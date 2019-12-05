//=====================================================================================================================================================================
//
//	Docking Planner V.1
//
//	Author : Promsutipong Kengkij
//  Contact: k.promsutipong@srd.mech.tohoku.ac.jp
//
//=====================================================================================================================================================================

#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <kalman_filter.h>

class docking_planner
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber pos_sub_;
	ros::Subscriber robot_sub_;
	ros::Publisher cmd_pub_;
	ros::Publisher path_pub_;
	geometry_msgs::PoseStamped poseGoal, last_msg;
	geometry_msgs::Twist cmd_vel;
	pcl::PointCloud<pcl::PointXYZ> path;  
	tf::TransformListener listener;
	kalman_filter filter;
	std::string	goal_frame, robot_frame, fixed_frame, goal_topic, robot_topic, output_vel_topic, docking_path_topic;
	double vel;
	int ind;
	double refresh;					//counter for number of loop passed
	bool robot_ready;				//check if robot odometry is received
	bool goal_ready; 				//check if goal data/ camera data is received or not
	bool final_adjust;				//state when the robot approaching the goal
	bool goal_reach;				//state when the robot reached the goal
	bool callback_firstloop;		//check if the system runs on the first loop or not (for initializing purpose)
	double offset;					//offset according to robot+station size
	double rx,ry,rt;				//robot x,y,theta
	double gx,gy,gt;				//goal x,y, theta
	double tx,ty,tt;				//required x,y,theta to reach lookahead point
	double lh_dist;					//lookahead distance
	double k;						//turning coefficient
	int point_num;					//number of point on the path

public:
	docking_planner():				//Constructor
	nh_("~"),
	ind(0),
	refresh(0),
	final_adjust(false),
	callback_firstloop(true),
	offset(0.05),
	point_num(1000),
	goal_reach(false),
	goal_ready(false)
	{
  		nh_.param<double>("velocity", vel, 0.15);
  		nh_.param<double>("rotational gain", k, 0.3);
  		nh_.param<double>("lookahead distance", lh_dist, 0.1);
  		nh_.param<std::string>("goal_frame", goal_frame, "goal");
  		nh_.param<std::string>("robot_frame", robot_frame, "base_link");
  		nh_.param<std::string>("fixed_frame", fixed_frame, "odom");
  		nh_.param<std::string>("goal_topic", goal_topic, "");
  		nh_.param<std::string>("robot_topic", robot_topic, "/odom");
  		nh_.param<std::string>("output_vel_topic", output_vel_topic, "/cmd_vel");
  		nh_.param<std::string>("docking_path_topic", docking_path_topic, "/docking_path");
		pos_sub_ = nh_.subscribe(goal_topic, 1, &docking_planner::goalPose, this);
		robot_sub_ = nh_.subscribe(robot_topic, 1, &docking_planner::robotPose, this); //Sub to robot pose or odom?
		cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_vel_topic, 1);
		path_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (docking_path_topic, 1);
	}
	void goalPose(const geometry_msgs::PoseStamped &msg);
	void robotPose(const nav_msgs::Odometry::ConstPtr& msg);
	bool init();
	void mainProcess();
	void pathGenerate();
	void purePursuit();
	void stop();
	double getYaw(geometry_msgs::Pose &msg);
};

void docking_planner::goalPose(const geometry_msgs::PoseStamped &msg)										//Callback to visp auto tracker's object position
{
	if(callback_firstloop){
		callback_firstloop = false;
		listener.waitForTransform(robot_frame, goal_frame, ros::Time(0), ros::Duration(1));		//Listen to tf from camera_link to base_footprint    
		try{
			listener.transformPose(robot_frame,msg,poseGoal);	
			last_msg = msg;
		}
		catch( tf::TransformException ex)
		{
			ROS_ERROR("transfrom exception : %s",ex.what());
		}
		return;
	}
	if(msg.pose.orientation.z != last_msg.pose.orientation.z){			//check if goal is still being seen
		listener.waitForTransform(robot_frame, goal_frame, ros::Time(0), ros::Duration(1));		//Listen to tf from camera_link to base_footprint    
		try{
			listener.transformPose(robot_frame,msg,poseGoal);		
			last_msg = msg;
			goal_ready = true;	
		}	
		catch( tf::TransformException ex)
		{
			ROS_ERROR("transfrom exception : %s",ex.what());
		}
	}
	else{
		std::cout<<"tracking lost"<<std::endl;
		goal_ready = false;
	}	
}

void docking_planner::robotPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	rx = msg->pose.pose.position.x;
	ry = msg->pose.pose.position.y;
	geometry_msgs::Pose temp = msg->pose.pose;
	rt = getYaw(temp);
	robot_ready = true;
}

double docking_planner::getYaw(geometry_msgs::Pose &msg)
{
	tf::Quaternion q( 
    	msg.orientation.x, 
    	msg.orientation.y,
    	msg.orientation.z,
    	msg.orientation.w);
	tf::Matrix3x3 m(q); 
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw); 
	return yaw;
}

bool docking_planner::init()		//First setup
{	
	if(goal_ready&&robot_ready){
		std::cout<<"Initialize"<<std::endl;
		pathGenerate();					//Generate the Path
	}
	return (goal_ready&&robot_ready);
}

void docking_planner::mainProcess()		//Main Process, will never come here before initialization
{
	if(!goal_reach){
		//Check if the goal is close or not
		if(gx<3*lh_dist){
			final_adjust = true; 
		}
		
		double temp_distance;
		double shortest_distance = 1000;
		double temp_dx, temp_dy; 
		double L=0.0;                                                                                  
        double dx,dy;                                                                               
        
        if(!final_adjust){
        	//If refresh the goal, generate new path
	        if(refresh>=1&&goal_ready){
	        	std::cout<<"Refresh"<<std::endl;
	        	pathGenerate();
	        	refresh = 0;
	        }
	        refresh+=0.1;
		}

		//Start from the current index, Find the point on path nearest to the robot
		for(int i = 0; i<path.size();i++)                                                   
        {
	           temp_dx = path.points[i].x-rx;                                             
	           temp_dy = path.points[i].y-ry;                                                      
	           temp_distance = sqrt(temp_dx*temp_dx + temp_dy*temp_dy);                     
			if (shortest_distance > temp_distance)                                         
	        {
	            ind = i;                                                                           
	            shortest_distance = temp_distance;                                                   
	        }
            //TODO:Make this more efficient, now it's running through the whole remaining path
        }

        // search look ahead target point index
        while(lh_dist > L && ind < path.size())                                                   
        {
            dx = path.points[ind+1].x - path.points[ind].x;                            
            dy = path.points[ind+1].y - path.points[ind].y;
            L += sqrt(dx*dx + dy*dy);
            ind ++;                                                                             
        }

        //Pure Pursuit complete
        if(ind>= path.size()){
        	goal_reach = true; 
        }
        //Pure pursuit incomplete
    	else{
    		tx = path.points[ind].x-rx;
        	ty = path.points[ind].y-ry;
        	tt = atan2(ty,tx)-rt;
        }

        if(!goal_reach){
        	if(!final_adjust){
        		cmd_vel.linear.x = vel;
        		cmd_vel.angular.z = 2* vel * sin(tt) * (1/lh_dist) * k; 
			}
			if(final_adjust){
				cmd_vel.linear.x = vel/3;
        		cmd_vel.angular.z = 2* vel/2 * sin(tt) * (1/lh_dist) * k; 	
			}
        	cmd_pub_.publish(cmd_vel);
        }
		else{
			//Goal reached, stop the robot
			cmd_vel.linear.x = 0;
        	cmd_vel.angular.z = 0; 
			std::cout<<"Goal reached"<<std::endl;
        	cmd_pub_.publish(cmd_vel);	
		}
	}
	else{
		//Mission already finished
		cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_pub_.publish(cmd_vel);	
	}
	return;
}

void docking_planner::pathGenerate()
{
	//initialize path
	path.header.frame_id = fixed_frame;
	pcl_conversions::toPCL(ros::Time::now(), path.header.stamp);
    
	//initialize temporary path
	pcl::PointCloud<pcl::PointXYZ> temp;
	temp.header.frame_id = robot_frame;
	pcl_conversions::toPCL(ros::Time::now(), temp.header.stamp);

    //Cubic Bezier Curve
    temp.points.resize(point_num);
	path.points.resize(point_num);
    double x0, x1, x2, x3;
    double y0, y1, y2, y3;
	double x0_r, x1_r, x2_r, x3_r;
	double y0_r, y1_r, y2_r, y3_r;
    //In rotated frame
    x3_r = cos(gt)*gx+sin(gt)*gy+lh_dist;
    y3_r = -sin(gt)*gx+cos(gt)*gy;

    x2_r = x3_r*0.5;   
    y2_r = y3_r;

    x1_r = x3_r*0.2;   //control point 1
    y1_r = y3_r*0.8;

    x0_r = 0;   //start point
    y0_r = 0;

    //Convert to robot frame w/o rotation
    x3 = cos(gt)*x3_r-sin(gt)*y3_r;
    y3 = sin(gt)*x3_r+cos(gt)*y3_r;

    x2 = cos(gt)*x2_r-sin(gt)*y2_r;
    y2 = sin(gt)*x2_r+cos(gt)*y2_r;

    x1 = cos(gt)*x1_r-sin(gt)*y1_r;
    y1 = sin(gt)*x1_r+cos(gt)*y1_r;

    x0 = 0;
    y0 = 0;

    double d_PathPoint = 1.0/(point_num-1);                                                             
    double t = 0.0;                                                                                     
    for (int i = 0 ; i < point_num; i++)  
    {
        temp.points[i].x = x0 + 3*t*(x1-x0) + 3*t*t*(x0+x2-2*x1) + t*t*t*(x3-x0+3*x1-3*x2);
        temp.points[i].y = y0 + 3*t*(y1-y0) + 3*t*t*(y0+y2-2*y1) + t*t*t*(y3-y0+3*y1-3*y2);
        temp.points[i].z = 0;
        path.points[i].x = cos(rt)*temp.points[i].x-sin(rt)*temp.points[i].y+rx;
		path.points[i].y = sin(rt)*temp.points[i].x+cos(rt)*temp.points[i].y+ry;
		path.points[i].z = 0;           
        t = t + d_PathPoint;
    }
    ind = 0;
	
    path_pub_.publish(path);
    return;
}

void docking_planner::stop()
{
	geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_pub_.publish(cmd_vel);
    return;
}

docking_planner* c_docking_planner;

void sig_handler(int sig){ 
	if(c_docking_planner != NULL){
		c_docking_planner->stop();
	}
	ros::shutdown();
}

int main(int argc, char** argv){
	std::cout << "Docking Robot..." << std::endl;

	ros::init(argc, argv, "docking_planner", ros::init_options::NoSigintHandler);
	c_docking_planner = new docking_planner();
	signal(SIGINT, sig_handler);

	ros::Rate rate(10);
	ros::Rate init_rate(1);
	std::cout << "Waiting For Docking Station's Pose..." << std::endl;
	bool init_ok = false;

	//initialization loop
	while(!init_ok){
		ros::spinOnce();
		init_ok = c_docking_planner->init();
		init_rate.sleep(); 	
	}

	//main loop
	while(ros::ok())
	{
		ros::spinOnce();
		c_docking_planner->mainProcess();
		rate.sleep();    
	}
	return 0;
}
