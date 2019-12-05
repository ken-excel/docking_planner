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
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class path_tester
{
private:
	ros::NodeHandle nh_;
	ros::Publisher path_pub_;
	ros::Publisher path2_pub_;
	ros::Publisher test_pub_;
	geometry_msgs::PoseStamped poseGoal;
	pcl::PointCloud<pcl::PointXYZ> path, path2;
	double lh_dist;					//lookahead distance
	int point_num;					//number of point on the path

public:
	path_tester():				//Constructor
	nh_("~"),
	lh_dist(0.1),
	point_num(1000)
	{
		path_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/docking_path_test", 1);
		path2_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/docking_path_test2", 1);
		test_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_test", 1);
	}
	double ix,iy,it;				//initial position x,y,theta
	double gx,gy,gt;				//goal x,y, theta
	void pathGenerate(int n);
};

void path_tester::pathGenerate(int n)
{
	std::cout<<"Generating Path: "<<n<<std::endl;
	std::cout<<"Goal: "<<gx<<", "<<gy<<std::endl;
	path.header.frame_id = "base_footprint";
	pcl_conversions::toPCL(ros::Time::now(), path.header.stamp);
	path.points.resize(point_num);
	path2.header.frame_id = "base_footprint";
	pcl_conversions::toPCL(ros::Time::now(), path2.header.stamp);
	path2.points.resize(point_num);
	double x0, x1, x2, x3;
	double y0, y1, y2, y3;
	double x0_r, x1_r, x2_r, x3_r;
	double y0_r, y1_r, y2_r, y3_r;
	double x0_o, x1_o, x2_o, x3_o;
	double y0_o, y1_o, y2_o, y3_o;
    switch(n){
    /*Do something*/
    	case 2:
    	// Cube Bezier
		{
			//Publish goal point
			poseGoal.pose.position.x = gx;
			poseGoal.pose.position.y = gy;
			poseGoal.pose.position.z = 0;
			tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, gt); 
	  		tf::quaternionTFToMsg (q, poseGoal.pose.orientation); 
			poseGoal.header.frame_id = "base_footprint";
    		test_pub_.publish(poseGoal);

    		//In rotated frame
		    x3_r = cos(gt)*gx+sin(gt)*gy;
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

		    //Path without concerning the point
		    x3_o = gx;
		    y3_o = gy;

		    x2_o = gx*0.5;
		    y2_o = gy;

		    x1_o = gx*0.2;
		    y1_o = gy*0.8;

		    x0_o = 0;
		    y0_o = 0;
		    double d_PathPoint = 1.0/(point_num-1);                                                             // Cubic Bezier curve f(t) is the curve generated with t = [0,1]. This command line to find the difference of t between 2 path points
		    double t = 0.0;                                                                                     // "t" ranges from 0 to 1
		    for (int i = 0 ; i < point_num; i++)                                                                // Calculate path point data from t = 0 to t = 1
		    {
		        path.points[i].x = x0 + 3*t*(x1-x0) + 3*t*t*(x0+x2-2*x1) + t*t*t*(x3-x0+3*x1-3*x2);
		        path.points[i].y = y0 + 3*t*(y1-y0) + 3*t*t*(y0+y2-2*y1) + t*t*t*(y3-y0+3*y1-3*y2);
		        path.points[i].z = 0;                                                        // Set reflectivity of point on the path to be 1000.
		        path2.points[i].x = x0_o + 3*t*(x1_o-x0_o) + 3*t*t*(x0_o+x2_o-2*x1_o) + t*t*t*(x3_o-x0_o+3*x1_o-3*x2_o);
		        path2.points[i].y = y0_o + 3*t*(y1_o-y0_o) + 3*t*t*(y0_o+y2_o-2*y1_o) + t*t*t*(y3_o-y0_o+3*y1_o-3*y2_o);
		        path2.points[i].z = 0;         
		        t = t + d_PathPoint;
		    }
		    break;
		}
	    case 1:
		// Linear
		{	
		    // Generate fixed path points
		    x2 = gx;   //goal point
		    y2 = gy+lh_dist;

		    x1 = gx;   //control point 1
		    y1 = iy;

		    x0 = ix;   //start point
		    y0 = iy;

		    point_num -= 1;

		    double d_PathPoint_hor = (x1 - x0)/((point_num+1)/2 -1);
		    double d_PathPoint_ver = (y2 - y1)/((point_num+1)/2 -1);
		    for (int i = 0 ; i < (point_num+1)/2; i++)
		    {
		        path.points[i].x = x0 + (double)(d_PathPoint_hor*i);
		        path.points[i].y = y0;
		        path.points[i].z = 0;
		    }

		    for (int i = 1; i < (point_num+1)/2 ; i++)
		    {
		        path.points[i+(point_num+1)/2 -1].x = x2;
		        path.points[i+(point_num+1)/2 -1].y = y1 + (double)(d_PathPoint_ver*i);
		        path.points[i+(point_num+1)/2 -1].z = 0;
		    }
		    break;
		}
		default:
		// Linear
		{	
		    // Generate fixed path points
		    x2 = gx;   //goal point
		    y2 = gy+lh_dist;

		    x1 = gx;   //control point 1
		    y1 = iy;

		    x0 = ix;   //start point
		    y0 = iy;

		    point_num -= 1;

		    double d_PathPoint_hor = (x1 - x0)/((point_num+1)/2 -1);
		    double d_PathPoint_ver = (y2 - y1)/((point_num+1)/2 -1);
		    for (int i = 0 ; i < (point_num+1)/2; i++)
		    {
		        path.points[i].x = x0 + (double)(d_PathPoint_hor*i);
		        path.points[i].y = y0;
		        path.points[i].z = 0;
		    }

		    for (int i = 1; i < (point_num+1)/2 ; i++)
		    {
		        path.points[i+(point_num+1)/2 -1].x = x2;
		        path.points[i+(point_num+1)/2 -1].y = y1 + (double)(d_PathPoint_ver*i);
		        path.points[i+(point_num+1)/2 -1].z = 0;
		    }
		    break;
		}
	}
    path_pub_.publish(path);
    path2_pub_.publish(path2);
    return;
}

int main(int argc, char** argv){
	std::cout << "Path Tester" << std::endl;

	ros::init(argc, argv, "path_tester");
	path_tester tester;

	ros::Rate rate(10);

	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		while (std::cin.get() != '\n'){
			if (std::cin.get() == 'c'){
				std::cout<<"manually terminating..."<<std::endl;
				ros::shutdown();
				break;
			}
		}
		std::cout<<"Input your end point"<<std::endl;
		std::cin>>tester.gx>>tester.gy;
		std::cout<<"Input your end point's angle"<<std::endl;
		std::cin>>tester.gt;
		tester.gt = tester.gt*M_PI/180;
		//std::cout<<"Input trajectory type 1:linear 2:cubic bezier (default: linear)"<<std::endl;
		//int mode;
		//std::cin>>mode;
		tester.pathGenerate(2);
	}
	return 0;
}
