#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>

using namespace std;

class kalman_filter{
private:
	int R,Q,A,B,C,u;
	double x, cov;
	bool first_data;
public:
	kalman_filter():
	R(1),
	Q(3),
	A(1),
	B(0),
	C(1),
	u(0),
	first_data(true)
	{}

	double get_input(double input);
};

double kalman_filter::get_input(double input) 
{
	if (first_data) {
	    x = (1 / C) * input;
   		cov = (1 / C) * Q * (1 / C);
	    first_data = false;
	}
    else {

      // Compute prediction
      double predX = (A * x) + (B * u);
      double predCov = ((A * cov) * A) + R;

      // Kalman gain
      double K = predCov * C * (1 / ((C * predCov * C) + Q));

      // Correction
      x = predX + K * (input - (C * predX));
      cov = predCov - (K * C * predCov);
    }
    return x;
}
