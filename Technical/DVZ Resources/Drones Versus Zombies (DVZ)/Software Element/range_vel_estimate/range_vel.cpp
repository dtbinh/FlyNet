/*
This class subscribes to laser scans and attitude and publishes an estimate of 
the laser's velocity in the body coordinates. The estimation is based on the 
paper "Direct motion estimation from a range scan sequence" by Gonzalez and 
Gutierrez, 1999.
*/

#include "stdint.h"
#include <math.h>
#include "iostream"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "roscopter/Attitude.h"

typedef struct {
	float vx; //body x velocity in m/s
	float vy; //body y velocity in m/s
	float P[2][2]; //Velocity covariance in (m/s)^2
} Velocity_TypeDef;

#define MAX_POINTS 1024 //should be greater than the number of scan points

static float psi_dot = 0.0f, range_min = 0.0f, range_max = 10000.0f;

static const float range_variance = 1e-5f; //variance of range scans, m^2

static ros::Publisher rangePub;

//Range is within the minimum & maximum
static inline uint8_t isValidRange(const float scanValue) {
	return range_min < scanValue && scanValue < range_max;
}

inline void processAttitude(const roscopter::Attitude::ConstPtr& att) {
	psi_dot = att->yawspeed;
}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
	//Note: static variables inside a function persist with the same value
	//Across multiple calls
	
	//Stores the current and previous scans
	static float scan1[MAX_POINTS], scan2[MAX_POINTS];

	//number of scan points
	static uint16_t size = 0;

	//cos and sin at each angle
	static float trig[MAX_POINTS][2];

	//Determines whether scan1 is the current scan or scan2
	static uint8_t bufferSwitch = 0;
	
	//1/(2*angle_increment)
	static float d_theta_inv;

	//1/time_increment
	static float dt_inv = 0.0f;
	
	//temporary value of psi_dot so it stays the same throughout the function
	float pd = psi_dot;
	
	//size is 0 when this is the very first scan; initialize everything and quit
	if(size == 0) {
		d_theta_inv = 1.0f / (2.0f * scan->angle_increment);
		dt_inv = 1.0f / scan->time_increment;
		range_min = scan->range_min;
		range_max = scan->range_max;
		float theta = scan->angle_min;
		size = (uint16_t)((scan->angle_max - scan->angle_min) / 
			scan->angle_increment + 0.5f);
		for(uint16_t n = 0; n < size; n++) {
			scan2[n] = scan->ranges[n];
			trig[n][0] = cos(theta);
			trig[n][1] = sin(theta);
			theta += scan->angle_increment;
		}
		return;
	}
	
	//set the current and previous scans
	float *currentScan, *previousScan;
	if(bufferSwitch) {
		currentScan = scan2;
		previousScan = scan1;
	} else {
		currentScan = scan1;
		previousScan = scan2;
	}
	bufferSwitch = !bufferSwitch;
	
	//A*vel = B, where A is symmetric
	float A00 = 0.0f, A01 = 0.0f, A11 = 0.0f, B0 = 0.0f, B1 = 0.0f;
	
	//set first two values of scan since they aren't set in the for loop
	currentScan[0] = scan->ranges[0];
	currentScan[1] = scan->ranges[1];
	
	//loop through the middle ranges since the central finite difference is 
	//used for the partial of r with respect to theta
	for(uint16_t n = 1; n < size - 1; ++n) {
	
		//set next range value
		currentScan[n+1] = scan->ranges[n+1];
		
		//Make sure the range values are valid
		if(isValidRange(currentScan[n-1]) && isValidRange(currentScan[n]) && 
			isValidRange(currentScan[n+1]) && isValidRange(previousScan[n])) {
			
			//Renaming variables for readability
			const float &r = currentScan[n], &r0 = currentScan[n-1], 
				&r2 = currentScan[n+1], &pr = previousScan[n], 
				&c = trig[n][0], &s = trig[n][1];
				
			//Other variables
			const float r_2 = r * r, rs = r * s, rc = r * c;
			
			//partial of r with respect to theta
			const float dr_dtheta = (r2 - r0) * d_theta_inv;
			
			//partial of r with respect to t
			const float dr_dt = (r - pr) * dt_inv;
			
			//Other variables
			const float JR1 = dr_dtheta * c - rs;
			const float JR2 = dr_dtheta * s + rc;
			const float JR3 = dr_dtheta * dr_dtheta + r_2;
			const float Q = -pd * r_2 - dr_dtheta * dr_dt;
			const float P1 = JR1 / JR3, P2 = JR2 / JR3;			
			
			//Gaussian elimination (with some other math)
			B0 += pd * rs - dr_dt * c - Q * P1;
			B1 -= pd * rc + dr_dt * s + Q * P2;
			A00 += 1.0f - JR1 * P1;
			A01 -= JR2 * P1;
			A11 += 1.0f - JR2 * P2;
		}
	}
	
	//Solve for vx and vy
	
	//determinate of A
	float det = (A00 * A11 - A01 * A01);
	
	//Check that determinate is valid
	if(det != 0.0f && !isnan(det)) {
		//Solve for velocity and covariance
		Velocity_TypeDef vel;
		vel.vx = (A01 * B1 - A11 * B0) / det;
		vel.vy = (A00 * B1 - A01 * B0) / det;
		det = range_variance / det;
		vel.P[0][0] = A11 * det;
		vel.P[1][1] = A00 * det;
		vel.P[0][1] = vel.P[1][0] = -A01 * det;
		//rangePub.publish(vel);
	}
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "range_velocity_node");
	
	ros::NodeHandle nh;
	
	ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, &processLaserScan);
	ros::Subscriber attSub = nh.subscribe<roscopter::Attitude>("/Attitude", 10, &processAttitude);
	
	//rangePub = nh.advertise<std::msgs::Velocity>("range_velocity", 100);
	
	ros::spin();

/*
	//Test data obtained from matlab
	//The scanner is initially at (0, 0) with yaw of 0 rad
	//and moves at a velocity of (0.2, -0.1) m/s and rotates at a rate of 0.075 rad/s
	//The "wall" is a 1 m radius circle centered at (0, 0)
	LaserScan_TypeDef scan1;
	scan1.angle_min = -3.14159265358979;
	scan1.angle_max = 3.14159265358979;
	scan1.angle_increment = 1.25663706143592; 
	scan1.time_increment = 0.5;
	scan1.range_min = 0;
	scan1.range_max = 100.; 
	scan1.ranges[0] = 1.+9.99977878279879e-12;
	scan1.ranges[1] = 1.;
	scan1.ranges[2] = 1.;
	scan1.ranges[3] = 1.;
	scan1.ranges[4] = 1.;
	scan1.ranges[5] = 9.99977878279879e-12;
	
	LaserScan_TypeDef scan2;
	scan2.ranges[0] = 1.0966114978900008; 
	scan2.ranges[1] = 0.973163361520001;
	scan2.ranges[2] = 0.88899891133;
	scan2.ranges[3] = 0.94712120393;
	scan2.ranges[4] = 1.0784432122100005;
	scan2.ranges[5] = 1.0966114978900006;
	
	processLaserScan(&scan1);
	psi_dot = 0.075;
	Velocity_TypeDef vel = processLaserScan(&scan2);
	
	std::cout << vel.vx << std::endl << vel.vy << std::endl;
*/
	return 0;
}
