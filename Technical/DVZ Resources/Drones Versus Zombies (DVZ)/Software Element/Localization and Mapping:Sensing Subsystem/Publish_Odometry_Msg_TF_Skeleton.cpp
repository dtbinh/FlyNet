<depend package="tf"/>
<depend package="nav_msgs">

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // Get initial pose from estimate

/*
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
*/


  // Use initial pose for initialization
  // Might not be necessary for odom frame, could just have a constant error source due to initialization
  // AMCL corrects for odom to map, so initial pose probably does not have to be used
  // Should be able to initialize with all zeros -> more simple
  ros::NodeHandle initialpose;
  ros::Subscriber initialposeSub;

  
  initialposeSub = initialpose.subscribe<initialposemsgtype::pose>("/initialpose", 10);

  double x = initialposeSub.x;
  double y = initialposeSub.y;
  double th = initialposeSub.th;


// Get velocities: x, y from optical flow sensor (Fused with accelerometer?)
// Get yaw rate from imu fusion (already done on autopilot)
/*
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
*/

// Guess at how to get velocities

  ros::NodeHandle PX4;			// Handle for node
  ros::Subscriber xVelSub;		// Name of subscriber
  ros::Subscriber yVelSub;      // Might not need, if velocity message has both
  ros::NodeHandle IMU_Yaw;		// Node handle for IMU, could bundle with other node
  ros::Subscriber yawSub;		// Name of subscriber

  // Definition of subscription
  // Uses PX4 node handle
  // Replace PX4_msgs::velocity with actual message type
  // Replace /xvel with name of topic
  // 10 Hz sampling rate
  // Need to include dependencies at top of code, define PX4_msgs
  xVelSub = PX4.subscribe<PX4_msgs::velocity>("/xvel", 10);

  // Might be unecessary
  yVelSub = PX4.subscribe<PX4_msgs::velocity>("/yvel", 10);

  // Get yaw rate from IMU
  // Replace IMU_msgs::angular with actual message type
  // Replace /yaw with name of topic
  // 10 Hz sampling rate
  // Include dependencies at top of code
  yawSub = IMU_Yaw.subscribe<IMU_msgs::angular>("/yaw", 10)
  

  // Assign the velocities
  // All of this might have to go in the loop, not sure based off of example
  // Instead of using variables vx, vy, and vth -> could use xVelSub, yVelSub, and yawSub directly?
  // What is a subscriber really? how can you assign the value to a double for use in calculations?
  double vx = xVelSub;
  double vy = yVelSub;
  double vth = yawSub;
 


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  // Increase publish rate
  // Originally 1 Hz
  // Changed to 10 Hz
  ros::Rate r(10.0);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


    // Might use subscribed data directly here instead of vx, vy, and th
    // Depends on data format of a "subscriber"

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}