#include <ros/ros.h>
#include <mavros/mavros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void posCallback(const geometry_msgs::PoseStamped& msg)
{

  float x_pos = msg.pose.position.x;
  float y_pos = msg.pose.position.y;
  float z_pos = msg.pose.position.z;
  float x_orr = msg.pose.orientation.x;
  float y_orr = msg.pose.orientation.y;
  float z_orr = msg.pose.orientation.z;
  float w_orr = msg.pose.orientation.w;
  ROS_INFO("z pos: %f, z_orr: %f", z_pos,z_orr);
}


void GPSCallback(const sensor_msgs::NavSatFix& msg)
{

  float lat_pos = msg.latitude;
  float lon_pos = msg.longitude;
  float alt_pos = msg.altitude;

  ROS_INFO("Lat: %f, Lon: %f, Alt: %f", lat_pos,lon_pos,alt_pos);
}

int main(int argc, char **argv)
{
  // Ros Init
  ros::init(argc, argv, "px4_flynet_node");
  // Ros Handle
  ros::NodeHandle n;
  
  //======= ROS Publishers ================
  ros::Publisher mavros_pos_control_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000);  
  geometry_msgs::PoseStamped pos_sp;
  
  ros::Publisher mavros_vis_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1000);  
  geometry_msgs::PoseStamped vis_pos;
  
  
  
  //======= ROS Subscribers ===============
  // Ros setup async spinners for subscibers
  ros::AsyncSpinner spinner(2);// Do we need 2 threads?
  // Setup Subscribers:
  ros::Subscriber sub_pos = n.subscribe("/mavros/local_position/local", 1000, posCallback);
  ros::Subscriber sub_gps = n.subscribe("/mavros/global_position/raw/fix", 1000, GPSCallback);
  // Start the Spinner
  spinner.start();
  
  //==== ROS Clients/Services ==============
  ros::ServiceClient mavros_set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode set_mode;
  set_mode.request.custom_mode = "OFFBOARD";
  
  ros::ServiceClient mavros_nav_guided_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/guided_enable");
  mavros_msgs::CommandBool nav_guided;
  nav_guided.request.value = true; // Need to figure out this stuff
  
  bool offboard_commands_enabled = false;
  bool nav_guided_enabled = false;
  
  // Publisher Loop
  ros::Rate loop_rate(100.0);
  int count = 0;
  while (ros::ok()){
    if (!offboard_commands_enabled) {
      if (mavros_set_mode_client.call(set_mode))
      {
        ROS_INFO("Set mode: OFFBOARD enabled!");
		offboard_commands_enabled = true;
      }
      else
      {
        ROS_INFO("Offboard mode still not enabled!");
      }
    }

    // Write desired setpoint value to pos_sp 
	pos_sp.header.seq = count;
	pos_sp.header.stamp = ros::Time::now();
	pos_sp.header.frame_id = "fcu";
	pos_sp.pose.position.x = 1;
	pos_sp.pose.position.y = 10;
	pos_sp.pose.position.z = 2;
	pos_sp.pose.orientation.w = 0; 
	
	// Write current position value to vis_pos 
	vis_pos.header.seq = count;
	vis_pos.header.stamp = ros::Time::now();
	vis_pos.header.frame_id = "fcu";
	vis_pos.pose.position.x = (double)((rand() % 10 +1)-5);
	vis_pos.pose.position.y = (double)((rand() % 10 +1)-5);
	vis_pos.pose.position.z = (double)((rand() % 4 +1));
	
	
	
	
    mavros_pos_control_pub.publish(pos_sp);
    
    mavros_vis_pos_pub.publish(vis_pos);
    
    if(!nav_guided_enabled)
    {
      if (mavros_nav_guided_client.call(nav_guided))
      {
		nav_guided_enabled = true;
		ROS_INFO("Nav guided: OFFBOARD enabled!");
      }
    }
	// Spin Once:
    ros::spinOnce();
	// Maintain loop rate
    loop_rate.sleep();
  }
  
  return 0;
}
