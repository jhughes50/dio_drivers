#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
  current_pose = *msg;
  std::cout << current_pose.pose.position.x << "\n";
  std::cout << current_pose.pose.position.y << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_swatooth");
  ros::NodeHandle nh;

  double min_alt, max_alt, travel, threshold, wavelength;
  const double PI = 3.14159265358979323846;
  
  ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("mavros/local_position/pose", 10, pose_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state",10, state_cb);

  ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>
    ("mavros/setpoint_raw/local", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  ros::param::get("/sawtooth/min_alt", min_alt);
  ros::param::get("/sawtooth/max_alt", max_alt);
  ros::param::get("/sawtooth/travel", travel);
  ros::param::get("/general/threshold", threshold);
  ros::param::get("/sawtooth/wavelength", wavelength);
  
  ros::Rate rate(20.0);
  
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::PositionTarget pose;
  pose.coordinate_frame = 1;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = min_alt;

  for(int i = 100; ros::ok() && i > 0; --i){
    local_pose_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode set_offboard;
  mavros_msgs::SetMode set_land;
  set_offboard.request.custom_mode = "OFFBOARD";
  set_land.request.custom_mode = "AUTO.LAND";

  if( current_state.mode != "OFFBOARD" ){
    if( set_mode_client.call(set_offboard) && set_offboard.response.mode_sent ){
      ROS_INFO("Offboard Enabled");
      }
  }

  int count = 0;

  int num_wps = travel/wavelength;
  std::cout << "numwps: "<< num_wps << "\n";  
  while( ros::ok() ){

    if( count == 2*(num_wps)+1 ){
      ROS_INFO("set to land");
      if( set_mode_client.call(set_land) && set_land.response.mode_sent ){
	ROS_INFO("Landing...");
	break;
      }
    }
    std::cout << "x dff  " << std::abs(current_pose.pose.position.x - pose.position.x) << "\n";
    std::cout << "y dff  " << std::abs(current_pose.pose.position.y - pose.position.y) << "\n";
    std::cout << "curr x  " << current_pose.pose.position.x << "\n";
    std::cout << "curr y  " << current_pose.pose.position.y << "\n";
    if ( std::abs(current_pose.pose.position.x - pose.position.x) < threshold &&
	 std::abs(current_pose.pose.position.y - pose.position.y) < threshold){
      ++count;
      pose.position.x = 0;
      pose.position.y = count * num_wps;
      pose.yaw = PI/2;
      if( count % 2 == 0 ){
	pose.position.z = min_alt;
      }
      else{
	pose.position.z = max_alt;
      }
      pose.coordinate_frame = 1;
      ROS_INFO("WP Reached");
      std::cout << "x  " << pose.position.x << "\n";
      std::cout << "y  " << pose.position.y << "\n";
      std::cout << "count  " << count << "\n";
    }
    
    local_pose_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;

}
