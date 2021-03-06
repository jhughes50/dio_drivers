#include <ros/ros.h>                                                                              
#include <ros/console.h>                                                                          
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <iostream>

geometry_msgs::PoseStamped mocap_pose;
void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mocap_pose = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_pose");
  ros::NodeHandle nh;

  std::string name;
  std::string vrpn_topic = "vrpn_client_node/";
  std::string pose_topic = "/pose";

  ros::param::get("/mocap_pose/name", name);
  
  vrpn_topic.append(name);
  vrpn_topic.append(pose_topic);
  
  ros::Subscriber mocap_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    (vrpn_topic, 10, vrpn_pose_cb);
  ros::Publisher mocap_to_pixhawk = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/mocap/pose", 10);
  
  ros::Rate rate(40.0);
  
  while( ros::ok() ){

    mocap_to_pixhawk.publish(mocap_pose);
    
    ros::spinOnce();
    rate.sleep();
    
  }
    
  return 0;
}
