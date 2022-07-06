#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandInt.h>
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
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_square");
    ros::NodeHandle nh;
    double alt, side_length, threshold, velocity;
    int count = 0;
    int repeat;

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped",10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient set_vel_client = nh.serviceClient<mavros_msgs::CommandInt>
            ("mavros/cmd/command_int");

    ros::param::get("/square/altitude", alt);
    ros::param::get("/square/side_length", side_length);
    ros::param::get("/general/threshold", threshold);
    ros::param::get("/square/velocity", velocity);
    ros::param::get("/square/repeat", repeat);

    double points[5][3] = {{0.0,0.0,0.0},{side_length,0.0,0.0},{side_length,side_length,M_PI_2},{0.0,side_length,M_PI},{0.0,0.0,3*M_PI_2}};

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pose;
    geometry_msgs::Twist vel;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = alt;
    pose.coordinate_frame = 1;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode set_land_mode;
    mavros_msgs::CommandInt set_vel;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_land_mode.request.custom_mode = "AUTO.LAND";
    set_vel.request.command = 178;
    set_vel.request.param1 = 1;
    set_vel.request.param2 = 0.1;

    if ( set_vel_client.call(set_vel) && set_vel.response.success)
    {
        ROS_INFO("Set Velocity");
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if( !current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(0.0))){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            }
        last_request = ros::Time::now();
     }

    if( current_state.mode != "OFFBOARD") //&&
    {
        //(ros::Time::now() - last_request > ros::Duration(0.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
    } 
    ROS_INFO("Entered Loop");
    while(ros::ok()){

        if( count == 6 )
        {
            ROS_INFO("Set to land");
            if ( set_mode_client.call(set_land_mode) && set_land_mode.response.mode_sent)
            {
                ROS_INFO("Landing...");
                break;
            }
        }
	  
	if( std::abs(current_pose.pose.position.x - pose.position.x) < threshold &&
	    std::abs(current_pose.pose.position.y - pose.position.y) < threshold )
	  {
	    pose.position.x = points[count][0];
	    pose.position.y = points[count][1];
	    pose.position.z = alt;
	    pose.yaw = points[count][2];
	    pose.coordinate_frame = 1;
	    ROS_INFO("WP REACHED");
	    count ++;
	  }
        local_pos_pub.publish(pose);
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
