#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
    int points[5][2] = {{0,0},{2,0},{2,2},{0,2},{0,0}};
    int count = 0;
    

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped",10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::param::get("/square/altitude", alt);
    ros::param::get("/square/side_length", side_length);
    ros::param::get("/square/threshold", threshold);
    ros::param::get("/square/velocity", velocity)

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist vel;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = alt;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode set_land_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_land_mode.request.custom_mode = "LAND";

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

    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(0.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
    } 

    while(ros::ok()){

        if( std::abs(current_pose.pose.position.x - pose.pose.position.x) < threshold &&
            std::abs(current_pose.pose.position.x - pose.pose.position.x) < threshold )
        {
            pose.pose.position.x = points[count][0];
            pose.pose.position.y = points[count][1];
            pose.pose.position.z = alt;
            vel.linear.x = velocity;
            count ++;
        }    
        else if( count == 6 )
        {
            if ( set_mode_client.call(set_land_mode) && set_land_mode.response.mode_sent)
            {
                ROS_INFO("Landing...");
                break;
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}