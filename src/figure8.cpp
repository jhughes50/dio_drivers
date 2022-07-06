#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

int get_angle(double y2, double y1, double x2, double x1)
{
    // return angle between two points in radians
    // p2 where youre going, p1 where youre at.
    return atan2(y2-y1, x2-x1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "figure_eight");
    ros::NodeHandle nh;
    double alt, alpha, threshold;
    int density;
    const double pi = 3.14159265358979323846;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::param::get("/general/threshold", threshold);
    ros::param::get("/figure8/altitude", alt);
    ros::param::get("/figure8/alpha", alpha);
    ros::param::get("/figure8/density", density);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pose;
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
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    // make sure it's armed 
    if( !current_state.armed){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            }
        last_request = ros::Time::now();
     }
    // change to offboard mode
    if( current_state.mode != "OFFBOARD"){// &&
      //(ros::Time::now() - last_request > ros::Duration(0.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("OFFBOARD enabled");
        }
    } 
    //generate waypoints 
    double x[density], y[density];
    double theta;
    std::cout<<"desnity: " << density << "\n";
    for (int i = 0; i <= density+1; i++)
    {
      //std::cout << i << " \n ";
      theta = (i/(1.0*density)) * (2*pi);
      std::cout << theta << " \n ";
      x[i] = alpha * sin(theta) * cos(theta);
      y[i] = alpha * sin(theta);
    }
    
    int count = 0;

    for (int i=0; i<density+1; i++){
      std::cout<<x[i]<<","<<y[i]<<"\n";
    }
    
    
    while(ros::ok())
    {
        
        if( std::abs(current_pose.pose.position.x - pose.position.x) < threshold &&
            std::abs(current_pose.pose.position.y - pose.position.y) < threshold )
        {
            pose.position.x = x[count];
            pose.position.x = y[count];
            pose.position.z = alt;
            pose.yaw = get_angle(y[count],current_pose.pose.position.y,x[count],current_pose.pose.position.x);
            pose.coordinate_frame = 1;
            count++;
	    ROS_INFO("WP REACHED");
            local_pos_pub.publish(pose);
        }
        else
        {
            pose.coordinate_frame = 1;
            pose.yaw = get_angle(pose.position.y,current_pose.pose.position.y,pose.position.x,current_pose.pose.position.x);
            local_pos_pub.publish(pose);
        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
