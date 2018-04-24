/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
using namespace std;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
bool tracking_flag=false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);        
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");     
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

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
    
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD"&&current_state.armed){
            std::cout<<"set over"<<endl;
            break;
        }
    }
    
    ROS_INFO("going to Track");
    tracking_flag=true;
   /* while(ros::ok()&&tracking_flag){

    }*/
   /* for(int i = 0; ros::ok() && i < 200; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");
    // go to the third waypoint
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.3;
    pose.pose.position.z = 2;
    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 200; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");
*/
    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
                land_cmd.response.success)){
        //local_pos_pub.publish(pose);
        ROS_INFO("tring to land");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}