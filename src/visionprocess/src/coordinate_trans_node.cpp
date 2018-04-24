#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visionprocess/ROI.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
using namespace cv;



class CoordinateTrans{
    
    ros::Subscriber roi_sub;
    ros::Subscriber current_pos_sub;
    
    float pixel_x,pixel_y;
    tf::Transform transform;
    tfScalar roll, pitch, yaw;
    
public:
  CoordinateTrans(ros::NodeHandle nh)
  {
    ros::Subscriber roi_sub = nh.subscribe<visionprocess::ROI>("/vision_node/roipos",5,&CoordinateTrans::roipos_cb,this );
    ros::Subscriber current_pos_sub=nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",100,&CoordinateTrans::localpos_cb,this);
  }

  ~CoordinateTrans()
  {
    
  }
  void roipos_cb(const visionprocess::ROI::ConstPtr& roimsg){
    
    pixel_x=roimsg->x+roimsg->width/2;
    pixel_y=roimsg->y+roimsg->height/2;
  }
  void localpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    
    tf::Quaternion q_base_link(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m_base_link(q_base_link);

    m_base_link.getRPY(roll, pitch, yaw);
    

    transform.setRotation(q_base_link);
    

  }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordinate_trans_node");
    ros::NodeHandle nh;
    CoordinateTrans ct_(nh);
    ros::Rate rate(100.0);
    ros::spinOnce();
    rate.sleep();
    

}
