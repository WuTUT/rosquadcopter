#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <visionprocess/ROI.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
string Cascade_mode ="/home/wu/haarcascade_frontalface_default.xml";
CascadeClassifier Mycascade;
  


class VisionProcess
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  visionprocess::ROI roimsg;
  ros::Publisher roi_pub_;
  cv::Mat roi_frame;
public:
  VisionProcess()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &VisionProcess::imageCb, this);
    roi_pub_ = nh_.advertise<visionprocess::ROI>("/vision_node/roipos", 5);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~VisionProcess()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  Mat detectAndDisplay( Mat frame){
    // Draw an example circle on the video stream
   // if (frame.rows > 60 && frame.cols > 60)
    //  cv::circle(frame, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //TODO
    std::vector<Rect> object;
    Mat frame_gray(frame.size(),CV_8U);
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
   //Mycascade.detectMultiScale( frame_gray, object, 1.1, 5, 0, Size(40,40),Size(70,70) );//test small samples high
   //Mycascade.detectMultiScale( frame_gray, object, 1.1, 5, 0, Size(40,40),Size(120,120) );//test big samples low
   Mycascade.detectMultiScale( frame_gray, object, 1.1, 3, 0, Size(40,40),Size(100,100) );
    for( int i = 0; i < object.size(); i++ )  {
        rectangle(frame,
                  object[i],
                  Scalar(0, 255, 0),
                  1);

       
       roimsg.x=object[i].x;
       roimsg.y=object[i].y;
       roimsg.width=object[i].width;
       roimsg.height=object[i].height;
       
       roi_pub_.publish(roimsg);
       
       
  }
       return frame;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    roi_frame= detectAndDisplay(cv_ptr->image);
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, roi_frame);
    cv::waitKey(30);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  
  if(!Mycascade.load( Cascade_mode ))  
    cout<< "[error] 无法加载级联分类器文件！" << endl;  
  
  VisionProcess vp_;
  ros::spin();
  return 0;
}

