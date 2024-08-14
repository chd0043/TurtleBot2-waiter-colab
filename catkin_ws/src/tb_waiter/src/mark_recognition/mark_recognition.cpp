/*
 * image_converter.cpp
 *
 *  Created on: Jun 6, 2014
 *    Author: chd
 */


#include <iostream>
#include <math.h>
#include <string.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tb_waiter/cvMarkRec.h>

#include <tb_waiter/detect_region.h>

static const std::string CAM_TOPIC = "camera/rgb/image_raw";

class MarkRecognition : public DetectRegion
{
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer getMarkRecog_;

  Mat normalImage;
  string figure, color;

public:
  MarkRecognition()
  : it_(nh_), loop_rate_(2), figure(""), color("")
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_    = it_.subscribe(CAM_TOPIC, 1, &MarkRecognition::mainProcess, this);
    getMarkRecog_ = nh_.advertiseService("cvMarkRec", &MarkRecognition::getDetectedMark, this);
  }

  ~MarkRecognition()
  {
    image_sub_.shutdown();
  }

  void spin()
  {
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate_.sleep();
    }
  }

private:
  bool getDetectedMark(tb_waiter::cvMarkRec::Request  &req,
                       tb_waiter::cvMarkRec::Response  &res)
  {
    if 	(req.command == "1")
      showBasicImages = true;
    else if (req.command == "2")
    {
      showAllImages = true;
      showBasicImages = true;
    }
    else
    {
      showAllImages = false;
      showBasicImages = false;
    }

    res.figure = figure;
    res.color = color;

    return (true);
  }

  void msgToImageConversion(const sensor_msgs::ImageConstPtr& msg, Mat &outputImage)
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
    outputImage = cv_ptr->image;
  }

  void mainProcess(const sensor_msgs::ImageConstPtr& msg)
  {
    std::vector<std::string> figureAndColor;
    figure = ""; color = "";

    //ROS Msg conversion to Mat
    msgToImageConversion(msg, normalImage);

    figureAndColor = runFigureDetection(normalImage);
    figure = figureAndColor[0];
    color = figureAndColor[1];

    if (figure.size() > 0 && figure.size() > 0)
    {
      ROS_INFO("figure = %s, color = %s", figure.c_str(), color.c_str());
    }

    if(waitKey(30) >= 0) return;

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  MarkRecognition ic;
  ROS_INFO("Mark_Recognition");

  //ros::spin();
  ic.spin();

  return (0);
}
