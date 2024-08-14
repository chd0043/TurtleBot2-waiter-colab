#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/JointControllerState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <ros/param.h>
#include <boost/assign/list_of.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub;
  ros::Rate r(25.0);

  while(n.ok())
  {
    ros::spinOnce(); // check for incoming messages
    //
    odom_pub  = n.advertise<nav_msgs::Odometry>("odom", 10);

    std::string name;
    ros::param::get( "robot_name", name);
    //std::cout << name << std::endl;

    // take pose info from gazebo
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;

    client.waitForExistence();

    getmodelstate.request.model_name = name;
    client.call(getmodelstate);

    //odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = getmodelstate.response.pose.position.x;
    odom.pose.pose.position.y = getmodelstate.response.pose.position.y;
    odom.pose.pose.orientation.z = getmodelstate.response.pose.orientation.z;
    odom.pose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
    odom.pose.covariance =  boost::assign::list_of(1e-1)  (0)  (0)  (0)  (0)  (0)
                                                    (0) (1e-1) (0)  (0)  (0)  (0)
                                                    (0)   (0) (1e6) (0)  (0)  (0)
                                                    (0)   (0)  (0) (1e6) (0)  (0)
                                                    (0)   (0)  (0)  (0) (1e6) (0)
                                                    (0)   (0)  (0)  (0)  (0) (5e-2) ;
    //set the velocity
    odom.child_frame_id = "base_footprint"; //base_footprint  base_link
    odom.twist.twist.linear.x = getmodelstate.response.twist.linear.x;
    odom.twist.twist.angular.z = getmodelstate.response.twist.angular.z;


    //publish the message
    odom_pub.publish(odom);

    r.sleep();
  }
}
