#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <ros/param.h>
#include <tf/tf.h>

void odom_callback(const nav_msgs::OdometryConstPtr& msg){
	// Publish /odom to base_footprint transform
  static tf::TransformBroadcaster br;
  tf::Transform BaseFootprintTransf;
  BaseFootprintTransf.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, 0.0));
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  BaseFootprintTransf.setRotation(q);

  std::string tf_name;
  ros::param::get( "tf_prefix", tf_name);
  std::string odom_ = std::string(tf_name) + "/odom";
  std::string base_footprint_ = std::string(tf_name) + "/base_footprint";

  br.sendTransform(tf::StampedTransform(BaseFootprintTransf, ros::Time::now(),odom_, base_footprint_));

  // Publish base_footprint transform to odom
  //static tf::TransformBroadcaster br2;
  //tf::Transform OdomTransf;
  //br2.sendTransform(tf::StampedTransform(OdomTransf, ros::Time::now(),"/origin", odom_));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "turtlebot_tf_broadcaster");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 100, &odom_callback);
  ros::spin();
  return (0);
};
