#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <ntnu_fieldflux/GPSFix.h>
#include <ntnu_fieldflux/GPSStatus.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <msgs/encoder.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("Latitude: [%f]", msg->latitude);
  ROS_INFO("Longitude: [%f]", msg->longitude);
}
void chatterGpsCallback(const ntnu_fieldflux::GPSFix::ConstPtr& msg)
{
  ROS_INFO("Latitude: [%f]", msg->latitude);
  ROS_INFO("Longitude: [%f]", msg->longitude);
}
void chatterImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("x_vel: [%f]", msg->angular_velocity.x);
  ROS_INFO("y_vel: [%f]", msg->angular_velocity.y);
}
void chatterOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("pose_X: [%f]", msg->pose.pose.position.x);
  ROS_INFO("pose_Y: [%f]", msg->pose.pose.position.y);
}
void chatterEncLeftCallback(const msgs::encoder::ConstPtr& msg)
{
  ROS_INFO("LEFT");
}
void chatterEncRightCallback(const msgs::encoder::ConstPtr& msg)
{
  ROS_INFO("RIGHT");
}
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("fix", 10, chatterCallback);
  ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, chatterGpsCallback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 10, chatterImuCallback);
  ros::Subscriber odom_sub = n.subscribe("/fmKnowledge/odom", 10, chatterOdomCallback);
  ros::Subscriber el_sub = n.subscribe("/fmInformation/encoder_left", 10, chatterEncLeftCallback);
  ros::Subscriber er_sub = n.subscribe("/fmInformation/encoder_right", 10, chatterEncRightCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
