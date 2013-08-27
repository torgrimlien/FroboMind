#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <ntnu_fieldflux/GPSFix.h>
#include <ntnu_fieldflux/GPSStatus.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <msgs/encoder.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
class listener{
public:
double last_lat;
double last_long;
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
  //ROS_INFO("x_vel: [%f]", msg->angular_velocity.x);
  //ROS_INFO("y_vel: [%f]", msg->angular_velocity.y);
  double qx=msg->orientation.x;
  double qy=msg->orientation.y;
  double qz=msg->orientation.z;
  double qw=msg->orientation.w;
  double roll = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) *180/3.14;;
  double yaw= asin(-2*(qx*qz-qw*qy))*180/3.14;
  double pitch=atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz)*180/3.14;
  ROS_INFO("roll_imu=[%f]",roll);
  std_msgs::Float32 mes;
  mes.data=roll;
  pub2.publish(mes);  
 
}
void chatterOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  double roll=atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)*180/3.1416;
  ROS_INFO("roll_odom_estimate: [%f]", roll);
  std_msgs::Float32 mes;
  mes.data=roll;
  pub.publish(mes); 
}
void chatterOdomCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{

  double longitude=msg->pose.pose.position.x;
  double latitude=msg->pose.pose.position.y;
  double dx=longitude-last_long;
  double dy=latitude-last_lat;
  double v=sqrt(dx*dx+dy*dy);
  last_long=longitude;
  last_lat=latitude;
  std_msgs::Float32 mes;
  mes.data=longitude;
  pub.publish(mes);
  std_msgs::Float32 mes2;
  mes2.data=latitude;
  pub2.publish(mes2);
}
void chatterPosCovCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{

  nav_msgs::Odometry mes;
  mes.pose.pose=msg->pose;
  pub3.publish(mes);
  
}
void chatterEncLeftCallback(const msgs::encoder::ConstPtr& msg)
{
  //ROS_INFO("LEFT");
}
void chatterEncRightCallback(const msgs::encoder::ConstPtr& msg)
{
  //ROS_INFO("RIGHT");
}
	    ros::Publisher pub;
	ros::Publisher pub2;
	ros::Publisher pub3;
private:


};
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
    ros::NodeHandle n;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */


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
  listener listen;
  listen.last_lat=0;
  listen.last_long=0;
  listen.pub = n.advertise<std_msgs::Float32>("gps_x",10);
  listen.pub2 =n.advertise<std_msgs::Float32>("gps_y",10);
  listen.pub3 =n.advertise<nav_msgs::Odometry>("fmKnowledge/pose_sim",5);
  ros::Subscriber sub = n.subscribe("fix", 10, &listener::chatterCallback,&listen);
  ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, &listener::chatterGpsCallback,&listen);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 10, &listener::chatterImuCallback,&listen);
  ros::Subscriber odom_sub = n.subscribe("fmKnowledge/pose_sim", 10, &listener::chatterOdomCallback,&listen);
  ros::Subscriber odom_gps_sub = n.subscribe("/fmInformation/gps_odom", 10, &listener::chatterOdomCallback2,&listen);
  ros::Subscriber pos_cov = n.subscribe("/fmProcessors/robot_pose_ekf/odom_estimate",2,&listener::chatterPosCovCallback,&listen);
 // ros::Subscriber el_sub = n.subscribe("/fmInformation/encoder_left", 10, listener::chatterEncLeftCallback,&listen);
 // ros::Subscriber er_sub = n.subscribe("/fmInformation/encoder_right", 10, listener::chatterEncRightCallback,&listen);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

