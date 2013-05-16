#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
int main(int argc,char** argv){
	ros::init(argc,argv,"Waypoint Publisher");	
	ros::NodeHandle n;	
	ros::Publisher pub;

	double x;
	double y;
	double orientation;
		
	
	n.param<double>("goal_pos_x",x,0);
	n.param<double>("goal_pos_y",y,0);
	n.param<double>("goal_orientation",orientation,0);

	nav_msgs::Odometry goal;
	goal.pose.pose.position.x=x;
	goal.pose.pose.position.y=y;
	goal.pose.pose.orientation=tf::createQuaternionMsgFromYaw(orientation);
	
	pub.publish(goal);
}
