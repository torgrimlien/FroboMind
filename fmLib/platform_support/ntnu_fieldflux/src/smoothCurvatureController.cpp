#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>



class SCcontroller
{
public:
	int hasGoal;
	//controller parameters
	double k1;
	double k2;
	double k3;
	double beta;
	double lambda;
	
	//robot parameters
	double yaw;
	double pos_x;
	double pos_y;
	//position and orientation of waypoint
	double goal_x;
	double goal_y;
	double goal_yaw;
	//max speed allowed
	double v_max;
	//curvature of the path at given point
	double K;

 	double alpha;
	double delta;
	double theta;
	double r;

	//parameters sent by controller
	double v;
	double omega;
	/*!
	 *The publisher to publish the geometry twist message from the controller
	*/
	ros::Publisher pub;

	
	void updateParameters(){
		r= sqrt(pow(pos_x-goal_x,2) + pow(pos_y-goal_y,2));
		alpha = atan2(goal_y-pos_y,goal_x-pos_x);
		delta = yaw - alpha;
		theta = goal_yaw - alpha;
		correct_angle(delta);
		correct_angle(theta);
		K = -1/r*(k2*(delta-atan(-k1*theta))+(1+k1/(1+k1*k1*theta*theta))*sin(theta));
		v = v_max/(1+beta*pow(abs(K),lambda));
		omega = v*K;
		printf("r=%f, \nalpha=%f \ndelta=%f \n theta=%f \n v=%f \n omega=%f \nK=%f\n\n",r,alpha,delta,theta,v,omega,K);
		
	}		
	void publishSystemInput(){
		geometry_msgs::Twist msg;
		msg.linear.x=v;
		msg.angular.z=omega;
		pub.publish(msg);
		
	}
	void processOdometry(const nav_msgs::Odometry::ConstPtr& msg){
		pos_x=msg->pose.pose.position.x;
		pos_y=msg->pose.pose.position.y;
		double qx=msg->pose.pose.orientation.x;
		double qy=msg->pose.pose.orientation.y;
		double qz=msg->pose.pose.orientation.z;
		double qw=msg->pose.pose.orientation.w;
		yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
		updateParameters();
		publishSystemInput();
	}
	void processWaypoint(const nav_msgs::Odometry::ConstPtr& msg){
		goal_x=msg->pose.pose.position.x;
		goal_y=msg->pose.pose.position.y;
		double qx=msg->pose.pose.orientation.x;
		double qy=msg->pose.pose.orientation.y;
		double qz=msg->pose.pose.orientation.z;
		double qw=msg->pose.pose.orientation.w;
		goal_yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
		hasGoal=1;
	}
private:
	void correct_angle(double& angle)
	{
		while(angle > M_PI)
		{
			angle -= 2*M_PI;
		}

		while(angle < -M_PI)
		{
			angle += 2*M_PI;
		}
	}
};
	int main(int argc,char** argv){
	printf("HEI!!!\n");
	ros::init(argc,argv,"WaypointNavigator");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	ros::Subscriber odom_sub;
	ros::Subscriber goal_sub;

	SCcontroller node;
	
	std::string publish_topic;
	std::string subscribe_odom;
	std::string subscribe_goal;

	nh.param<std::string>("subscribe_topic",subscribe_odom,"/fmProcessors/odom_estimate_test");
	nh.param<std::string>("goal_subscribe_topic",subscribe_goal,"/fmProcessors/goal");
	nh.param<std::string>("publish_topic", publish_topic,"/fmController/output");
	printf("%s \n",subscribe_odom.c_str());
	nh.param<double>("k1",node.k1,1);
	nh.param<double>("k2",node.k2,5);
	nh.param<double>("k3",node.k3,0.1);
	nh.param<double>("beta",node.beta,0.4);
	nh.param<double>("lambda",node.lambda,2);
	nh.param<double>("v_max",node.v_max,0.1);
	nh.param<double>("goal_x",node.goal_x,0);
	nh.param<double>("goal_y",node.goal_y,0);
	nh.param<double>("goal_yaw",node.goal_yaw,0);
	printf("v_max= %f \n",node.v_max);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_goal,10,&SCcontroller::processWaypoint,&node);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_odom,10,&SCcontroller::processOdometry,&node);
	node.pub = n.advertise<geometry_msgs::Twist>(publish_topic,5);

	ros::spin();	

	

	}
