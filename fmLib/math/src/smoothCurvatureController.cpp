#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>


class SCcontroller
{
public:
	int hasGoal;
	//controller parameters
	double k1;
	double k2;
	double k3;
	double k4;
	double beta;
	double lambda;
	//controller parameters when close to the goal
	
	double r_no_gps;

	//robot parameters
	double yaw;
	double pos_x;
	double pos_y;
	
	//parameters for estimating position from only wheel odometry
	double current_w_x;
	double current_w_y;
	double current_w_yaw;
	double prev_w_x;
	double prev_w_y;
	double prev_w_yaw;
	double dx;
	double dy;
	double dyaw;

	int use_gps;
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
	double r_last;

	//parameters sent by controller
	double v;
	double omega;
	int pos_ok;
	int yaw_ok;
	int close_to_goal;
	/*!
	 *The publisher to publish the geometry twist message from the controller
	*/
	ros::Publisher pub;
	ros::Publisher pub2;

	
	void updateParameters(){
		r= sqrt(pow(pos_x-goal_x,2) + pow(pos_y-goal_y,2));
		if (r>r_no_gps && close_to_goal == 0){

		
		alpha = atan2(goal_y-pos_y,goal_x-pos_x);
		printf("distance_goalx = %f  distance_goaly = %f",goal_x-pos_x,goal_y-pos_y);
		delta = yaw - alpha;
		theta = goal_yaw - alpha;
		correct_angle(delta);
		correct_angle(theta);
		K = -1/r*(k2*(delta-atan(-k1*theta))+(1+k1/(1+k1*k1*theta*theta))*sin(delta));
		v = v_max/(1+beta*pow(abs(K),lambda));
		omega = v*K;
		if (fabs(omega) > 1){
			v=v/fabs(omega);
			omega=1/fabs(omega);
		}
		printf("r=%f, \nalpha=%f \ndelta=%f \n theta=%f \n v=%f \n omega=%f \nK=%f\n\n",r,alpha,delta,theta,v,omega,K);
		}
		else{
			close_to_goal =1;
			double diff_yaw = goal_yaw-yaw;
			correct_angle(diff_yaw);
			if(yaw_ok==0){			
				omega=k4*(diff_yaw);
				if(omega>1)omega=1;
				if(omega<-1)omega=-1;			
			}
			if(pos_ok ==0){			
			v=0.3;}
			if(r_last<r){
				v=0;
				pos_ok = 1;}
			if(fabs(diff_yaw)<0.015){
				omega=0;
				printf("YAW_OK!");
				yaw_ok = 1;}
			}
			printf("v =%f    omega= %f r = %f\n\n, goal_yaw-yaw %f \n",v,omega,r,fabs(goal_yaw-yaw));
		r_last = r;
	}		
	void publishSystemInput(){
		geometry_msgs::Twist msg;
		msg.linear.x=v;
		msg.angular.z=omega;
		pub.publish(msg);
		
	}
	void publishGpsOn(int pwr){
		std_msgs::Int16 msg;
		msg.data=pwr;
		pub2.publish(msg);
		printf("Turing GPS off!!!!\n");
		
	}
	void processOdometry(const nav_msgs::Odometry::ConstPtr& msg){
		pos_x=msg->pose.pose.position.x;
		pos_y=msg->pose.pose.position.y;
		double qx=msg->pose.pose.orientation.x;
		double qy=msg->pose.pose.orientation.y;
		double qz=msg->pose.pose.orientation.z;
		double qw=msg->pose.pose.orientation.w;
		yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
		printf("yaw=%f \n",yaw*180/3.14);
		updateParameters();
		publishSystemInput();
	}
	void processWheelOdometry(const nav_msgs::Odometry::ConstPtr& msg){
		if(use_gps){
			prev_w_x=msg->pose.pose.position.x;
			prev_w_y=msg->pose.pose.position.x;
			double qx=msg->pose.pose.orientation.x;
			double qy=msg->pose.pose.orientation.y;
			double qz=msg->pose.pose.orientation.z;
			double qw=msg->pose.pose.orientation.w;
			prev_w_yaw=atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);		
		}
		else{
			current_w_x=msg->pose.pose.position.x;
			current_w_y=msg->pose.pose.position.y;
			double qx=msg->pose.pose.orientation.x;
			double qy=msg->pose.pose.orientation.y;
			double qz=msg->pose.pose.orientation.z;
			double qw=msg->pose.pose.orientation.w;
			current_w_yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
			dx=current_w_x-prev_w_x;
			dy=current_w_y-prev_w_y;
			dyaw=current_w_yaw-prev_w_yaw;
			pos_x	= pos_x+dx;
			pos_y	= pos_y+dy;
			yaw	= yaw+dyaw;
			updateParameters();
			publishSystemInput();
		}
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
		ros::init(argc,argv,"WaypointNavigator");
		ros::NodeHandle nh("~");
		ros::NodeHandle n;

		ros::Subscriber odom_sub;
		ros::Subscriber goal_sub;

		SCcontroller node;
		std::string publish_gps_on_off;
		std::string publish_topic;
		std::string subscribe_wheel_odom;
		std::string subscribe_odom;
		std::string subscribe_goal;
		node.pos_ok=0;
		node.yaw_ok=0;
		node.close_to_goal=0;
		nh.param<std::string>("subscribe_topic",subscribe_odom,"/fmProcessors/odom_estimate_test");
		nh.param<std::string>("goal_subscribe_topic",subscribe_goal,"/fmProcessors/goal");
		nh.param<std::string>("publish_topic", publish_topic,"/fmController/output");
		nh.param<std::string>("publish_topic_gps_on_of", subscribe_wheel_odom,"/fmKnowledge/odom");
	
		printf("%s \n",subscribe_odom.c_str());
		nh.param<double>("k1",node.k1,1);
		nh.param<double>("k2",node.k2,5);
		nh.param<double>("k3",node.k3,0.1);
		nh.param<double>("k4",node.k4,0.1);
		nh.param<double>("beta",node.beta,0.4);
		nh.param<double>("lambda",node.lambda,2);
		nh.param<double>("v_max",node.v_max,0.1);
		nh.param<double>("goal_x",node.goal_x,0);
		nh.param<double>("goal_y",node.goal_y,0);
		nh.param<double>("goal_yaw",node.goal_yaw,0);
		nh.param<double>("r_no_gps", node.r_no_gps, 1.5);
	
		printf("v_max= %f \n",node.v_max);
		odom_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_goal,10,&SCcontroller::processWaypoint,&node);
		odom_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_odom,10,&SCcontroller::processOdometry,&node);
		node.pub = n.advertise<geometry_msgs::Twist>(publish_topic,5);
		
		ros::spin();	

	

	}

