#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

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
    //parameters for position estimation through only wheel odometry
    double current_x;
    double current_y;
    double current_yaw;
    double prev_x;
	double prev_y;
	double prev_yaw;
    double d_x;
    double d_y;
    double d_yaw;
    double d_xm;
    double d_ym;
    double qx;
    double qy;
    double qz;
    double qw;
	//position and orientation of waypoint
	double goal_x;
	double goal_y;
	double goal_yaw;
	//max speed allowed
	double v_max;
    double v_max_close;
    double v_max_far;
	//curvature of the path at given point
	double K;

    double alpha; //angle between robot position and waypoint ENU
    double delta; //angle betwwen straight line between robot and waypoint and robot's pose
    double theta; //angle betwwen straight line between robot and waypoint and waypoint's pose
    double r;     //distance between robot and waypoint
    double r_threshold;

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
    ros::Publisher goal_reached_pub;
    ros::Publisher active_pose_pub;

    nav_msgs::Odometry active_pose;
	
	void updateParameters(){
		r= sqrt(pow(pos_x-goal_x,2) + pow(pos_y-goal_y,2));
        if (r<r_no_gps){
            close_to_goal = 1;
            v_max = v_max_close;
        }
		alpha = atan2(goal_y-pos_y,goal_x-pos_x);
        printf("distance_goalx = %f  distance_goaly = %f \n",goal_x-pos_x,goal_y-pos_y);
		delta = yaw - alpha;
		theta = goal_yaw - alpha;
		correct_angle(delta);
		correct_angle(theta);
		K = -1/r*(k2*(delta-atan(-k1*theta))+(1+k1/(1+k1*k1*theta*theta))*sin(delta));
		v = v_max/(1+beta*pow(abs(K),lambda));
		omega = v*K;
        if (fabs(omega) > 0.5){
			v=v/fabs(omega);
			omega=1/fabs(omega);
		}
        if (r<r_threshold){
            v=0;
            omega = 0;
            hasGoal = 0;
            publishGoalReached();
        }
		printf("r=%f, \nalpha=%f \ndelta=%f \n theta=%f \n v=%f \n omega=%f \nK=%f\n\n",r,alpha,delta,theta,v,omega,K);


        printf("v =%f    omega= %f r = %f\n\n, goal_yaw-yaw %f \n",v,omega,r,fabs(goal_yaw-yaw));

	}		
	void publishSystemInput(){
		geometry_msgs::Twist msg;
		msg.linear.x=v;
		msg.angular.z=omega;
		pub.publish(msg);
		
	}

	void processOdometry(const nav_msgs::Odometry::ConstPtr& msg){
        if(close_to_goal == 0){
            active_pose.pose.pose.position.x    = pos_x = msg->pose.pose.position.x;
            active_pose.pose.pose.position.y    = pos_y = msg->pose.pose.position.y;
            active_pose.pose.pose.orientation.x = qx    = msg->pose.pose.orientation.x;
            active_pose.pose.pose.orientation.y = qy    = msg->pose.pose.orientation.y;
            active_pose.pose.pose.orientation.z = qz    = msg->pose.pose.orientation.z;
            active_pose.pose.pose.orientation.w = qw    = msg->pose.pose.orientation.w;

            yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
            printf("yaw=%f \n",yaw*180/3.14);
            active_pose_pub.publish(active_pose);
            if(hasGoal){
                updateParameters();
                publishSystemInput();
            }
        }
	}
    void processOdometryNoGps(const nav_msgs::Odometry::ConstPtr& msg){
        if(close_to_goal==0){
            prev_x = msg->pose.pose.position.x;
            prev_y = msg->pose.pose.position.y;
            qx=msg->pose.pose.orientation.x;
            qy=msg->pose.pose.orientation.y;
            qz=msg->pose.pose.orientation.z;
            qw=msg->pose.pose.orientation.w;
            prev_yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
        }
        else{
            current_x = msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;
            qx=msg->pose.pose.orientation.x;
            qy=msg->pose.pose.orientation.y;
            qz=msg->pose.pose.orientation.z;
            qw=msg->pose.pose.orientation.w;
            current_yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);

            d_xm = current_x - prev_x;
            d_ym = current_y - prev_y;
            d_yaw = current_yaw - prev_yaw;
            yaw   = yaw + d_yaw;
            d_x = sqrt(d_xm*d_xm +d_ym*d_ym)*cos(yaw);
            d_y = sqrt(d_xm*d_xm +d_ym*d_ym)*sin(yaw);
            prev_x = current_x;
            prev_y = current_y;
            prev_yaw = current_yaw;
            active_pose.pose.pose.position.x= pos_x = pos_x + d_x;
            active_pose.pose.pose.position.y= pos_y = pos_y + d_y;

            correct_angle(yaw);
            active_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            active_pose_pub.publish(active_pose);
            if(hasGoal){
                updateParameters();
                publishSystemInput();
            }
            else{
                publishGoalReached();
            }

        }

	}
	void processWaypoint(const nav_msgs::Odometry::ConstPtr& msg){
        if(hasGoal == 0){
            goal_x=msg->pose.pose.position.x;
            goal_y=msg->pose.pose.position.y;
            double qx=msg->pose.pose.orientation.x;
            double qy=msg->pose.pose.orientation.y;
            double qz=msg->pose.pose.orientation.z;
            double qw=msg->pose.pose.orientation.w;
            goal_yaw = atan2(2*(qx*qy + qw*qz),qw*qw + qx*qx -qy*qy - qz*qz);
            hasGoal=1;
            close_to_goal=0;
            v_max = v_max_far;
        }
	}
    void publishGoalReached(){
        std_msgs::Bool goal_reached;
        goal_reached.data = true;
        goal_reached_pub.publish(goal_reached);
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
//    	printf("HEI!!!\n");
        ros::init(argc,argv,"WaypointNavigator");
        ros::NodeHandle nh("~");
        ros::NodeHandle n;

        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        ros::Subscriber wheel_sub;
        SCcontroller node;
        std::string active_pose_topic;
        std::string publish_topic;
        std::string subscribe_odom;
        std::string subscribe_goal;
        std::string wheel_odom_topic;
        std::string goal_reached_topic;

        node.pos_ok=0;
        node.yaw_ok=0;
        node.close_to_goal=0;

        nh.param<std::string>("subscribe_topic",subscribe_odom,"/fmKnowledge/pose");
        nh.param<std::string>("wheel_only_odom_subscribe_topic",wheel_odom_topic,"/fmKnowledge/encoder_odom");
        nh.param<std::string>("goal_subscribe_topic",subscribe_goal,"/fmInformation/waypoint");
        nh.param<std::string>("publish_topic", publish_topic,"/fmController/output");
        nh.param<std::string>("publish_goal_reached_topic", goal_reached_topic,"/fmInformation/goal_reached");
        nh.param<std::string>("acive_pose_pub", active_pose_topic,"/fmController/pos_used");

    //	printf("%s \n",subscribe_odom.c_str());
        nh.param<double>("k1",node.k1,1.0);
        nh.param<double>("k2",node.k2,5.0);
        nh.param<double>("k3",node.k3,0.1);
        nh.param<double>("k4",node.k4,0.1);
        nh.param<double>("beta",node.beta,0.4);
        nh.param<double>("lambda",node.lambda,2.0);
        nh.param<double>("v_max",node.v_max,1.0);
        nh.param<double>("goal_x",node.goal_x,0.0);
        nh.param<double>("goal_y",node.goal_y,0.0);
        nh.param<double>("goal_yaw",node.goal_yaw,0.0);
        nh.param<double>("r_no_gps", node.r_no_gps, 2.0);
        nh.param<double>("r_threshold",node.r_threshold,0.4);

    //	printf("v_max= %f \n",node.v_max);
        node.v_max_far = node.v_max;
        node.v_max_close = node.v_max/2;
        node.hasGoal =0;
        odom_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_goal,10,&SCcontroller::processWaypoint,&node);
        goal_sub = nh.subscribe<nav_msgs::Odometry>(subscribe_odom,10,&SCcontroller::processOdometry,&node);
        wheel_sub= nh.subscribe<nav_msgs::Odometry>(wheel_odom_topic,10,&SCcontroller::processOdometryNoGps,&node);
        node.pub = n.advertise<geometry_msgs::Twist>(publish_topic,5);
        node.goal_reached_pub = n.advertise<std_msgs::Bool>(goal_reached_topic,5);
//      node.pub2 = n.advertise<std_msgs::Int16>(publish_gps_on_off,5);
        node.active_pose_pub = n.advertise<nav_msgs::Odometry>(active_pose_topic,2);
        ros::spin();

	

	}
