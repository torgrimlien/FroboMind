#include<ros/ros.h>

class vehicle:
	public:
	int ticks_per_rev;
	double wheel_distance;
	double wheel_radius;
	double x_pos;
	double y_pos;
	double orientation;
	double x_vel;
	double y_vel;
	double lin_vel;
	double ang_vel;
	double leftSpeed;
	double righSpeed;
	void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg){
		lin_vel=msg->linear.x;
		ang_vel=msg->angular.z;
		rightSpeed=lin_vel+ang_vel*wheel_distance;
		leftSpeed=lin_vel-ang_vel*wheel_distance;
	}
	void updateOdometry(){
		t_now=get_	
	}
	private:
	int main(int* argc,char** argv){
		ros::init(argc,argv,"simulation");
		ros::NodeHandle n;

		std::string subscribe_topic=n.param


	}
