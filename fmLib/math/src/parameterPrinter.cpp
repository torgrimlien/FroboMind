#include<ros/ros.h>
#include<ntnu_fieldflux/parameterPrinter.h>
#include<ntnu_fieldflux/motorValues.h>
#include<ntnu_fieldflux/CANMessageHandler.h>

void motor_parameters::callback(const ntnu_fieldflux::motorValues::ConstPtr& msg){
	leftMotor_Id_ref 	= 	msg->leftMotor_Id_ref;
	leftMotor_Iq_ref 	= 	msg->leftMotor_Iq_ref;
	leftMotor_Id 		=	msg->leftMotor_Id;
	leftMotor_Iq		=	msg->leftMotor_Iq;
	leftMotor_state		=	msg->leftMotor_state;
	leftMotor_w		=	msg->leftMotor_w;
	leftMotor_w_ref		=	msg->leftMotor_w_ref;
	rightMotor_Id_ref	=	msg->rightMotor_Id_ref;
	rightMotor_Iq_ref	=	msg->rightMotor_Iq_ref;
	rightMotor_Id		=	msg->rightMotor_Id;
	rightMotor_Iq		=	msg->rightMotor_Iq;
	rightMotor_state	=	msg->rightMotor_state;
	rightMotor_w		=	msg->rightMotor_w;
	rightMotor_w_ref	=	msg->rightMotor_w_ref;
}
motor_parameters::motor_parameters(){
	leftMotor_Id_ref 	= 	0;
	leftMotor_Iq_ref 	= 	0;
	leftMotor_Id 		=	0;
	leftMotor_Iq		=	0;
	leftMotor_state		=	0;
	leftMotor_w		=	0;
	leftMotor_w_ref		=	0;
	rightMotor_Id_ref	=	0;
	rightMotor_Iq_ref	=	0;
	rightMotor_Id		=	0;
	rightMotor_Iq		=	0;
	rightMotor_state	=	0;
	rightMotor_w		=	0;
	rightMotor_w_ref	=	0;
}
int   	main	(int 	argc,
		char** 	argv,
		char** 	envp )
{
	ros::init(argc,argv,"parameterPrinter");
	ros::NodeHandle nh_;
	motor_parameters mp;
	ros::Subscriber sub = nh_.subscribe<ntnu_fieldflux::motorValues>("motor_values",10,&motor_parameters::callback, &mp);
	ros::spin();
	
	


}
