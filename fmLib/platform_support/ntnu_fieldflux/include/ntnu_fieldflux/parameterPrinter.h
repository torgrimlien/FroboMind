#ifndef PARAMETER_PRINTER_INCLUDED_
#define PARAMETER_PRINTER_INCLUDED_
#include<ntnu_fieldflux/motorValues.h>
class motor_parameters{
	public:
	motor_parameters();
	void callback(const ntnu_fieldflux::motorValues::ConstPtr& msg);
	int leftMotor_Id_ref;
	int leftMotor_Iq_ref;
	int leftMotor_Id;
	int leftMotor_Iq;
	int leftMotor_state;
	int leftMotor_w;
	int leftMotor_w_ref;
	int rightMotor_Id_ref;
	int rightMotor_Iq_ref;
	int rightMotor_Id;
	int rightMotor_Iq;
	int rightMotor_state;
	int rightMotor_w;
	int rightMotor_w_ref;

};



#endif
