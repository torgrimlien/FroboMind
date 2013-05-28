#ifndef MOTOR_REFERENCE_INCLUDED_
#define MOTOR_REFERENCE_INCLUDED_

#include <ECI109.h>
//#include <ntnu_fieldflux/motorVal.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
///////////////////////////////////////////////////////////////////////////////
/**
  class holding all the parameters sent to the controller via CAN

  @param leftSpeed reference speed sent to left motor
  @param rightSpeed reference speed sent to right motor
  @param leftDir reference direction sent to left motor
  @param rightDir reference direction sent to left motor
  @param leftState state of the left motor
  @param rightState state of the right motor
  @param throttle_scale constant for converting joystick input to motor input
  @param turn_scale constant for converting joystick input to motor input
  
*/
class motor_reference{

private:
	DWORD leftSpeed;
	DWORD rightSpeed;
	DWORD leftDir;
	DWORD rightDir;
	DWORD leftState;
	DWORD rightState;
	
	
public:
	double throttle_scale;
	double turn_scale;
	double lin_scale;
	double ang_scale;
	double wheel_distance;
	double wheel_radius;
	int joyMode;
	int autoMode;
	int joystick_calibrated;
	int left_full;
	int right_full;
	int left_release;
	int right_release;	
	int stayInLoop;
	int slowMode;
	int receiving_cmd;
	motor_reference();	
	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& mot);	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);	
	void setValues(DWORD ls, DWORD ld, DWORD rs, DWORD rd, double thrt,double turn);
	DWORD getLeftSpeed(){return leftSpeed;}
	DWORD getRightSpeed(){return rightSpeed;}
	DWORD getLeftDir(){return leftDir;}
	DWORD getRightDir(){return rightDir;}
	DWORD getLeftState(){return leftState;}
	DWORD getRightState(){return rightState;}	

};

#endif
