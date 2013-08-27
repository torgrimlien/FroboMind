#include <ntnu_fieldflux/rosCallbacks.h>
#include <ntnu_fieldflux/motor.h>
#include <ntnu_fieldflux/motorVal.h>
#include <ros/ros.h>
void throttleCallback(const ntnu_fieldflux::motorVal::constPtr& msg,motor_ref_t * ref)
{
	ref->leftspeed 	= msg->leftSpeed;
	ref->rightspeed	= msg->rightSpeed;
	ref->leftdir	= msg->leftDir;
	ref->rightdir  	= msg->rightDir;

}
