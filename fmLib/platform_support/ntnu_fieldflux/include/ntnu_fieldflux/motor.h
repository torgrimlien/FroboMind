#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#include<ntnu_fieldflux/EciDemo109.h>
//struct containing all the information sent from the motorcontrollers via CANopen
typedef struct motor_t
{
    	int targetVelocity;
    	int velocity;
    	int16_t targetIq;
    	int16_t targetId;
    	int16_t iq;
    	int16_t id;
	DWORD capacitorVoltage;
	DWORD heatsinkTemp;
	DWORD batteryCurrent;
	DWORD maxTorque;
	DWORD voltageLimit;
	DWORD maxFluxCurrent;
	DWORD maxIqAllowed;
	DWORD tempMeasured;
	int16_t ud;
	int16_t uq;
	int16_t voltageModulation;
	int16_t inductanceMeasured;
	DWORD state;
        int ticks;
	
} motor_t;

//struct containing the information of the vehicle
typedef struct vehicle_t
{
	DWORD position;
	DWORD angle;
	int SDOResponseSent;
	
	motor_t leftMotor;

	motor_t rightMotor;

}vehicle_t;
//struct containing parameters for the ixxat usb2can controller
typedef struct __ixxat_param_t
{
    ECI_HW_PARA stcHwPara;
    ECI_HW_INFO stcHwInfo;
    DWORD       dwHwIndex;
    DWORD       dwCtrlIndex;
    ECI_CTRL_CONFIG stcCtrlConfig;
    ECI_CTRL_CAPABILITIES stcCtrlCaps;
    ECI_CTRL_HDL  dwCtrlHandle;
}  ixxat_param_t;

//struct parsed to the thread reading CANmessages
typedef struct thread_param_t
{
    vehicle_t v;
    ixxat_param_t ip;
} thread_param_t;

typedef struct motor_ref_t
{
    DWORD leftSpeed;
    DWORD leftDir;
    DWORD rightSpeed;
    DWORD rightDir;
    DWORD state;
    
}motor_ref_t;

#endif //MOTOR_H_INCLUDE
