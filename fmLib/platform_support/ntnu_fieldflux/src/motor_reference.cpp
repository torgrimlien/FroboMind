#include <ntnu_fieldflux/motor_reference.h>
//#include <ntnu_fieldflux/motorVal.h>

//#include <Eci.h>

motor_reference::motor_reference()
{
	setValues(0,0,0,0,250,150);
}
void motor_reference::setValues(DWORD ls, DWORD ld, DWORD rs, DWORD rd, double thrt, double turn)
{
	leftSpeed = ls;
	rightSpeed = rs;
	leftDir = ld;
	rightDir = rd;
	throttle_scale=thrt;
	turn_scale = turn;
    joystick_calibrated=0;
	drive_ok = true;
    joystick_connected = 1;
	slowMode = 1;
	stayInLoop=0;
	joyMode = 1;
	autoMode = 0;
	receiving_cmd =0;
	lin_scale=150;
	ang_scale= lin_scale*1.88;
}
void motor_reference::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
		receiving_cmd=1;
    if(autoMode && drive_ok){
		
		double lin_velocity = msg->linear.x * lin_scale;
		double ang_velocity = msg->angular.z * ang_scale;
		printf("LINEAR %f \n ANGULAR%f\n",msg->linear.x,ang_velocity);	
		leftSpeed = lin_velocity - ang_velocity; 
		rightSpeed = lin_velocity + ang_velocity;
		printf("Left %d \n Right%d\n",leftSpeed,rightSpeed);	
		if(leftSpeed < 0){
			leftSpeed = -leftSpeed;
			leftDir = 0x05;}
		else if(leftSpeed == 0){
			leftDir = 0x00;}
		else if(leftSpeed > 0){
			leftDir = 0x06;}
	
		if(rightSpeed > 0){
			rightDir = 0x05;}
		else if(rightSpeed == 0){
			rightDir = 0x00;}
		else if(rightSpeed < 0){
			rightSpeed=-rightSpeed;
			rightDir = 0x06;}
		}
    /*if(joystick_connected ==0){
        rightSpeed =0;
        leftSpeed=0;
        rightDir=0;
        leftDir=0;
    }*/
			
	}
void motor_reference::drive_ok_callback(const std_msgs::Bool::ConstPtr& msg){
	drive_ok = msg->data;
	
}
void motor_reference::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	if(joy->buttons[7] && joy->buttons[6]){
		stayInLoop=1;
	}
    joystick_connected = joy->buttons[5];
    double fw = (joy->axes[5] - joy->axes[4])*throttle_scale;
	double turn = joy->axes[0]*turn_scale;
	if(abs(turn)<5){
		turn = 0;
	}
	if(joy->buttons[0]){
		slowMode =0;
	}	
	if(joy->buttons[1]){
		slowMode = 1;
	}
	if(joy->buttons[2]){
		joyMode=1;
		autoMode=0;
	}
	if(joy->buttons[3] && receiving_cmd == 1){
		joyMode=0;
		autoMode=1;
	}
	if(joystick_calibrated==0){
        printf("joystick_calibrated = 0 \n\n");
        if(joy->axes[4]==1.0 && joy->axes[5]==1.0)
		{
			joystick_calibrated=1;
		}
	}
    if(joyMode && drive_ok){
		leftSpeed = (DWORD)abs(fw - turn);
		rightSpeed = (DWORD)abs(fw + turn);
		if((fw-turn)>0){
			leftDir = 0x06;}
		else if((fw-turn)==0){
			leftDir = 0x00;}
		else if((fw-turn)<0){
			leftDir= 0x05;}
		
		if((fw+turn)<0){
			rightDir = 0x06;}
		else if((fw+turn)==0){
			rightDir = 0x00;}
		else if((fw+turn)>0){
			rightDir= 0x05;}

		if(slowMode){
			leftSpeed = leftSpeed/2;
			rightSpeed = rightSpeed/2;
		}
	}
	if (!drive_ok){
		leftSpeed = 0;
		rightSpeed = 0;
		leftDir = 0;
		rightDir = 0;	
	}
/*	if(slowMode){
		leftSpeed = leftSpeed/2;
		rightSpeed = rightSpeed/2;
	}*/
//	}
}
