///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
/**
  

  @author Torgrim Aalvik Lien
  @file CANMsgHandler.c
*/


//////////////////////////////////////////////////////////////////////////
// compiler directives

//////////////////////////////////////////////////////////////////////////
// include files
#include <ntnu_fieldflux/CANMessageHandler.h>
#include <msgs/encoder.h>
#include <string.h>
//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables

//////////////////////////////////////////////////////////////////////////
// global variables

//////////////////////////////////////////////////////////////////////////
// static function prototypes

//////////////////////////////////////////////////////////////////////////
// global functions

//////////////////////////////////////////////////////////////////////////
// static functions


///////////////////////////////////////////////////////////////////////////////
/**
  main function

  @param argc Argument count
  @param argv Array of arguments
  @param envp environment variables

*/

//////////////////////////////////////////////////////////////////////////
// compiler directives

//////////////////////////////////////////////////////////////////////////
// include files




//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables
#define NMT_BOOTUP = 0x00
#define NMT_STOPPED = 0x04
#define NMT_OPERATIONAL = 0x05
#define NMT_PRE_OPERATIONAL = 0x7F

#define PDO_CAN_ID_1 0x180
#define TPDO_CAN_ID_1 0x200
//////////////////////////////////////////////////////////////////////////
// global variables
/** ECI Demo send timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_TX_TIMEOUT 500

/** ECI Demo TX message count for CAN @ingroup EciDemo */
#define ECIDEMO_TX_MSGCOUNT_CAN (0x800 *10)

/** ECI Demo receive timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_RX_TIMEOUT 500

/** ECI Demo total receive timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_RX_TOTALTIMEOUT 30000
//////////////////////////////////////////////////////////////////////////
// static function prototypes

//////////////////////////////////////////////////////////////////////////
// global functions

//////////////////////////////////////////////////////////////////////////
// static functions

#define ECIDEMO_CHECKERROR(FuncName) \
{\
  if(ECI_OK == hResult)\
  {\
    OS_Printf(#FuncName "...succeeded.\n"); \
  }\
  else\
  {\
    OS_Printf( #FuncName "...failed with errorcode: 0x%08X. %s\n", \
               hResult, \
               ECI109_GetErrorString(hResult)); \
  }\
}


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
/*void throttleCallback(const ntnu_fieldflux::motorVal &mot, motor_ref_t *ref)
{
	ref->leftspeed 	= mot.leftSpeed;
	ref->rightspeed	= mot.rightSpeed;
	ref->leftdir	= mot.leftDir;
	ref->rightdir  	= mot.rightDir;

}*/

///////////////////////////////////////////////////////////////////////////////
/**
  loop for reading CAN-messages

  @param arg Argument containing a pointer to the motor variables and ixxat-specific parameters
  
*/
void *CANReadLoop(void *arg){
    int rc;
    rc = pthread_mutex_lock(&mutex);
    thread_param_t* self = (thread_param_t*)arg;
    rc = pthread_mutex_unlock(&mutex);
    int stay_in_loop = 1;

    ECI_CTRL_MESSAGE stcCtrlMsg         = {0};

//    DWORD           dwStartTime         = 0;
//    DWORD           dwCurrentTime       = 0;
//    DWORD           dwRxTimeout         = ECIDEMO_RX_TOTALTIMEOUT;
    DWORD           dwCount             = 0;
    DWORD           lastHeartBeat1 = 0;
    DWORD           lastHeartBeat2 = 0;
    DWORD	    currentTime = 0;
    ECI_RESULT hResult;
    //*** Receive Messages
    //OS_Printf("Now, receiving CAN Messages\n");

    //parameters for converting rpm to ticks
    int current_time_left 	= OS_GetTimeInMs();
    int current_time_right	= OS_GetTimeInMs();
    int prev_time_left		= OS_GetTimeInMs();
    int prev_time_right		= OS_GetTimeInMs();
    int dt_left;
    int dt_right;


    lastHeartBeat1 = OS_GetTimeInMs();
    lastHeartBeat2 = OS_GetTimeInMs();
    hResult       = ECI_ERR_TIMEOUT;
        //*** Loop until abort
    while(stay_in_loop == 1)
    {

      {

        dwCount = 1;
        //rc = pthread_mutex_lock(&mutex);
        hResult = ECI109_CtrlReceive( self->ip.dwCtrlHandle , &dwCount, &stcCtrlMsg, ECIDEMO_RX_TIMEOUT);
        //rc = pthread_mutex_unlock(&mutex);

      }
      if((ECI_OK == hResult) && (dwCount > 0))
      {
        //OS_Printf("\n");
        //EciPrintCtrlMessage(&stcCtrlMsg);
	currentTime = OS_GetTimeInMs();
        int msgId=stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId;
        switch(msgId){
            case 0x701:
                //rc = pthread_mutex_lock(&mutex);
                self->v.leftMotor.state   = stcCtrlMsg.u.sCanMessage.u.V0.abData[0];
                //rc = pthread_mutex_unlock(&mutex);
                lastHeartBeat1          = OS_GetTimeInMs();
		//OS_Printf("Heartbeat1: %d", self->v.leftMotor.state);
                break;

            case 0x702:
                rc = pthread_mutex_lock(&mutex);
                self->v.rightMotor.state    = stcCtrlMsg.u.sCanMessage.u.V0.abData[3]+(stcCtrlMsg.u.sCanMessage.u.V0.abData[2])<<(8);
                rc = pthread_mutex_unlock(&mutex);
                lastHeartBeat2      = OS_GetTimeInMs();
		//OS_Printf("Heartbeat2\n %d",self->v.rightMotor.state);
                break;
            case 0x80:
                
                break;
            case 0x289:

		current_time_left = OS_GetTimeInMs();
		dt_left = current_time_left - prev_time_left;
		prev_time_left = current_time_left;                
		rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.leftMotor.targetVelocity, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int));
		memcpy(&self->v.leftMotor.velocity, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int)); 
                self->v.leftMotor.ticks += self->v.leftMotor.velocity * dt_left /60; //100 ticks/rev
		rc = pthread_mutex_unlock(&mutex);
		//OS_Printf("LEFT:\n ticks: %d\n rpm: %d dt: %d \n",self->v.leftMotor.ticks, self->v.leftMotor.velocity,dt_left);
		
                break;
	    case 0x290:
		current_time_right = OS_GetTimeInMs();
		dt_right = current_time_right - prev_time_right;
		prev_time_right = current_time_right;
                rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.rightMotor.targetVelocity, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int));
		memcpy(&self->v.rightMotor.velocity, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int));
		self->v.rightMotor.ticks += self->v.rightMotor.velocity * dt_right /60;
                rc = pthread_mutex_unlock(&mutex);
		//OS_Printf("RIGHT:\n ticks: %d\n rpm: %d dt: %d \n",self->v.rightMotor.ticks, self->v.rightMotor.velocity,dt_right);

		 
				
		break;
	    case 0x228:
                rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.leftMotor.ud, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int16_t));
		memcpy(&self->v.leftMotor.uq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[2], sizeof(int16_t));
		memcpy(&self->v.leftMotor.voltageModulation, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int16_t));
		memcpy(&self->v.leftMotor.inductanceMeasured, &stcCtrlMsg.u.sCanMessage.u.V0.abData[6], sizeof(int16_t));
                rc = pthread_mutex_unlock(&mutex);
		break;		
	    case 0x229:
                rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.rightMotor.ud, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int16_t));
		memcpy(&self->v.rightMotor.uq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[2], sizeof(int16_t));
		memcpy(&self->v.rightMotor.voltageModulation, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int16_t));
		memcpy(&self->v.rightMotor.inductanceMeasured, &stcCtrlMsg.u.sCanMessage.u.V0.abData[6], sizeof(int16_t));
                rc = pthread_mutex_unlock(&mutex);
		break;
	    case 0x277:
                rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.leftMotor.targetId, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int16_t));
		memcpy(&self->v.leftMotor.targetIq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[2], sizeof(int16_t));
		memcpy(&self->v.leftMotor.id, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int16_t));
		memcpy(&self->v.leftMotor.iq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[6], sizeof(int16_t));
                rc = pthread_mutex_unlock(&mutex);
		break;
	    case 0x278:
                rc = pthread_mutex_lock(&mutex);
                memcpy(&self->v.rightMotor.targetId, &stcCtrlMsg.u.sCanMessage.u.V0.abData[0], sizeof(int16_t));
		memcpy(&self->v.rightMotor.targetIq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[2], sizeof(int16_t));
		memcpy(&self->v.rightMotor.id, &stcCtrlMsg.u.sCanMessage.u.V0.abData[4], sizeof(int16_t));
		memcpy(&self->v.rightMotor.iq, &stcCtrlMsg.u.sCanMessage.u.V0.abData[6], sizeof(int16_t));
                rc = pthread_mutex_unlock(&mutex);
		break;
          /*  case "SDO_1":
                self->sdo1ResponseCount += 1;
                self->sdo1Data = stcCtrlMsg.u.sCanMessage.u.V0.abData;
            case "SDO_2":
                self->sdo2ResponseCount += 1;
                self->sdo2data = stcCtrlMsg.u.sCanMessage.u.V0.abData;*/
            default:
                
                //EciPrintCtrlMessage(&stcCtrlMsg);
		break;


        }//endSwitch
        if(currentTime - lastHeartBeat1>1500){
		OS_Printf("\nNo heartbeat from  node 1!\n");
		lastHeartBeat1 = OS_GetTimeInMs();
		}
    if(currentTime - lastHeartBeat2>1500){
		OS_Printf("\nNo heartbeat from  node 2!\n");
		lastHeartBeat2 = OS_GetTimeInMs();	
	}
        OS_Fflush(stdout);
      }//endif
      else
      {

        OS_Fflush(stdout);
        OS_Sleep(10);
      }

    }//end while
	return NULL;
}//end CANReadLoop

void *ROSReadLoop(void *arg)
{
	
	ros::spin();
}
//////////////////////////////////////////////////////////////////////////////
/**
  rotations per minute to ticks

	converts rpm from the motor controller to ticks usable by the 
	differential_odometry node

  @param dt Time interval
  @param rpm_now Last measured rpm
  @param rpm_pre Previous measured rpm
*/
/*
int rpm_to_ticks(int dt, float rpm_now, float rpm_prev){

	return (rpm_now + rpm_prev)/2 * dt * TPR / 60e6; 

}
*/
///////////////////////////////////////////////////////////////////////////////
/**
  main function

  @param argc Argument count
  @param argv Array of arguments
  @param envp environment variables

*/

int   main( int        argc,
            char**     argv,
            char**     envp )
{
	ros::init(argc,argv,"motor");
	ros::NodeHandle n;
	


 
  ECI_RESULT hResult = ECI_OK;
  //motor_ref_t ref;
  motor_reference ref;
  
  std::string drive_ok_topic;
  std::string wp_controller_sub_topic;

  n.param<std::string>("drive_ok_topic",drive_ok_topic,"/fmKnowledge/drive_ok");
  n.param<std::string>("wp_controller_input",wp_controller_sub_topic,"/fmController/output");
  thread_param_t tp;
  ntnu_fieldflux::motorValues motor_data;
  msgs::encoder encoder_right;
  msgs::encoder encoder_left;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber wp_controller_sub;

  sub = n.subscribe("joy",100,&motor_reference::joyCallback, &ref);

  sub2 = n.subscribe(drive_ok_topic,10,&motor_reference::drive_ok_callback, &ref);

  wp_controller_sub=n.subscribe(wp_controller_sub_topic,40,&motor_reference::cmd_vel_callback,&ref);
  OS_Printf(">> Linux ECI API Demo program <<\n\n");
  ros::Publisher param_pub = n.advertise<ntnu_fieldflux::motorValues>("motor_values",100); 
  ros::Publisher encoder_left_pub = n.advertise<msgs::encoder>("/fmInformation/encoder_left",10);
  ros::Publisher encoder_right_pub = n.advertise<msgs::encoder>("/fmInformation/encoder_right",10);
 
  
  tp.v.SDOResponseSent=0;
  hResult = initUSB2CAN(&tp);
 
  pthread_t thr;
  //pthread_t thr2;
  int res = pthread_create(&thr,NULL,CANReadLoop,(void *) &tp);
  //int res2 = pthread_create(&thr2,NULL,ROSReadLoop,NULL);
  //OS_Printf("Res = %d", res);
  //*** ECI Demo for USB-to-CAN compact

  
  int sendSpeedTime = OS_GetTimeInMs();
  
  	ECI_CTRL_MESSAGE send_mes = {0};
  	ECI_CTRL_MESSAGE send_mes2 ={0};
	DWORD ls; 
	DWORD rs; 
	DWORD ld; 
	DWORD rd;
	DWORD dummyByte = 0x00;
        send_mes.wCtrlClass                            = ECI_CTRL_CAN;
        send_mes.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
        send_mes.u.sCanMessage.u.V0.dwMsgId            = 0x201;
        send_mes.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = 7;
	send_mes.u.sCanMessage.u.V0.uMsgInfo.Bits.type = ECI_CAN_MSGTYPE_DATA;

	send_mes2.wCtrlClass                            = ECI_CTRL_CAN;
        send_mes2.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
        send_mes2.u.sCanMessage.u.V0.dwMsgId            = 0x202;
        send_mes2.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = 3;
  while(ref.stayInLoop==0){
      if(OS_GetTimeInMs()-sendSpeedTime > 50){
          if(!ref.getJoystickConnected()){
              OS_Printf("No connection to joystick \n\n");
          }
	//update timer for when last PDO setting speed was sent	
	sendSpeedTime = OS_GetTimeInMs();
	//update the parameter of the motor_values message
	ls = ref.getLeftSpeed();
	rs = ref.getRightSpeed();
    motor_data.leftMotor_dir = ld = ref.getLeftDir();
    motor_data.rightMotor_dir = rd = ref.getRightDir();
	motor_data.leftMotor_w = tp.v.leftMotor.velocity;
	motor_data.leftMotor_w_ref = tp.v.leftMotor.targetVelocity;
	motor_data.rightMotor_w = tp.v.rightMotor.velocity;
	motor_data.rightMotor_w_ref = tp.v.rightMotor.targetVelocity;
//send_mes.u.sCanMessage.u.V0.abData[0] = ls&255;
	//send_mes.u.sCanMessage.u.V0.abData[1] = ls<<8;
	//send_mes.u.sCanMessage.u.V0.abData[2] = ld;
	//send_mes.u.sCanMessage.u.V0.dwTime = interval;
	//send_mes2.u.sCanMessage.u.V0.abData[0] = rs&255;
	//send_mes2.u.sCanMessage.u.V0.abData[1] = rs<<8;
	//send_mes2.u.sCanMessage.u.V0.abData[2] = rd;
	        
	memcpy( &send_mes.u.sCanMessage.u.V0.abData[0],
                &ls,
                2);
        memcpy( &send_mes.u.sCanMessage.u.V0.abData[2],
                &ld,
                1);
	memcpy( &send_mes.u.sCanMessage.u.V0.abData[3],
                &dummyByte,
                1);
	memcpy( &send_mes.u.sCanMessage.u.V0.abData[4],
                &rs,
                2);
        memcpy( &send_mes.u.sCanMessage.u.V0.abData[6],
                &rd,
                1);
/*        memcpy( &send_mes2.u.sCanMessage.u.V0.abData[0],
                &rs,
                2);
        memcpy( &send_mes2.u.sCanMessage.u.V0.abData[2],
                &rd,
                1);*/
        pthread_mutex_lock(&mutex);
        hResult = ECI109_CtrlSend(tp.ip.dwCtrlHandle, &send_mes, ECIDEMO_TX_TIMEOUT);
//	hResult = ECI109_CtrlSend(tp.ip.dwCtrlHandle, &send_mes2, ECIDEMO_TX_TIMEOUT);
	
        pthread_mutex_unlock(&mutex);
        if(ECI_OK == hResult){
	   
          
          EciPrintCtrlMessage(&send_mes);
      OS_Printf("\n");
/*	  EciPrintCtrlMessage(&send_mes2);
	  OS_Printf("\n");*/
	  	
        }
        else{
	    OS_Printf("\n Error sending\n");
	    ECIDEMO_CHECKERROR(ECI109_CtrlSend);
	}
	pthread_mutex_lock(&mutex);
	encoder_left.encoderticks = -tp.v.leftMotor.ticks;
	encoder_left_pub.publish(encoder_left);
	tp.v.leftMotor.ticks = 0;
	encoder_right.encoderticks = tp.v.rightMotor.ticks;
	encoder_right_pub.publish(encoder_right);
	tp.v.rightMotor.ticks = 0;
	param_pub.publish(motor_data);
	
	ros::spinOnce();
	ECI109_CtrlStop(tp.ip.dwCtrlHandle,ECI_STOP_FLAG_CLEAR_TX_FIFO);
	ECI109_CtrlStart(tp.ip.dwCtrlHandle);
	pthread_mutex_unlock(&mutex);
	OS_Sleep(10);
	
	
      }

  }
  ld=0;
  rd =0;
  ls=0;
  rs = 0;
  memcpy( &send_mes.u.sCanMessage.u.V0.abData[0],
                &ls,
                2);
        memcpy( &send_mes.u.sCanMessage.u.V0.abData[2],
                &ld,
                1);
	
        memcpy( &send_mes2.u.sCanMessage.u.V0.abData[0],
                &rs,
                2);
        memcpy( &send_mes2.u.sCanMessage.u.V0.abData[2],
                &rd,
                1);
  hResult = ECI109_CtrlSend(tp.ip.dwCtrlHandle, &send_mes, ECIDEMO_TX_TIMEOUT);
  hResult = ECI109_CtrlSend(tp.ip.dwCtrlHandle, &send_mes2, ECIDEMO_TX_TIMEOUT);
  OS_Sleep(1000);
  ECI109_Release();
  ECI109_CtrlStop(tp.ip.dwCtrlHandle,ECI_STOP_FLAG_RESET_CTRL);
  OS_Printf("-> Closing CAN bus <-\n");

  return 0;
}




