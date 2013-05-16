#ifndef CANMESSAGEHANDLER_INCLUDED_
#define CANMESSAGEHANDLER_INCLUDED_

#include <ntnu_fieldflux/EciDemo109.h>
#include <ntnu_fieldflux/EciDemoCommon.h>
#include <ntnu_fieldflux/CANopenMessages.h>
#include <ntnu_fieldflux/PdoConfiguration.h>
#include <ntnu_fieldflux/initUSB2CAN.h>
#include <ntnu_fieldflux/motor.h>
#include <ntnu_fieldflux/motorVal.h>
#include <ntnu_fieldflux/motorValues.h>
#include <ntnu_fieldflux/motor_reference.h>
#include <ros/ros.h>
#include <pthread.h>
#include <boost/bind.hpp>


void *CANReadLoop(void *arg);
void *ROSReadLoop(void *arg);
int main(int argc, char**argv,char** envp);
#endif
