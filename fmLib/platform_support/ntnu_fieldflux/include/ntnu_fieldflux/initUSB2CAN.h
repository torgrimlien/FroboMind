#ifndef INITUSB2CAN_H_INCLUDED
#define INITUSB2CAN_H_INCLUDED



//////////////////////////////////////////////////////////////////////////
// constants and macros

//////////////////////////////////////////////////////////////////////////
// data types

#include <ntnu_fieldflux/EciDemo109.h>
#include <ntnu_fieldflux/motor.h>

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus



ECI_RESULT initUSB2CAN(thread_param_t *self);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // INITUSB2CAN_H_INCLUDED

