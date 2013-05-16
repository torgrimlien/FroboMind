#ifndef PDOCONFIGURATION_H_INCLUDED
#define PDOCONFIGURATION_H_INCLUDED

#ifdef __cplusplus
extern "C"{
#endif
#include <ntnu_fieldflux/CANopenMessages.h>
ECI_RESULT RpdoCommunication(DWORD nodeID, DWORD pdoNumber, DWORD CobID, DWORD transType, ECI_CTRL_HDL ctrlHdl);      //ctrl handler.

ECI_RESULT RpdoMap(DWORD nodeID, DWORD pdoNumber, DWORD MapIndex, DWORD index, DWORD subIndex, DWORD dataLength, ECI_CTRL_HDL ctrlHdl);

ECI_RESULT TpdoCommunication(DWORD nodeID,DWORD pdoNumber, DWORD CobID, DWORD transType, ECI_CTRL_HDL ctrlHdl);

ECI_RESULT TpdoMap(DWORD nodeID, DWORD pdoNumber,DWORD MapIndex, DWORD index, DWORD subIndex, DWORD dataLength, ECI_CTRL_HDL ctrlHdl);

ECI_RESULT setSpeed(DWORD speed, DWORD direction);

#ifdef __cplusplus
}
#endif

#endif // PDOCONFIGURATION_H_INCLUDED
