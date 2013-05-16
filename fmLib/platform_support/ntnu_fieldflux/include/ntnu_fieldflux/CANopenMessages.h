#ifndef CANOPENMESSAGES_H_INCLUDED
#define CANOPENMESSAGES_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
//////////////////////////////////////////////////////////////////////////
// include files
//#include <CANopenMessages.h>
//////////////////////////////////////////////////////////////////////////
// constants and macros

//////////////////////////////////////////////////////////////////////////
// data types



ECI_RESULT sdo_wnx (DWORD nodeID, DWORD objectInd, DWORD subInd, DWORD data, ECI_CTRL_HDL ctrlHdl);

ECI_CTRL_MESSAGE sdo_rnx (DWORD nodeID, DWORD ind, DWORD subInd, ECI_CTRL_HDL ctrlHdl);

ECI_RESULT setOperational(DWORD nodeID, ECI_CTRL_HDL ctrlHdl);

ECI_RESULT setPreOperational(DWORD nodeID, ECI_CTRL_HDL ctrlHdl);

#ifdef __cplusplus
}
#endif

#endif // CANOPENMESSAGES_H_INCLUDED
