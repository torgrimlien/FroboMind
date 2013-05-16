///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2012 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo common helper functions

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file EciDemoCommon.h
*/

#ifndef __ECIDEMOCOMMON_H__
#define __ECIDEMOCOMMON_H__

#ifdef __cplusplus
extern "C"{
#endif
//////////////////////////////////////////////////////////////////////////
// include files
#include "ECI_hwtype.h"

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/* _countof helper */
#if !defined(_countof)
  #define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

//////////////////////////////////////////////////////////////////////////
// data types

#include <ECI_poppack.h>

//////////////////////////////////////////////////////////////////////////
// static function prototypes

void        EciPrintHwInfo            ( const ECI_HW_INFO*            pstcHwInfo);
void        EciPrintCtrlCapabilities  ( const ECI_CTRL_CAPABILITIES*  pstcCtrlCaps);
void        EciPrintCtrlMessage       ( const ECI_CTRL_MESSAGE*       pstcCtrlMsg);

ECI_RESULT  EciGetNthCtrlOfClass      ( const ECI_HW_INFO*            stcHwInfo,
                                        e_CTRLCLASS                   eCtrlClass,
                                        DWORD                         dwRelCtrlIndex,
                                        DWORD*                        pdwCtrIndex);

#ifdef __cplusplus
}
#endif
#endif //__ECIDEMOCOMMON_H__

