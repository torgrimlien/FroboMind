#include <ntnu_fieldflux/initUSB2CAN.h>
#include <ntnu_fieldflux/EciDemoCommon.h>
#include <ntnu_fieldflux/motor.h>

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


ECI_RESULT initUSB2CAN(thread_param_t *tp){


  ECI_RESULT  hResult     = ECI_OK;
  ECI_HW_PARA stcHwPara   = {0};
  tp->ip.stcHwPara = stcHwPara;
  ECI_HW_INFO stcHwInfo   = {0};
  tp->ip.stcHwInfo = stcHwInfo;
  tp->ip.dwHwIndex   = 0;
  tp->ip.dwCtrlIndex = 0;


  //*** Prepare Hardware parameter structure
  tp->ip.stcHwPara.wHardwareClass = ECI_HW_USB;
  #ifdef ECIDEMO_HWUSEPOLLINGMODE
  tp->ip.stcHwPara.dwFlags = ECI_SETTINGS_FLAG_POLLING_MODE;
  #endif //ECIDEMO_HWUSEPOLLINGMODE

  //*** At first call Initialize to prepare ECI driver
  //*** with one board
  hResult = ECI109_Initialize(1, &tp->ip.stcHwPara);
  ECIDEMO_CHECKERROR(ECI109_Initialize);

  //*** Retrieve hardware info
  if(ECI_OK == hResult)
  {
    //*** Retrieve hardware info
    hResult = ECI109_GetInfo(tp->ip.dwHwIndex, &tp->ip.stcHwInfo);
    ECIDEMO_CHECKERROR(ECI109_GetInfo);
    if(ECI_OK == hResult)
      {EciPrintHwInfo(&tp->ip.stcHwInfo);}
  }

  //*** Find first CAN Controller of Board
  if(ECI_OK == hResult)
  {
    hResult = EciGetNthCtrlOfClass(&tp->ip.stcHwInfo,
                                   ECI_CTRL_CAN,
                                   0, //first relative controller
                                   &tp->ip.dwCtrlIndex);
  }
  tp->ip.dwCtrlHandle  = ECI_INVALID_HANDLE;


  //*** Open given controller of given board
  if(ECI_OK == hResult)
  {
    ECI_CTRL_CONFIG stcCtrlConfig = {0};
    tp->ip.stcCtrlConfig = stcCtrlConfig;

    //*** Set CAN Controller configuration
    tp->ip.stcCtrlConfig.wCtrlClass                = ECI_CTRL_CAN;
    tp->ip.stcCtrlConfig.u.sCanConfig.dwVer        = ECI_STRUCT_VERSION_V0;
    tp->ip.stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0 = ECI_CAN_BT0_1000KB;
    tp->ip.stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1 = ECI_CAN_BT1_1000KB;
    tp->ip.stcCtrlConfig.u.sCanConfig.u.V0.bOpMode = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED | ECI_CAN_OPMODE_ERRFRAME;

    //*** Open and Initialize given Controller of given board
    hResult = ECI109_CtrlOpen(&tp->ip.dwCtrlHandle, tp->ip.dwHwIndex, tp->ip.dwCtrlIndex, &tp->ip.stcCtrlConfig);
    ECIDEMO_CHECKERROR(ECI109_CtrlOpen);
  }

  //*** Get Controller Capabilites
  if(ECI_OK == hResult)
  {
    ECI_CTRL_CAPABILITIES stcCtrlCaps = {0};
    tp->ip.stcCtrlCaps = stcCtrlCaps;

    hResult = ECI109_CtrlGetCapabilities(tp->ip.dwCtrlHandle, &tp->ip.stcCtrlCaps);
    ECIDEMO_CHECKERROR(ECI109_CtrlGetCapabilities);
    if(ECI_OK == hResult)
      {EciPrintCtrlCapabilities(&tp->ip.stcCtrlCaps);}
  }

  //*** Start Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI109_CtrlStart(tp->ip.dwCtrlHandle);
    ECIDEMO_CHECKERROR(ECI109_CtrlStart);
  }


  return hResult;

}
