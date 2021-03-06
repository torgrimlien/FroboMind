///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2012 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo for USB-to-CAN compact

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file EciDemo109.c
*/


//////////////////////////////////////////////////////////////////////////
// compiler directives

//////////////////////////////////////////////////////////////////////////
// include files
#include <ntnu_fieldflux/EciDemo109.h>
#include <ntnu_fieldflux/EciDemoCommon.h>
#include <ntnu_fieldflux/CANopenMessages.h>
#include <ntnu_fieldflux/PdoConfiguration.h>


//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables

/** ECI Demo send timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_TX_TIMEOUT 500

/** ECI Demo TX message count for CAN @ingroup EciDemo */
#define ECIDEMO_TX_MSGCOUNT_CAN (0x800 * 10)

/** ECI Demo receive timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_RX_TIMEOUT 500

/** ECI Demo total receive timeout in [ms] @ingroup EciDemo */
#define ECIDEMO_RX_TOTALTIMEOUT 30000

/**
  If defined ECI Demo will run in polling mode instead of using an
 event triggered mode. This mode can be used, if it is not possible to
  assign an unique IRQ to the device to use. The device driver than
  will not use an IRQ based message reception and transmission.

  @ingroup EciDemo
*/
//#define ECIDEMO_HWUSEPOLLINGMODE

/** ECI Demo error check macro @ingroup EciDemo */
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
//////////////////////////////////////////////////////////////////////////
// global variables

//////////////////////////////////////////////////////////////////////////
// static function prototypes

ECI_RESULT EciCanDemo109  ( DWORD   dwHwIndex,
                            DWORD   dwCtrlIndex);

//////////////////////////////////////////////////////////////////////////
// global functions

//////////////////////////////////////////////////////////////////////////
// static functions

///////////////////////////////////////////////////////////////////////////////
/**
  ECI Demo for USB-to-CAN compact

  @return ECI_OK on success, otherwise an error code

  @ingroup EciDemo
*/

ECI_RESULT EciDemo109(void)
{
  ECI_RESULT  hResult     = ECI_OK;
  ECI_HW_PARA stcHwPara   = {0};
  ECI_HW_INFO stcHwInfo   = {0};
  DWORD       dwHwIndex   = 0;
  DWORD       dwCtrlIndex = 0;

  OS_Printf("\n>> ECI Demo for USB-to-CAN compact <<\n");




  //*** Prepare Hardware parameter structure
  stcHwPara.wHardwareClass = ECI_HW_USB;
  #ifdef ECIDEMO_HWUSEPOLLINGMODE
    OS_Printf("PULLINGMODE");
    stcHwPara.dwFlags = ECI_SETTINGS_FLAG_POLLING_MODE;
  #endif //ECIDEMO_HWUSEPOLLINGMODE

  //*** At first call Initialize to prepare ECI driver
  //*** with one board
  hResult = ECI109_Initialize(1, &stcHwPara);
  ECIDEMO_CHECKERROR(ECI109_Initialize);

  //*** Retrieve hardware info
  if(ECI_OK == hResult)
  {
    //*** Retrieve hardware info
    hResult = ECI109_GetInfo(dwHwIndex, &stcHwInfo);
    ECIDEMO_CHECKERROR(ECI109_GetInfo);
    if(ECI_OK == hResult)
      {EciPrintHwInfo(&stcHwInfo);}
  }

  //*** Find first CAN Controller of Board
  if(ECI_OK == hResult)
  {
    hResult = EciGetNthCtrlOfClass(&stcHwInfo,
                                   ECI_CTRL_CAN,
                                   0, //first relative controller
                                   &dwCtrlIndex);
    if(ECI_OK == hResult)
    {
      //*** Start CAN Demo
      hResult =  EciCanDemo109(dwHwIndex, dwCtrlIndex);
      ECIDEMO_CHECKERROR(EciCanDemo109);
    }
    else
    {
      //*** Ignore if no controller was found
      hResult = ECI_OK;
    }
  }

  //*** Clean up ECI driver
  ECI109_Release();

  OS_Printf("-> Returning from ECI Demo for USB-to-CAN compact <-\n");

  return hResult;
}


///////////////////////////////////////////////////////////////////////////////
/**
  ECI CAN Demo

  @param dwHwIndex
    Hardware index of board to use.

  @param dwCtrlIndex
    Controller index to use.

  @return ECI_OK on success, otherwise an error code

  @ingroup EciDemo
*/
ECI_RESULT EciCanDemo109(DWORD dwHwIndex,
                         DWORD dwCtrlIndex)
{
  ECI_RESULT    hResult       = ECI_OK;
  ECI_CTRL_HDL  dwCtrlHandle  = ECI_INVALID_HANDLE;

  OS_Printf("\n>> ECI CAN Demo <<\n");

  //*** Open given controller of given board
  if(ECI_OK == hResult)
  {
    ECI_CTRL_CONFIG stcCtrlConfig = {0};

    //*** Set CAN Controller configuration
    stcCtrlConfig.wCtrlClass                = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer        = ECI_STRUCT_VERSION_V0;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0 = ECI_CAN_BT0_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1 = ECI_CAN_BT1_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bOpMode = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED | ECI_CAN_OPMODE_ERRFRAME;

    //*** Open and Initialize given Controller of given board
    hResult = ECI109_CtrlOpen(&dwCtrlHandle, dwHwIndex, dwCtrlIndex, &stcCtrlConfig);
    ECIDEMO_CHECKERROR(ECI109_CtrlOpen);
  }

  //*** Get Controller Capabilites
  if(ECI_OK == hResult)
  {
    ECI_CTRL_CAPABILITIES stcCtrlCaps = {0};

    hResult = ECI109_CtrlGetCapabilities(dwCtrlHandle, &stcCtrlCaps);
    ECIDEMO_CHECKERROR(ECI109_CtrlGetCapabilities);
    if(ECI_OK == hResult)
      {EciPrintCtrlCapabilities(&stcCtrlCaps);}
  }

  //*** Start Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI109_CtrlStart(dwCtrlHandle);
    ECIDEMO_CHECKERROR(ECI109_CtrlStart);
  }

    hResult = setOperational(1, dwCtrlHandle);
    /*ECI_CTRL_MESSAGE receive = {0};
    receive = sdo_rnx(1, 0x600, 0,dwCtrlHandle);
    EciPrintCtrlMessage(&receive);
    */

  //*** Send some CAN Messages
  if(ECI_OK == hResult)
  {
    ECI_CTRL_MESSAGE stcCtrlMsg   = {0};
    DWORD            dwTxMsgCount = 5;
    DWORD            dwIndex      = 0;
    DWORD            motdir        =0;
    DWORD            motval      =0x0000;

    OS_Printf("Now, sending %u CAN Messages\n", dwTxMsgCount);






    //*** Send Loop
    for(dwIndex=0; dwIndex < dwTxMsgCount; dwIndex++)
    {
      //*** Prepare CAN Message to send
      /*stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
      stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
      stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId            = 0x200;
      stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = 3;
      memcpy( &stcCtrlMsg.u.sCanMessage.u.V0.abData[0],
              &motval,
              2);
      memcpy( &stcCtrlMsg.u.sCanMessage.u.V0.abData[2],
              &motdir,
              1);

      //*** Send one message
      #ifdef ECIDEMO_HWUSEPOLLINGMODE
      {
        DWORD dwStartTime = OS_GetTimeInMs();

        //*** Loop until message is sent or timeout has passed
        do
        {
          hResult = ECI109_CtrlSend( dwCtrlHandle, &stcCtrlMsg, 0);
          if(ECI_OK != hResult)
            { OS_Sleep(1);}
        }while((ECI_OK != hResult) && ((OS_GetTimeInMs() - dwStartTime) < ECIDEMO_TX_TIMEOUT));
      }
      #else
        hResult = ECI109_CtrlSend( dwCtrlHandle, &stcCtrlMsg, ECIDEMO_TX_TIMEOUT);
      #endif //ECIDEMO_HWUSEPOLLINGMODE         */
      hResult = setSpeed(0x0000,0x00);
      OS_Printf("\n");
      OS_Sleep(100);

      if(ECI_OK != hResult)
      {
        OS_Printf("Error while sending CAN Messages\n");
        ECIDEMO_CHECKERROR(ECI109_CtrlSend);
        hResult = ECI_OK;
        break;
      }
      else
      {
        //*** Read out all received messages
        //OS_Sleep(5000);
        ECI_CTRL_MESSAGE  astcCtrlMsg[20] = {{0}};
        DWORD             dwCount         = _countof(astcCtrlMsg);
        DWORD             dwMsgIndex      = 0;

        //*** Try to read some messages
        hResult = ECI109_CtrlReceive( dwCtrlHandle, &dwCount, astcCtrlMsg, 0);

        //*** Loop through all received messages
        dwMsgIndex = 0;
       /* while((ECI_OK == hResult) && (dwCount > dwMsgIndex))
        {
          EciPrintCtrlMessage(&stcCtrlMsg);
          OS_Printf("\n");
            OS_Sleep(1);
          //*** Proceed with next message
          dwMsgIndex++;
        }//end while*/
        //*** Reset Error code and proceed with transmission
        hResult = ECI_OK;
      }//end else
    }//end for
  }//endif


  //*** Try to receive some CAN Messages
  if(ECI_OK == hResult)
  {
    ECI_CTRL_MESSAGE stcCtrlMsg   = {0};
    DWORD           dwStartTime   = 0;
    DWORD           dwCurrentTime = 0;
    DWORD           dwRxTimeout   = ECIDEMO_RX_TOTALTIMEOUT;
    DWORD           dwCount       = 0;

    //*** Receive Messages
    OS_Printf("Now, receiving CAN Messages for %u seconds\n", dwRxTimeout/1000);

    //*** Get current time
    dwStartTime   = OS_GetTimeInMs();
    dwCurrentTime = dwStartTime;
    hResult       = ECI_ERR_TIMEOUT;

    //*** Loop until timeout
    while(dwRxTimeout >= (dwCurrentTime - dwStartTime))
    {
      //*** Try to read Message
      #ifdef ECIDEMO_HWUSEPOLLINGMODE
      {
        DWORD dwStartTime = OS_GetTimeInMs();


        //*** Loop until message is sent or timeout has passed
        do
        {
          dwCount = 1;
          hResult = ECI109_CtrlReceive( dwCtrlHandle, &dwCount, &stcCtrlMsg, 0);
          if((ECI_OK != hResult) || (0 == dwCount))
            { OS_Sleep(1);}
        }while((ECI_OK != hResult) && ((OS_GetTimeInMs() - dwStartTime) < ECIDEMO_RX_TIMEOUT));
      }
      #else
      {
        dwCount = 1;

        hResult = ECI109_CtrlReceive( dwCtrlHandle, &dwCount, &stcCtrlMsg, ECIDEMO_RX_TIMEOUT);
      }
      #endif //ECIDEMO_HWUSEPOLLINGMODE
      if((ECI_OK == hResult) && (dwCount > 0))
      {
        OS_Printf("\n");
        EciPrintCtrlMessage(&stcCtrlMsg);
        OS_Fflush(stdout);
      }//endif
      else
      {
        OS_Printf(".");
        OS_Fflush(stdout);
      }

      //*** Get current Time
      dwCurrentTime = OS_GetTimeInMs();
    }//end while
    OS_Printf("\n");

    //*** Reset error code
    hResult = ECI_OK;
  }//endif


  //*** Stop Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI109_CtrlStop(dwCtrlHandle, ECI_STOP_FLAG_NONE);
    ECIDEMO_CHECKERROR(ECI109_CtrlStop);
  }

  //*** Wait some time to ensure bus idle
  OS_Sleep(250);

  //*** Reset Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI109_CtrlStop(dwCtrlHandle, ECI_STOP_FLAG_RESET_CTRL);
    ECIDEMO_CHECKERROR(ECI109_CtrlStop);
  }

  //*** Close ECI Controller
  ECI109_CtrlClose(dwCtrlHandle);
  dwCtrlHandle = ECI_INVALID_HANDLE;

  return hResult;
}
