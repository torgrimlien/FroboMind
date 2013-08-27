#include <ntnu_fieldflux/EciDemo109.h>
#include <ntnu_fieldflux/EciDemoCommon.h>
#include <ntnu_fieldflux/CANopenMessages.h>




/**
///////////////////////////////////////////////////////////////////////////////
SDO_Write
    @param nodeID
        Which node to write to
    @param objectInd
        Which object index to write to
    @param subInd
        Which subindex to write to
    @param data
        Data to be written to the object dictionary


*/
ECI_RESULT sdo_wnx (DWORD nodeID, DWORD objectInd, DWORD subInd, DWORD data, ECI_CTRL_HDL ctrlHdl)
{

    ECI_RESULT  hResult                             = ECI_OK;
    ECI_CTRL_MESSAGE    sdo_write                   =   {0};
    DWORD data0write                                =   0x2F-(sizeof(data)-1)*4;
    sdo_write.wCtrlClass                            =   ECI_CTRL_CAN;
    sdo_write.u.sCanMessage.dwVer                   =   ECI_STRUCT_VERSION_V0;
    sdo_write.u.sCanMessage.u.V0.dwMsgId             =   0x600 + nodeID;
    sdo_write.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  =   4+sizeof(data);

    memcpy(&sdo_write.u.sCanMessage.u.V0.abData[0],&data0write,1);
    memcpy(&sdo_write.u.sCanMessage.u.V0.abData[1],&objectInd ,2);
    //memcpy(&sdo_read.u.sCanMessage.u.V0.abData[2],(&ind >> 8),1);
    memcpy(&sdo_write.u.sCanMessage.u.V0.abData[3],&subInd,1);

    memcpy(&sdo_write.u.sCanMessage.u.V0.abData[4],&data,sizeof(data));

    hResult = ECI109_CtrlSend( 0, &sdo_write, 500);
    if(ECI_OK==hResult)
    {
        EciPrintCtrlMessage(&sdo_write);
        OS_Printf("\n");
    }
    return hResult;

}


/**
///////////////////////////////////////////////////////////////////////////////
SDO read
    @param nodeID
        Which node to read from
    @param objectInd
        Which object index to read from
    @param subInd
        Which subindex to read from
    TODO:
        filter received messages on its COB-ID and if read reqest is successfull,
        and handle other messages not received
*/
ECI_CTRL_MESSAGE sdo_rnx (DWORD nodeID, DWORD ind, DWORD subInd, ECI_CTRL_HDL ctrlHdl)
{
    DWORD   data0read                               =   0x40;


    ECI_RESULT res = ECI_OK;
    ECI_CTRL_MESSAGE    sdo_read                    =   {0};
    ECI_CTRL_MESSAGE    readmes                     =   {0};
    sdo_read.wCtrlClass                             =   ECI_CTRL_CAN;
    sdo_read.u.sCanMessage.dwVer                    =   ECI_STRUCT_VERSION_V0;
    sdo_read.u.sCanMessage.u.V0.dwMsgId             =   0x600 + nodeID;
    sdo_read.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc   =   8;

    memcpy(&sdo_read.u.sCanMessage.u.V0.abData[0],&data0read,1);
    memcpy(&sdo_read.u.sCanMessage.u.V0.abData[1],&ind,2);

    memcpy(&sdo_read.u.sCanMessage.u.V0.abData[3],&subInd,1);
    memcpy(&sdo_read.u.sCanMessage.u.V0.abData[4],0,4);

    res = ECI109_CtrlSend( ctrlHdl, &sdo_read, 500);
    if (res = ECI_OK){
        res = ECI109_Ctrl_Receive(ctrlHdl, 1, readmes, 500);
    }
    else{
        OS_Printf("Error sending message");
    }
    EciPrintCtrlMessage(&readmes);
    return readmes;

}
/**
////////////////////////////////////////////////////////////////////////////
set operational
    @param nodeID
        Which node to set operational
*/
ECI_RESULT setOperational(DWORD nodeID,ECI_CTRL_HDL ctrlHdl)
{

    ECI_RESULT  hResult                                     =   ECI_OK;
    hResult = sdo_wnx(nodeID, 0x2800, 0x00, 0x00, ctrlHdl);


    ECI_CTRL_MESSAGE    setOperational                  =   {0};
    DWORD   setop                                           =   1;
    setOperational.wCtrlClass                           =   ECI_CTRL_CAN;
    setOperational.u.sCanMessage.dwVer                  =   ECI_STRUCT_VERSION_V0;
    setOperational.u.sCanMessage.u.V0.dwMsgId           =   0x00;
    setOperational.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc =   2;

    memcpy(&setOperational.u.sCanMessage.u.V0.abData[0], &setop, 1);
    memcpy(&setOperational.u.sCanMessage.u.V0.abData[1], &nodeID ,1);

    hResult = ECI109_CtrlSend( ctrlHdl, &setOperational, 500);
    return hResult;




}
/**
////////////////////////////////////////////////////////////////////////////
set preoperational
    @param nodeID
        Which node to set preoperational
*/
ECI_RESULT setPreOperational(DWORD nodeID, ECI_CTRL_HDL ctrlHdl)
{
    ECI_RESULT  hResult                                        = ECI_OK;
    ECI_CTRL_MESSAGE    sdo_setPreOperational                  =   {0};
    DWORD setpre                                               = 128;
    sdo_setPreOperational.wCtrlClass                           =   ECI_CTRL_CAN;
    sdo_setPreOperational.u.sCanMessage.dwVer                  =   ECI_STRUCT_VERSION_V0;
    sdo_setPreOperational.u.sCanMessage.u.V0.dwMsgId           =   0;
    sdo_setPreOperational.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc =   2;

    memcpy(&sdo_setPreOperational.u.sCanMessage.u.V0.abData[0], &setpre,1);
    memcpy(&sdo_setPreOperational.u.sCanMessage.u.V0.abData[1], &nodeID,1);

    hResult = ECI109_CtrlSend( ctrlHdl, &sdo_setPreOperational, 500);
    return hResult;


}
