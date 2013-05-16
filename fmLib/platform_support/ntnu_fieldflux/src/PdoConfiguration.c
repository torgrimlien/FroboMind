
#include <ntnu_fieldflux/EciDemoCommon.h>
#include <ntnu_fieldflux/CANopenMessages.h>
#include <ntnu_fieldflux/PdoConfiguration.h>


/**
////////////////////////////////////////////////////////////////////////////
RPDO Communication parameters
    @param nodeID
        Which node write to
    @param pdoNumber
        Which of the RPDOs to set up
    @param COB-ID
        COB-ID of the PDO message to interpet
    @param transType
        When receiver should process the data in the PDO
*/
ECI_RESULT RpdoCommunication(DWORD nodeID, DWORD pdoNumber, DWORD CobID,DWORD transType, ECI_CTRL_HDL ctrlHdl)
{


    ECI_RESULT hResult          =   ECI_OK;
    //Object index for PDO communication parameters
    DWORD objectIndex = 0x13FF + pdoNumber;
    //Subindex where COB ID is written
    DWORD subInd = 0x01;
    hResult = sdo_wnx (nodeID,objectIndex, subInd, CobID, ctrlHdl);

    //Subindex where transmission Type is written
    subInd= 0x02;
    //Transmission type
    hResult = sdo_wnx (nodeID,objectIndex, subInd, transType, ctrlHdl);

}
/**
////////////////////////////////////////////////////////////////////////////
RPDO Mapping parameters
    @param nodeID
        Which node write to
    @param pdoNumber
        Which of the RPDOs to set up
    @param MapIndex
        Which map to write to
    @param index
        Index in the object library
    @param subIndex
        Sub Index in the object library
    @param dataLength
        Length of data to be sent to the index/sub index in the object library
*/
ECI_RESULT RpdoMap(DWORD nodeID, DWORD pdoNumber,DWORD MapIndex, DWORD index, DWORD subIndex, DWORD dataLength ,ECI_CTRL_HDL ctrlHdl)
{
    ECI_RESULT   hResult        = ECI_OK;
    DWORD objectIndex           = 0x15FF + pdoNumber;
    DWORD writeData             = (index<<16)+(subIndex<<8)+dataLength;

    hResult = sdo_wnx (nodeID,objectIndex, MapIndex, writeData, ctrlHdl);
}
/**
////////////////////////////////////////////////////////////////////////////
TPDO Communication parameters
    @param nodeID
        Which node write to
    @param pdoNumber
        Which of the TPDOs to set up
    @param COB-ID
        COB-ID of the PDO message to send
    @param transType
        synchronous or rtr
*/
ECI_RESULT TpdoCommunication(DWORD nodeID, DWORD pdoNumber, DWORD CobID, DWORD transType,ECI_CTRL_HDL ctrlHdl)
{
    ECI_RESULT hResult          =   ECI_OK;
    //Object index for PDO communication parameters
    DWORD objectIndex           = 0x17FF + pdoNumber;

    DWORD subInd                = 0x01;

    hResult                     = sdo_wnx (nodeID,objectIndex, subInd, CobID, ctrlHdl);

    subInd                      = 0x02;
    hResult = sdo_wnx (nodeID,objectIndex, subInd, transType, ctrlHdl);

}
/**
////////////////////////////////////////////////////////////////////////////
TPDO Mapping parameters
    @param nodeID
        Which node write to
    @param pdoNumber
        Which of the TPDOs to set up
    @param MapIndex
        Which map to write to
    @param index
        Index in the object library
    @param subIndex
        Sub Index in the object library
    @param dataLength
        Length of data to be sent from the index/sub index in the object library
*/
ECI_RESULT TpdoMap(DWORD nodeID, DWORD pdoNumber,DWORD MapIndex, DWORD index, DWORD subIndex, DWORD datalength ,ECI_CTRL_HDL ctrlHdl)
{
    ECI_RESULT hResult          =   ECI_OK;
    DWORD writeData            = (index<<16)+(subIndex<<8)+datalength;
    DWORD objectIndex = 0x19FF + pdoNumber;


    hResult = sdo_wnx (nodeID,objectIndex, MapIndex, writeData, ctrlHdl);
}

//Function setup to set the speed of the motor, sends a PDO with COB-ID 0x200 along with speed and direction,
ECI_RESULT setSpeed(DWORD speed, DWORD direction)
{
    ECI_RESULT          hResult                      = ECI_OK;
    ECI_CTRL_MESSAGE stcCtrlMsg                      = {0};

    stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
    stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
    stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId            = 0x200;
    stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = 3;
    memcpy( &stcCtrlMsg.u.sCanMessage.u.V0.abData[0],
              &speed,
              2);
    memcpy( &stcCtrlMsg.u.sCanMessage.u.V0.abData[2],
              &direction,
              1);
    hResult = ECI109_CtrlSend( 0, &stcCtrlMsg, 500);

}
