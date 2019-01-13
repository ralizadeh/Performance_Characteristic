/*******************************************************************************
 *
 * File        : ArrAdpt_AXIinterface.cpp
 *
 * Created Date: March 05, 2018
 *
 * Description : The aim is to get error count from arrive side
 * 
 ******************************************************************************/

#include<stdio.h>

// Monitor 2.5G Eth interface (used by PDH device only)
#include "AtEthPort.h"
#include "ArrAdpt_private.h" 

/******************************************************************************
 *
 *  Name:         ArrAdpt_getAndclearFCSerror
 *
 *  Description:  This function gets and then clears the FCS error count on an Ethernet port.   
 *  
 *  Parameters:   dev: PDH device number
 *                AXIPort
 *                clear as a switch to call get or clear function
 *                     TRUE  = read and clear
 *                     FALSE = read
 *                                  
 *  Output        errorcount
 *
 *  Return:       ArrAdpt_rc_t
 *
*******************************************************************************/
ArrAdpt_rc_t ArrAdpt_getAndclearFCSerror (ArrAdpt_dev_id_t dev, uint8  AXIPort, uint32 *errorcount, boolean clear)
{
    AtEthPort          AtEPort;
    AtModuleEth        AtModEth;
    AtDevice           AtDev;
    
    if (dev >= ArrAdpt_num_configured_devices())  
    {
        ARRADPT_GN_LOG_ERROR("Invalid dev %d, AXI port = %d", dev, AXIPort);
        return   ARRADPT_RC_NO_DEVICE;    
    }

    if (errorcount==NULL) 
    { 
        ARRADPT_GN_LOG_ERROR("Null error count for dev %d, AXI port %d = %d", dev, AXIPort, errorcount);
        return ARRADPT_RC_INVALID_PARAM; 
    }

    AtDev    = AtDriverDeviceGet(AtDriverSharedDriverGet(), dev);
    AtModEth = (AtModuleEth)AtDeviceModuleGet(AtDev, cAtModuleEth);
    AtEPort  =  AtModuleEthPortGet(AtModEth, AXIPort);

    if (clear) 
       *errorcount = AtChannelCounterClear((AtChannel)AtEPort, cAtEthPortCounterRxErrFcsPackets);
    else
       *errorcount = AtChannelCounterGet((AtChannel)AtEPort, cAtEthPortCounterRxErrFcsPackets);

    return ARRADPT_RC_SUCCESS; 
}



