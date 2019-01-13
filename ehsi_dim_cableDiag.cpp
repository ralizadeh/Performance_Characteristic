/*******************************************************************************
 *
 * File        : ehsi_dim_cableDiag.cpp
 *
 * Created Date: March 05, 2018
 *
 * Description : The aim is to calculate BER for DIM-PDH cable and
 * raise signal fail/signal degrade alarms from the Flare engine 
 * 
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ehsi_dim_mgr.h"
#include "dimDrv_dashboard.h" 
#include "diag_api_PTS_MRO_PDH.h"
#include "ehsi_dim_cableDiag.h"
#include "ArrAdpt_public.h"
#include "alrstui.h"
#include "vkcardid.h"

// DIM cable threshold calculation definition
#define PTS_DIMCABLE_MAX                      PTS_PDH_2xDIM_NUM_LINK  
#define PTS_DIMCABLE_NUMTHRESH                13  
#define PTS_DIMCABLE_MAX_ERRORCOUNT           30 
#define PTS_DIMCABLE_NUMCOLUMNS               5 
#define PTS_DIMCABLE_DISCARD_NUM              5
#define PTS_DIMCABLE_SFTHRESHOLD_R            9.99E-5   // 1E-4 
#define PTS_DIMCABLE_SFTHRESHOLD_C            1E-5
#define PTS_DIMCABLE_SDTHRESHOLD_R            9.99E-11  // 1E-10
#define PTS_DIMCABLE_SDTHRESHOLD_C            1E-11

boolean DIMLogPrint = FALSE;
#define prCable(...) {if (DIMLogPrint) printf(__VA_ARGS__);}

typedef struct {
    float BERThreshold;
    uint32 FrameErrors10Sec;
} Cable_info_t;

/* ****************************************************************************** 
 *  
 *   The DIM cable uses an ethernet packet based "AXI" interface, which transmits 
 * control and data frames with a frame check sum (FCS).  The cable degrade/fail 
 * detection uses the frame error count (#frames with an FCS error) to estimate 
 * the error rate on the cable.  An analysis of the frame sizes/rates for both 
 * DS3/E3  and DS1/E1 DIMs by Chris Brown provided a fixed set of numbers to 
 * use in this calculation.  Specifically the number of frames with errors 
 * is counted over a 10second window and compared to the numbers in this 
 * analysis to determine an approximate rate of errors(10e-1 to 10e-13). 
 * These tables are shown below.   
 *
 * Here are the full tables, only the BER and Errored RxFrames are
 * needed here and coded below. 
 
     //BER        FER   Errored_Rx_Frames  Tx_Frm    Frm_Size              
const float DS1_FER_Tbl[PTS_DIMCABLE_NUMTHRESH][PTS_DIMCABLE_NUMCOLUMNS]=
{
      {1E-1,     1,         2566110,       2566110,    2264},
      {1E-2,     1,         2566110,       2566110,    2264},
      {1E-3,     8.96E-1,   2299707,       2566110,    2264},
      {1E-4,     2.03E-1,   519920,        2566110,    2264},
      {1E-5,     2.24E-2,   57445,         2566110,    2264},
      {1E-6,     2.26E-3,   5804,          2566110,    2264},
      {1E-7,     2.26E-4,   581,           2566110,    2264},
      {1E-8,     2.26E-5,   59,            2566110,    2264},
      {1E-9,     2.26E-6,   6,             2566110,    2264},
      {1E-10,    2.26E-7,   1,             2566110,    2264},
      {1E-11,    2.26E-8,   1,             2566110,    2264},
      {1E-12,    2.26E-9,   1,             2566110,    2264},
      {1E-13,    2.26E-10,  1,             2566110,    2264}
};

const float DS3_FER_Tbl[PTS_DIMCABLE_NUMTHRESH][PTS_DIMCABLE_NUMCOLUMNS]=
{
      {1E-1,     1,         5436790,       5436790,    3608},
      {1E-2,     1,         5436790,       5436790,    3608},
      {1E-3,     9.73E-1,   5289687,       5436790,    3608},
      {1E-4,     3.03E-1,   1646772,       5436790,    3608},
      {1E-5,     3.54E-2,   192664,        5436790,    3608},
      {1E-6,     3.60E-3,   19581,         5436790,    3608},
      {1E-7,     3.61E-4,   1962,          5436790,    3608},
      {1E-8,     3.61E-5,   197,           5436790,    3608},
      {1E-9,     3.61E-6,   20,            5436790,    3608},
      {1E-10,    3.61E-7,   2,             5436790,    3608},
      {1E-11,    3.61E-8,   1,             5436790,    3608},
      {1E-12,    3.61E-9,   1,             5436790,    3608},
      {1E-13,    3.61E-10,  1,             5436790,    3608}
};

 *******************************************************************************/

Cable_info_t DS1_BER_info[PTS_DIMCABLE_NUMTHRESH] = 
{ 
      {1E-1,     2566110},
      {1E-2,     2566110},
      {1E-3,     2299707},
      {1E-4,     519920},
      {1E-5,     57445},
      {1E-6,     5804},
      {1E-7,     581},
      {1E-8,     59},
      {1E-9,     6},
      {1E-10,    1},
      {1E-11,    1},
      {1E-12,    1},
      {1E-13,    1}
};

Cable_info_t DS3_BER_info[PTS_DIMCABLE_NUMTHRESH] = 
{ 
      {1E-1,     5436790},
      {1E-2,     5436790},
      {1E-3,     5289687},
      {1E-4,     1646772},
      {1E-5,     192664},
      {1E-6,     19581},
      {1E-7,     1962},
      {1E-8,     197},
      {1E-9,     20},
      {1E-10,    2},
      {1E-11,    1},
      {1E-12,    1},
      {1E-13,    1}
};

// DimCableDeg structure is initializaed to 0 in init function.
DimCabledegrade_t   DimCableDeg[PTS_DIMCABLE_NUMCABLES];
uint8               max_numDimLink;
static  boolean     clearAlarms = FALSE;
boolean             errorMode = FALSE;

/***************************************************************************
 *
 *  Name:         DIM_PDH_BER_calculate
 *
 *  Description:  Calculate BER using error counts and fixed tables
 *
 *  Parameters:   cable number
 *                total_erroredFrm which is summation of 10  Dim/PDH error counts
 *                                               
 *  Output        BER
 * 
 *  Return:       dimMgr_rc_t 
 *
****************************************************************************/
static uint8 ErrLogCnt = 0;
dimMgr_rc_t DIM_PDH_BER_calculate(uint8 DimCable, uint32 total_erroredFrm, DimCable_BitErrorRt *BER)   
{
     dimMgr_rc_t         DimCable_Rc = DIM_MGR_OK;
     
     uint8               LookupTblRow;
     dimType_t           dimType;

     // Get cable traffic type (DS1 or DS3).
     if ((( DimCable_Rc = dimMgrGetProvisionedDimType(DimCable, &dimType)) != DIM_MGR_OK))  
     {
            prCable("Cable=%d dimMgrGetProvisionedDimType gets DimCable_Rc=%d \n ", DimCable, DimCable_Rc);
            DIM_MGR_ERROR(errorMode, "%s DIM failed to get DS1/DS3 type! for DimCable %d, DimCable_Rc = %d \n", __FUNCTION__, DimCable, DimCable_Rc);
            return  DIM_MGR_INVALID_ARGS;
     }
     else
     {
            switch (dimType)   
            {
                 case DIM_84xDS1:

                    // Get traffic error rate from error count
                    prCable("Cable %d DIM Type = %d (DS1)\n", DimCable, dimType);    // 0=unknown, 1=DS1, 2=DS3
                   
                    if (total_erroredFrm != 0)
                    {
                        // Calculate BER from error count 
                        if (total_erroredFrm > DS1_BER_info[0].FrameErrors10Sec)
                        {
                            *BER = DS1_BER_info[0].BERThreshold;

                            if (ErrLogCnt++ < PTS_DIMCABLE_MAX_ERRORCOUNT) 
                                DIM_MGR_ERROR(errorMode, "%s DimSide FCS errorcnt %d exceeds max %d for DimCable %d \n", __FUNCTION__, total_erroredFrm, DS1_BER_info[0].FrameErrors10Sec, DimCable);
                            break;
                        } 
                        
                        // Find where the Dim_total_erroredFrm fits in the table, that row has the traffic BER 
                        for (LookupTblRow = 0; LookupTblRow < PTS_DIMCABLE_NUMTHRESH -1; LookupTblRow++)
                        {
                            if (total_erroredFrm == DS1_BER_info[LookupTblRow].FrameErrors10Sec)
                            {
                               *BER = DS1_BER_info[LookupTblRow].BERThreshold;     
                                break;
                            }
                            else if ((total_erroredFrm < DS1_BER_info[LookupTblRow].FrameErrors10Sec) && (total_erroredFrm >=  DS1_BER_info[LookupTblRow + 1].FrameErrors10Sec))
                            {
                                *BER = DS1_BER_info[LookupTblRow + 1].BERThreshold; 
                                break;
                            }
                        }
                    }
                    else
                    {
                        *BER = 0;
                        break;
                    }
                   
                    break;

                case DIM_24xDS3:
             
                   prCable("Cable %d DIM Type = %d (DS3)\n", DimCable, dimType);    // 0=unknown, 1=DS1, 2=DS3
                   
                   if (total_erroredFrm != 0)
                   {
                        // Calculate BER from error count
                        if (total_erroredFrm > DS3_BER_info[0].FrameErrors10Sec)
                        {
                            *BER = DS3_BER_info[0].BERThreshold;

                            if (ErrLogCnt++ < PTS_DIMCABLE_MAX_ERRORCOUNT) 
                                DIM_MGR_ERROR(errorMode, "%s DimSide FCS errorcnt %d exceeds max %d for DimCable %d \n", __FUNCTION__, total_erroredFrm, DS3_BER_info[0].FrameErrors10Sec, DimCable);
                            break;
                        }

                        // Find where the Dim_total_erroredFrm fits in the table, that row has the traffic BER 
                        for (LookupTblRow = 0; LookupTblRow < PTS_DIMCABLE_NUMTHRESH -1; LookupTblRow++)
                        {
                            if (total_erroredFrm == DS3_BER_info[LookupTblRow].FrameErrors10Sec)
                            {
                                *BER = DS3_BER_info[LookupTblRow].BERThreshold;
                                break;
                            }
                            else if ((total_erroredFrm < DS3_BER_info[LookupTblRow].FrameErrors10Sec) && (total_erroredFrm >=  DS3_BER_info[LookupTblRow + 1].FrameErrors10Sec))
                            {
                                *BER = DS3_BER_info[LookupTblRow + 1].BERThreshold; 
                                break;
                            }
                        }
                    }
                    else
                    {
                        *BER = 0;
                        break;
                    }
                    
                    break;

            default:
                   prCable("unknown traffic type for %d", dimType);    // 0=unknown, 1=DS1, 2=DS3
                   DIM_MGR_ERROR(errorMode, "%s DIM faced with unknown traffic type! for DimCable %d, Dim Type = %d \n", __FUNCTION__, DimCable, dimType);
                   return DIM_MGR_INVALID_ARGS;
            }
     }

     return DIM_MGR_OK;
}

/***************************************************************************
 *
 *  Name:         DIM_PDH_cable_BERevaluateAndReport
 *
 *  Description:  This function runs in each second and
 *                raise/clear SF/SD alarms from Flare engine.
 *                If it is after warm restart or cold restart the alarm state
 *                is not compared with the previous state.
 *
 *  Parameters:   DimCable as a port number
 *                BER      as a bit error rate            
 *
 *  Return:       void
 *
****************************************************************************/
static boolean FIRSTReportDone = FALSE;
void DIM_PDH_cable_BERevaluateAndReport(uint8  DimCable,DimCable_BitErrorRt BER)
{
    boolean  newSD = FALSE;
    boolean  newSF = FALSE;
    
    // Raise SF,  Clear SD  
    if (BER >= PTS_DIMCABLE_SFTHRESHOLD_R )   
    {
        newSD = FALSE;   // local
        newSF = TRUE;
                  
        // Check if this is the first pass after restart, if so update both SF/SD in Flare.
        if (!FIRSTReportDone) 
        {
            dgSetDIMCableFail(DimCable+1, newSF); 
            dgSetDIMCableDegrade(DimCable+1, newSD);
            FIRSTReportDone = TRUE;
        }
        else // report only changes to existing state.
        {
            if (DimCableDeg[DimCable].current_almstatus_SF != newSF) 
            {
              // raise SF alarm from Flare engine 
              // port id for this function is one-based  
              dgSetDIMCableFail(DimCable+1, newSF);   
            }
            if (DimCableDeg[DimCable].current_almstatus_SD != newSD)
            {
              // clear SD alarm from Flare engine 
              dgSetDIMCableDegrade(DimCable+1, newSD); 
            }
        }
        // update stored states with new ones.
        DimCableDeg[DimCable].current_almstatus_SD = newSD;
        DimCableDeg[DimCable].current_almstatus_SF = newSF;
    }

    // Check if it is between SF clear and SF raise thresholds (hysteresis effect), if so keep SF on if it was on, or raise SD if SF was off.
    else if ((BER >= PTS_DIMCABLE_SFTHRESHOLD_C) && (BER < PTS_DIMCABLE_SFTHRESHOLD_R ))  
    {
          newSF = FALSE;
          newSD = TRUE;
 
          // First pass?  raise SD clear SF
          if (!FIRSTReportDone)
          {
             dgSetDIMCableDegrade(DimCable+1, newSD);
             dgSetDIMCableFail   (DimCable+1, newSF);
             FIRSTReportDone = TRUE;
          }
          else
          {
              // check SF current state, if ON leave it ON, else raise SD.
              if (DimCableDeg[DimCable].current_almstatus_SF)         
              {
                  // no change to FLARE needed. Leave alarm ON, update local states to reflect this.
                  newSF = TRUE;
                  newSD = FALSE;
              }
              // If SF is OFF, then raise SD if not already raised.
              else if (DimCableDeg[DimCable].current_almstatus_SD != newSD)
              {      
                  // raise SD alarm from Flare engine
                  dgSetDIMCableDegrade(DimCable+1, newSD);
              }
          }
          // update stored states with new ones.
          DimCableDeg[DimCable].current_almstatus_SF = newSF;
          DimCableDeg[DimCable].current_almstatus_SD = newSD;
    }
    
    // clear SF, Raise SD
    else if ((BER >= PTS_DIMCABLE_SDTHRESHOLD_R) && (BER < PTS_DIMCABLE_SFTHRESHOLD_C ))
    { 
          newSD = TRUE;
          newSF = FALSE;
         
          if (!FIRSTReportDone) 
          {
               dgSetDIMCableDegrade(DimCable+1, newSD); 
               dgSetDIMCableFail(DimCable+1, newSF);
               FIRSTReportDone = TRUE;
          }
          else 
          {
              if (DimCableDeg[DimCable].current_almstatus_SD != newSD) 
              {       
                 // raise SD alarm from Flare engine 
                 dgSetDIMCableDegrade(DimCable+1, newSD); 
              }
              if (DimCableDeg[DimCable].current_almstatus_SF != newSF)  
              {
                 // clear SF alarm from Flare engine 
                 dgSetDIMCableFail(DimCable+1, newSF); 
              }
          }

          // update stored states with new ones.
          DimCableDeg[DimCable].current_almstatus_SD = newSD; 
          DimCableDeg[DimCable].current_almstatus_SF = newSF;
    }
  
    else if ((BER >= PTS_DIMCABLE_SDTHRESHOLD_C) && (BER < PTS_DIMCABLE_SDTHRESHOLD_R ))
    {    // Keep SD ON if it is already ON
 
         newSD = FALSE;
         newSF = FALSE;
 
         if (!FIRSTReportDone)
         {
             dgSetDIMCableFail(DimCable+1, newSF);
             dgSetDIMCableDegrade(DimCable+1, newSD);
             FIRSTReportDone = TRUE;
         }
         else
         {
            // if SD was ON, keep it ON.  If OFF keep it OFF and make sure SF is OFF too.
            if (DimCableDeg[DimCable].current_almstatus_SD)
            {
                // no change to FLARE needed. Leave alarm ON, update local states.
                 newSD = TRUE;
            }
            // else just make sure SF is clear, small chance we went from full SF to this level.
            else if (DimCableDeg[DimCable].current_almstatus_SF != newSF)
            {
                 // clear SF from Flare engine
                 dgSetDIMCableFail(DimCable+1, newSF);
            }
         }
         // update stored states.
         DimCableDeg[DimCable].current_almstatus_SF = newSF;
         DimCableDeg[DimCable].current_almstatus_SD = newSD;
    }
   
    // clear SF, clear SD
    else if (BER < PTS_DIMCABLE_SDTHRESHOLD_C) 
    {
         newSD = FALSE;// update stored states with new ones.
         newSF = FALSE;
         
         if (!FIRSTReportDone) 
         {
             dgSetDIMCableFail(DimCable+1, newSF); 
             dgSetDIMCableDegrade(DimCable+1, newSD); 
             FIRSTReportDone = TRUE;
         }
         else 
         {
             if (DimCableDeg[DimCable].current_almstatus_SF != newSF) 
             {                            
                // clear SF alarm from Flare engine 
                dgSetDIMCableFail(DimCable+1, newSF); 
             }
             if (DimCableDeg[DimCable].current_almstatus_SD != newSD) 
             {                            
                // clear SD alarm from Flare engine 
                dgSetDIMCableDegrade(DimCable+1, newSD); 
             }
         }

         // update stored states with new ones.
         DimCableDeg[DimCable].current_almstatus_SF = newSF;
         DimCableDeg[DimCable].current_almstatus_SD = newSD;
    }
  
    prCable("Cable %d current_almstatus_SF = %d, current_almstatus_SD = %d \n", DimCable, DimCableDeg[DimCable].current_almstatus_SF,DimCableDeg[DimCable].current_almstatus_SD);
     
    return;
}

/***************************************************************************
 *
 *  Name:        DimCable_CancelandResetcounts 
 *
 *  Description:  
 *
 *  Parameters:   DimCable,
 *                clearAlarms  TRUE= reset SF/SD
 *
 *  Return:       void    
 *
****************************************************************************/
void DimCable_CancelandResetcounts(uint8  DimCable, boolean clearAlarms)
{
     // reset error count
     for (DimCableDeg[DimCable].Dim_window_Index=0; DimCableDeg[DimCable].Dim_window_Index <PTS_DIMCABLE_WINDOWSIZE ; DimCableDeg[DimCable].Dim_window_Index++) 
     {
         DimCableDeg[DimCable].Dim_error_cnt[DimCableDeg[DimCable].Dim_window_Index] = 0;
     }
     
     for (DimCableDeg[DimCable].PDH_window_Index=0; DimCableDeg[DimCable].PDH_window_Index <PTS_DIMCABLE_WINDOWSIZE ; DimCableDeg[DimCable].PDH_window_Index++) 
     {
         DimCableDeg[DimCable].PDH_error_cnt[DimCableDeg[DimCable].PDH_window_Index] = 0;
     }
     
     // reset window index,total errored Frame, and readytodetect flag
     // DIM side
     DimCableDeg[DimCable].Dim_window_Index = 0; 
     DimCableDeg[DimCable].Dim_total_erroredFrm = 0;
     DimCableDeg[DimCable].Dim_readytodetect = FALSE;
     
     // PDH side
     DimCableDeg[DimCable].PDH_window_Index = 0; 
     DimCableDeg[DimCable].PDH_total_erroredFrm = 0;
     DimCableDeg[DimCable].PDH_readytodetect = FALSE;
     
     // reset SF/SD
     if (clearAlarms) 
     {
         // Clear SF
         DimCableDeg[DimCable].current_almstatus_SF = FALSE; 
         dgSetDIMCableFail(DimCable+1, DimCableDeg[DimCable].current_almstatus_SF);
          
         // clear SD
         DimCableDeg[DimCable].current_almstatus_SD = FALSE; 
         dgSetDIMCableDegrade(DimCable+1, DimCableDeg[DimCable].current_almstatus_SD);
     }

     ErrLogCnt = 0;

     return;
}

/***************************************************************************
 *
 *  Name:         translate_cablelink_2arrivedev
 *
 *  Description:  This function returns the arrive device# for a given cable#.
 *
 *  Parameters:   Dimlink:   number of Dim Link
 * 
 *  output        arrdev:    arrive device number proportional to Dim link number
 *                
 *  Return:       dimMgr_rc_t
 *
****************************************************************************/
dimMgr_rc_t translate_cablelink_2arrivedev(ePtsPdh2xDimNumLink_t Dimlink, ArrAdpt_dev_id_t *arrdev)
{
    if (arrdev==NULL) 
    {
       return DIM_MGR_INVALID_ARGS;
    }
    else
    {
        // currently there is a 1 to 1 relationship between cable and arrive device.
        switch (Dimlink)
        {
            case PTS_PDH_2xDIM_LINK0:

                *arrdev = arrive_dev_LINK0;
                break;

            case PTS_PDH_2xDIM_LINK1:
                
                *arrdev = arrive_dev_LINK1;
                break;
                
            default:
                prCable("xlatecablelink gets Invalid cable, DimLink=%d\n", Dimlink);
                return DIM_MGR_INVALID_ARGS;
        }
    }
    return DIM_MGR_OK;
}

/***************************************************************************
 *
 *  Name:    PDHcard_detection     
 *
 *  Description:  This function checks the card type.
 *                It's executed only on the PDH card.
 *
 *  Parameters:   void               
 *
 *  Return:       boolean     TRUE = PDH card     
 *
****************************************************************************/
boolean PDHcard_detection(void)
{
   uint32 CardID=0, Version=0;
 
   // Need to know what card this is, 
   if (!(VKRetreiveCardID( &CardID, &Version)) )
   {
       DIM_MGR_DEBUG (errorMode, "%s VkCardID failed to read CardID \n", __FUNCTION__);
   }
    
   if (CardID == HAL_PTS_PDH_2xDIM)
       return  TRUE;
   else 
       return FALSE;
}
/***************************************************************************
 *
 *  Name:         Ehsi_DIM_Cabledegrade_diag
 *
 *  Description:  This function runs each second to calculate/evaluate BER and
 *                raise/clear SF/SD alarms report to Flare engine. 
 *
 *  Parameters:   void               
 *
 *  Return:       dimMgr_rc_t
 *
****************************************************************************/
dimMgr_rc_t  Ehsi_DIM_Cabledegrade_diag (void)
{
  ArrAdpt_rc_t               PDHCable_Rc; 
  uint8                      DimCable;
  stDimMacCounters_t         DimCable_mac_Counters;
  uint32                     DimCable_PDH_error_count;
  uint8                      winInd; 
  dimMgr_rc_t                DimCable_Rc = DIM_MGR_OK;
  eDimLinkState_t            DimCable_currState = 0;
  DimCable_BitErrorRt        MaxBERonThisCable = 0;
  DimCable_BitErrorRt        BitErrorRt = 0;
  ArrAdpt_dev_id_t           arrdev = 0;

  // In each second two ports of DIM PDH card is checked if it is connected or not.
  for (DimCable = 0; DimCable < PTS_DIMCABLE_MAX; DimCable ++) 
  {
        // Check if cable is connected 
        if (((DimCable_Rc = dimMgrGetCurrLinkState(DimCable,&DimCable_currState))!= DIM_MGR_OK)) 
        {
            DIM_MGR_ERROR(errorMode, "%s ERROR return from getCurrLinkState for Cable %d rc=%d \n", __FUNCTION__, DimCable, DimCable_Rc);
            DimCable_CancelandResetcounts(DimCable, FALSE);
        }
        else if (DimCable_currState == DIMConnected) 
        {
             if ((( DimCable_Rc = translate_cablelink_2arrivedev(DimCable, &arrdev)) != DIM_MGR_OK))  
             {
                 DIM_MGR_ERROR(errorMode, "%s : Invalid Dim link index or null arrdev : cable=%d, DimCable_Rc = %d\n", __FUNCTION__, DimCable, DimCable_Rc); 
                 DimCable_CancelandResetcounts(DimCable, FALSE);
                 break;
             }

             // circular sliding window -- But discard first n items after restart (as they are unreliable)
             // 
             // DIM side
            if (DimCableDeg[DimCable].FIRSTimeCount) 
            {
                 // Flush bad values come from DIM/PDH error count
                 ArrAdpt_getAndclearFCSerror (arrdev, PTS_DIMCABLE_AXIPort, &DimCable_PDH_error_count, TRUE);
                 dimMgrGetDimMacCounters(DimCable, &DimCable_mac_Counters);

                 DimCableDeg[DimCable].Dim_discarddatacount = DimCableDeg[DimCable].Dim_discarddatacount + 1; 
                 if (DimCableDeg[DimCable].Dim_discarddatacount == PTS_DIMCABLE_DISCARD_NUM) 
                 {
                     DimCableDeg[DimCable].Dim_discarddatacount = 0;
                     DimCableDeg[DimCable].FIRSTimeCount = FALSE;
                 }
            }
            else // (DimCableDeg[DimCable].FIRSTimeCount = FALSE) 
            {
                 // DIM side
                 if (DimCableDeg[DimCable].Dim_window_Index == PTS_DIMCABLE_WINDOWSIZE) 
                 {
                     DimCableDeg[DimCable].Dim_window_Index = 0; 
                     DimCableDeg[DimCable].Dim_readytodetect = TRUE;
                 }

                 // PDH side
                 if (DimCableDeg[DimCable].PDH_window_Index == PTS_DIMCABLE_WINDOWSIZE)  
                 {
                     DimCableDeg[DimCable].PDH_window_Index = 0; 
                     DimCableDeg[DimCable].PDH_readytodetect = TRUE;
                 }

                 prCable("\n\nCable %d is connected! Dim_readytodetect=%d, PDH_readytodetect=%d\n", DimCable, DimCableDeg[DimCable].Dim_readytodetect, DimCableDeg[DimCable].PDH_readytodetect);

                 // -------------------------DIM side-------------------------
                 // For each installed cable on DIM side, error count is fetched.
                 if ((( DimCable_Rc = dimMgrGetDimMacCounters(DimCable, &DimCable_mac_Counters)) != DIM_MGR_OK))  
                 {
                       prCable("DimCable=%d dimMgrGetDimMacCounters gets DimCable_Rc=%d \n", DimCable, DimCable_Rc);
                       DIM_MGR_ERROR(errorMode, "%s DimMgr failed to count DIM FCS error for link %d, DimCable_Rc = %d \n", __FUNCTION__, DimCable, DimCable_Rc);
                       DimCable_CancelandResetcounts(DimCable, TRUE);
                 }
                 else
                 {
                    // Store  error count in the table on DIM side.
                    DimCableDeg[DimCable].Dim_error_cnt[DimCableDeg[DimCable].Dim_window_Index] = DimCable_mac_Counters.rxFcsErrFrames; 
      
                    // if the sample count equals or greater than window size,
                    // BER calculation and alarm report are executed.
                    if (DimCableDeg[DimCable].Dim_readytodetect == TRUE) 
                    {
                       //  DIM side
                       // 
                       prCable("--------------DIM side------------\n");
                       prCable("cable %d DimCable_mac_Counters.rxFcsErrFrames = %d \n", DimCable, DimCable_mac_Counters.rxFcsErrFrames); 
                       
                       //  Total errored frame calculation
                       DimCableDeg[DimCable].Dim_total_erroredFrm = 0; 
                       for (winInd=0; winInd<PTS_DIMCABLE_WINDOWSIZE; winInd++) 
                       {
                           DimCableDeg[DimCable].Dim_total_erroredFrm = DimCableDeg[DimCable].Dim_total_erroredFrm + DimCableDeg[DimCable].Dim_error_cnt[winInd];
                       }

                       prCable("Cable %d DIM Total erroredFrm = %d \n", DimCable, DimCableDeg[DimCable].Dim_total_erroredFrm); 

                       // DIM BER calculation
                       if (((DimCable_Rc = DIM_PDH_BER_calculate(DimCable, DimCableDeg[DimCable].Dim_total_erroredFrm, &BitErrorRt)) != DIM_MGR_OK))
                       {
                          prCable("DimCable=%d DIM_BER_calculate gets DimCable_Rc=%d \n ", DimCable,DimCable_Rc);
                          DIM_MGR_ERROR(errorMode, "%s DimMgr failed to calculate DIM BER for link %d, DimCable_Rc = %d \n", __FUNCTION__, DimCable, DimCable_Rc);
                          DimCable_CancelandResetcounts(DimCable, TRUE);
                       }
                       else
                       {
                         // Store BER
                         DimCableDeg[DimCable].Dim_BER = BitErrorRt;
                         prCable("DIM BER =%3.2e \n",DimCableDeg[DimCable].Dim_BER);
                       }
                    }
                    else 
                    { 
                        DimCableDeg[DimCable].Dim_BER = 0;
                    }

                    prCable("Cable %d Dim_window_Index = %d \n",DimCable, DimCableDeg[DimCable].Dim_window_Index);
                    DimCableDeg[DimCable].Dim_window_Index =  DimCableDeg[DimCable].Dim_window_Index + 1;
                 }

                 // PDH side 
                 if (((PDHCable_Rc = ArrAdpt_getAndclearFCSerror (arrdev, PTS_DIMCABLE_AXIPort, &DimCable_PDH_error_count, TRUE))!= ARRADPT_RC_SUCCESS)) 
                 {
                      prCable("\nCable %d has error to count ArrAdpt FCS error. Dim manager error mode = %d \n\n",DimCable, PDHCable_Rc);
                      DIM_MGR_ERROR(errorMode, "%s Error return from Arradpt_getFCSError, arrdev=%d, Rc=%d\n", __FUNCTION__, arrdev, DimCable_Rc);
                      DimCable_CancelandResetcounts(DimCable, TRUE);
                 }
                 else 
                 {
                     // Store PDH error count in each sliding window
                     DimCableDeg[DimCable].PDH_error_cnt[DimCableDeg[DimCable].PDH_window_Index] = DimCable_PDH_error_count;  
                     
                     if (DimCableDeg[DimCable].PDH_readytodetect == TRUE)
                     {
                         prCable("--------------PDH side------------\n");
                         prCable("cable %d PDH FcsErrFrames = %d \n",DimCable, DimCableDeg[DimCable].PDH_error_cnt[DimCableDeg[DimCable].PDH_window_Index]);  
                         
                         //  Total errored frame calculation
                         DimCableDeg[DimCable].PDH_total_erroredFrm = 0; 
                         for (winInd=0; winInd<PTS_DIMCABLE_WINDOWSIZE; winInd++) 
                         {
                              DimCableDeg[DimCable].PDH_total_erroredFrm = DimCableDeg[DimCable].PDH_total_erroredFrm + DimCableDeg[DimCable].PDH_error_cnt[winInd];
                         }

                         prCable("Cable %d PDH Total erroredFrm = %d \n", DimCable, DimCableDeg[DimCable].PDH_total_erroredFrm); 

                         //  PDH BER calculation
                         if (((DimCable_Rc = DIM_PDH_BER_calculate(DimCable, DimCableDeg[DimCable].PDH_total_erroredFrm, &BitErrorRt)) != DIM_MGR_OK))
                         {
                            prCable("DimCable=%d PDH_BER_calculate gets PDH_Cable=%d \n ", DimCable, DimCable_Rc);
                            DIM_MGR_ERROR(errorMode, "%s DimMgr failed to calculate PDH BER for link %d, DimCable_Rc = %d \n", __FUNCTION__, DimCable, DimCable_Rc);
                            DimCable_CancelandResetcounts(DimCable, TRUE);
                         }
                         else
                         {
                            DimCableDeg[DimCable].PDH_BER = BitErrorRt;
                            prCable("PDH BER = %3.2e \n",DimCableDeg[DimCable].PDH_BER);
                         }
                     }
                     else
                     {
                         DimCableDeg[DimCable].PDH_BER = 0;
                     }

                     MaxBERonThisCable = max (DimCableDeg[DimCable].Dim_BER, DimCableDeg[DimCable].PDH_BER);
                     DIM_PDH_cable_BERevaluateAndReport(DimCable, MaxBERonThisCable);

                     prCable("Cable %d max BER on this cable = %3.2e \n", DimCable, MaxBERonThisCable);

                     prCable("Cable %d PDH_window_Index = %d\n",DimCable, DimCableDeg[DimCable].PDH_window_Index);
                     prCable("----------------------------------\n");
                     DimCableDeg[DimCable].PDH_window_Index = DimCableDeg[DimCable].PDH_window_Index + 1;
                    
                 } // end of Read error count from PDH (arrive) side
            }      // end of getting data from DIM and PDH side  
        }
        else
        {
            DimCable_CancelandResetcounts(DimCable, TRUE);
        }
  }
            
  return DIM_MGR_OK; ; 
}

/***************************************************************************
 *
 *  Name:         Ehsi_DIM_Cabledegrade_init
 *
 *  Description:  This function runs at initialization time
 *                (after cold/warm restart) to reset the variables.
 *                It is called from ptsmropdh_app_init.c file.
 *
 *  Parameters:   void               
 *
 *  Return:       void
 *
****************************************************************************/
void Ehsi_DIM_Cabledegrade_init(void)
{
  uint8            DimCable;

  // Check the card type
  if (!PDHcard_detection()) 
  {
      DIM_MGR_DEBUG(errorMode, "%s It is not a PDH card. This function is executed only on the PDH card. \n", __FUNCTION__);
      PDHcardDiag = FALSE;
      return;
  }
  else
  {
      PDHcardDiag = TRUE;
  }
  
  // Check warm restart 
  if (QueryResetType() == SOFT_RESET)   // warm restart
  {
      prCable("\n---------------------------\n");
      prCable("\n----Warm restart event!----\n");
      prCable("\n---------------------------\n");
      
      clearAlarms = FALSE;
  }
  else
  {
      clearAlarms = TRUE;
  }
  
  for (DimCable = 0; DimCable < PTS_DIMCABLE_MAX; DimCable ++) 
  {   
      DimCable_CancelandResetcounts(DimCable, clearAlarms);
      DimCableDeg[DimCable].FIRSTimeCount = TRUE;
  }

  return;
}
/***************************************************************************/

