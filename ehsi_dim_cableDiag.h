/*****************************************************************************************
 * File        : ehsi_dim_cableDiag.h
 *
 * Created Date: March 05, 2018
 *
 * Description : This header file is used in ehsi_dim_cableDiag.c to calculate BER
 * for DIM-PDH cable. 
 * 
 *****************************************************************************************/
#define PTS_DIMCABLE_WINDOWSIZE           10
#define PTS_DIMCABLE_AXIPort               1  // AXI4 interface on Arrive, is ethernet Port2
#define PTS_DIMCABLE_NUMCABLES             2

typedef float DimCable_BitErrorRt; 

typedef struct {
  uint32                 Dim_error_cnt[PTS_DIMCABLE_WINDOWSIZE];  
  uint32                 PDH_error_cnt[PTS_DIMCABLE_WINDOWSIZE];  
  DimCable_BitErrorRt    Dim_BER;
  DimCable_BitErrorRt    PDH_BER;                        
  uint32                 Dim_total_erroredFrm; 
  uint32                 PDH_total_erroredFrm;             
  boolean                Dim_readytodetect;  //ready to compute BER and report alarms on Dim side
  boolean                PDH_readytodetect;  //ready to compute BER and report alarms on PDH side
  uint8                  Dim_window_Index; 
  uint8                  PDH_window_Index;
  uint8                  Dim_discarddatacount;
  boolean                FIRSTimeCount;     // FIRSTimeCount uses to discard the first 5 data to prevent getting unwanted ones.
  boolean                current_almstatus_SF; 
  boolean                current_almstatus_SD;
} DimCabledegrade_t;


/* Cable to ArriveDevice */ 
typedef enum 
{
   arrive_dev_LINK0 = 0, 
   arrive_dev_LINK1
} Ehsi_DIM_CableArrDev_t; 

boolean PDHcardDiag;
/**************************************************************************************
 *
 *  Name:         Ehsi_DIM_Cabledegrade_diag
 *
 *  Description:  This function runs each second to calculate/evaluate BER and
 *                raise/clear SF/SD alarms report to Flare engine. 
 *
 *  Parameters:   void
 *                
 *
 *  Return:       dimMgr_rc_t
 *
**************************************************************************************/
dimMgr_rc_t  Ehsi_DIM_Cabledegrade_diag (void);








