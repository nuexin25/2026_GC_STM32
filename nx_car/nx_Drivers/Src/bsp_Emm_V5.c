#include "bsp_Emm_V5.h"
#include <string.h>


Emm_V5_Pos_Status_Struct  Emm_V5_Pos_Status;

CAN_RxHeaderTypeDef RxHeader;
uint8_t CANRxBuff[8];

void Emm_V5_Pos_Status_Read_Init(Emm_V5_Pos_Status_Struct * Emm_V5_Pos_Status)
{
	Emm_V5_Pos_Status->M1_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M2_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M3_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M4_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M5_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M6_EncodedStatus = FAILRECEIVE;
	Emm_V5_Pos_Status->M1_VelStatus = FAILRECEIVE;
    Emm_V5_Pos_Status->M2_VelStatus = FAILRECEIVE;
    Emm_V5_Pos_Status->M3_VelStatus = FAILRECEIVE;
    Emm_V5_Pos_Status->M4_VelStatus = FAILRECEIVE;

    /* Position-reached flags: cleared at init so the first segment of an
       action cannot mistake a stale flag for its own arrival. */
    Emm_V5_Pos_Status->M1_Reach_Status = FAILRECEIVE;
    Emm_V5_Pos_Status->M2_Reach_Status = FAILRECEIVE;
    Emm_V5_Pos_Status->M3_Reach_Status = FAILRECEIVE;
    Emm_V5_Pos_Status->M4_Reach_Status = FAILRECEIVE;
    Emm_V5_Pos_Status->M5_Reach_Status = FAILRECEIVE;
    Emm_V5_Pos_Status->M6_Reach_Status = FAILRECEIVE;
}

/**
  * @brief    Read motor status (called from CAN RX callback)
  * @param    Emm_V5_Pos_Status: status struct
  * @param    pdata: received data
  */
void Emm_V5_Pos_Status_ReadCalback(Emm_V5_Pos_Status_Struct * Emm_V5_Pos_Status,  uint8_t *pdata)
{
  /* Read position: 7 bytes total */
  if ( ( RxHeader.DLC == 7 ) && (pdata[0] == 0x36) )
  {
	switch(RxHeader.ExtId)
	{
		case(0x100):
			memcpy((void *)&Emm_V5_Pos_Status->M1_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M1_EncodedStatus =  SUCCESSRECEIVE;
			break;
		case(0x200):
			memcpy((void *)&Emm_V5_Pos_Status->M2_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M2_EncodedStatus =  SUCCESSRECEIVE;
			break;
		case(0x300):
			memcpy((void *)&Emm_V5_Pos_Status->M3_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M3_EncodedStatus =  SUCCESSRECEIVE;
			break;
		case(0x400):
			memcpy((void *)&Emm_V5_Pos_Status->M4_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M4_EncodedStatus =  SUCCESSRECEIVE;
			break;
		case(0x500):
			memcpy((void *)&Emm_V5_Pos_Status->M5_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M5_EncodedStatus =  SUCCESSRECEIVE;
			break;
		case(0x600):
			memcpy((void *)&Emm_V5_Pos_Status->M6_EncodedBuff,pdata,7);
			Emm_V5_Pos_Status->M6_EncodedStatus =  SUCCESSRECEIVE;
			break;
	}
  }
  /* Read velocity: 5 bytes total (0x35 frame, sign + 16-bit RPM) */
  else if ( ( RxHeader.DLC == 5 ) && (pdata[0] == 0x35) )
  {
	switch(RxHeader.ExtId)
	{
		case(0x100):
			memcpy((void *)&Emm_V5_Pos_Status->M1_VelBuff,pdata,5);
			Emm_V5_Pos_Status->M1_VelStatus =  SUCCESSRECEIVE;
			break;
		case(0x200):
			memcpy((void *)&Emm_V5_Pos_Status->M2_VelBuff,pdata,5);
			Emm_V5_Pos_Status->M2_VelStatus =  SUCCESSRECEIVE;
			break;
		case(0x300):
			memcpy((void *)&Emm_V5_Pos_Status->M3_VelBuff,pdata,5);
			Emm_V5_Pos_Status->M3_VelStatus =  SUCCESSRECEIVE;
			break;
		case(0x400):
			memcpy((void *)&Emm_V5_Pos_Status->M4_VelBuff,pdata,5);
			Emm_V5_Pos_Status->M4_VelStatus =  SUCCESSRECEIVE;
			break;
	}
  }
  /* Read reach status flag: 3 bytes total.
   * ===== ROOT-CAUSE FIX 2026-10-11 (arm "motor 5 stalls for seconds") =====
   * pdata[1] is the driver status byte; bit1 (0x02) = "position reached".
   * The old test was  ((pdata[1] & 0x02) == 0x01)  which can NEVER be true,
   * because the AND result is either 0x00 or 0x02 -- never 0x01.
   * Result: M5/M6_Reach_Status stayed FAILRECEIVE forever ->
   * Arm_IsReachDone() was always 0 -> the arm action layer lost the driver's
   * authoritative "reached" judgement and had to fall back to the software
   * position window / closed-loop budget / parallel grace timer, i.e. every
   * step waited seconds before the next joint was allowed to move.
   * Correct test = "bit1 set" (any non-zero of that mask). */
  else if ( ( RxHeader.DLC == 3) && (pdata[0] == 0x3A) )
  {
	switch(RxHeader.ExtId)
	{
		case(0x500):
				memcpy((void *)&Emm_V5_Pos_Status->M5_ReachBuff,pdata,3);
				if( (pdata[1] & 0x02) != 0x00){Emm_V5_Pos_Status->M5_Reach_Status = SUCCESSRECEIVE;}
				else{Emm_V5_Pos_Status->M5_Reach_Status = FAILRECEIVE;}

				break;
		case(0x600):
				memcpy((void *)&Emm_V5_Pos_Status->M6_ReachBuff,pdata,3);
				if( (pdata[1] & 0x02) != 0x00)
				{
					Emm_V5_Pos_Status->M6_Reach_Status = SUCCESSRECEIVE;
				}
				else
				{
					Emm_V5_Pos_Status->M6_Reach_Status = FAILRECEIVE;
				}
				break;
	}
  }
  /* Read zero reach status flag: 3 bytes total */
  else if ( ( RxHeader.DLC == 3) && (pdata[0] == 0x3B) )
  {
	switch(RxHeader.ExtId)
	{
		case(0x500):
				if( (pdata[1] & 0x04) == 0X00 )
				{
					Emm_V5_Pos_Status->M5_Zero_Reach_Status = SUCCESSRECEIVE;
				}
				else
				{
					Emm_V5_Pos_Status->M5_Zero_Reach_Status = FAILRECEIVE;
				}

				break;
		case(0x600):
				if( (pdata[1] & 0x04) == 0X00)
				{
					Emm_V5_Pos_Status->M6_Zero_Reach_Status = SUCCESSRECEIVE;
				}
				else
				{
					Emm_V5_Pos_Status->M6_Zero_Reach_Status = FAILRECEIVE;
				}
				break;
	}
  }
  /* Position reach flag: 3 bytes total
     (addr + 0xFD + 0x9F + 0x6B).
     NOTE: the "command received" ack uses the SAME header
     (addr + 0xFD + 0x02 + 0x6B), so only data byte 0x9F counts as reached;
     0x02 (received) / 0xE2 (condition not met) / 0xEE (bad command) => not reached.
     M1~M4 cases are added by this project (vendor only implements M5/M6). */
  else if ( ( RxHeader.DLC == 3) && (pdata[0] == 0xFD) )
  {
   switch(RxHeader.ExtId)
   {
           case(0x100):
                           memcpy((void *)&Emm_V5_Pos_Status->M1_ReachBuff,pdata,3);
                           if( pdata[1] == 0x9F )
                           {
                                   Emm_V5_Pos_Status->M1_Reach_Status = SUCCESSRECEIVE;
                           }
                           else
                           {
                                   Emm_V5_Pos_Status->M1_Reach_Status = FAILRECEIVE;
                           }
                           break;
           case(0x200):
                           memcpy((void *)&Emm_V5_Pos_Status->M2_ReachBuff,pdata,3);
                           if( pdata[1] == 0x9F )
                           {
                                   Emm_V5_Pos_Status->M2_Reach_Status = SUCCESSRECEIVE;
                           }
                           else
                           {
                                   Emm_V5_Pos_Status->M2_Reach_Status = FAILRECEIVE;
                           }
                           break;
           case(0x300):
                           memcpy((void *)&Emm_V5_Pos_Status->M3_ReachBuff,pdata,3);
                           if( pdata[1] == 0x9F )
                           {
                                   Emm_V5_Pos_Status->M3_Reach_Status = SUCCESSRECEIVE;
                           }
                           else
                           {
                                   Emm_V5_Pos_Status->M3_Reach_Status = FAILRECEIVE;
                           }
                           break;
           case(0x400):
                           memcpy((void *)&Emm_V5_Pos_Status->M4_ReachBuff,pdata,3);
                           if( pdata[1] == 0x9F )
                           {
                                   Emm_V5_Pos_Status->M4_Reach_Status = SUCCESSRECEIVE;
                           }
                           else
                           {
                                   Emm_V5_Pos_Status->M4_Reach_Status = FAILRECEIVE;
                           }
                           break;
           case(0x500):
				memcpy((void *)&Emm_V5_Pos_Status->M5_ReachBuff,pdata,3);
				if( pdata[1] == 0x9F )
				{
					Emm_V5_Pos_Status->M5_Reach_Status = SUCCESSRECEIVE;
				}
				else
				{
					Emm_V5_Pos_Status->M5_Reach_Status = FAILRECEIVE;
				}

				break;
		case(0x600):
				memcpy((void *)&Emm_V5_Pos_Status->M6_ReachBuff,pdata,3);
				if( pdata[1] == 0x9F )
				{
					Emm_V5_Pos_Status->M6_Reach_Status = SUCCESSRECEIVE;
				}
				else
				{
					Emm_V5_Pos_Status->M6_Reach_Status = FAILRECEIVE;
				}
				break;
	}
  }
}

/**
  * @brief    CAN RX FIFO0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, CANRxBuff) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  Emm_V5_Pos_Status_ReadCalback(&Emm_V5_Pos_Status,CANRxBuff);
}
