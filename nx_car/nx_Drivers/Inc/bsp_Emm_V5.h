#ifndef __BSP_EMM_V5_H
#define __BSP_EMM_V5_H

#include "Emm_V5.h"

/* Receive success */
#define SUCCESSRECEIVE 0x100
/* Receive failed */
#define FAILRECEIVE    0x000

typedef struct
{
	/* Encoder response buffer */
	__IO uint8_t M1_EncodedBuff[7];
	__IO uint8_t M2_EncodedBuff[7];
	__IO uint8_t M3_EncodedBuff[7];
	__IO uint8_t M4_EncodedBuff[7];
	__IO uint8_t M5_EncodedBuff[7];
	__IO uint8_t M6_EncodedBuff[7];

	__IO uint16_t M1_EncodedStatus;
	__IO uint16_t M2_EncodedStatus;
	__IO uint16_t M3_EncodedStatus;
	__IO uint16_t M4_EncodedStatus;
	__IO uint16_t M5_EncodedStatus;
	__IO uint16_t M6_EncodedStatus;

	/* Velocity response buffer (0x35 frame, 7 bytes) */
	__IO uint8_t M1_VelBuff[7];
	__IO uint8_t M2_VelBuff[7];
	__IO uint8_t M3_VelBuff[7];
	__IO uint8_t M4_VelBuff[7];

	__IO uint16_t M1_VelStatus;
	__IO uint16_t M2_VelStatus;
	__IO uint16_t M3_VelStatus;
	__IO uint16_t M4_VelStatus;

	__IO uint16_t M5_Zero_Reach_Status;
        __IO uint16_t M6_Zero_Reach_Status;

        /* Reach status
           Frame: addr + 0xFD + 0x9F + 0x6B (DLC=3)
           Note: the "command received" ack is the SAME header
           (addr + 0xFD + 0x02 + 0x6B), so the 2nd data byte decides:
           0x9F = position reached, others (0x02/0xE2/0xEE) = not reached.
           M1~M4 are added by this project (vendor only implements M5/M6). */
        __IO uint16_t M1_Reach_Status;
        __IO uint16_t M2_Reach_Status;
        __IO uint16_t M3_Reach_Status;
        __IO uint16_t M4_Reach_Status;
        __IO uint16_t M5_Reach_Status;
        __IO uint16_t M6_Reach_Status;

        __IO uint8_t M1_ReachBuff[3];
        __IO uint8_t M2_ReachBuff[3];
        __IO uint8_t M3_ReachBuff[3];
        __IO uint8_t M4_ReachBuff[3];
        __IO uint8_t M5_ReachBuff[3];
        __IO uint8_t M6_ReachBuff[3];

}Emm_V5_Pos_Status_Struct;

extern Emm_V5_Pos_Status_Struct  Emm_V5_Pos_Status;

void Emm_V5_Pos_Status_Read_Init(Emm_V5_Pos_Status_Struct * Emm_V5_Pos_Status);
void Emm_V5_Pos_Status_ReadCalback(Emm_V5_Pos_Status_Struct * Emm_V5_Pos_Status, uint8_t *pdata);

#endif
