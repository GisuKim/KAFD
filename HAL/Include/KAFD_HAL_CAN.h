/**
 * File Name :
 * File Description : Low Level CAN Driver(Setup, Send, Read)
 * Copyright Sentence
 * Author : dsbaek
 * Dept :
 * Created Date : 2014.02.19
 * Version : v0.9
 */

#ifndef SCU_HAL_CAN_H_
#define SCU_HAL_CAN_H_

#include "f28x_project.h"
#include "f2838x_Device.h"     // DSP2833x Headerfile Include File
#include "driverlib.h"
#include "device.h"

//#define CAN_DIR_TX	0
//#define CAN_DIR_RX	1
//#define CAN_RX_INDEX_TIME 0
//#define CAN_RX_INDEX_BROADCAST 1
//#define CAN_RX_INDEX_CONTROL 2
//#define CAN_RX_INDEX_SENSOR 3

//CAN Msg Box Assigne
#define CAN_TX_COMP_CMD 	        			0
#define CAN_RX_COMP_STATUS					    1

#define MAX_CANQ_SIZE                           8

#define CANA_RX_MSG_OBJ_ID_1     1
#define CANB_RX_MSG_OBJ_ID_1     1
#define CANA_RX_MSG_OBJ_Length   6
#define CANB_RX_MSG_OBJ_Length   8


/*-----------------------------------------------------------------------------
Define the structure of the Packet Driver Object
-----------------------------------------------------------------------------*/
typedef union
{
	Uint64 m_lData64Bit;
	Uint8  m_abyData8Bit[8];
}unCanDataField;


typedef struct
{
	Uint8			m_byMboxNum;
	Uint32  		m_lMsgID;
	Uint8   		m_byUseINT;
	Uint8   		m_byDLC;
	Uint8   		m_byDirection;
  	unCanDataField	m_unDataField;
}stCanMsgObject;


/*------------------------------------------------------------------------------
Default Initializers for the Packet Object
------------------------------------------------------------------------------*/


#define CAN_SWOBJ {0x00, \ 0x00, \ 0x00, \ 0x00, \ 0x00, \ {0x00} \ }

#define CAN_SWOBJ_DEFAULTS CAN_SWOBJ

extern	Uint16 g_auStdID[];

/*------------------------------------------------------------------------------
 Prototypes for the functions in SCU_HAL_CAN.c
------------------------------------------------------------------------------*/
Uint32 Set_Bit32(Uint32 i_lData, Uint8 i_byBit, Uint32 i_lSet);

void InitCan(void);
void InitCanMessageBox(void);
/*
void Set_CanaMsgBox(stCanMsgObject *i_pstCAN_MSG_Handle);
void Send_CanaMsg(stCanMsgObject *i_pstCAN_MSG_Handle);
*/
#endif /* SCU_HAL_CAN_H_ */
