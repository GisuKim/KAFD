/***************************************************************************//**
 * @file       SCU_HAL_CAN.c
 * @addtogroup SCU_HAL_CAN
 * @{
 ******************************************************************************/

#include <HAL/Include/KAFD_HAL_CAN.h>

//#include "f2838x_Device.h"
//#include "f2838x_Examples.h"

#define MBOX_SIZE	2

Uint16 g_auStdID[MBOX_SIZE] = { 0x238, 0x248};
Uint16 g_uCNT_CANB_TX = 0;


Uint32 Set_Bit32(Uint32 i_lData, Uint8 i_byBit, Uint32 i_lSet)
{
	Uint32 lBit32 = 0;
	Uint32 lOn = 0x01;

	lBit32 = i_lData & (0xFFFFFFFF ^ (lOn<<i_byBit) );
	lBit32 = lBit32 | (i_lSet<<i_byBit);

	return lBit32;
}

void InitCan(void)
{
    // CAN 컨트롤러 초기화 (CANA & CANB)
    CAN_initModule(CANA_BASE);
    CAN_initModule(CANB_BASE);

    // CANA, CANB 모듈의 비트 전송률을 1000kHz로 설정
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);


    // CANB 모듈의 인터럽트 활성화
    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);

}
void InitCanMessageBox(void)
{

    // CAN 메시지 전송에 사용할 송신 메시지 오브젝트 초기화
    // CAN Module:                  A
    // Message Object ID Number:    RX_MSG_OBJ_ID
    // Message Identifier:          0x521
    // Message Frame:               Standard
    // Message Type:                Receive
    // Message ID Mask:             0x00f
    // Message Object Flags:        Receive Interrupt
    // Message Data Length:         6 Bytes (Note that DLC field is a "don't care" for a Receive mailbox
    CAN_setupMessageObject( CANA_BASE, CANA_RX_MSG_OBJ_ID_1, 0x521,
                            CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0x00f,
                            CAN_MSG_OBJ_RX_INT_ENABLE, CANA_RX_MSG_OBJ_Length );


//CAN_MSG_OBJ_USE_ID_FILTER
    // CAN 메시지 수신에 사용할 수신 메시지 오브젝트 초기화
    // CAN Module:                  B
    // Message Object ID Number:    RX_MSG_OBJ_ID
    // Message Identifier:          0x3c2
    // Message Frame:               Extended
    // Message Type:                Receive
    // Message ID Mask:             0x0
    // Message Object Flags:        Receive Interrupt
    // Message Data Length:         4 Bytes

    CAN_setupMessageObject( CANB_BASE, CANB_RX_MSG_OBJ_ID_1, 0x3c2,
                            CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                            CAN_MSG_OBJ_RX_INT_ENABLE, CANB_RX_MSG_OBJ_Length );



}
/*-----------------------------------------------------------------------
AUTHOR : SUNG-JUN PARK
DATE : 2015-05-01
--------------------------------------------------------------------------
적용 함수 :
    void Set_CanaMsgBox(stCanMsgObject *i_pstCAN_MSG_Handle)
--------------------------------------------------------------------------
Etc.
    - &ECanaMboxes.MBOX0 의주소값 설정??
    - 레지스터 설정에 대한 입력을 받아들이는 방식 ?
    - ECanaRegs.CANME.all에 대한 값을 어떻게 입력 할것인지
--------------------------------------------------------------------------
INPUT :                                             INPUT RANGE         INPUT
    1.      &ECanaMboxes.MBOX0                      [][]
            i_pstCAN_MSG_Handle->m_byMboxNum        [][]
            ECanaRegs.CANMD.all                     [][]
            ECanaRegs.CANME.all                     [][]
            i_pstCAN_MSG_Handle->m_byMboxNum        [][]
            i_pstCAN_MSG_Handle->m_lMsgID           [][]
            i_pstCAN_MSG_Handle->m_byDLC            [][]
            i_pstCAN_MSG_Handle->m_byDirection      [][]
--------------------------------------------------------------------------
OUTPUT :                                            RANGE		OUTPUT
    1.      pstCANMailbox->MSGID.bit.STDMSGID       [][]
        	pstCANMailbox->MSGID.bit.IDE            [][]
        	pstCANMailbox->MSGID.bit.AME            [][]
        	pstCANMailbox->MSGID.bit.AAM            [][]

        	pstCANMailbox->MSGCTRL.bit.TPL          [][]
        	pstCANMailbox->MSGCTRL.bit.RTR          [][]
        	pstCANMailbox->MSGCTRL.bit.DLC          [][]

        	ECanaRegs.CANMD.all                     [][]
        	ECanaRegs.CANME.all                     [][]
--------------------------------------------------------------------------*/
/*
void Set_CanaMsgBox(stCanMsgObject *i_pstCAN_MSG_Handle)
{
	volatile struct MBOX *pstCANMailbox;

	pstCANMailbox = &ECanaMboxes.MBOX0 + i_pstCAN_MSG_Handle->m_byMboxNum;

	ECanaRegs.CANME.all = Set_Bit32(ECanaRegs.CANME.all, i_pstCAN_MSG_Handle->m_byMboxNum, 0x0);

	pstCANMailbox->MSGID.bit.STDMSGID	= i_pstCAN_MSG_Handle->m_lMsgID;
	pstCANMailbox->MSGID.bit.IDE		= 0;													// IDE(identifier extension bit)=0: standard identifier
	pstCANMailbox->MSGID.bit.AME		= 0;													// AME(acceptance mask enable)=0  			<<<<<<<<<<<<<<<<<<<10.09.13
	pstCANMailbox->MSGID.bit.AAM		= 0;													// AAM(auto answer mode)=0

	pstCANMailbox->MSGCTRL.bit.TPL		= 0;													// TPL(transmit priority level)
	pstCANMailbox->MSGCTRL.bit.RTR		= 0;													// RTR(remote transmission request)
	pstCANMailbox->MSGCTRL.bit.DLC		= i_pstCAN_MSG_Handle->m_byDLC;

	ECanaRegs.CANMD.all = Set_Bit32(ECanaRegs.CANMD.all, i_pstCAN_MSG_Handle->m_byMboxNum, i_pstCAN_MSG_Handle->m_byDirection);
	ECanaRegs.CANME.all = Set_Bit32(ECanaRegs.CANME.all, i_pstCAN_MSG_Handle->m_byMboxNum, 0x1);
}
*/
/*-----------------------------------------------------------------------
AUTHOR : SUNG-JUN PARK
DATE : 2015-04-27
--------------------------------------------------------------------------
적용 함수 :
    void Send_CanaMsg(stCanMsgObject  *i_pstCAN_MSG_Handle)
--------------------------------------------------------------------------
Etc.
    -&ECanaMboxes.MBOX0 의주소값 설정??
    - DataFiled 는 0 ~ 255 이다.
    - 레지스터 설정에 대한 입력을 받아들이는 방식 ?
--------------------------------------------------------------------------
INPUT :                                                         INPUT RANGE         INPUT
    1.      &ECanaMboxes.MBOX0
            i_pstCAN_MSG_Handle->m_byMboxNum
            i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit
            i_pstCAN_MSG_Handle->m_byDLC
--------------------------------------------------------------------------
OUTPUT :                                                        RANGE		OUTPUT
    1.      pstCANMailbox->MDL.byte.BYTE0                       [][]
            pstCANMailbox->MDL.byte.BYTE1                       [][]
            pstCANMailbox->MDL.byte.BYTE2           			[][]
            pstCANMailbox->MDL.byte.BYTE3           			[][]
            pstCANMailbox->MDH.byte.BYTE4           			[][]
            pstCANMailbox->MDH.byte.BYTE5           			[][]
            pstCANMailbox->MDH.byte.BYTE6           			[][]
            pstCANMailbox->MDH.byte.BYTE7           			[][]
            pstCANMailbox->MSGCTRL.bit.DLC          			[][]
			
            ECanaRegs.CANMC.bit.CDR                 			[][]
            ECanaRegs.CANMC.bit.MBNR                			[][]
            ECanaRegs.CANTRS.all                    			[][]
--------------------------------------------------------------------------*/
/*
void Send_CanaMsg(stCanMsgObject  *i_pstCAN_MSG_Handle)
{
	Uint32 CANTRS = 0x0;

	volatile struct MBOX *pstCANMailbox;

	pstCANMailbox = &ECanaMboxes.MBOX0 + i_pstCAN_MSG_Handle->m_byMboxNum;

	ECanaRegs.CANMC.bit.MBNR = 0;													//MBNR(mailbox number)
	ECanaRegs.CANMC.bit.CDR  = 1;													//CDR(change data field request)=1

	pstCANMailbox->MDL.byte.BYTE0 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>0)  & 0xff);
	pstCANMailbox->MDL.byte.BYTE1 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>8)  & 0xff);
	pstCANMailbox->MDL.byte.BYTE2 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>16) & 0xff);
	pstCANMailbox->MDL.byte.BYTE3 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>24) & 0xff);
	pstCANMailbox->MDH.byte.BYTE4 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>32) & 0xff);
	pstCANMailbox->MDH.byte.BYTE5 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>40) & 0xff);
	pstCANMailbox->MDH.byte.BYTE6 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>48) & 0xff);
	pstCANMailbox->MDH.byte.BYTE7 = (Uint8)((i_pstCAN_MSG_Handle->m_unDataField.m_lData64Bit>>56) & 0xff);

	pstCANMailbox->MSGCTRL.bit.DLC = i_pstCAN_MSG_Handle->m_byDLC;								//DLC(data length code)

	ECanaRegs.CANMC.bit.CDR = 0;   										//CDR(change data field request)=1

	CANTRS = ECanaRegs.CANTRS.all;

	ECanaRegs.CANTRS.all = CANTRS | (0x01<<i_pstCAN_MSG_Handle->m_byMboxNum);
}
*/
