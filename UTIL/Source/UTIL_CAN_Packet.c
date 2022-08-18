//#include "f2838x_Device.h"     // DSP2833x Headerfile Include File
#include "UTIL_CAN_Packet.h"

//____________________________________________________________________________//
// 	NOT		complement		~
// 	OR						|
//	AND						&
//	XOR		exclusive or		^			0101^0011 = 0110
//	LEFT-SHIFT				<<			y×2^right
//	RIGHT-SHIFT				>>			y/2^right
//____________________________________________________________________________//


Uint16 g_auTest_can[5] = {0,0,0,0,0};

Uint16 g_auBitMask[17] = {
	0x0000, 0x0001, 0x0003, 0x0007, 0x000F,
			0x001F, 0x003F, 0x007F, 0x00FF,
			0x01FF, 0x03FF, 0x07FF, 0x0FFF,
			0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF };

Uint32 g_alBitMask[33] = {
    0x00000000, 0x00000001, 0x00000003, 0x00000007, 0x0000000F,
                0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
                0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF,
                0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
                0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF,
                0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
                0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF,
                0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF};


// AFD <------ IVT-S Msg --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//			Signal Name  					               Sbit	   Len     Min  	   Max		  uVal  fScalingFactor Offset  	  Type 	   fVal			Description	Unit
// Message Object ID Number[0]
// 0x52x
CAN_BIT_DEF	g_stIVT_Result_MuxID      			    =	{	0	,	8	,	0		,	7   	,	0	,	1		,	0		,	0	,	0.0	};	//	0x00 = I, 0x00 = I
CAN_BIT_DEF	g_stIVT_Result_IVT_MsgCount			    =	{	8	,	8	,	0		,	15  	,	0	,	1		,	0		,	0	,	0.0	};	//	0=Good, 1= Fault	0,1
CAN_BIT_DEF	g_stIVT_Result_IVT_Result_state         =	{	16	,	8	,	0		,	15		,	0	,	1		,	0		,	0	,	0.0	};	//	0=Off, 1= on
CAN_BIT_DEF g_stIVT_Result_IVT_Value_Current        =   {   24  ,   32  ,   -32000  ,   32000   ,   0   ,   0.001   ,   0       ,   0   ,   0.0 };  //  Current -32.000 ~ +32.000 A
CAN_BIT_DEF g_stIVT_Result_IVT_Value_U1             =   {   24  ,   32  ,   -32000  ,   32000   ,   0   ,   0.001   ,   0       ,   0   ,   0.0 };  //  U1 -32.000 ~ +32.000 V
CAN_BIT_DEF g_stIVT_Result_IVT_Value_U2             =   {   24  ,   32  ,   -32000  ,   32000   ,   0   ,   0.001   ,   0       ,   0   ,   0.0 };  //  U2 -32.000 ~ +32.000 V
CAN_BIT_DEF g_stIVT_Result_IVT_Value_U3             =   {   24  ,   32  ,   -32000  ,   32000   ,   0   ,   0.001   ,   0       ,   0   ,   0.0 };  //  U3 -32.000 ~ +32.000 V
CAN_BIT_DEF g_stIVT_Result_IVT_Value_T              =   {   24  ,   32  ,   -500    ,   2000    ,   0   ,   0.1     ,   0       ,   0   ,   0.0 };  //  Temperature -50.0 ~ +200.0 ℃
CAN_BIT_DEF g_stIVT_Result_IVT_Value_W              =   {   24  ,   32  ,   -99999  ,   99999   ,   0   ,   1.0     ,   0       ,   0   ,   0.0 };  //  Power(U1) -99999 ~ +99999 W
CAN_BIT_DEF g_stIVT_Result_IVT_Value_As             =   {   24  ,   32  ,   -99999  ,   99999   ,   0   ,   1.0     ,   0       ,   0   ,   0.0 };  //  CurrentCount -99999 ~ +99999 As
CAN_BIT_DEF g_stIVT_Result_IVT_Value_Wh             =   {   24  ,   32  ,   -99999  ,   99999   ,   0   ,   1.0     ,   0       ,   0   ,   0.0 };  //  EnergyCounter -99999 ~ +99999 Wh


// AFD <----- FluxGate Msg ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Message Object ID Number[0]
// 0x3c2
CAN_BIT_DEF	g_stFG_SGV          					=	{	0	,	1	,	0		,	1   	,	0 	,	1   	,	0   	,	0	,	0.0 };	//  ‘SafetyGoalViolation’ signal
CAN_BIT_DEF	g_stFG_StatusIE		    				=	{	1	,	1	,	0		,	1   	,	0 	,	1   	,	0		,	0	,	0.0 };	//	‘StatusInternalError’ signal
CAN_BIT_DEF	g_stFG_StatusPS			                =	{	2	,	2	,	0		,	3       ,	0 	,	1       ,	0	    ,	0	,	0.0 };	//	‘StatusPowerSupply’ signal
CAN_BIT_DEF	g_stFG_SCI				                =	{	4	,	4	,	0		,	15  	,	0	,	1		,	0		,	0	,	0.0	};	//  ‘SequenceCounterIP’ signal
CAN_BIT_DEF g_stFG_AnalogCurrent                    =   {   8   ,   24  , -1550.000 ,-1550.000  ,   0   ,   1       ,   0       ,   0   ,   0.0 };  //  'Analog Current' signal
CAN_BIT_DEF g_stFG_DigitalCurrent                   =   {   32  ,   16  , -1500     ,  -1550    ,   0   ,   1       ,   0       ,   0   ,   0.0 };  //  'Digital Current' signal
CAN_BIT_DEF g_stFG_Reserved                         =   {   48  ,   8   ,   0       ,   255     ,   0   ,   1       ,   0       ,   0   ,   0.0 };  //  Reserved
CAN_BIT_DEF g_stFG_CRC                              =   {   56  ,   8   ,   0       ,   255     ,   0   ,   1       ,   0       ,   0   ,   0.0 };  //  CRC

/*
 * ● ‘SafetyGoalViolation’ signal
    Within the current range of [−1500 A; −220 A] and [+ 220 A; + 1500 A], if there more than 20% of difference between
    analog current level and digital current level --> then SafetyGoalViolation = 1
    Within the current range of [−220 A; 220 A], if there is a gap above 44 A between analog current level and digital current
    level --> then SafetyGoalViolation = 1
    Safe State: To provide Safety Goal violation flag, keep providing data measurement
    FTTI: 100ms.

 * ● ‘StatusInternalError’ signal
    Internal hardware error (Memory , reference voltage, DAC, PGA errors).

 * ● ‘StatusPowerSupply’ signal
    • If power supply voltage is strictly above 8 V and strictly below than 16 V, then ‘StatusPowerSupply’ = 0
    LOW VOLTAGE use cases:
    • If power supply goes down below 8 V (< 8 V) and over 6 V:
    ‘StatusPowerSupply’ = 1 after 100 ms.
    • If power supply goes over 8 V (≥ 8 V):
    Then ‘StatusPowerSupply’ is set to 0 (no filter)
    • If power supply goes below 6 V (< 6 V), the sensor stops emitting CAN frame
    • If power supply goes over 6 V (≥ 6 V), CAB starts emitting CAN frame.
    OVER VOLTAGE use cases:
    • If power supply goes over 16 V (> 16 V) and below 18 V:
    ‘StatusPowerSupply’ = 2 after 100 ms
    Current level data (analog and digital) are frozen in the CAN frame
    • If power supply goes below 16 V (≤ 16 V):
    Then ‘StatusPowerSupply’ is set to 0 (no filter)
    • If power supply goes over 18 V (> 18 V), the sensor stops emitting CAN frame
    • If power supply goes below 18 V (≤ 18 V), CAB starts emitting CAN frame
    Power Supply voltage measurement not available:
    • ‘StatusPowerSupply’ = 3.

   ● ‘SequenceCounterIP’ signal
    • Initialized with 0 and incremented by 1 for every subsequent send request
    • When the counter reaches the value 15(0xF), then restart with 1 for the next send request
*/



// From IVT-S Unit to AFD
void ConversionCanFrameToChannel_IVT_Result(stCanMsgObject *i_pstCanMsg_Handle)
{
	//Message Object ID Number[0]
	//IVT-S Result
	//CAN ID 0x52x
	Get_Data16_Uint_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit	,&	g_stIVT_Result_MuxID	            );
	Get_Data16_Uint_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit	,&	g_stIVT_Result_IVT_MsgCount         );
	Get_Data16_Uint_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit	,&	g_stIVT_Result_IVT_Result_state	    );

	switch(g_stIVT_Result_MuxID.m_uVal)
	{
	case 0x00:
	    Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_Current    );
	    break;
    case 0x01:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_U1    );
        break;
    case 0x02:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_U2    );
        break;
    case 0x03:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_U3    );
        break;
    case 0x04:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_T    );
        break;
    case 0x05:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_W    );
        break;
    case 0x06:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_As    );
        break;
    case 0x07:
        Get_Data32_Float_INTEL( i_pstCanMsg_Handle->m_unDataField.m_lData64Bit  ,&  g_stIVT_Result_IVT_Value_Wh    );
        break;
    default:
        break;
	}

}


Uint64 Set_Data16_Uint_INTEL(CAN_BIT_DEF *i_pstConv_Data)
{
	Uint8   BitStart  = 0;
	Uint8   BitLength = 0;
	Uint16  Step       = 0;
	Uint64  Result     = 0;
	float32 Step1      = 0.00;
	float32 Step2      = 0.00;

	BitStart  = i_pstConv_Data->m_byBitStart;
	BitLength = i_pstConv_Data->m_byBitLength;

	if (BitLength >= 16 )
	{
		BitLength = 16;
	}

	Step1 = (float32)i_pstConv_Data->m_uVal;		

	if(i_pstConv_Data->m_uVal >= i_pstConv_Data->m_fSignal_Max)
    {
        Step1 = i_pstConv_Data->m_fSignal_Max;
	}
	if(i_pstConv_Data->m_uVal <= i_pstConv_Data->m_fSignal_Min)
    {
        Step1 = i_pstConv_Data->m_fSignal_Min;
	}
    
	Step2  = (Step1 - i_pstConv_Data->m_fOffset ) * (1.0/i_pstConv_Data->m_fScalingFactor);

	Step   = ((Uint16)(Step2) ) & g_auBitMask[BitLength];

	Result = (((Uint64)Step) << BitStart );

	return Result;
}


Uint64 Set_Data16_Float_INTEL(CAN_BIT_DEF *i_pstConv_Data)
{
	Uint8   BitStart;
	float32 Step1;
	float32 Step2;
	Uint16  Step;
	Uint64  Result;
    
	BitStart = i_pstConv_Data->m_byBitStart;

    Step1 = i_pstConv_Data->m_fVal;        

	if(i_pstConv_Data->m_fVal >= i_pstConv_Data->m_fSignal_Max)
    {
        Step1 = i_pstConv_Data->m_fSignal_Max;
	}
	if(i_pstConv_Data->m_fVal <= i_pstConv_Data->m_fSignal_Min)
    {
        Step1 = i_pstConv_Data->m_fSignal_Min;
	}

	Step2  = (Step1 - i_pstConv_Data->m_fOffset ) * (1.00/i_pstConv_Data->m_fScalingFactor)+0.5;
	Step   = (Uint16)(Step2);
	Result = (((Uint64)Step) << BitStart);

	return Result;
}

void Get_Data16_Uint_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data)
{
    Uint8 BitStart = 0;

	BitStart= i_pstConv_Data->m_byBitStart;

	i_pstConv_Data->m_uVal = ((i_uData >> BitStart) & g_auBitMask[i_pstConv_Data->m_byBitLength]) * i_pstConv_Data->m_fScalingFactor + i_pstConv_Data->m_fOffset;

	if(i_pstConv_Data->m_uVal >= i_pstConv_Data->m_fSignal_Max)
    {
        i_pstConv_Data->m_uVal = i_pstConv_Data->m_fSignal_Max;
	}
	if(i_pstConv_Data->m_uVal <= i_pstConv_Data->m_fSignal_Min)
    {
        i_pstConv_Data->m_uVal = i_pstConv_Data->m_fSignal_Min;
	}
    
	i_pstConv_Data->m_fVal  = (float32)i_pstConv_Data->m_uVal;
}


void Get_Data16_Float_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data)
{

	Uint8 BitStart = 0;

	BitStart = i_pstConv_Data->m_byBitStart;

	i_pstConv_Data->m_fVal = ((( i_uData >> BitStart) & g_auBitMask[i_pstConv_Data->m_byBitLength]) * i_pstConv_Data->m_fScalingFactor) + i_pstConv_Data->m_fOffset;

	if(i_pstConv_Data->m_fVal >= i_pstConv_Data->m_fSignal_Max)
    {
        i_pstConv_Data->m_fVal = i_pstConv_Data->m_fSignal_Max;
	}
	if(i_pstConv_Data->m_fVal <= i_pstConv_Data->m_fSignal_Min)
    {
        i_pstConv_Data->m_fVal = i_pstConv_Data->m_fSignal_Min;
	}
}

void Get_Data32_Float_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data)
{

    Uint8 BitStart = 0;

    BitStart = i_pstConv_Data->m_byBitStart;

    i_pstConv_Data->m_fVal = ((int32)(( i_uData >> BitStart) & g_alBitMask[i_pstConv_Data->m_byBitLength]) * i_pstConv_Data->m_fScalingFactor) + i_pstConv_Data->m_fOffset;

    if(i_pstConv_Data->m_fVal >= i_pstConv_Data->m_fSignal_Max)
    {
        i_pstConv_Data->m_fVal = i_pstConv_Data->m_fSignal_Max;
    }
    if(i_pstConv_Data->m_fVal <= i_pstConv_Data->m_fSignal_Min)
    {
        i_pstConv_Data->m_fVal = i_pstConv_Data->m_fSignal_Min;
    }
}

//void GetTXSensorData(stCanMsgObject *i_pstCanMsg_Handle)
//{
//    Uint16 i;
//    Uint16 uData_Size;
//    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];
//
//    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
//    {
//        uTest_Read_Buff[i] =0;
//    }
////Get from Queue to Struct Data
//    uData_Size = sizeof((*i_pstADCData_out));
//    g_stTXDataQueue.Pop(&g_stTXDataQueue, &uTest_Read_Buff);
//    memcpy(&(*i_pstADCData_out), &uTest_Read_Buff, uData_Size);
//
//    uData_Size=0;
//}
