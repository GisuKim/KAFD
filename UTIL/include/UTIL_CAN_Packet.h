#ifndef _UTIL_CAN_PACKET_H_
#define _UTIL_CAN_PACKET_H_

#include <HAL/Include/KAFD_HAL_CAN.h> //
//#include "f2838x_Examples.h"

//___________________________________________________________________________//



typedef	struct
{
    Uint8   m_byBitStart;               // 0 ~ 63
    Uint8   m_byBitLength;              // 1 ~ 16
    float32 m_fSignal_Min;              // -7000 ~ 0
    float32 m_fSignal_Max;              // 1 ~ 65535
    Uint16  m_uVal;                     // 0 ~ 65535 -32768 ~ 32767
    float32 m_fScalingFactor;           // 0.00152 ~ 1
    float32 m_fOffset;                  // -7000 ~ 0
    Uint8   m_byType;                   // 0~ 64
    float32 m_fVal;                     // -2147483648 ~ 2147483647
} CAN_BIT_DEF;
/*
#if( RUN_ON_TYPE==RUN_ON_RAM)
    #pragma CODE_SECTION(ConversionCanChannelToFrame, "ramfuncs")
    #pragma CODE_SECTION(ConversionCanFrameToChannel_COMP_Status, "ramfuncs")
    #pragma CODE_SECTION(ConversionCanFrameToChannel_DevMode_CMD1, "ramfuncs")
    #pragma CODE_SECTION(ConversionCanFrameToChannel_DevMode_CMD2, "ramfuncs")
    #pragma CODE_SECTION(ConversionCanFrameToChannel_DevMode_CMD3, "ramfuncs")
#endif
*/
void Get_Data16_Uint_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data);
void Get_Data16_Float_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data);
void Get_Data32_Float_INTEL(Uint64 i_uData, CAN_BIT_DEF *i_pstConv_Data);

Uint64 Set_Data16_Uint_INTEL(CAN_BIT_DEF *i_pstConv_Data);
Uint64 Set_Data16_Float_INTEL(CAN_BIT_DEF *i_pstConv_Data);

void ConversionCanFrameToChannel_IVT_Result(stCanMsgObject *i_pstCanMsg_Handle);


#endif

