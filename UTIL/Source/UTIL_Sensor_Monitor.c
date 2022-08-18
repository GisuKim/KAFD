/*
 * UTIL_Sensor_Monitor.c
 *
 *  Created on: 2022. 8. 17.
 *      Author: syslabs
 */

#include "UTIL_Sensor_Monitor.h"

ARRAY_QUEUE_def g_stADCDataQueue =ARRAY_QUEUE_DEFAULTS;   /* Queue of Sensor Data */
ARRAY_QUEUE_def g_stTXDataQueue =ARRAY_QUEUE_DEFAULTS;   /* Queue of Sensor Data */

Uint16 g_uSensorQueueCounter = 0;

ADC_DATA_STRUCT_DEF g_stADCData_out;

SENSOR_DATA_STRUCT_DEF g_stSensorData_out;
Uint8 SensCNT = 0;


void AFD_SensorMain(void)
{
    g_uSensorQueueCounter = GetADCSensorDataCount();
    while(GetADCSensorDataCount())
    {
        GetADCSensorData(&g_stADCData_out);
        ConversionAdcData(&g_stADCData_out,&g_stSensorData_out);

    }


}

void ConversionAdcData(ADC_DATA_STRUCT_DEF  *i_pstADC_Handle, SENSOR_DATA_STRUCT_DEF *i_pstSensor_Handle)
{
    i_pstSensor_Handle->m_fHFCT = i_pstADC_Handle->m_uHFCT + HFCT_SENSOR_OFFSET * HFCT_SENSOR_GAIN;
    i_pstSensor_Handle->m_fShunt = i_pstADC_Handle->m_uShunt + SHUNT_SENSOR_OFFSET * SHUNT_SENSOR_GAIN;
}

void SetADCSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData)
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Write_Buff[i] =0;
    }

//Set from Struct Data to Queue
    uData_Size = sizeof((*i_pstADCData));
    memcpy(&uTest_Write_Buff, &(*i_pstADCData), uData_Size);

    g_stADCDataQueue.Push(&g_stADCDataQueue, &uTest_Write_Buff);
    uData_Size=0;

}

void SetTXSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData)
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Write_Buff[i] =0;
    }

//Set from Struct Data to Queue
    uData_Size = sizeof((*i_pstADCData));
    memcpy(&uTest_Write_Buff, &(*i_pstADCData), uData_Size);

    g_stTXDataQueue.Push(&g_stTXDataQueue, &uTest_Write_Buff);
    uData_Size=0;

}

void GetADCSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData_out)
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Read_Buff[i] =0;
    }
//Get from Queue to Struct Data
    uData_Size = sizeof((*i_pstADCData_out));
    g_stADCDataQueue.Pop(&g_stADCDataQueue, &uTest_Read_Buff);
    memcpy(&(*i_pstADCData_out), &uTest_Read_Buff, uData_Size);

    uData_Size=0;
}

void GetTXSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData_out)
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Read_Buff[i] =0;
    }
//Get from Queue to Struct Data
    uData_Size = sizeof((*i_pstADCData_out));
    g_stTXDataQueue.Pop(&g_stTXDataQueue, &uTest_Read_Buff);
    memcpy(&(*i_pstADCData_out), &uTest_Read_Buff, uData_Size);

    uData_Size=0;
}

Uint16 GetADCSensorDataCount(void)
{
    return g_stADCDataQueue.m_uCount;
}

Uint16 GetTXSensorDataCount(void)
{
    return g_stTXDataQueue.m_uCount;
}


