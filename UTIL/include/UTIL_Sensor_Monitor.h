/*
 * UTIL_Sensor_Monitor.h
 *
 *  Created on: 2022. 8. 17.
 *      Author: syslabs
 */

#ifndef UTIL_INCLUDE_UTIL_SENSOR_MONITOR_H_
#define UTIL_INCLUDE_UTIL_SENSOR_MONITOR_H_

#include "UTIL/include/UTIL_Queue.h"

#define DEF_GAIN                (3.3/4095.0)
#define HFCT_SENSOR_GAIN         0.1
#define HFCT_SENSOR_OFFSET       0.0

#define SHUNT_SENSOR_GAIN         0.1
#define SHUNT_SENSOR_OFFSET       0.0

typedef struct ADC_DATA_STRUCT{
                Uint16  m_Count;
                Uint16  m_uShunt;             // 0~4095(0V~1V)                                     ,       1               Shunt ADC1,
                Uint16  m_uHFCT;              // 0~4095(-5V~+5V)                                   ,       1               HFCT  ADC3
            }ADC_DATA_STRUCT_DEF;

typedef struct SENSOR_DATA_STRUCT{
                float32  m_fShunt;             // 0~4095(0V~1V)                                     ,       1               Shunt ADC1,
                float32  m_fHFCT;              // 0~4095(-5V~+5V)                                   ,       1               HFCT  ADC3
            }SENSOR_DATA_STRUCT_DEF;

void AFD_SensorMain(void);

void SetADCSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData);
void GetADCSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData_out);
void SetTXSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData);
void GetTXSensorData(ADC_DATA_STRUCT_DEF *i_pstADCData_out);

void ConversionAdcData(ADC_DATA_STRUCT_DEF  *i_pstADC_Handle, SENSOR_DATA_STRUCT_DEF *i_pstSensor_Handle);
Uint16 GetADCSensorDataCount(void);
Uint16 GetTXSensorDataCount(void);
#endif /* UTIL_INCLUDE_UTIL_SENSOR_MONITOR_H_ */
