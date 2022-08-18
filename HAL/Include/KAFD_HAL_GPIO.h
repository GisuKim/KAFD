/*
 * KAFD_HAL_GPIO.h
 *
 *  Created on: 2022. 8. 10.
 *      Author: syslabs
 */

#ifndef HAL_INCLUDE_KAFD_HAL_GPIO_H_
#define HAL_INCLUDE_KAFD_HAL_GPIO_H_

#include "f2838x_Device.h"     // DSP2833x Headerfile Include File
#include "driverlib.h"


#define     FAULT_LED_OFF()                  (GpioDataRegs.GPASET.bit.GPIO5 = 1)
#define     FAULT_LED_ON()                   (GpioDataRegs.GPACLEAR.bit.GPIO5 = 1)
#define     FAULT_LED_TOGGLE()                   (GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1)


void InitAFDGPIO(void);


#endif /* HAL_INCLUDE_KAFD_HAL_GPIO_H_ */
