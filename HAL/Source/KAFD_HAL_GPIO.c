/*
 * KAFD_HAL_GPIO.c
 *
 *  Created on: 2022. 8. 10.
 *      Author: syslabs
 */

#include <HAL/Include/KAFD_HAL_GPIO.h>

void InitAFDGPIO(void)
{

    //CAN A/B GPIO Setting
    GPIO_setPinConfig(GPIO_36_CANA_RX);
    GPIO_setPinConfig(GPIO_37_CANA_TX);
    GPIO_setPinConfig(GPIO_38_CANB_TX);
    GPIO_setPinConfig(GPIO_39_CANB_RX);

    //SCI RX GPIO Setting
    GPIO_setMasterCore(GPIO_49_SCIA_RX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_49_SCIA_RX);
    GPIO_setDirectionMode(GPIO_49_SCIA_RX, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(GPIO_49_SCIA_RX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_49_SCIA_RX, GPIO_QUAL_ASYNC);    //No synchronization
    //SCI TX GPIO Setting
    GPIO_setMasterCore(GPIO_48_SCIA_TX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_48_SCIA_TX);
    GPIO_setDirectionMode(GPIO_48_SCIA_TX, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(GPIO_48_SCIA_TX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_48_SCIA_TX, GPIO_QUAL_ASYNC);   //No synchronization


    GPIO_setPinConfig(GPIO_1_GPIO1);        //RS485 RE
    GPIO_setPinConfig(GPIO_2_GPIO2);        //CAN A OE
    GPIO_setPinConfig(GPIO_3_GPIO3);        //CAN B OE
    GPIO_setPinConfig(GPIO_4_GPIO4);        //FAULT SIG
    GPIO_setPinConfig(GPIO_5_GPIO5);        //FAULT LED
    GPIO_setPinConfig(GPIO_6_GPIO6);        //RS485 DE
    GPIO_setPinConfig(GPIO_7_GPIO7);        //SPARE SIG

    GPIO_setDirectionMode(1,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO01
    GPIO_writePin(1, 0);                          // RS485 RE Enable = 0

    GPIO_setDirectionMode(2,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO2
    GPIO_writePin(2, 0);                          // CANA Driver OE = 0 (Enable)

    GPIO_setDirectionMode(3,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO3
    GPIO_writePin(3, 0);                          // CANB Driver OE = 0 (Enable)

    GPIO_setDirectionMode(4,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO4
    GPIO_writePin(4, 0);                          // Fault Signal default = 0 (disable)

    GPIO_setDirectionMode(5,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO5
    GPIO_writePin(5, 0);                          // Fault led default = 0 (off)

    GPIO_setDirectionMode(6,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO6
    GPIO_writePin(6, 1);                          //RS485 DE Enable = 1

    GPIO_setDirectionMode(7,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO7
    GPIO_writePin(7, 0);                          //SPARE SIG default = 0 (disable)


}
