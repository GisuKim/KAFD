/*
 * CPU1_Main.c
 *
 *  Created on: 2022. 8. 2.
 *      Author: syslabs
 */
//
// Included Files
//
#include "f28x_project.h"

//
// Main
//
void main(void)
{
    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the f2838x_sysctrl.c file.
    //
    InitSysCtrl();

    //
    // Initialize GPIO:
    // This example function is found in the f2838x_gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    InitGpio();

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the f2838x_piectrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in f2838x_defaultisr.c.
    // This function is found in f2838x_pievect.c.
    //
    InitPieVectTable();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //
    // For this case just init GPIO pins for ePWM1
    // Only CPU1 can configure GPIO muxing so this is done here
    // These functions are in the f2838x_epwm.c file
    //
    InitEPwm1Gpio();

    //
    // Transfer ownership of EPWM1 and ADCA to CPU02
    //
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM1 = 1;
    DevCfgRegs.CPUSEL11.bit.ADC_A = 1;
    EDIS;

    //
    // IDLE loop. Just sit and loop forever (optional):
    //
    while(1)
    {
        asm(" nop");
    }
}

//
// End of file
//

