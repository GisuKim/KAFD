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
#include "driverlib.h"
#include <HAL/Include/KAFD_HAL_CAN.h>
#include <HAL/Include/KAFD_HAL_GPIO.h>

//
// Function Prototypes
//
void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCEpwm(void);
void initSCIAFIFO(void);
void initSCIAFIFO(void);
void error(void);

interrupt void adca1_isr(void);
interrupt void CanaISR(void);
interrupt void CanbISR(void);
interrupt void sciaRXFIFOISR(void);
interrupt void sciaTXFIFOISR(void);


//
// Defines
//
#define RESULTS_BUFFER_SIZE 256
#define CANA_RX_MSG_RESULT       1       // CAN-A(TX) ����� 32�� �޽��� ������Ʈ�� �� 1�� MSG_OBJ�� TX������ ���
#define CANA_RX_MSG_RESULT       1       // CAN-A(TX) ����� 32�� �޽��� ������Ʈ�� �� 1�� MSG_OBJ�� TX������ ���
#define MSG_DATA_A_LENGTH     6       // �׽�Ʈ�� �ۼ��� �޽��� ����
#define MSG_DATA_B_LENGTH     8       // �׽�Ʈ�� �ۼ��� �޽��� ����
#define TX_MSG_OBJ_ID       1       // CAN-A(TX) ����� 32�� �޽��� ������Ʈ�� �� 1�� MSG_OBJ�� TX������ ���
#define RX_MSG_OBJ_ID       1       // CAN-B(RX) ����� 32�� �޽��� ������Ʈ�� �� 1�� MSG_OBJ�� RX������ ���
//
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE][4];
Uint16 resultsIndex;
volatile Uint16 bufferFull;

unsigned long              BackTicker;
volatile unsigned long      i;
volatile uint32_t           txMsgCount = 0;
volatile uint32_t           rxMsgCount = 0;
volatile uint32_t           errorFlag = 0;
uint16_t                    rxMsgDataA[MSG_DATA_A_LENGTH];
uint16_t                    rxMsgDataB[MSG_DATA_B_LENGTH];


//
// Send data for SCI-A
//
uint16_t sDataA[2];

//
// Received data for SCI-A
//
uint16_t rDataA[2];

//
// Used for checking the received data
//
uint16_t rDataPointA;


void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the f2838x_sysctrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the f2838x_gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio(); // Skipped for this example

    InitAFDGPIO();

    InitCan();
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Configure the ADC and power it up
    //
        ConfigureADC();

    //
    // Configure the ePWM
    //
        ConfigureEPWM();

    //
    // Setup the ADC for ePWM triggered conversions on channel 0
    //
        SetupADCEpwm();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    IER |= M_INT1; //Enable group 1 interrupts

    EINT;
    ERTM;


//
// Map ISR functions
//
//    EALLOW;
//    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
//    EDIS;

    Interrupt_register(INT_ADCA1, &adca1_isr);    // INT_CANB0 ���ͷ�Ʈ ���Ϳ� CanbISR( ) �Լ� ����
    Interrupt_register(INT_CANA0, &CanaISR);    // INT_CANB0 ���ͷ�Ʈ ���Ϳ� CanbISR( ) �Լ� ����
    Interrupt_register(INT_CANB0, &CanbISR);    // INT_CANB0 ���ͷ�Ʈ ���Ϳ� CanbISR( ) �Լ� ����
    Interrupt_register(INT_SCIA_RX, &sciaRXFIFOISR);
    Interrupt_register(INT_SCIA_TX, &sciaTXFIFOISR);

    initSCIAFIFO();


    for(i = 0; i < 2; i++)
    {
        sDataA[i] = i;
    }

    rDataPointA = sDataA[0];

    //
    // Enable the CAN-A interrupt signal
    //
    Interrupt_enable(INT_CANA0);
    Interrupt_enable(INT_CANB0);
    Interrupt_enable(INT_SCIA_RX);
    Interrupt_enable(INT_SCIA_TX);

    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    InitCanMessageBox();
    // CAN �޽��� ���ۿ� ����� �۽� �޽��� ������Ʈ �ʱ�ȭ
    // CAN Module:                  A
    // Message Object ID Number:    TX_MSG_OBJ_ID
    // Message Identifier:          0x95555555
    // Message Frame:               Extended
    // Message Type:                Transmit
    // Message ID Mask:             0x0
    // Message Object Flags:        None
    // Message Data Length:         4 Bytes (Note that DLC field is a "don't care" for a Receive mailbox
    CAN_setupMessageObject( CANA_BASE, TX_MSG_OBJ_ID, 0x523,
                            CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                            CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_A_LENGTH );
//CAN_MSG_OBJ_USE_ID_FILTER
    // CAN �޽��� ���ſ� ����� ���� �޽��� ������Ʈ �ʱ�ȭ
    // CAN Module:                  B
    // Message Object ID Number:    RX_MSG_OBJ_ID
    // Message Identifier:          0x95555555
    // Message Frame:               Extended
    // Message Type:                Receive
    // Message ID Mask:             0x0
    // Message Object Flags:        Receive Interrupt
    // Message Data Length:         4 Bytes

    CAN_setupMessageObject( CANB_BASE, RX_MSG_OBJ_ID, 0x3c2,
                            CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                            CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_B_LENGTH );


    CAN_startModule(CANA_BASE);
    CAN_startModule(CANB_BASE);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
                      // Load output latch


//
// Initialize results buffer
//
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex][0] = 0;
        AdcaResults[resultsIndex][1] = 0;
        AdcaResults[resultsIndex][2] = 0;
        AdcaResults[resultsIndex][3] = 0;
    }
    resultsIndex = 0;
    bufferFull = 0;

//
// Enable PIE interrupt
//
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

//
// Sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

//
// Take conversions indefinitely in loop
//
    do
    {

        //
        // Start ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

        //
        // Wait while ePWM causes ADC conversions, which then cause interrupts,
        // which fill the results buffer, eventually setting the bufferFull
        // flag
        //
        while(!bufferFull);
        bufferFull = 0; //clear the buffer full flag

        //
        // Stop ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 3; //freeze counter

        //
        // At this point, AdcaResults[] contains a sequence of conversions
        // from the selected channel
        //

        //
        // Software breakpoint, hit run again to get updated conversions
        //

//        asm("   ESTOP0");
    } while(1);
}



//
// initSCIAFIFO - Configure SCIA FIFO
//
void initSCIAFIFO()
{
    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 115200.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIA_BASE);
    //SCI_enableLoopback(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_enableInterrupt(SCIA_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXERR);

    //
    // The transmit FIFO generates an interrupt when FIFO status
    // bits are less than or equal to 2 out of 16 words
    // The receive FIFO generates an interrupt when FIFO status
    // bits are greater than equal to 2 out of 16 words
    //
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX2, SCI_FIFO_RX2);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
}



//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    // Write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// ConfigureEPWM - Configure EPWM SOC and compare values
//
void ConfigureEPWM(void)
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;     // Set compare A value to 2048 counts
    EPwm1Regs.TBPRD = 0x1000;             // Set period to 4096 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(void)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    // Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 4;  //SOC3 will convert pin A4
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}


//
// error - Function to halt debugger on error
//
void error(void)
{
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}


interrupt void sciaRXFIFOISR(void)
{
    uint16_t i;

    SCI_readCharArray(SCIA_BASE, rDataA, 2);

    //
    // Check received data
    //
    for(i = 0; i < 2; i++)
    {
        if(rDataA[i] != ((rDataPointA + i) & 0x00FF))
        {
            //error();
        }
    }

    rDataPointA = (rDataPointA + 1) & 0x00FF;

    SCI_clearOverflowStatus(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

interrupt void sciaTXFIFOISR(void)
{
    uint16_t i;

    SCI_writeCharArray(SCIA_BASE, sDataA, 2);

    //
    // Increment send data for next cycle
    //
    for(i = 0; i < 2; i++)
    {
        sDataA[i] = (sDataA[i] + 1) & 0x00FF;
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

    //
    // Issue PIE ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void)
{
    resultsIndex++;
    AdcaResults[resultsIndex][0] = AdcaResultRegs.ADCRESULT0;
    AdcaResults[resultsIndex][1] = AdcaResultRegs.ADCRESULT1;
    AdcaResults[resultsIndex][2] = AdcaResultRegs.ADCRESULT2;
    AdcaResults[resultsIndex][3] = AdcaResultRegs.ADCRESULT3;

    if(RESULTS_BUFFER_SIZE <= resultsIndex)
    {
        resultsIndex = 0;
        bufferFull = 1;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


interrupt void CanaISR(void)
{
    uint32_t status;

    // ���ͷ�Ʈ �߻� ������ �ľ��ϱ� ���� CAN-B ����� ���ͷ�Ʈ ����(Interrupt Status) Ȯ��
    status = CAN_getInterruptCause(CANA_BASE);


    // (�޽��� ������ �ƴ�) CAN ��Ʈ�ѷ� ���� ��ȭ�� ���� ���ͷ�Ʈ �߻��̶��, �ش� ���� Ȯ��
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // ��Ʈ�ѷ� ���� Ȯ�� (Read the CAN controller status)
        // �� �������� �ڵ� ���⼺�� ���ϱ� ���� �� ������ ���� ��ȭ�� ���� ���� �ڵ�� ���ԵǾ� ���� ����
        status = CAN_getStatus(CANA_BASE);

        // ����(Error)�� �߻��ߴ��� Ȯ��
        if( ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            // ������ �߻������� �˸��� ���� Flag ������ 1�� ����
            errorFlag = 1;
        }
    }
    // CAN-B ��� ���� �޽��� ������Ʈ�� ���� ���ͷ�Ʈ�� ���, ���ŵ� ������ Ȯ��
    else if(status == RX_MSG_OBJ_ID)
    {
        // ���ŵ� ������ Ȯ��
        CAN_readMessage(CANA_BASE, RX_MSG_OBJ_ID, rxMsgDataA);

        // CAN �޽��� ������Ʈ ���ͷ�Ʈ Ŭ����
        CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);

        // ���� ������ ���� �ľǿ� ī���� ���� ����
        rxMsgCount++;

        // �޽����� ���ŵǾ����Ƿ�, ���� Flag Clear
        errorFlag = 0;
    }
    // ������� ���� �Ϸ� ���ͷ�Ʈ �߻� ��, �̰����� ó��
    else
    {
        asm("  NOP"); // No Operation
    }


    // CAN ����(Global) ���ͷ�Ʈ Flag Clear
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    // INT_CANB0 ���ͷ�Ʈ ���Ͱ� ���Ե� CPU ���ͷ�Ʈ Ȯ��׷� 9���� Acknowledge ��Ʈ Ŭ����
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


interrupt void CanbISR(void)
{

    uint32_t status;

    // ���ͷ�Ʈ �߻� ������ �ľ��ϱ� ���� CAN-B ����� ���ͷ�Ʈ ����(Interrupt Status) Ȯ��
    status = CAN_getInterruptCause(CANB_BASE);

    // (�޽��� ������ �ƴ�) CAN ��Ʈ�ѷ� ���� ��ȭ�� ���� ���ͷ�Ʈ �߻��̶��, �ش� ���� Ȯ��
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // ��Ʈ�ѷ� ���� Ȯ�� (Read the CAN controller status)
        // �� �������� �ڵ� ���⼺�� ���ϱ� ���� �� ������ ���� ��ȭ�� ���� ���� �ڵ�� ���ԵǾ� ���� ����
        status = CAN_getStatus(CANB_BASE);

        // ����(Error)�� �߻��ߴ��� Ȯ��
        if( ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            // ������ �߻������� �˸��� ���� Flag ������ 1�� ����
            errorFlag = 1;
        }
    }
    // CAN-B ��� ���� �޽��� ������Ʈ�� ���� ���ͷ�Ʈ�� ���, ���ŵ� ������ Ȯ��
    else if(status == RX_MSG_OBJ_ID)
    {
        // ���ŵ� ������ Ȯ��
        CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, rxMsgDataB);

        // CAN �޽��� ������Ʈ ���ͷ�Ʈ Ŭ����
        CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID);

        // ���� ������ ���� �ľǿ� ī���� ���� ����
        rxMsgCount++;

        // �޽����� ���ŵǾ����Ƿ�, ���� Flag Clear
        errorFlag = 0;
    }
    // ������� ���� �Ϸ� ���ͷ�Ʈ �߻� ��, �̰����� ó��
    else
    {
        asm("  NOP"); // No Operation
    }

    // CAN ����(Global) ���ͷ�Ʈ Flag Clear
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    // INT_CANB0 ���ͷ�Ʈ ���Ͱ� ���Ե� CPU ���ͷ�Ʈ Ȯ��׷� 9���� Acknowledge ��Ʈ Ŭ����
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// End of file
//

