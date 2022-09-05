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
#include <HAL/Include/KAFD_HAL_ADC.h>
#include <HAL/Include/KAFD_HAL_PWM.h>
#include <UTIL/include/UTIL_Sensor_Monitor.h>
#include "KAFD_CPU1_Setting.h"
#include <UTIL/include/UTIL_CRC8.h>
//
// Function Prototypes
//




#define mSec1                               6000        // 1.0 mS
#define mSec5                               30000       // 5.0 mS


void initSCIAFIFO(void);
void initSCIAFIFO(void);
void error(void);

// State Machine function prototypes
//------------------------------------
// Alpha states
void Task_GroupA_main(void);    //state A0
void Task_GroupB_main(void);    //state B0
void Task_GroupA1(void);        //state A1  // A branch states
void Task_GroupB1(void);        //state B1  // B branch states
void Task_GroupB2(void);        //state B2  // B branch states

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch



interrupt void adca1_isr(void);
interrupt void CanaISR(void);
interrupt void CanbISR(void);
interrupt void sciaRXFIFOISR(void);
interrupt void sciaTXFIFOISR(void);
interrupt void ISR_CpuTimer0(void);


//
// Defines
//
#define RESULTS_BUFFER_SIZE 64
//#define CANA_RX_MSG_RESULT       1       // CAN-A(TX) 모듈의 32개 메시지 오브젝트들 중 1번 MSG_OBJ를 TX용으로 사용
#define MSG_DATA_A_LENGTH     6       // 테스트용 송수신 메시지 길이
#define MSG_DATA_B_LENGTH     8       // 테스트용 송수신 메시지 길이
#define TX_MSG_OBJ_ID       1       // CAN-A(TX) 모듈의 32개 메시지 오브젝트들 중 1번 MSG_OBJ를 TX용으로 사용
#define RX_MSG_OBJ_ID       1       // CAN-B(RX) 모듈의 32개 메시지 오브젝트들 중 1번 MSG_OBJ를 RX용으로 사용
////
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE][4];
Uint16 resultsIndex;

Uint32 g_SendValueCount = 0;
Uint8 g_SendStart = 0;


volatile Uint16 bufferFull;

unsigned long              BackTicker;
volatile unsigned long      i;
volatile uint32_t           txMsgCount = 0;
volatile uint32_t           rxMsgCount = 0;
volatile uint32_t           errorFlag = 0;
uint16_t                    rxMsgDataA[MSG_DATA_A_LENGTH];
uint16_t                    rxMsgDataB[MSG_DATA_B_LENGTH];

Uint16 g_auVTimer0[4];                                  // Virtual Timers slaved off CPU Timer 0 (A events)
Uint16 g_auVTimer1[4];

Uint16 g_uBackTicker = 0;                           // Count in ISR

Uint16 g_uTXQueueCounter = 0;
ADC_DATA_STRUCT_DEF g_stADCData;
ADC_DATA_STRUCT_DEF g_stTXData_out;
//

Uint8 g_ConnectTestSW = 0;
// Send data for SCI-A
//
uint16_t sDataA[24];
uint8_t sTempDataA[24];
Uint16 g_HFCTAVG =0;
unsigned long g_TempHFCT = 0;
Uint32 g_SensTXCNT = 0;
Uint8 g_failCNT = 0;
Uint8 SendCnt = 10;
Uint8 AdcTestCnt = 0;
//
// Received data for SCI-A
//
uint16_t rDataA[6];
Uint8 g_trigger_on = 0;
Uint8 g_TriggerMode = 0;
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
// Map ISR functions
    Interrupt_register(INT_ADCA1, &adca1_isr);    // INT_CANB0 인터럽트 벡터에 CanbISR( ) 함수 연결
    Interrupt_register(INT_CANA0, &CanaISR);    // INT_CANB0 인터럽트 벡터에 CanbISR( ) 함수 연결
    Interrupt_register(INT_CANB0, &CanbISR);    // INT_CANB0 인터럽트 벡터에 CanbISR( ) 함수 연결
    Interrupt_register(INT_SCIA_RX, &sciaRXFIFOISR);
//    Interrupt_register(INT_SCIA_TX, &sciaTXFIFOISR);
    Interrupt_register(INT_TIMER0, &ISR_CpuTimer0);


    CpuTimer1Regs.PRD.all =  mSec1;     // task A
    CpuTimer2Regs.PRD.all =  mSec5;     // task B

    // Tasks State-machine init
    Alpha_State_Ptr = &Task_GroupA_main;
    A_Task_Ptr = &Task_GroupA1;             //check - 2
    B_Task_Ptr = &Task_GroupB1;


    InitCpuTimers();
//  ConfigCpuTimer(&CpuTimer1, 150, 20000);                                     // 50Hz for  1553b  communication
    ConfigCpuTimer(&CpuTimer1, 200, 10000);                                     // 100Hz for  1553b  communication

    ConfigCpuTimer(&CpuTimer0, 200, 1000/ISR_FREQUENCY);                        // 1000/1 = 1[Khz]
    ConfigCpuTimer(&CpuTimer2, 200, 1000/(ISR_FREQUENCY*2));                    // 2khz

    StartCpuTimer1();           //100[hz] - 1553B

    StartCpuTimer0();           //1[Khz] - ADC, RDC
    StartCpuTimer2();           //2[Khz] - CSU, CAN

#if( RUN_ON_TYPE == RUN_ON_FLASH)
//    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//    MemCopy(&Flash28_API_LoadStart, &Flash28_API_LoadEnd, &Flash28_API_RunStart);
//    InitFlash();
#endif
//
    InitCan();
    InitCanMessageBox();
        initSCIAFIFO();

//*****************************************************************************************************
// ADC 설정 관련
//*****************************************************************************************************
// Configure the ADC and power it up
        ConfigureADC();             //분해능을 여기서 설정

// Configure the ePWM
        ConfigureEPWM();            // ADC 샘플링 주기
// Setup the ADC for ePWM triggered conversions on channel 0
        SetupADCEpwm();             //여기서는 읽어욜 채널을 설정
//******************************************************************************************************

        //
// Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
//
//




//    IER |= M_INT1 | M_INT9; //Enable group 1 interrupts
    Interrupt_enable(INT_ADCA1);        //adca1 interrupt enabel

    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);
//
    Interrupt_enable(INT_CANA0);
    Interrupt_enable(INT_CANB0);
    Interrupt_enable(INT_SCIA_RX);
//    Interrupt_enable(INT_SCIA_TX);
    Interrupt_enable(INT_TIMER0);



//    CAN_startModule(CANA_BASE);
//    CAN_startModule(CANB_BASE);

//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
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

    g_auVTimer0[0] = 0;
    g_auVTimer1[0] = 0;
    g_auVTimer1[1] = 0;

    EINT;
    ERTM;

//*****************************************************************************************************
// ADC 설정 관련
//
// 여기서 메인 루틴이 시작 되지 건에 ADC 싱크를 동기화 시켜 준다.
//*****************************************************************************************************
// Sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    EDIS;
////
//// Take conversions indefinitely in loop
////
    do
    {
        g_uBackTicker++;

//      g_uSychPulse = READ_SYNCPULSE;

        // State machine entry & exit point
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
    } while(1);
}


///////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------//
//--------------------------------- FRAMEWORK -----------------------------------//
//-------------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////

void Task_GroupA_main(void)                 // loop rate synchronizer for A-tasks
{
    if(CpuTimer1Regs.TCR.bit.TIF == 1)
    {
        g_auVTimer0[0]++;                       // virtual timer 0, instance 0 (spare)
        CpuTimer1Regs.TCR.bit.TIF = 1;      // clear flag
        (*A_Task_Ptr)();                    // jump to an A Task
    }
    Alpha_State_Ptr = &Task_GroupB_main;    // Comment out to allow only A tasks
}

void Task_GroupB_main(void)                 // loop rate synchronizer for B-tasks
{
    if(CpuTimer2Regs.TCR.bit.TIF == 1)
    {
        g_auVTimer1[0]++;                       // virtual timer 1, instance 0 (spare)
        CpuTimer2Regs.TCR.bit.TIF = 1;      // clear flag
        (*B_Task_Ptr)();                    // jump to a B Task
    }
    Alpha_State_Ptr = &Task_GroupA_main;    // Allow C state tasks
}


///////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------//
//--------------------------------- OPERATING -----------------------------------//
//-------------------------------------------------------------------------------//
//////////////////////////
/////////////////////////////////////////////////////////
void Task_GroupA1(void)         // (executed in every 20 msec)
{

    A_Task_Ptr = &Task_GroupA1;

}
void Task_GroupB1(void)         // (executed in every 1msec)
{
    g_auVTimer1[1]++;

    Uint8 i = 0;
//        AFD_SensorMain();
//
        unsigned char CrcTemp = 0;

        g_uTXQueueCounter = GetTXSensorDataCount();

        if(GetTXSensorDataCount() > 9)
        {
            if(SciaRegs.SCICTL2.bit.TXRDY == 1 && SciaRegs.SCICTL2.bit.TXEMPTY == 1 )
            {

                if(g_ConnectTestSW==1 && g_SendStart == 1)
                {

                    if(g_SendValueCount > g_SensTXCNT)
                    {
//                        if(SendCnt>0)
//                        {
//                            g_TempHFCT += (unsigned long)g_stTXData_out.m_uHFCT;
//                            SendCnt--;
//                        }
//                        else
//                        {
//                            SendCnt=10;
//                            g_HFCTAVG = (unsigned long)g_TempHFCT / (unsigned long)SendCnt;
//                            g_TempHFCT = 0;
//                        }

                        g_failCNT=0;

                        sTempDataA[0] = 0xfe;
                        sTempDataA[1] = 0x10;
                        for(i = 1; i < 11 ; i++)
                        {
                            GetTXSensorData(&g_stTXData_out);
                            sTempDataA[i*2] =  (g_stTXData_out.m_uHFCT >> 8);
                            sTempDataA[(i*2)+1] = g_stTXData_out.m_uHFCT;
                        }

                        sTempDataA[22] = 0x00ff & (g_SensTXCNT % 256);
                        sTempDataA[23] = 0xa5;

                        g_SensTXCNT++;
                    }
                    else
                    {
                        g_SendStart = 0;
                        g_SensTXCNT=0;
                        sTempDataA[0] = 0xfe;
                        sTempDataA[1] = 0x07;
                        for(i = 2; i < 22 ; i++)
                        {

                            sTempDataA[i] =  0x00;

                        }
                        sTempDataA[22] = 0x00;
                        sTempDataA[23] = 0xa5;

                    }

//                    CrcTemp = CRC8_BlockChecksum(sTempDataA,6);
//
                    int i;

                    for(i = 0; i < 24; i++)
                    {
                      sDataA[i]= 0x00ff & sTempDataA[i];
                    }
//
//                    sDataA[10]= 0x00ff & CrcTemp;
                    SCI_writeCharArray(SCIA_BASE, sDataA, 24);
        //            SCI_writeCharArray(SCIA_BASE, sDataA, 8);

                }
//                else
//                {
//                    g_failCNT++;
//
//                }


//                    SciaRegs.SCIFFTX.bit.SCIFFENA = 0;

//            SciaRegs.SCIFFTX.bit.SCIFFENA = 1;

            }
        }
//    DSP_SCI_TXEN_ON();
//  scia_xmit(0x5a);
//    CSU_Sensor_main();
//    CSU_StateManagement_main();



//    DSP_SCI_TXEN_OFF();


    B_Task_Ptr = &Task_GroupB2;
}
void Task_GroupB2(void) //  (executed in every 2msec)
{
    g_auVTimer1[2]++;
    if((g_auVTimer1[2] & 0x0001) == 0x0)
    {
 //       CSU_TJ_COMMUNICATION_main();

//        FAULT_LED_TOGGLE(); //2ms

    }
    B_Task_Ptr = &Task_GroupB1;
}

interrupt void ISR_CpuTimer0(void) // 1msec
{

    resultsIndex=0;
    CpuTimer0.InterruptCount++;


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
//
// initSCIAFIFO - Configure SCIA FIFO
//
void initSCIAFIFO()
{
    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 115200.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 460800, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIA_BASE);
    //SCI_enableLoopback(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_enableInterrupt(SCIA_BASE, (SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_RXRDY_BRKDT));
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXERR);

    //
    // The transmit FIFO generates an interrupt when FIFO status
    // bits are less than or equal to 2 out of 16 words
    // The receive FIFO generates an interrupt when FIFO status
    // bits are greater than equal to 2 out of 16 words
    //
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX12, SCI_FIFO_RX6);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
}




//
// error - Function to halt debugger on error
//
void error(void)
{
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

void SendHanShaking(void)
{
    int i;
    unsigned char CrcTemp = 0;

    sTempDataA[0] = 0xfe;
    sTempDataA[1] = 0x01;
    for(i = 2; i < 22 ; i++)
    {
        sTempDataA[i] =  0x00;

    }
    sTempDataA[22] = 0x00;
    sTempDataA[23] = 0xa5;


//    CrcTemp = CRC8_BlockChecksum(sTempDataA,12);
//
    for(i = 0; i < 24; i++)
    {
        sDataA[i]= 0x00ff & sTempDataA[i];
    }
//
//    sDataA[10]= 0x00ff & CrcTemp;

    SCI_writeCharArray(SCIA_BASE, sDataA, 24);

}



void SendCloseCom(void)
{
    int i;
    unsigned char CrcTemp = 0;

    sTempDataA[0] = 0xfe;
    sTempDataA[1] = 0x05;
    for(i = 2; i < 22 ; i++)
    {
        sTempDataA[i] =  0x00;

    }
    sTempDataA[22] = 0x00;
    sTempDataA[23] = 0xa5;


//    CrcTemp = CRC8_BlockChecksum(sTempDataA,12);

    for(i = 0; i < 24; i++)
    {
        sDataA[i]= 0x00ff & sTempDataA[i];
    }
//
//    sDataA[10]= 0x00ff & CrcTemp;

    SCI_writeCharArray(SCIA_BASE, sDataA, 24);

}


void DataSendModeStart(Uint8 count)
{
    g_SendValueCount = (Uint32)count * 1000;
    g_SendStart = 1;

}

void DataSendWithTriggerModeStart(Uint8 count)
{
    g_SendValueCount = (Uint32)count * 1000;
    g_SendStart = 1;
    g_TriggerMode = 1;
    g_trigger_on = 0;
    AdcTestCnt=0;
}

Uint8 g_remainCNT = 0;
Uint8 g_remainValue[12];
Uint8 g_testTemp[12];
interrupt void sciaRXFIFOISR(void)
{

    SCI_readCharArray(SCIA_BASE, rDataA, 6);
    Uint8 datasize=6;
    Uint8 count=0;
    Uint8 i;
    //
    // Check received data
    //
    if (g_remainCNT != 0)
    {
        for (i = 0; i < g_remainCNT; i++)
        {
            g_testTemp[i] = g_remainValue[i];
        }

        for (i = 0; i < datasize; i++)
        {
            g_testTemp[g_remainCNT + i] = rDataA[i];
        }
    }
    else
    {
        for (i = 0; i < datasize; i++)
        {
            g_testTemp[i] = rDataA[i];
        }
    }

    datasize = datasize + g_remainCNT;
    while((datasize - count) > 5)
    {
        if(g_testTemp[count] == 0xfd && g_testTemp[count+5]==0xa5){
              switch(g_testTemp[count+1])
              {
              case 0x00:
                  SendHanShaking();
                  g_ConnectTestSW = 1;
                  break;
              case 0x02:
                  DataSendModeStart(g_testTemp[count+3]);
                  break;
              case 0x03:
                  DataSendWithTriggerModeStart(rDataA[3]);
                  break;
              case 0x04:
      //            DataSendWithTriggerModeStop();
                  break;
              case 0x05:
                  SendCloseCom();
                  g_ConnectTestSW = 0;
                  break;

              }
              count += 5;
          }
        else
        {
            count += 1;
        }


        if (datasize - count != 0)
        {
            g_remainCNT = datasize - count;
            for (i = 0; i < g_remainCNT; i++)
            {
                g_remainValue[i] = g_testTemp[count + i];
            }
        }
    }


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

    SCI_writeCharArray(SCIA_BASE, sDataA, 8);

    //
    // Increment send data for next cycle
    //
    //for(i = 0; i < 2; i++)
  //  {
    //    sDataA[i] = (sDataA[i] + 1) & 0x00FF;
  //  }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

    //
    // Issue PIE ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//**************************************************************************************
// adca1_isr - Read ADC Buffer in ISR
//
//  ADC 입력 신호
//
//
//**************************************************************************************
Uint8 adcSampleCNT = 0;
interrupt void adca1_isr(void)
{

//    FAULT_LED_TOGGLE();

//    AdcaResults[resultsIndex][0] = AdcaResultRegs.ADCRESULT0;   //P2
//    AdcaResults[resultsIndex][1] = AdcaResultRegs.ADCRESULT1;   //P3
//    AdcaResults[resultsIndex][2] += AdcaResultRegs.ADCRESULT2;   //P4
//    resultsIndex++;
//
//    g_stADCData.m_uShunt = AdcaResults[resultsIndex][0];
//    g_stADCData.m_uHFCT = AdcaResults[resultsIndex][2];

//        AdcaResults[resultsIndex][0] = AdcaResultRegs.ADCRESULT0;   //P2
//        AdcaResults[resultsIndex][1] = AdcaResultRegs.ADCRESULT1;   //P3
//        AdcaResults[resultsIndex][2] = AdcaResultRegs.ADCRESULT2;   //P4
//        resultsIndex++;
//    FAULT_LED_TOGGLE();
        g_stADCData.m_uShunt = AdcaResultRegs.ADCRESULT0;
        g_stADCData.m_uHFCT = AdcaResultRegs.ADCRESULT2;


    adcSampleCNT++;
    if(adcSampleCNT>9)
    {
            FAULT_LED_TOGGLE();
        adcSampleCNT=0;
//    SetADCSensorData(&g_stADCData);
        if(GPIO_readPin(0) && g_trigger_on==0)
        {
            g_trigger_on=1;
        }


        if(g_TriggerMode == 1)
        {
            if(g_SendStart == 1 && g_trigger_on == 1)
            {


                    g_stADCData.m_Count = AdcTestCnt;
                    SetTXSensorData(&g_stADCData);

                    AdcTestCnt++;

            }

        }
        else
        {

            if(g_SendStart == 1)
            {

                    FAULT_LED_TOGGLE();
                    g_stADCData.m_Count = AdcTestCnt;
                    SetTXSensorData(&g_stADCData);
                    AdcTestCnt++;
            }
        }


    }

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
    //현재 프로세스 1.5us
}


interrupt void CanaISR(void)
{
    uint32_t status;

    // 인터럽트 발생 원인을 파악하기 위해 CAN-B 모듈의 인터럽트 상태(Interrupt Status) 확인
    status = CAN_getInterruptCause(CANA_BASE);


    // (메시지 수신이 아닌) CAN 컨트롤러 상태 변화에 따른 인터럽트 발생이라면, 해당 상태 확인
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // 컨트롤러 상태 확인 (Read the CAN controller status)
        // 본 예제에는 코드 복잡성을 피하기 위해 각 오류나 상태 변화에 따른 대응 코드는 포함되어 있지 않음
        status = CAN_getStatus(CANA_BASE);

        // 오류(Error)가 발생했는지 확인
        if( ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            // 오류가 발생했음을 알리기 위해 Flag 변수를 1로 설정
            errorFlag = 1;
        }
    }
    // CAN-B 모듈 수신 메시지 오브젝트에 의한 인터럽트일 경우, 수신된 데이터 확인
    else if(status == RX_MSG_OBJ_ID)
    {
        // 수신된 데이터 확인
        CAN_readMessage(CANA_BASE, RX_MSG_OBJ_ID, rxMsgDataA);



        // CAN 메시지 오브젝트 인터럽트 클리어
        CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);

        // 수신 데이터 갯수 파악용 카운터 변수 증가
        rxMsgCount++;

        // 메시지가 수신되었으므로, 오류 Flag Clear
        errorFlag = 0;
    }
    // 예상되지 않은 일로 인터럽트 발생 시, 이곳에서 처리
    else
    {
        asm("  NOP"); // No Operation
    }


    // CAN 전역(Global) 인터럽트 Flag Clear
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    // INT_CANB0 인터럽트 벡터가 포함된 CPU 인터럽트 확장그룹 9번의 Acknowledge 비트 클리어
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


interrupt void CanbISR(void)
{

    uint32_t status;

    // 인터럽트 발생 원인을 파악하기 위해 CAN-B 모듈의 인터럽트 상태(Interrupt Status) 확인
    status = CAN_getInterruptCause(CANB_BASE);

    // (메시지 수신이 아닌) CAN 컨트롤러 상태 변화에 따른 인터럽트 발생이라면, 해당 상태 확인
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // 컨트롤러 상태 확인 (Read the CAN controller status)
        // 본 예제에는 코드 복잡성을 피하기 위해 각 오류나 상태 변화에 따른 대응 코드는 포함되어 있지 않음
        status = CAN_getStatus(CANB_BASE);

        // 오류(Error)가 발생했는지 확인
        if( ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) && ((status & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            // 오류가 발생했음을 알리기 위해 Flag 변수를 1로 설정
            errorFlag = 1;
        }
    }
    // CAN-B 모듈 수신 메시지 오브젝트에 의한 인터럽트일 경우, 수신된 데이터 확인
    else if(status == RX_MSG_OBJ_ID)
    {
        // 수신된 데이터 확인
        CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, rxMsgDataB);

        // CAN 메시지 오브젝트 인터럽트 클리어
        CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID);

        // 수신 데이터 갯수 파악용 카운터 변수 증가
        rxMsgCount++;

        // 메시지가 수신되었으므로, 오류 Flag Clear
        errorFlag = 0;
    }
    // 예상되지 않은 일로 인터럽트 발생 시, 이곳에서 처리
    else
    {
        asm("  NOP"); // No Operation
    }

    // CAN 전역(Global) 인터럽트 Flag Clear
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    // INT_CANB0 인터럽트 벡터가 포함된 CPU 인터럽트 확장그룹 9번의 Acknowledge 비트 클리어
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// End of file
//

