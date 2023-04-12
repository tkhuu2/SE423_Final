//Comments made marked by TK or AJS
//#############################################################################
// FILE:   LAB6_main.c
//
// TITLE:  Lab 6 main
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI1_HighestPriority(void);
__interrupt void SWI2_MiddlePriority(void);
__interrupt void SWI3_LowestPriority(void);
__interrupt void ADCC_ISR(void); //ADCC hardware ISR AJS
__interrupt void SPIB_isr(void); //SPI interrupt service routine AJS

//SPIB setup predefinition AJS
void setupSpib(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint32_t adcc_flags = 0; //counter for ADCC ISR AJS
uint32_t SPIB_flags = 0; //counter for SPIB ISR AJS

//-------------LPR VARIABLES-----------------------//
float adcc2VOLT = 0.0; //voltage from ADCINC2 AJS
float adcc3VOLT = 0.0; //voltage from ADCINC3 AJS
float adcc4VOLT = 0.0; //voltage from ADCINC4 AJS
float adcc5VOLT = 0.0; //voltage from ADCINC5 AJS
uint16_t adcc2result = 0; //raw 12 bit data from ADCINC2 AJS
uint16_t adcc3result = 0; //raw 12 bit data from ADCINC3 AJS
uint16_t adcc4result = 0; //raw 12 bit data from ADCINC4 AJS
uint16_t adcc5result = 0; //raw 12 bit data from ADCINC5 AJS
float fineZ = 0.0; //deg/s for 4Z AJS
float fineX = 0.0; //deg/s for 4X AJS
float roughZ = 0.0; //deg/s for Z AJS
float roughX = 0.0; //deg/s for X AJS
float sum4Z = 0.0; //sum of 4Z data over two seconds AJS
float sumZ = 0.0; //sum of Z data over two seconds AJS
float sum4X = 0.0; //sum of 4X data over two seconds AJS
float sumX = 0.0; //sum of X data over two seconds AJS
float bearingZ = 0.0;
float bearing4Z = 0.0;
float bearingZold = 0.0;
float bearing4Zold = 0.0;
float fineZold = 0.0;
float roughZold = 0.0;

//------------------MPU VARIABLES--------------------------------//
int16_t accX_raw = 0; //Raw data from accelerometers AJS
int16_t accY_raw = 0;
int16_t accZ_raw = 0;
int16_t temp_raw = 0; //Raw data from temperature sensor AJS
int16_t gyroX_raw = 0; //Raw data from gyroscopes AJS
int16_t gyroY_raw = 0;
int16_t gyroZ_raw = 0;
float sum_gyroX = 0.0;
float sum_gyroY = 0.0;
float sum_gyroZ = 0.0;
float accX = 0.0; //Scaled data from accelerometers AJS
float accY = 0.0;
float accZ = 0.0;
float temp = 0.0; //Scaled data from temperature sensor AJS
float gyroX = 0.0; //Scaled data from gyroscopes AJS
float gyroY = 0.0;
float gyroZ = 0.0;
float gyroXold = 0.0;
float gyroYold = 0.0;
float gyroZold = 0.0;
float worldX = 0.0;
float worldY = 0.0;
float worldXold = 0.0;
float worldYold = 0.0;

// eQEP / Encoder function initializations  TK
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);

//----------------MOTOR AND PI CONTROL VARIABLES----------------//
//global float variables for wheel values   TK
float LeftWheel = 0.0;
float RightWheel = 0.0;
float EncWheel = 0.0;
//control outputs to PWM channels 1A and 2A TK
float uLeft = 0.0;
float uRight = 0.0;
//old position of wheels    TK
float pRight_old = 0.0;
float pLeft_old = 0.0;
//velocity value of wheels  TK
float vRight = 0.0;     //right TK
float vLeft = 0.0;     //left  TK
//values for positive and negative coulomb friction TK
float Cpos = 2.3744 * 0.6;
float Cneg = -1.8744 * 0.6;
//values for positive and negative viscous friction TK
float Vpos = 1.9527 * 0.6;
float Vneg = 2.2062 * 0.6;

float Vref = 1.0;
float Kp = 3.0;
float Ki = 25.0;

float IKLeft = 0.0;
float IKLeftprev = 0.0;
float errKLeft = 0.0;
float errKLeftprev = 0.0;

float IKRight = 0.0;
float IKRightprev = 0.0;
float errKRight = 0.0;
float errKRightprev = 0.0;

float uRight_bar = 0.0;
float uLeft_bar = 0.0;

float e_steer = 0.0;
float Kp_turn = 3.0;
float turn = 0.0;

//--------------------LINUX COMMAND INITS----------------------------//
float ref_right_wall = 1.25; //distance in feet to keep from wall on right
float left_turn_Start_threshold = 1.75; //distance in front to initiate left turn
float left_turn_Stop_threshold = 4.0; //distance in front to end left turn
float Kp_right_wall = -1.5; //right wall gain post-error
float Kp_front_wall = -0.6; //front wall gain post-error
float front_turn_velocity = 0.4; //velocity when obstacle front recognized
float forward_velocity = 1.0; //velocity when normal right-wall following
float turn_command_saturation = 1.0; //maximum turn command to prevent spinning
float right_turn_Start_threshold = 2.0; //distance behind to initiate right turn
uint16_t right_wall_follow_state = 2; //switch case state

//--------------------LADAR VARIABLES/INITS---------------------------//
uint32_t timecount = 0;

extern datapts ladar_data[228]; //distance data from LADAR

extern float printLV1;
extern float printLV2;

extern float LADARrightfront;
extern float LADARfront;
extern float LADARrightrear;

extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern char G_command[]; //command for getting distance -120 to 120 degree
extern uint16_t G_len; //length of command
extern xy ladar_pts[228]; //xy data

extern uint16_t LADARpingpong;
extern float LADARxoffset;
extern float LADARyoffset;

extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

extern uint16_t NewLVData;

uint16_t LADARi = 0;
pose ROBOTps = {0,0,0}; //robot position
pose LADARps = {3.5/12.0,0,1};  // 3.5/12 for front mounting, theta is not used in this current code
float printLinux1 = 0;
float printLinux2 = 0;

void PostSWI1(void);
void PostSWI3(void);

void setEPWM1A(float controleffort) { //If the value passed is greater than 10 set it to 10.    TK
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {  //If the value passed is less than -10 set it to -10.   TK
        controleffort = -10;
    }
    EPwm1Regs.CMPA.bit.CMPA = 125 * controleffort + 1250;
}

void setEPWM2A(float controleffort) { //If the value passed is greater than 10 set it to 10.    TK
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {  //If the value passed is less than -10 set it to -10.   TK
        controleffort = -10;
    }
    EPwm2Regs.CMPA.bit.CMPA = 125 * controleffort + 1250;
}
//------------------END INITIALIZATIONS-------------------------------//

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LS7366#2 CS
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    //PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);  //Set GPIO0�s pin function to EPWM1A    TK
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //Set GPIO2�s pin function to EPWM2A.    TK

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI1_HighestPriority;
    PieVectTable.RAM_CORRECTABLE_ERROR_INT = &SWI2_MiddlePriority;
    PieVectTable.FLASH_CORRECTABLE_ERROR_INT = &SWI3_LowestPriority;

    PieVectTable.ADCC1_INT = &ADCC_ISR; //Connect ADCC1_INT to our ADCC ISR function AJS
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    DELAY_US(1000000);  // Delay 1 second giving LADAR Time to power on after system power on

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
    init_serialSCIC(&SerialC,19200);
    init_serialSCID(&SerialD,2083332);

    for (LADARi = 0; LADARi < 228; LADARi++) {
        ladar_data[LADARi].angle = ((3*LADARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
    }

    setupSpib();

    ///////////////////////////////////
    //Do the same settings but for EPWM1 and EPWM2
    //Both will be used to drive the robot�s DC motors through an H-bridge IC   TK
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm1Regs.TBCTL.bit.PHSEN = 0;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 1;
    EPwm1Regs.AQCTLA.bit.ZRO = 2;
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    ///////////////////////////////////
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    init_eQEPs();

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;  // For EPWM1A   TK
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;  // For EPWM2A   TK
    EDIS;

    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (�pulse� is the same as �trigger�)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for ADCC
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag

    //ADCC
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4; //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5; //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6; //SPIB AJS
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable ADCC in the PIE: Group 1 interrupt 3 AJS
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;
    // Enable SPI in the PIE: Group 6, interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1; //SWI1
    PieCtrlRegs.PIEIER12.bit.INTx10 = 1; //SWI2
    PieCtrlRegs.PIEIER12.bit.INTx11 = 1; //SWI3  Lowest priority

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    char S_command[19] = "S1152000124000\n";//this changes the baud rate to 115200
    uint16_t S_len = 19;    // Set S_len to the length of S_command array   TK
    serial_sendSCIC(&SerialC, S_command, S_len);    // Send the S_command array to the serial communication using the serial_sendSCIC function  TK

    DELAY_US(1000000);  // Delay letting LADAR change its Baud rate
    init_serialSCIC(&SerialC,115200);

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        // Print values of the motor/car to Tera Term   TK
        // Print the values of printLV1 and printLV2 TK
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"lpZ: %.2f/aX: %.2f/aY: %.2f/mpZ: %.2f/vR: %.2f/vL: %.2f/L1: %.2f/L2 %.2f/LF: %.2f/LRF: %.2f\r\n",fineZ,accX,accY,gyroZ,vRight,vLeft,printLV1,printLV2,LADARfront,LADARrightfront);
            UART_printfLine(1,"LV1: %.2f", printLV1);
            UART_printfLine(2,"LV2: %.2f", printLV2);
            UARTPrint = 0;
        }
    }
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%5) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    serial_sendSCIC(&SerialC, G_command, G_len);
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        //UARTPrint = 1;
    }
}

__interrupt void ADCC_ISR (void) {
    /////////////////////BEGIN GYRO CONTROLS/////////////////////////////////////////
    adcc2result = AdccResultRegs.ADCRESULT0; //rough X reading AJS
    adcc3result = AdccResultRegs.ADCRESULT1; //fine X reading AJS
    adcc4result = AdccResultRegs.ADCRESULT2; //fine Z reading AJS
    adcc5result = AdccResultRegs.ADCRESULT3; //rough Z reading AJS

    adcc2VOLT = adcc2result * (3.0/4095.0); //rough X AJS
    adcc3VOLT = adcc3result * (3.0/4095.0); //fine X AJS
    adcc4VOLT = adcc4result * (3.0/4095.0); //fine Z AJS
    adcc5VOLT = adcc5result * (3.0/4095.0); //rough Z AJS
    //----------------------SPIB START TRANSMISSION----------------------------------//
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; //slave select low AJS
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Interrupt at 8 values in FIFO AJS
    //Send garbage to required addresses in order to receive data in SPIB interrupt AJS
    SpibRegs.SPITXBUF = (0x8000 | 0x3A00); //Start at address 58 (INT_STATUS) for write and RX discard AJS
    SpibRegs.SPITXBUF = 0x0000; //59 and 60, Accel X high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //61 and 62, Accel Y high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //63 and 64, Accel Z high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //65 and 66, Temp high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //67 and 68, Gyro X high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //69 and 70, Gyro Y high and low AJS
    SpibRegs.SPITXBUF = 0x0000; //71 and 72, Gyro Z high and low AJS

    ///////////////////////////////////////END MOTOR CONTROLS//////////////////////////////////

    adcc_flags++;   //increment counter  TK

    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag   TK
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void SPIB_isr(void){
    int16_t trash = 0;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.

    /*spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
    gyroZ_raw = spivalue2;*/

    //------------------READ SPI RX BUFFER----------------------//
    trash = SpibRegs.SPIRXBUF; //read INT_STATUS garbage AJS
    accX_raw = SpibRegs.SPIRXBUF; //read raw data from accelerometers AJS
    accY_raw = SpibRegs.SPIRXBUF;
    accZ_raw = SpibRegs.SPIRXBUF;
    temp_raw = SpibRegs.SPIRXBUF; //read raw data from temperature sensor AJS
    gyroX_raw = SpibRegs.SPIRXBUF; //read raw data from gyroscopes AJS
    gyroY_raw = SpibRegs.SPIRXBUF;
    gyroZ_raw = SpibRegs.SPIRXBUF;

    PostSWI1();

    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

    SPIB_flags ++;

    if ((SPIB_flags % 100) == 0){
        UARTPrint = 1;
    }
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t trash = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Also don�t forget to cut and
    //paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    //-----------------------------------------------------------------------------------------------------------------
    //SPI settings AJS
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 50; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK�s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 0 spi clocks.
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes
    //interrupt. This is just the initial setting for the register. Will be changed below

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00 X gyro offset high
    SpibRegs.SPITXBUF = 0x1300;
    // To address 00x14 write 0x00 X gyro offset low
    // To address 00x15 write 0x00 Y gyro offset high
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x16 write 0x00 Y gyro offset low
    // To address 00x17 write 0x00 Z gyro offset high
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x18 write 0x00 Z gyro offset low
    // To address 00x19 write 0x13 SMPRLT DIV (set to 19) AJS
    SpibRegs.SPITXBUF = 0x0013;
    // To address 00x1A write 0x02 CONFIG (set bandwidth to 92 Hz) AJS
    // To address 00x1B write 0x00 GYRO CONFIG
    SpibRegs.SPITXBUF = 0x0200;
    // To address 00x1C write 0x08 ACCEL CONFIG (set full scale to 4 g) AJS
    // To address 00x1D write 0x06 ACCEL CONFIG 2 (set bandwidth to 5 Hz) AJS
    SpibRegs.SPITXBUF = 0x0806;
    // To address 00x1E write 0x00 LP ACCEL ODR
    // To address 00x1F write 0x00 WOM THR
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    //Clear RX buffer AJS
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00 FIFO EN
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40 I2C MST CTRL
    // To address 00x25 write 0x8C I2C SLV0 ADDR
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02 I2C SLV0 REG
    // To address 00x27 write 0x88 I2C SLV0 CTRL
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C I2C SLV1 ADDR
    // To address 00x29 write 0x0A I2C SLV1 REG
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    //Clear RX buffer AJS
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    trash = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81 I2C SLV1 CTRL
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800 INT ENABLE
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00 INT STATUS
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400 I2C SLV1 DO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700 I2C MST DELAY CTRL
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00 USER CTRL
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00 PWR MGMT 1
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500 WHO AM I
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700 XA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7800 | 0x0034); // 0x7800 L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7A00 | 0x0014); // 0x7A00 YA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7B00 | 0x0094); // 0x7B00 L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7D00 | 0x0017); // 0x7D00 ZA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    SpibRegs.SPITXBUF = (0x7E00 | 0x0018); // 0x7E00 L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    trash = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
//sets the interrupt priority, enables interrupt handling, and initializes a counter variable   TK
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    uint16_t i = 0;//for loop

    //-------------------------POWER ON BREAK----------------------------------------//
    //first two seconds: do nothing, let robot stabilize AJS
    if (timecount <= 2000) {
        setEPWM2A(0); //freeze motors AJS
        setEPWM1A(0);
    }
    //from two seconds to four seconds: sum up gyro readings to be divided in next stage AJS
    else if ((timecount > 2000) && (timecount <= 4000)) {
        //        sumX += adcc2VOLT;
        //        sum4X += adcc3VOLT;
        //        sum4Z += adcc4VOLT;
        //        sumZ += adcc5VOLT;

        sum_gyroX += gyroX_raw * 250.0 / 32767.0;
        sum_gyroY += gyroY_raw * 250.0 / 32767.0;
        sum_gyroZ += gyroZ_raw * 250.0 / 32767.0;

        setEPWM2A(0); //freeze motors AJS
        setEPWM1A(0);
    }
    //-------------------------POWER ON BREAK----------------------------------------//
    //set zero adjustments for each gyro reading by dividing it by 2000 clocks AJS
    else if (timecount == 4001) {
        //        sumX /= 2000;
        //        sum4X /= 2000;
        //        sum4Z /= 2000;
        //        sumZ /= 2000;

        sum_gyroX /= 2000.0;
        sum_gyroY /= 2000.0;
        sum_gyroZ /= 2000.0;
    }
    //from that point on subtract the zero adjustment for each reading AJS
    else {
        //LPR
        //        roughZold = roughZ;
        //        fineZold = fineZ;
        //        //Take each of the voltage values and convert them to their corresponding angular velocity AJS
        //        roughX = (adcc2VOLT - sumX) * 400;
        //        //rough voltage range: 0.23V = -400 deg/s :: 1.23V = 0 deg/s :: 2.23V = 400 deg/s AJS
        //        fineX = (adcc3VOLT - sum4X) * 100;
        //        //fine voltage range: 0.23V = -100 deg/s :: 1.23V = 0 deg/s :: 2.23V = 100 deg/s AJS
        //        fineZ = (adcc4VOLT - sum4Z) * 100;
        //        roughZ = (adcc5VOLT - sumZ) * 400;

        //MPU
        gyroXold = gyroX;
        gyroYold = gyroY;
        gyroZold = gyroZ;
        accX = accX_raw * 4.0 / 32767.0; //scale accelerometer data (maximum 4 g) AJS
        accY = accY_raw * 4.0 / 32767.0;
        accZ = accZ_raw * 4.0 / 32767.0;
        gyroX = ((gyroX_raw) * 250.0 / 32767.0 - sum_gyroX) * PI / 180; //scale gyroscope data (maximum 250 deg/s) AJS
        gyroY = ((gyroY_raw) * 250.0 / 32767.0 - sum_gyroY) * PI / 180;
        gyroZ = ((gyroZ_raw) * 250.0 / 32767.0 - sum_gyroZ) * PI / 180;

        ////////////////////////////BEGIN RIGHT WALL FOLLOW//////////////////////////////////
        // inside SWI1 before PI speed control

        // State machine for right wall following   TK
        switch (right_wall_follow_state) {
        case 1:
            //State 1 - Left Turn   TK
            // Calculate turn command and velocity reference    TK
            turn = Kp_front_wall * (14.5 - LADARfront);
            Vref = front_turn_velocity;
            // Check if robot has completed left turn and transition to next state  TK
            if (LADARfront > left_turn_Stop_threshold) {
                right_wall_follow_state = 2;
            }
            break;
        case 2:
            //State 2 - Right Wall Follow   TK
            // Calculate turn command and velocity reference    TK
            if (fabs(ref_right_wall - LADARrightfront) > 2){
                turn = Kp_right_wall * (ref_right_wall - LADARrightrear);
                Vref = forward_velocity;
            }
            else {
                turn = Kp_right_wall * (ref_right_wall - LADARrightfront);
                Vref = forward_velocity;
            }
            // Check if robot has reached a point where it should turn left again and transition to next state  TK
            if (LADARfront < left_turn_Start_threshold) {
                right_wall_follow_state = 1;
            }
            break;
        }
        if (turn > turn_command_saturation){ //saturate turn command if greater than sat value AJS
            turn = turn_command_saturation;
        }
        else if (turn < -turn_command_saturation){ //saturate turn command if less than -sat value AJS
            turn = -turn_command_saturation;
        }
        ////////////////////////////END RIGHT WALL FOLLOW////////////////////////////////////

        ////////////////////////////BEGIN MOTOR CONTROLS/////////////////////////////////////
        //call the read functions and assign their
        //return values to the corresponding variables  TK
        LeftWheel = readEncLeft();
        RightWheel = readEncRight();
        EncWheel = readEncWheel();
        //calculate current position of wheels  TK
        float pRight_current = RightWheel / 9.592;
        float pLeft_current = LeftWheel / 9.592;
        //calculates the velocity of each wheel TK
        vRight = (pRight_current - pRight_old) / 0.001;
        vLeft = (pLeft_current - pLeft_old) / 0.001;
        //updates the position of the robot  TK
        pRight_old = pRight_current;
        pLeft_old = pLeft_current;

        //turn = EncWheel / 20;
        //calculate the steering error  TK
        e_steer = vRight + turn - vLeft;

        //PID control for left wheel    TK
        errKLeftprev = errKLeft;
        errKLeft = Vref - vLeft + e_steer * Kp_turn;
        IKLeftprev = IKLeft;
        IKLeft = IKLeftprev + (errKLeftprev + errKLeft)/2 * 0.001;

        //PID control for right wheel   TK
        errKRightprev = errKRight;
        errKRight = Vref - vRight - e_steer * Kp_turn;
        IKRightprev = IKRight;
        IKRight = IKRightprev + (errKRightprev + errKRight)/2 * 0.001;

        //adjusted left and right wheel movement    TK
        uRight_bar = errKRight * Kp + IKRight * Ki;
        uLeft_bar = errKLeft * Kp + IKLeft * Ki;

        //check for integral windup TK
        if(fabs(uRight_bar) > 10) {
            IKRight = IKRightprev;
        }

        if(fabs(uLeft_bar) > 10) {
            IKLeft = IKLeftprev;
        }

        //Limit uLeft and uRight to a value between �10 and 10  TK
        //    if (EncWheel > 10) {
        //        uLeft = 10;
        //        uRight = 10;
        //    }
        //    if (EncWheel < -10) {
        //        uLeft = -10;
        //        uRight = -10;
        //    }
        /*else {
             uLeft = EncWheel;    //Set both uLeft and uRight equal to EncWheel   TK
             uRight = EncWheel;
         }*/

        // Only friction compensation is activated for this section.
        // The easiest way to do this is to assign uLeft and uRight to 0
        // instead of assigning uLeft and uRight the value of EncWheel      TK

        // friction compensation algorithm  TK
        // Calculate control input for right motor  TK
        if (vRight > 0.0) {
            uRight = uRight_bar + Vpos * vRight + Cpos;
        }
        else {
            uRight = uRight_bar + Vneg * vRight + Cneg;
        }
        // Calculate control input for left motor   TK
        if (vLeft > 0.0) {
            uLeft = uLeft_bar + Vpos * vLeft + Cpos;
        }
        else {
            uLeft = uLeft_bar + Vneg * vLeft + Cneg;
        }
        // Set motor controls TK
        setEPWM2A(-uRight);
        setEPWM1A(uLeft);

        //        bearingZold = bearingZ;
        //        bearing4Zold = bearing4Z;
        //
        //        bearingZ = bearingZold + (roughZ + roughZold) / 2 * 0.001;
        //        bearing4Z = bearing4Zold + (fineZ + fineZold) / 2 * 0.001;

        // Update robot heading using the average of the old and new gyro Z values  TK
        ROBOTps.theta += (gyroZold + gyroZ) / 2 * 0.001;

        //worldXold = worldX;
        //worldYold = worldY;

        // Update robot position using the average of the left and right wheel velocities,
        // multiplied by the cosine and sine of the current heading, respectively, and
        // the time elapsed (0.001 seconds) TK
        ROBOTps.x += (vLeft + vRight) / 2 * cosf(ROBOTps.theta) * 0.001;
        ROBOTps.y += (vLeft + vRight) / 2 * sinf(ROBOTps.theta) * 0.001;
    }


    /////////////////////////////END GYRO CONTROLS///////////////////////////////////////

    if (newLinuxCommands == 1) {
        newLinuxCommands = 0;
        //Vref = LinuxCommands[0];
        //turn = LinuxCommands[1];
        ref_right_wall = LinuxCommands[2];
        left_turn_Start_threshold = LinuxCommands[3];
        left_turn_Stop_threshold = LinuxCommands[4];
        Kp_right_wall = LinuxCommands[5];
        Kp_front_wall = LinuxCommands[6];
        front_turn_velocity = LinuxCommands[7];
        forward_velocity = LinuxCommands[8];
        turn_command_saturation = LinuxCommands[9];
        right_turn_Start_threshold = LinuxCommands[10];
    }

    if (NewLVData == 1) {
        NewLVData = 0;
        printLV1 = fromLVvalues[0];
        printLV2 = fromLVvalues[1];
    }

    if((timecount%250) == 0) {
        DataToLabView.floatData[0] = ROBOTps.x;
        DataToLabView.floatData[1] = ROBOTps.y;
        DataToLabView.floatData[2] = (float)timecount;
        DataToLabView.floatData[3] = ROBOTps.theta;
        DataToLabView.floatData[4] = ROBOTps.theta;
        DataToLabView.floatData[5] = ROBOTps.theta;
        DataToLabView.floatData[6] = ROBOTps.theta;
        DataToLabView.floatData[7] = ROBOTps.theta;
        LVsenddata[0] = '*';  // header for LVdata
        LVsenddata[1] = '$';
        for (i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }

    timecount++;

    //##############################################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
}

//
// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
//
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        LADARrightrear = 19;
        for (LADARi = 11; LADARi <= 15 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightrear) {
                LADARrightrear = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        LADARrightrear = 19;
        for (LADARi = 11; LADARi <= 15 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightrear) {
                LADARrightrear = ladar_data[LADARi].distance_pong;
            }
            LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
            LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
            for (LADARi = 0; LADARi < 228; LADARi++) {

                ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
                ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

            }
        }

    }
    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
//
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1; // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(-PI/20000));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(PI/20000));
}

float readEncWheel(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep3Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/4000));
}

void PostSWI1(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI1
}

void PostSWI3(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1; // Manually cause the interrupt for the SWI3
}
