//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.570796326794896619231321691639
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
//exercise 1.4, predefinition for ADCD interrupt function
__interrupt void ADCD_ISR(void);
//exercise 3, predefinition for ADCA interrupt function
__interrupt void ADCA_ISR(void);
//exercise 4, predefinition for ADCB interrupt function
__interrupt void ADCB_ISR(void);

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);
void serialRXA(serial_t *s, char data);
void serialRXC(serial_t *s, char data);
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void calcPose(void);
void calcAngle();


// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t spivalue4 = 0;
int16_t spivalue5 = 0;
int16_t spivalue6 = 0;
int16_t spivalue7 = 0;
int16_t spivalue8 = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
int16_t pwm1 = 0;
int16_t pwm2 = 0;
int16_t pwmdown1 = 0; //pwm flag for incrementing
int16_t pwmdown2 = 0;
float LeftWheel = 0; //readings of two wheels in radians
float RightWheel = 0;
float Leftdist = 0; //distance travelled in ft
float Rightdist = 0;
float uleft = -5.0;
float uright =-5.0; //reference value for motor control
float PosLeft_K = 0; //current position
float PosLeft_K_1 = 0; //previous position
float VLeftK = 0; // raw velocity
float PosRight_K = 0;
float PosRight_K_1 = 0;
float VRightK = 0;
float KI = 25;  //variables for PI controller
float KP = 3;
float el_k = 0; //error for left wheel
float el_k_1 = 0;
float er_k = 0;
float er_k_1 = 0;//error for right wheel
float Il_k = 0;
float Il_k_1 = 0; //integrator
float Ir_k = 0;
float Ir_k_1 = 0;
float Vref = 0.0;
//steering controller variables
float KPturn = 3;
float e_turn = 0;
float turn = 0;
//varibles for calculating car pose
float Wr = 0.57750; //width between wheels,modified to make more accurate
float Rwh = 0.19583; //radius of wheel
float theta_r = 0; //rotation angle of wheels
float theta_l = 0;
float Xr = 0; //position
float Yr = 0;
float phi_R = 0;
float x_dot = 0;
float y_dot = 0;
float x_dot_k_1 = 0;
float y_dot_k_1 = 0;
float theta_ave = 0;
float theta_dot_l = 0;
float theta_dot_r = 0;
float theta_dot_ave = 0;
int16_t adcb4result = 0; //reading for b4
float d0volt = 0;
int32_t adcd1count = 0;
int32_t adca1count = 0;
int32_t adcb1count = 0;

//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float xk = 0;  //current ADC reading of d0
float xk2 = 0; //current ADC reading of a2
float xk3 = 0; //current ADC reading of a3
float xk4 = 0; //current ADC reading of b4
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//yk is the filtered value for d0
float yk = 0;
float yk2 = 0; //filter value for a2
float yk3 = 0; //filter value for a3
float yk4 = 0; //filter value for b4
float yk4_1 = 0; //record previous filtered value
float maxyk = 0; //maximum amplitude
float energy = 0; //energy integral of the sound signal. obtained every 100 ms
float currentE = 0;
float prevE = 0;
int16_t high = 0; //indicate that a beat is detected
int16_t swing = 0;//indicate position of the servo
float a2 = 0; //converted voltage reading of adca
float a3 = 0;
// Needed global Variables for lab 7
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.77;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;
float LeftWheel_k_1 = 0; //previous readings of two wheels in radians
float RightWheel_k_1 = 0;
float vel_Right = 0; //filtered wheel speed
float vel_Left = 0;
float vel_Right_k_1 = 0; //store previous readings of speed
float vel_Left_k_1 = 0;
float raw_vel_Right = 0; //velocity calculated before filtering
float raw_vel_Left = 0;
float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;
float ubal = 0;
float WhlDiff = 0;
float vel_WhlDiff = 0;
float raw_vel_WhlDiff = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff_1 = 0;
float turnref = 0; //turn angle
float errorDiff = 0;
float errorDiff_1 = 0;
float intDiff = 0;
float intDiff_1 = 0;
float Kp = 3.0;
float Ki = 0.0; //20.0
float Kd = 0.08;
float FwdBackOffset = 0.0;
char myriodata[16];
int16_t numRXC = 0;
int16_t goalx, goaly, startx, starty = 0;
float goal_angle, dy, dx = 0;
float turnrate = 0; //Turn rate to replace turnref
float turnrate_1 = 0; // Old Turn rate
float turnref_1 = 0; // Old Turn ref
float goal_dist = 0;
int16_t state = 0;
int16_t com_state, nav_state = 0;
char status = '!';
float FBerror = 0;
float seg_ref = 0;
float FBint = 0;
float FBint_1 = 0;
float FBerror_1 = 0;
float KpFB = -0.35;
float KiFB = -1.5;
float leftir, rightir, frontir = 0.0;
int16_t adcb2result = 0; // For IR Sensor 1
int16_t adcb3result = 0; // For IR Sensor 2
int16_t adcb14result = 0; // For IR Sensor 3
float rightKp = -0.75;
float frontKp = 0.75;
float leftKp = 0.0;
float exitx, exity = 0.0;
int16_t beatPrint = 0;
int16_t bpm = 150;
float turnerror, turnint, turnint_1, turnerror_1 = 0.0;
float trateKp = 1.0;
float trateKi = 0.1;

float d[52] = {   -1.8688015876014348e-03,
    -2.0298497802274159e-03,
    -1.3269481447811923e-03,
    2.1110497667511799e-18,
    1.0604558395142466e-03,
    7.5730076883295198e-04,
    -9.6792710266198356e-04,
    -2.2018364692855345e-03,
    9.8809753989459228e-19,
    6.7421152215178630e-03,
    1.4877285593600112e-02,
    1.8141173815452924e-02,
    1.2240393735627844e-02,
    -1.1491009320515306e-17,
    -8.9842785497757649e-03,
    -5.9832316690203481e-03,
    7.1435364294908758e-03,
    1.5329372754847885e-02,
    -8.4727270917777590e-18,
    -4.3952511778198519e-02,
    -9.7299218958775538e-02,
    -1.2325289981120988e-01,
    -9.0894190154076820e-02,
    0.0000000000000000e+00,
    1.1192996767285625e-01,
    1.8852841833248338e-01,
    1.8852841833248338e-01,
    1.1192996767285625e-01,
    0.0000000000000000e+00,
    -9.0894190154076820e-02,
    -1.2325289981120988e-01,
    -9.7299218958775538e-02,
    -4.3952511778198519e-02,
    -8.4727270917777590e-18,
    1.5329372754847885e-02,
    7.1435364294908758e-03,
    -5.9832316690203481e-03,
    -8.9842785497757649e-03,
    -1.1491009320515306e-17,
    1.2240393735627844e-02,
    1.8141173815452924e-02,
    1.4877285593600112e-02,
    6.7421152215178630e-03,
    9.8809753989459228e-19,
    -2.2018364692855345e-03,
    -9.6792710266198356e-04,
    7.5730076883295198e-04,
    1.0604558395142466e-03,
    2.1110497667511799e-18,
    -1.3269481447811923e-03,
    -2.0298497802274159e-03,
    -1.8688015876014348e-03}; //0.1 - 0.3 * half sample Hz
float xks[22] = {0};
float xk2s[22] = {0};  //filter array for a2
float xk3s[22] = {0};  //filter array for a3
float xk4s[52] = {0};  //filter array for b4
int16_t xinteger, yinteger = 0;
float tsegKp = 0.5;

//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
 if (dacouta0 > 3.0) dacouta0 = 3.0;
 if (dacouta0 < 0.0) dacouta0 = 0.0;
 DacaRegs.DACVALS.bit.DACVALS = dacouta0/3.0*4095.0; // perform scaling of 0-3 to 0-4095
}
void setDACB(float dacouta1) {
 if (dacouta1 > 3.0) dacouta1 = 3.0;
 if (dacouta1 < 0.0) dacouta1 = 0.0;
 DacbRegs.DACVALS.bit.DACVALS = dacouta1/3.0*4095.0; // perform scaling of 0-3 to 0-4095
}

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

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //Exercise 4, setup GPIO52 as output
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //MyRio  CS  Chip Select
    GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO48 = 1;

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
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCD1_INT = &ADCD_ISR; // assign ADCD1_INT to the memory address location of ISR
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR; //assign ADCA1_INT to the memory address location of ISR
    PieVectTable.ADCB1_INT = &ADCB_ISR; // assign ADCB1_INT to the memory address location of ISR
    PieVectTable.ADCD1_INT = &ADCD_ISR; //assign ADCD1_INT to the memory address location of ISR

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    init_serial(&SerialC,115200,serialRXC);
    //    init_serial(&SerialD,115200,serialRXD);

    setupSpib();
    init_eQEPs();// initialize eqeps
    //copied from lab 3
    //With TBCTL: Count up Mode, Free Soft emulation mode to Free Run so that the PWM continues when you set a break point in your code, disable the phase loading, Clock divide by 1.
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    //With TBCTR: Start the timer at zero.
    EPwm2Regs.TBCTR = 0;
    //With TBPRD: Set the period (carrier frequency) of the PWM signal to 20KHz which is a period of 50 microseconds.
    EPwm2Regs.TBPRD = 2500;
    //With CMPA initially start the duty cycle at 0%.
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;
    //With AQCTLA set it up such that the signal is cleared when CMPA is reached. Have the pin be set when the TBCTR register is zero.
    EPwm2Regs.AQCTLA.bit.CAU = 0x1;
    EPwm2Regs.AQCTLA.bit.ZRO = 0x2;
    EPwm2Regs.AQCTLB.bit.CBU = 0x1;
    EPwm2Regs.AQCTLB.bit.ZRO = 0x2;
    //With TBPHS set the phase to zero
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    //Set the pinmux so EPwm2A and EPwm2B
    GPIO_SetupPinMux(2,GPIO_MUX_CPU1,1);
    GPIO_SetupPinMux(3,GPIO_MUX_CPU1,1);

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD, 010
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”), 01
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 2500; // Set sampling frequency to 2000Hz (0.5 ms period). Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode: 00
    EDIS;

    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD, 010
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”), 01
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; //Set frequency to 1000Hz. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode: 00
    EDIS;

//With TBCTL: Count up Mode, Free Soft emulation mode to Free Run so that the PWM continues when you set a break point in your code, disable the phase loading, Clock divide by 1.
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 0x2;
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.CLKDIV = 0x4;  //Exercise 3.1, set divider to be 16
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    //With TBCTR: Start the timer at zero.
    EPwm8Regs.TBCTR = 0;
    //exercise 3.1 With TBPRD: Set the period (carrier frequency) of the PWM signal to 20KHz which is a period of 20 ms
    EPwm8Regs.TBPRD = 62500;  //period = 62500*16/(50Mhz)= 20 ms, and frequency is 50Hz
    //With CMPA initially start the duty cycle at 0%.
    EPwm8Regs.CMPA.bit.CMPA = 0;
    EPwm8Regs.CMPB.bit.CMPB = 0;
    //With AQCTLA set it up such that the signal is cleared when CMPA is reached. Have the pin be set when the TBCTR register is zero.
    EPwm8Regs.AQCTLA.bit.CAU = 0x1;
    EPwm8Regs.AQCTLA.bit.ZRO = 0x2;
    EPwm8Regs.AQCTLB.bit.CBU = 0x1;
    EPwm8Regs.AQCTLB.bit.ZRO = 0x2;
    //With TBPHS set the phase to zero
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    //Set the pinmux so EPwm8 is used instead of GPIO
    GPIO_SetupPinMux(14,GPIO_MUX_CPU1,1);
    GPIO_SetupPinMux(15,GPIO_MUX_CPU1,1);

    EALLOW; //ADCA initialization code copied from lab 4
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    EDIS;
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA, exercise 3
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose to be A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose to be A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1 that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB,exercise 4
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose to be B4
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2; //SOC2 will convert Channel you choose Does not have to be B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 14; //SOC3 will convert Channel you choose Does not have to be B3
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD, exercise 1.2
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0: 0xD
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1: 0xD
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2; //set SOC2 to convert pin D2
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // Enable DACA and DACB outputs, exercise 1.3
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;  //SCIB
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable ADCD1 in the PIE: Group 1 interrupt 6
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    //exercise 3,Enable ADCA1 in the PIE: Group 1 interrupt 1
    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    //exercise 4,Enable ADCB1 in the PIE: Group 1 interrupt 2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //Enable SPIB RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //Enable ADCA1 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
     {
        if (UARTPrint == 1 ) {
            xinteger = Xr * 12.0;
            yinteger = Yr * 12.0;
            serial_printf(&SerialC, "%d %d %c\r", xinteger, yinteger, status);
            status = '!';
            UARTPrint = 0;
        }
        if (beatPrint == 1) {
            serial_printf(&SerialA,"Current Energy:%.3f \r\n",currentE); //print current energy integral
            if (high == 1) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //makes the blue LED blink at each beat
                high = 0;
            }
            if (currentE >= (5.5*prevE) && fabs(currentE - prevE) > 0.5) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
                high = 1;
                if (swing == 0) {               //change servo position at each beat
                    setEPWM8A_RCServo(0);
                    setEPWM8B_RCServo(0);
                    swing = 1;
                } else {
                    setEPWM8A_RCServo(-30);
                    setEPWM8B_RCServo(-30);
                    swing = 0;
                }
            }
            beatPrint = 0;
            prevE = currentE;
        }
    }
}


//exercise 2, adcd1 pie interrupt with 21st order filter
 __interrupt void ADCD_ISR (void)
{

 AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
 PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

 //exercise 3, adca1 pie interrupt with 21st order filter
/*__interrupt void ADCA_ISR (void)
 {

  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
 }*/



 //filter out the drum sound, adcb1 interrupt with 51st low pass filter at 200Hz
  __interrupt void ADCB_ISR (void)
 {
  GpioDataRegs.GPBSET.bit.GPIO52 = 1;
  adcb4result = AdcbResultRegs.ADCRESULT0;
  adcb3result = 4095.0 - AdcbResultRegs.ADCRESULT1;
  adcb2result = 4095.0 - AdcbResultRegs.ADCRESULT2;
  adcb14result = 4095.0 - AdcbResultRegs.ADCRESULT3;
  // Here covert ADCIND0
  xk4 = adcb4result*3.0/4095.0;
  leftir = adcb3result*3.0/4095.0; //voltage of b3
  rightir = adcb2result*3.0/4095.0; //voltage of b2
  frontir = adcb14result*3.0/4095.0; //voltage of b14
  xk4s[0] = xk4;
  int i;
  yk4 = 0;
  for (i = 0; i < 52;i++) {
      yk4 = yk4 + d[i]*xk4s[i];
  }
  //Save past states before exiting from the function so that next sample they are the older state
  for (i = 51; i > 0;i--) {
      xk4s[i] = xk4s[i-1];
  }
  // Here write yk to DACA channel
  setDACA(leftir);
  if (yk4 > maxyk) {
      maxyk = yk4;
  }
  //energy = energy + (fabs(yk4) + fabs(yk4_1))*0.001; //integral
  energy = energy + yk4*yk4; //discrete energy of signal
  yk4_1 = yk4;
  // Print ADCIND0's voltage value to TeraTerm every 50ms
  adcb1count++;
  if ((adcb1count % bpm) == 0) { //1/8 of a second
      currentE = energy;
      energy = 0;
      beatPrint = 1;
  }
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
  GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
 }


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......

    //exercise 3 calculate filtered left and right wheel speed in radains/second using (100z-100)/(z-0.6)
    raw_vel_Right = (RightWheel - RightWheel_k_1) / 0.004;
    raw_vel_Left = (LeftWheel - LeftWheel_k_1) / 0.004;
    vel_Right = 0.6*vel_Right_k_1 + 0.4*raw_vel_Right; // Calculate right wheel velocity using transfer function (100z - 100)/(z-0.6)
    vel_Left = 0.6*vel_Left_k_1 + 0.4*raw_vel_Left;
    calcPose(); //calculate the robot pose
    //update the previous values
    RightWheel_k_1 = RightWheel;
    LeftWheel_k_1 = LeftWheel;
    vel_Right_k_1 = vel_Right;
    vel_Left_k_1 = vel_Left;
    ubal = -K1*tilt_value - K2*gyro_value - K3*(vel_Left + vel_Right)/2.0; // Output balance value
    float uLeft = 0;
    float uRight = 0;

    //exercise 4 steering the segbot
    WhlDiff = LeftWheel - RightWheel;
    raw_vel_WhlDiff = (WhlDiff - WhlDiff_1)/0.004;
    vel_WhlDiff = 0.333 * vel_WhlDiff_1 + 0.667 * raw_vel_WhlDiff; //calculated from (166.667z-166.667)/(z-0.333)

	goal_dist = sqrt(((goalx-Xr)*(goalx-Xr))+((goaly-Yr)*(goaly-Yr))); // Calculate distance to goal coordinates
    calcAngle();
    if (nav_state == 10){
        /*turnerror = phi_R - goal_angle;
        turnint = turnint_1 + 0.004*(turnerror + turnerror_1)/2.0;
        turnrate = trateKp * turnerror + trateKi * turnint;
        turnint_1 = turnint;
        turnerror_1 = turnerror;

            if (fabs(turnrate) >= 3) {
                    turnint = turnint_1; //PID integrator
                }
                //saturate the turn if it's bigger than 3
                if (turnrate >= 3) {
                    turnrate = 3;
                } else if (turnrate <= -3) {
                    turnrate = -3;
                }
        seg_ref = tsegKp * fabs(PI - (phi_R - goal_angle));*/
        if (frontir < 1.25 || rightir < 0.75){
            nav_state = 20;
            seg_ref = 0;
            turnrate = 0;
        }
        if (fabs(phi_R - goal_angle) > 0.5){
            seg_ref = 0;
            turnerror = fabs(phi_R - goal_angle);
            turnint = turnint_1 + 0.004*(turnerror + turnerror_1)/2.0;
            turnrate = trateKp * turnerror + trateKi * turnint;
            if (fabs(turnrate) >= 3) {
                turnint = turnint_1; //PID integrator
            }
            //saturate the turn if it's bigger than 3
            if (turnrate >= 3) {
                turnrate = 3;
            } else if (turnrate <= -3) {
                turnrate = -3;
            }
            turnint_1 = turnint;
            turnerror_1 = turnerror;
            /*if (fabs(goal_angle) <= PI && fabs(goal_angle) >= 3.0){
                turnrate = 0.8;
            }
            else if (phi_R - goal_angle > 0){
                turnrate = 0.8;
            }
            else{
                turnrate = -0.8;
            }*/
        }
        else if (fabs(phi_R - goal_angle) > 0.1){
            nav_state = 11;
        }
        else{
            turnrate = 0.0;
            seg_ref = 2.0;
            }
    }
    else if (nav_state == 11){
        if (frontir < 1.5 || rightir < 0.5){
            nav_state = 20;
            seg_ref = 0;
            turnrate = 0;
        }
        seg_ref = 0.8;
        if ((phi_R - goal_angle) > 0){
            turnrate = 0.3;
        }
        else {
            turnrate = -0.3;
             }
        if (fabs(phi_R - goal_angle) > 0.8){
            nav_state = 10;
        }
    if(goal_dist <= 0.04){ // If not at the goal, go forward until it is reached
        turnrate = 0;
        seg_ref = 0;
        status = '$';
        nav_state = 0;
        }
    }
    else if (nav_state == 20){ //Obstacle Avoidance State
        //leftir, rightir, frontir
        turnrate = rightKp * (rightir - 2.2);
        seg_ref = 1.0;
        if (frontir < 1.0){
            nav_state = 21;
        }
        if (frontir && rightir > 2.2){
            nav_state = 23;
        }
        if(goal_dist <= 0.04){ // If not at the goal, go forward until it is reached
                turnrate = 0;
                seg_ref = 0;
                status = '$';
                nav_state = 0;
                }
    }
    else if (nav_state == 21){ // Turn the robot to the left
        turnrate = frontKp * (frontir - 2.5);
        seg_ref = 0.2;
        if (frontir > 2.2){
            nav_state = 20;
            exitx = Xr; // Stores current X
            exity = Yr; // Stores current Y
        }
        if(goal_dist <= 0.04){ // If not at the goal, go forward until it is reached
                turnrate = 0;
                seg_ref = 0;
                status = '$';
                nav_state = 0;
                }
    }
    else if (nav_state == 23){ //Clear obstacle before resuming navigation
        float exitdist = 0.0;
        if (frontir < 1.0){
            nav_state = 21;
        }
        seg_ref = 1.0;
        turnrate = 0;
        exitdist = sqrt(((Xr-exitx)*(Xr-exitx))+((Yr-exity)*(Yr-exity))); // Calculate distance traveled since exiting obstacle avoidance
        if (exitdist >= 0.5){ // Resume navigation after a set distance
            nav_state = 10;
        }
        if(goal_dist <= 0.04){ // If not at the goal, go forward until it is reached
                turnrate = 0;
                seg_ref = 0;
                status = '$';
                nav_state = 0;
                }
    }

    turnref = 0.004*(turnrate + turnrate_1)/2 + turnref_1; // Calculate turnref from turnrate and old turnref with trapezoid rule
    //turnref = goal_angle;
    //implement controller
    errorDiff = turnref - WhlDiff;

    intDiff = intDiff_1 + 0.004*(errorDiff+errorDiff_1)/2;

    turn = Kp*errorDiff + Ki*intDiff - Kd*vel_WhlDiff; //PID loop for turn
    //anti-windup controller
    if (fabs(turn) >= 3) {
        intDiff = intDiff_1; //PID integrator
    }
    //saturate the turn if it's bigger than 4
    if (turn >= 4) {
        turn = 4;
    } else if (turn <= -4) {
        turn = -4;
    }

    FBerror = seg_ref - (vel_Left + vel_Right)/2.0;
    FBint = FBint_1 + 0.004*(FBerror + FBerror_1)/2.0;
    FwdBackOffset = KpFB * FBerror + KiFB * FBint;

    if (fabs(FwdBackOffset) >= 3) {
            FBint = FBint_1; //PID integrator
        }
        //saturate the turn if it's bigger than 4
        if (FwdBackOffset >= 4) {
            FwdBackOffset = 4;
        } else if (FwdBackOffset <= -4) {
            FwdBackOffset = -4;
        }

    uRight = ubal/2.0 - turn + FwdBackOffset;
    uLeft = ubal/2.0 + turn + FwdBackOffset;
    setEPWM2A(uRight); //Send effort value to right wheel
    setEPWM2B(-uLeft); // Send effort value to left wheel

    //update the previous values
    WhlDiff_1 = WhlDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    errorDiff_1 = errorDiff;
    intDiff_1 = intDiff;
    turnrate_1 = turnrate;
    turnref_1 = turnref;
    FBerror_1 = FBerror;
    FBint_1 = FBint;

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;
    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; //select dan's chip

    //SpibRegs.SPITXBUF = 0x00DA; // start code for dan's chip
    //SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    /*SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    SpibRegs.SPITXBUF = pwm1; // communicates two PWM values to the chip
    SpibRegs.SPITXBUF = pwm2;
    if (pwm1 >= 3000) {
        pwmdown1 = 1;
    }
    if (pwm1 <= 0) {
        pwmdown1 = 0;
    }
    if (pwm2 >= 3000) {
        pwmdown2 = 1;
    }
    if (pwm2 <= 0) {
        pwmdown2 = 0;
    }
    if (pwmdown1 == 0) {
        pwm1 = pwm1 + 10;
    } else {
        pwm1 = pwm1 - 10;
    }
    if (pwmdown2 == 0) {
        pwm2 = pwm2 + 10;
    } else {
        pwm2 = pwm2 - 10;
    }*/

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    /*LeftWheel = readEncLeft();
    RightWheel = readEncRight();
    Leftdist = LeftWheel/5.12;
    Rightdist = RightWheel/5.12;
    PosLeft_K = Leftdist;
    PosRight_K = Rightdist; //keep track of current distance
    VLeftK = (PosLeft_K - PosLeft_K_1)/0.004; //calculate the raw velocity
    VRightK = (PosRight_K - PosRight_K_1)/0.004;
    //add steering control with a turn setpoint
    e_turn = turn + (VLeftK - VRightK);
    //implement controller
    el_k = Vref - VLeftK - KPturn*e_turn; //KPturn*e_turn term is added in exercise 4
    er_k = Vref - VRightK + KPturn*e_turn;
    //anti-windup controller
    if (fabs(uleft) >= 10) {
        Il_k = Il_k_1;
    } else {
        Il_k = Il_k_1 + 0.004*(el_k+el_k_1)/2;
    }
    if (fabs(uright) >= 10) {
        Ir_k = Ir_k_1;
    } else {
        Ir_k = Ir_k_1 + 0.004*(er_k+er_k_1)/2;
    }
    uleft = KP*el_k + KI*Il_k;
    uright = KP*er_k + KI*Ir_k;
    //update error and integrator
    el_k_1 = el_k;
    er_k_1 = er_k;
    Il_k_1 = Il_k;
    Ir_k_1 = Ir_k;
    PosLeft_K_1 = PosLeft_K; //updating position values
    PosRight_K_1 = PosRight_K;
    setEPWM2A(uright); //2A calls right
    setEPWM2B(uleft); //2B calls left, no need to negate because the wire is flipped
    calcPose(); //calculate the robot pose
     */
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    //if ((CpuTimer2.InterruptCount % 50) == 0) {
        //UARTPrint = 1;
    //}
}

__interrupt void ADCA_ISR (void) {
    //adca2result = AdcaResultRegs.ADCRESULT0;
    //dca3result = AdcaResultRegs.ADCRESULT1;
    //a2 = adca2result * 3.0/4095.0;
    //a3 = adca3result * 3.0/4095.0;
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  //select MPU-9250
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; //Issue the SPIB_RX_INT when 8 values are in the RX FIFO (start address, 3 accel ,readings, a temp reading, 3 gyro readings)
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); //start from register 3A and will be discard
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
 //   GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  //deselect MPU-9250

//    GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;  //select MyRio-1900
//    SpibRegs.SPIFFRX.bit.RXFFIL = 1; //Issue the SPIB_RX_INT when 1 value are in the RX FIFO (start address, 3 accel ,readings, a temp reading, 3 gyro readings)
//    SpibRegs.SPITXBUF = (0x0077); //Test Data

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void SPIB_isr(void){
    /*spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
    spivalue3 = SpibRegs.SPIRXBUF; // Read third 16 bit value off RX FIFO. Again probably zero */
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 9 high to end Slave Select. Now to Scope. Later to deselect DAN28027
    spivalue1 = SpibRegs.SPIRXBUF;  //junk data
    spivalue2 = SpibRegs.SPIRXBUF;  //x accel
    spivalue3 = SpibRegs.SPIRXBUF;  //y accel
    spivalue4 = SpibRegs.SPIRXBUF;  //z accel
    spivalue5 = SpibRegs.SPIRXBUF;  //temp out
    spivalue6 = SpibRegs.SPIRXBUF;  //x gyro
    spivalue7 = SpibRegs.SPIRXBUF;  //y gyro
    spivalue8 = SpibRegs.SPIRXBUF;  //z gyro
    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
    accelx = spivalue2/32767.0*4.0;  //scale the accel readings in units of g
    accely = spivalue3/32767.0*4.0;
    accelz = spivalue4/32767.0*4.0;
    gyrox = spivalue6/32767.0*250.0; //scale the gyro readings in units of deg
    gyroy = spivalue7/32767.0*250.0;
    gyroz = spivalue8/32767.0*250.0;
    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = -readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }

    //call print function every 200 times into SPIB_isr
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    /*if (data == 'q') {
        turn = turn + 0.05;
    } else if (data == 'r') {
        turn = turn - 0.05;
    } else if (data == '3') {
        Vref = Vref + 0.1;
    } else if (data == 's') {
        turn = 0; //stop command
        Vref = 0;
    }else {
        turn = 0;
        Vref = 0.5;
    } */

    // This function is called each time a char is received over UARTA.
    if (data == 'q') {  // Changes turn rate or velocity based on tera term inputs
        turnrate = turnrate - 0.2;
    } else if (data == 'r') {
        turnrate = turnrate + 0.2;
    } else if (data == '3') {
        seg_ref = seg_ref - 0.5;
    } else if (data == 's') {
        seg_ref = seg_ref + 0.5;
    } else if (data == 'p') {
        nav_state = 10;
    } else {
        turnrate = 0;
        seg_ref = 0.0;
        nav_state = 0;
    }
}

// This function is called each time a char is received over UARTA.
void serialRXC(serial_t *s, char data) {
    if (com_state == 0){
        if (data == '*'){
                com_state = 2;
                numRXC = -1;
            }
    }
    else if (com_state == 2){
        if (data == '*'){
            com_state = 10;
            status = '!';
            numRXC = -1;
            }
        else{
            com_state = 0;
            numRXC = -1;
        }
        }
    else if (com_state == 10){ // Indicates incoming integer
        myriodata[numRXC] = data;
            if (data == 13){
            numRXC = -1;
            com_state = 0;
            }
            else if (data == ','){ // Checks for multiple values
                goalx = atoi(myriodata)/12;
                com_state = 11;
                numRXC = -1;
            }
    }
    else if (com_state == 11){
        myriodata[numRXC] = data;
        if (data == 13){
            goaly = atoi(myriodata)/12;
            numRXC = -1;
            com_state = 0;
            nav_state = 10;
            }
    }
    numRXC++;
}
//exercise 4
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    //between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    //66 which are also a part of the SPIB setup.
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //Exercise 2.1 SPIB setup, now it's inside this function rather than in main()
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MyRio SS
    GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO48 an Output Pin
    GpioDataRegs.GPBSET.bit.GPIO48 = 1; //Initially Set GPIO48/SS High so MyRio is not selected
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
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL =16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below
    //-----------------------------------------------------------------------------------------------------------------
    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x0000); // for register 13,14, x accelerometer offset is 0
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // register 15,16, y accelerometer offset is 0
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); //register 17,18, z accelerometer offset is 0
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0000 | 0x0013); //sample rate divider is set to 13 by register 19, sample rate = 1kHz/(1+13)
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200 | 0x0000); //register 1A sets DLPF_CFG = 2, determines low-pass filter configuration to gyroscope and temperature sensor; 1B sets FCHOICE_B is 00, enabling FCHOICE as 11, gyro full-scale select as +250dps
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0800 | 0x0006); // 1C Accel Full Scale Select +/- 4g; 1D sets accelerometer data rates and bandwidth of another low-pass filter, A_DLPF_CFG mode 6, sets A_FCHOICE_B = 0 and enable A_Fchoice = 1,
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); //1E sets output freq (wake up chip) = 0.24Hz; 1F sets wake on motion interrupt for accel =0
    // wait for the correct number of 16 bit values to be received into the RX FIFO, in this case 7 times
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1; // Slave Select Low
    SpibRegs.SPITXBUF = (0x0017 | 0x00A8); // Write Test data to MyRio address 0x0
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPBSET.bit.GPIO48 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EC); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0078); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E1); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0056); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0025); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x001C); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
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

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2.0*PI/600.0));// negate this reading to make the forward movement positive
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2.0*PI/600.0));
}


//2A calls right
void setEPWM2A(float controleffort){
    if(controleffort > 10) {
        controleffort= 10;
    }
    if(controleffort < -10) {
        controleffort= -10;
    }
    EPwm2Regs.CMPA.bit.CMPA = (controleffort+10.0)*0.05*EPwm2Regs.TBPRD;
}
//2B calls Left
void setEPWM2B(float controleffort){
    if(controleffort > 10) {
        controleffort= 10;
    }
    if(controleffort < -10) {
        controleffort= -10;
    }
    EPwm2Regs.CMPB.bit.CMPB = (controleffort+10.0)*0.05*EPwm2Regs.TBPRD;
}


void calcPose() {
    phi_R = Rwh/Wr*(RightWheel - LeftWheel);
    //phi_R = fmod(phi_R, TWOPI);
    theta_ave = 0.5 * (RightWheel + LeftWheel);
    theta_dot_l = (LeftWheel - LeftWheel_k_1)/0.004;
    theta_dot_r = (RightWheel - RightWheel_k_1)/0.004;
    //theta_dot_ave = 0.5 * (theta_dot_l+theta_dot_r);
    theta_dot_ave = (vel_Left + vel_Right)/2.0;
    x_dot = Rwh*theta_dot_ave*cos(phi_R);
    y_dot = Rwh*theta_dot_ave*sin(phi_R);
    phi_R = atan2(sin(phi_R),cos(phi_R));
    Xr = Xr + (x_dot_k_1+x_dot)/2*0.004; //integrate with the trapezoid rule
    Yr = Yr + (y_dot_k_1+y_dot)/2*0.004;
    //update values
    x_dot_k_1 = x_dot;
    y_dot_k_1 = y_dot;
}

void calcAngle() { //Calculates the angle to goal point
    dx = goalx - Xr; // Difference in x coordinates
    dy = goaly - Yr; // Difference in y coordinates
    goal_angle = atan2(dy, dx);
}

//exercise 3.2 change angle of servo motor
void setEPWM8A_RCServo(float angle){
    if(angle > 90) {
        angle= 90;
    }
    if(angle < -90) {
        angle= -90;
    }
    EPwm8Regs.CMPA.bit.CMPA = ((angle+90.0)*0.08/180 + 0.04)*EPwm8Regs.TBPRD;
}

void setEPWM8B_RCServo(float angle){
    if(angle > 90) {
        angle= 90;
    }
    if(angle < -90) {
        angle= -90;
    }
    EPwm8Regs.CMPB.bit.CMPB = ((angle+90.0)*0.08/180 + 0.04)*EPwm8Regs.TBPRD;
}

