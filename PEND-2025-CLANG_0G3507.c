/**********************************************************************
 * PROGRAM NAME: PEND-2025-CLANG_0G3507.c
 * PROGRAM FUNCTION: Balancing Robot
 * HARDWARE SPECIFICATIONS:
 *   - 2xPOLOLU 4753 (50:1 Metal Gearmotor 37Dx70Lmm/12V/64CPR)
 *   - MSPM0G3507@80MHz/12.5ns
 * COMPILER VERSION: Clang v4.0.0 -O2
 * PROGRAMMER: Ing. Tomas Solarski
 * LAST MODIFIED: 2026-02-11 08:13:24
 **********************************************************************/

// ** LAST UPDATE **
// - Feed Forward Control added (K0 gain)
// 

#include    <stdbool.h>
#include    <string.h>
#include    <stdlib.h>

#include    "ti/driverlib/dl_gpio.h"
#include    "ti_msp_dl_config.h"

#define     GLOBAL_IQ 16
#include    <ti/iqmath/include/IQmathLib.h>

#define     LED_HEART_ON_MS     (100U)
#define     LED_HEART_OFF_MS    (100U)
#define     LED_TICK_COUNT      (15U)

#define     LED_SEQUENCE_0      (0b111111)      // long  blink
#define     LED_SEQUENCE_1      (0b111)         // one   blink
#define     LED_SEQUENCE_2      (0b111000111)   // two   blink
#define     LED_SEQUENCE_3      (0b1100110011)  // three blink
#define     LED_SEQUENCE_4      (0b11011011011) // four blink
#define     LED_SEQUENCE_5      (0b101010101)   // five blink
#define     LED_SEQUENCE_6      (0b10101010101) // six blink

#define     TIME_SEND_DATA_MS       (100U)      // 10Hz send data rate
#define     TIME_MOTOR_OMEGA_MS     (50U)       // rate to calculate impulses to determine motor speed
#define     TIME_MOTOR_OMEGA_S      (0.05f)     // in seconds
#define     TIME_MPU_DELAY_MS       (250U)      // delay before MPU is initiated
#define     TIME_MPU_READ_MS        (2U)        // reading rate of MPU
#define     TIME_MPU_READ_SEC       (0.002f)    // in seconds
#define     TIME_CRSF_MS            (50U)       // Cross Fire (Radio Control) protocol
#define     TIME_FAIL_SAFE_MS       (1500U)     // No command received

#define     CRSF_MSG_HEAD_SIZE      (3U)        // 0xC8 SYNC (Start-of-frame), 0x18 LEN (Length of TYPE+PAYLOAD+CRC=24B), 0x16 TYPE (16 RC chans)
#define     CRSF_MSG_DATA_SIZE      (22U)       // PAYLOAD: 22*8bit = 176bit => 176bit/16chan = 11bit resolution per RC channel
#define     CRSF_MSG_CRC_SIZE       (1U)        // CRC-8 0xD5
#define     CRSF_MESSAGE_SIZE       (CRSF_MSG_HEAD_SIZE+CRSF_MSG_DATA_SIZE+CRSF_MSG_CRC_SIZE)
#define     CRSF_CHANNEL_COUNT      (16U)
#define     CRSF_CRC_POLY           (0xD5)      // 0xD5 gives better burst‑error detection for short frames

// #define     ANGLE_LIMIT_NEG_DEG     (-45.0f)    // negative angle limit - beyond Controller controll is OFF    
#define     ANGLE_LIM_BODY_DEG      (45.0f)     // Angle limit - beyond Controller controll is OFF    
#define     ANGLE_LIM_HYST_DEG      (22.5f)     // Angle hysteresis - preventing oscilations

#define     VOLT_MOTOR_LIM_HI       (8.00f)     // Action value limit for Inner Loop
#define     VOLT_MOTOR_LIM_LO       (4.00f)     // Action value limit for Side Loop

// #define     VOLT_MOTOR_MIN    (-0.15f)    // Motor voltage minimum value - negative
#define     VOLT_MOTOR_MIN          (0.25f)     // Motor voltage minimum value - positive
#define     VOLT_MOTOR_HYST         (0.10f)     // Small hysteresis to prevent oscillations
#define     VOLT_BATTERY_MIN        (10.2f)     // Li-Pol low voltage - 3.4V per cell
#define     VOLT_BATTERY_DEF        (11.3f)     // Li-Pol default voltage - 3.75V per cell
#define     VOLT_ADC_REF            (3.60f)     // ADC reference voltage
#define     VOLT_SCHTKY_DROP        (0.15f)     // Input rectifier drop
#define     ADC_CNT                 (5U)        // 5 channel used: VS1(Battery), Current M0, Current M1 , Temp M0, Temp M1     

// POLOLU MOTOR 4753 ( 50:1 Metal Gearmotor 37Dx70L mm 12V 200RPM with 64 CPR Encoder )
#define     MOTOR_GEAR_RATIO        (50.0f)     // gear ratio
#define     IMPULSES_PER_REVOLUTION (64.0f)     // 64 counts per revolution (quadrature)
#define     WHEEL_DIAMETER_M        (0.125f)    // 125mm
#define     WHEEL_RADIUS_M          (WHEEL_DIAMETER_M/2.0f)
#define     WHEEL_TRACK_M           (0.21f)     // 210mm - wheels separation distance

#define     ROBO_STATE_INIT         (0U)        // MPU not initiated or found
#define     ROBO_STATE_BALA         (1U)        //
#define     ROBO_STATE_CALIB        (2U)        //

#define     ALFA_CF_COEF            (0.03f)     // Complementary filter coefficient - Acc Gyro
#define     ALPHA_IQ             _IQ(0.02f)     // Alpha
#define     ONE_ALPHA_IQ         _IQ(0.98f)     // 1-Alpha

#define     RAD_TO_DEG              (57.2958f)

#define     PWM_MAX_DUTY            (90U)       // [ % ] -> 0.90*12V => 10.8V
#define     PWM_MIN_DUTY            (2U)        // [ % ] -> 0.02*12V => 0.24V

// TIMER CHANNELS ASSIGNED TO OUTPUTS
#define     PHASE_A                 (1U)
#define     PHASE_B                 (0U)
#define     PHASE_C                 (2U)
#define     PHASE_D                 (3U)
#define     PHASE_COUNT             (4U)

// ----- WS2812B LED -----
#define     WS2812B_COLOR_CNT       (3U)    // RGB - 3 colors BUT on WS2812B GREEN is first, RED, last BLUE
#define     WS2812B_PWM_PATTR       (8U)    // 8 bits of PWM CC to map one bit of WS2818
#define     WS2812B_LED_COUNT       (16U)   // number of physical LED (8+8) to control 
#define     WS2812B_DUMY_BYTE       (2U)    // +2 dummy patterns for latch
#define     WS2812B_BIT_COUNT       (WS2812B_COLOR_CNT*WS2812B_PWM_PATTR*(WS2812B_LED_COUNT+WS2812B_DUMY_BYTE))

// ----- RGB Digi LED WS WS2812B -----
// 8bit PWM pattern: zero pulse - 0x1C (28*12.5ns=0.35us), one pulse - 0x38 (56*12.5ns=0.7us)
volatile    uint8_t     gLEDColors_8b_GRB[ (WS2812B_LED_COUNT+WS2812B_DUMY_BYTE) ][ WS2812B_COLOR_CNT*WS2812B_PWM_PATTR ] =
{   
    // GREEN color (MSB first),                RED color (MSB first),                  BLUE color (MSB first)
    { 0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED0 - FRONT RH
    { 0x1C,0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED1
    { 0x1C,0x1C,0x38,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED2
    { 0x1C,0x1C,0x1C,0x38,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED3 - FRONT MID
    { 0x1C,0x1C,0x1C,0x1C,0x38,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED4 - FRONT MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x38,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED5
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x38,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED6
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x38, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED7 - FRONT LH

    { 0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED8 - REAR LH
    { 0x1C,0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED9
    { 0x1C,0x1C,0x38,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED10
    { 0x1C,0x1C,0x1C,0x38,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED11 - REAR MID
    { 0x1C,0x1C,0x1C,0x1C,0x38,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED12 - REAR MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x38,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED13
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x38,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED14
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x38, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED15 - REAR RH

    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } , // DUMMY
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }   // DUMMY  
};

volatile uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

volatile    uint16_t    gLED_Seq_Now    = LED_SEQUENCE_1;
volatile    uint8_t     gRobot_State    = ROBO_STATE_BALA;
volatile    uint8_t     gRobot_State_1  = ROBO_STATE_BALA;

// ----- FLAGS -----
#define     QUAD_CNT    (2U)
volatile    bool    flg_Heart_LED     = false;
volatile    bool    flg_Read_MPU      = false;
volatile    bool    flg_ADC[ADC_CNT]  = { false };
volatile    bool    flg_Send_UART     = false;
volatile    bool    flg_RUN_OuterLoop = false;
volatile    bool    flg_RUN_InnerLoop = false;
volatile    bool    flg_Motor_EN      = false;
volatile    bool    flg_Read_CRSF     = false;
volatile    bool    A_0[QUAD_CNT]     = { false };
volatile    bool    B_0[QUAD_CNT]     = { false };

// ----- COUNTERS -----
volatile    uint32_t    cnt_Heart_LED   = LED_HEART_ON_MS;
volatile    uint8_t     cnt_Heart_Tick  = 0U;
volatile    uint32_t    cnt_Read_MPU    = TIME_MPU_READ_MS;
volatile    uint32_t    cnt_Delay_MPU   = TIME_MPU_DELAY_MS;
volatile    uint32_t    cnt_Send_UART   = TIME_SEND_DATA_MS;
volatile    uint32_t    cnt_OuterLoop   = TIME_MOTOR_OMEGA_MS;
volatile    uint32_t    cnt_Fail_Safe   = 0U;
volatile    uint32_t    cnt_Read_CRSF   = TIME_CRSF_MS;

volatile    uint32_t    er_ct[ 24 ];
volatile    uint32_t    ab_ct[ 24 ]; 
#define     MAX_TIMERS  (10U)
volatile    uint16_t    dt_us[ MAX_TIMERS ];    // delta time in us
volatile    uint16_t    t0_us[ MAX_TIMERS ];    // save timer value
#define     STORE_TIMER6(i) ( t0_us[i] = (uint16_t)DL_TimerG_getTimerCount( TIMER_6_INST ) )
#define     GET_DURATION(i) ( dt_us[i] = (uint16_t)(DL_TimerG_getTimerCount( TIMER_6_INST ) - t0_us[i]) )

// ----- ANALOG -----
#define     BETA_EMA_COEF       (0.01f)     // Exponential Moving Average coef - Battery voltage
#define     ZETA_EMA_COEF       (0.01f)     // Exponential Moving Average coef - Motor Current
volatile    uint32_t    ADC_LSB[  ADC_CNT ];       // [ LSB ]
volatile    float       ADC_Volt[ ADC_CNT ];       // [ V ]

volatile    float       gBatt_Volt       = VOLT_BATTERY_DEF;
volatile    float       gBatt_Volt_EMA   = VOLT_BATTERY_DEF;
volatile    float       gBETA_EMA_Coef   = BETA_EMA_COEF;
volatile    float       gZETA_EMA_Coef   = ZETA_EMA_COEF;

volatile    float       gM0_RH_Curr_mA      = 0.0f;
volatile    float       gM1_LH_Curr_mA      = 0.0f;
volatile    float       gM0_RH_Curr_mA_Med  = 0.0f;
volatile    float       gM1_LH_Curr_mA_Med  = 0.0f;
volatile    float       gM0_RH_Curr_mA_EMA  = 0.0f;
volatile    float       gM1_LH_Curr_mA_EMA  = 0.0f;

// MEDIAN of 3 - motor current spike elimination
volatile    uint32_t    ADC_LSB_M0_X = 0U , ADC_LSB_M0_Y = 0U , ADC_LSB_M0_Z = 0U , ADC_LSB_M0_MED3 = 0U;
volatile    uint32_t    ADC_LSB_M1_X = 0U , ADC_LSB_M1_Y = 0U , ADC_LSB_M1_Z = 0U , ADC_LSB_M1_MED3 = 0U;
volatile    float       Motor0_curr_mA_max  = 0.0f , Motor1_curr_mA_max = 0.0f;
volatile    uint8_t     Motors_buff_idx     = 0U;
volatile    float       Drive0_temp_C       = 0.0f , Drive1_temp_C      = 0.0f;
volatile    float       Drive0_temp_C_ema   = 0.0f , Drive1_temp_C_ema  = 0.0f;

// -----------------------------------
// ----- Controller 0 INNER LOOP -----
// -----------------------------------
// Attitude (Tilt) Stabilization
#define     K0_FEEDFOR_DEF      (-0.01f)    // K Gain  for FEED FORWARD
#define     K1_DAMPIMG_DEF      (-15.0f)    // K Gain  for Dynamic DAMPING - Negative because Dumping is counteracting the tilt
#define     K2_BALANCE_DEF      (0.1f)      // K Gain  for ANGLE - BALLANCING
#define     K3_TURN_RH_DEF      (+0.020f)   // K Gains for TURNING - As one wheel is countered, the other is turned
#define     K4_TURN_LH_DEF      (-0.020f)   // one wheel gain must be opposite (negative) of the other
volatile    float       gK0_FeedForward     = K0_FEEDFOR_DEF;   // Feed Forward
volatile    float       gK1_DynamicDamp     = K1_DAMPIMG_DEF;   // Angular velocity (from gyroscope) as a damping term
volatile    float       gK2_BalanceAngle    = K2_BALANCE_DEF;   // K Gain for ANGLE - Inner Loop
volatile    float       gK3_Turn_Gain_RH    = K3_TURN_RH_DEF;
volatile    float       gK4_Turn_Gain_LH    = K4_TURN_LH_DEF;

volatile    float       gActVal_FeedF_Volt  = 0.0f;     // Inner loop - Motor voltage
volatile    float       gActVal_Angle_Volt  = 0.0f;     // Inner loop - Motor voltage - From tilt angle (deg)
volatile    float       gActVal_Rates_Volt  = 0.0f;     // Inner loop - Motor voltage - From tilt rates (deg/sec)
volatile    float       gActVal_Robot_Volt  = 0.0f;     // Total Inner Loop Motor Voltage
volatile    float       gPID_Error_Angle_Deg= 0.0f;     // Inner loop - Angle error in degrees

// volatile    float       gTurn_SetPoint_Rad  = 0.0f;     // Angle in radians, -1 means rotate LEFT, +1 means rotate RIGHT
volatile    float       gTurn_Error_RH_Rad  = 0.0f;     // Outer loop - RIGHT WHEEL Angle error in radians
volatile    float       gTurn_Error_LH_Rad  = 0.0f;     // Outer loop - LEFT WHEEL Angle error in radians
volatile    float       gActVal_TurnRH_Volt = 0.0f;     // Outer loop - RIGHT WHEEL Motor voltage compensation
volatile    float       gActVal_TurnLH_Volt = 0.0f;     // Outer loop - LEFT WHEEL Motor voltage compensation

volatile    float       gTurn_rad           = 0.0f;
volatile    float       gTurnR_rads         = 0.0f;

volatile    float       gT1_sample = TIME_MPU_READ_SEC;     // sample time [sec] Inner loop
volatile    float       gT2_sample = TIME_MOTOR_OMEGA_S;    // sample time [sec] Outer loop

// ----- UART0 -----
// RX
#define     RX0_BUFF_N  (4U) 
#define     UART0_RX_BUFFER_SIZE (1U<<RX0_BUFF_N)
volatile    uint8_t     UART0_RXbuffer[ UART0_RX_BUFFER_SIZE ];         // Buffer size in 2^N size (1,2,4,8,16,32,64,...)
volatile    uint8_t     UART0_RXbytes = 0U;                              // Buffer counter
volatile    bool        UART0_terminator_detected = false;
volatile    bool        UART0_command_detected = false;

volatile    int16_t     P_Gain_received = 0;    // P gain
volatile    int16_t     I_Gain_received = 0;    // I gain
volatile    int16_t     D_Gain_received = 0;    // D gain
volatile    int16_t     C_Gain_received = 0;    // LH-RH Compensation gain
volatile    int16_t     N_Gain_received = 0;    // N coef for D filter
volatile    int16_t     G_Gain_received = 0;    // damping from Gyro
volatile    int16_t     A_Filt_received = 0;    // Alpha coef. for MPU (complementary filter)
volatile    int16_t     B_Filt_received = 0;    // Beta  coef. for Battery Voltage
volatile    int16_t     Z_Filt_received = 0;    // Zeta  coef. for Motor Current
volatile    int16_t     Y_Offs_received = 0;    // Y (PITCH) offset received
// TX
#define     SIGNAL_CNT  (28U)                                           // count of Signals that will be sended over UART
#define     SIGNAL_RNG  (7U)                                            // Signals -99999 to +99999 range (6 characters max) + coma separator
volatile    int32_t     UART0_signals[ SIGNAL_CNT ];                    // Signals are stored in fiel to use for-cycle - now global
volatile    uint8_t     UART0_message[ SIGNAL_CNT * SIGNAL_RNG + 1 ];   // array with message (Signals + separators + terminator) - max. size
volatile    uint8_t     UART0_MessageLength = 0;
volatile    uint8_t     UART0_TXbytes = 0;

// ----- UART1 -----
#define     RX1_BUFF_N  (5U) 
#define     UART1_RX_BUFFER_SIZE (1U<<RX1_BUFF_N)
volatile    uint8_t     UART1_RXbuffer[ UART1_RX_BUFFER_SIZE ];
volatile    uint8_t     UART1_RXbuffer_index = 0;
volatile    uint8_t     UART1_TX_counter = 0;
// Cross Fire
volatile    uint32_t    CSFR_Header_match = 0;
volatile    uint32_t    CSFR_CRC_match = 0;
volatile    uint32_t    CSFR_CRC_error = 0;

volatile    uint8_t     CSFR_Data_RX[ CRSF_MSG_DATA_SIZE + CRSF_MSG_CRC_SIZE ];
volatile    uint8_t     CSFR_Data_Reversed[ CRSF_MSG_DATA_SIZE ];
volatile    uint8_t     CSFR_CRC_Received = 0;
volatile    uint8_t     CSFR_CRC_Calculated = 0;
volatile    uint16_t    RC_channels[ CRSF_CHANNEL_COUNT ];

volatile    int16_t     gRC_CMD_Roll     = 0;    // Received Ch. 1 - buffer idx 0
volatile    int16_t     gRC_CMD_Pitch    = 0;    // Received Ch. 2 - buffer idx 1
volatile    uint16_t    RC_throttle = 0;    // Received Ch. 3 - buffer idx 2
volatile    int16_t     RC_yaw      = 0;    // Received Ch. 4 - buffer idx 3

volatile    int16_t     RC_latch_left = 0;  // Received Ch. 5 - buffer idx 4
volatile    int16_t     RC_3state_left = 0; // Received Ch. 6 - buffer idx 5
volatile    int16_t     RC_3state_right= 0; // Received Ch. 7 - buffer idx 6
volatile    int16_t     RC_latch_right = 0; // Received Ch. 8 - buffer idx 7

// ----- I2C1 -----
#define     I2C_TX_MAX_PACKET_SIZE (16U)
#define     I2C_RX_MAX_PACKET_SIZE (16U)
uint8_t     gTxPacket[ I2C_TX_MAX_PACKET_SIZE ] = { 0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F };
uint8_t     gRxPacket[ I2C_RX_MAX_PACKET_SIZE ] = { 0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F };
uint32_t    gTxLen = 0U, gTxCount = 0U;                 // Counters for TX length and bytes sent
uint32_t    gRxLen = 0U, gRxCount = 0U;                 // Counters for RX length and bytes sent
volatile    uint16_t    I2C_Anti_Block = 0U;            // Anti Block System for IIC 
volatile    uint16_t    I2C_Flag_Block = 0U;            // Anti Block System for IIC 
// Indicates status of I2C
enum I2cControllerStatus
{
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} gI2cControllerStatus;

// ----- MPU-6050 -----
//MPU physical AD0 = 0
#define     MPU_ADDRESS ( 0b1101000 )  // MPU slave address in 7bit format
volatile	uint8_t		MPU_WhoAmI_data = 0U;
volatile	uint8_t		MPU_switch = 0U;
volatile    uint32_t    MPU_switch_transition_cnt = 0U;
volatile    uint16_t    MPU_WhoAmI_Return = 0U;
volatile    uint16_t    MPU_ExitSleep_Return = 0U;
volatile    uint16_t    MPU_GetAccTempGyro_Return = 0U;
// raw data registers
volatile	int16_t	    MPU_temp = 0;
volatile	int16_t	    MPU_accX = 0, MPU_accY = 0, MPU_accZ = 0;
volatile	int16_t	    MPU_accX_raw = 0, MPU_accY_raw = 0, MPU_accZ_raw = 0;
volatile	int16_t	    MPU_gyroX = 0, MPU_gyroY = 0, MPU_gyroZ = 0;
volatile	int16_t	    MPU_gyroX_raw = 0, MPU_gyroY_raw = 0, MPU_gyroZ_raw = 0;
// ----- OFFSET COMPENSATION -----
volatile	 int16_t	gMPU_AccX_offset  =  400;
volatile	 int16_t	gMPU_AccY_offset  = -250;
volatile	 int16_t	gMPU_AccZ_offset  =  800; // 2025-09-13
volatile	 int16_t	gMPU_GyroX_offset =  100;
volatile	 int16_t	gMPU_GyroY_offset =  -50;
volatile	 int16_t	gMPU_GyroZ_offset =  75; // 2025-09-13

// ----- Calculated MPU data -----
volatile    float       gMPU_Temp_C = 0.0f;     // Temperature in degrees Celcius

volatile    float       gMPU_AccX_g = 0.0f;     // X Axis Acceleration in Gs
volatile    float       gMPU_AccY_g = 0.0f;     // Y Axis Acceleration in Gs
volatile    float       gMPU_AccZ_g = 0.0f;     // Z Axis Acceleration in Gs

volatile	float		gRate_X_Degs = 0.0f;    // X Axis Rate in degrees per second
volatile	float		gRate_Y_Degs = 0.0f;    // Y Axis Rate in degrees per second
volatile	float		gRate_Z_Degs = 0.0f;    // Z Axis Rate in degrees per second

volatile    float       gAngle_PITCH_Deg = 0.0f;// Pitch in degrees
volatile    float       gAngle_ROLL_Deg = 0.0f; // Roll in degrees
volatile    float       gAngle_YAW_Deg = 0.0f;  // Yaw in degrees

// ----- COMPLEMENTARY FILTER -----
volatile    float       gALPHA_CF_Coef = ALFA_CF_COEF;
// filtered angles
volatile    float       gAngle_PITCH_Deg_CF = 0.0f;
volatile    float       gAngle_ROLL_Deg_CF  = 0.0f;
volatile    float       gAngle_YAW_Deg_CF   = 0.0f;
// N-1 filtered angles
volatile    float       gAngle_PITCH_Deg_0 = 0.0f;
volatile    float       gAngle_ROLL_Deg_0  = 0.0f;
volatile    float       gAngle_YAW_Deg_0    = 0.0f;

// ----- MOTOR/ROBOT -----
volatile    float       gTURN_Limit_Volt;    // limit voltage for yaw control to not become bigger than Balancing Voltage
volatile    float       gMotor_LH_Volt;     // Left Hand motor voltage
volatile    float       gMotor_RH_Volt;     // Right Hand motor voltage
volatile    int32_t     gTheta_0_RH_pos , gTheta_1_LH_pos;      // encoder position in CPR
volatile    float       gTheta_0_RH_rad , gTheta_1_LH_rad;      // motor Angle [rad]
volatile    float       gWheel_LH_m     , gWheel_RH_m;          // wheel distance [m]
volatile    float       gWheel_LH_msec  , gWheel_RH_msec;       // wheel speed [m/sec]
volatile    float       gWheel_LH_hist_m[5];    // history buffers: [0]=k-2, [1]=k-1, [2]=k, [3]=k+1, [4]=k+2
volatile    float       gWheel_RH_hist_m[5];    // history buffers: [0]=k-2, [1]=k-1, [2]=k, [3]=k+1, [4]=k+2

volatile    float       gPosition_m;            // position in m
volatile    float       gPosition_error_m;      // position error in m
volatile    float       gPosition_setPoint_m;   // position set point in m
volatile    float       gPosition_ActVal_deg;   // position ACTION value in deg
volatile    float       gPosition_Gain;         // position GAIN [deg/m]
volatile    float       gPosition_hist_m[5];    // x[k-2], x[k-1], x[k], x[k+1], x[k+2]
// --- Savitzky-Golay Derivative Filter ---
#define     DT_S        (0.05f)                 // TIME_MOTOR_OMEGA_S
#define     SG_K        (1.0f / (12.0f * DT_S)) // 1 / (12*dt)

volatile    float       gSpeed_msec;            // speed in m/sec
volatile    float       gSpeed_error_msec;      // speed error in m/sec
volatile    float       gSpeed_setPoint_msec;   // speed set point in m/sec
volatile    float       gSpeed_ActVal_deg;      // speed ACTION value in deg
volatile    float       gSpeed_Gain;            // speed GAIN [deg/msec]

volatile    float       gAngle_error_deg;       // angle error in deg
volatile    float       gAngle_setPoint_deg;    // angle set point in deg
volatile    float       gYaw_hist_rad[5];       // x[k-2], x[k-1], x[k], x[k+1], x[k+2]

// ----- FUNCTIONS -----
void    MCU_Init( void );
void    PID_Outer_Loop( void );
void    PID_Inner_Loop( void );
void    CSFR_check ( void );
void    ADC_check( void );
void    UART_check( void );
void    QUAD_check( void );
void    Update_Quad_0_RH( void );
void    Update_Quad_1_LH( void );
void    MPU_check ( void );
void    SET_PWM_DUTY( uint8_t argPhase , uint16_t argDuty );
void    Motor_M0_volt( float argVoltM0 );
void    Motor_M1_volt( float argVoltM1 );
bool    checkForGainCommand( void );
bool    checkForCommand2( void );
void    clearUART0_RXbuffer_Zero(void);
uint16_t MPU_ExitSleep( void );
uint16_t MPU_WhoAmI( void );
uint16_t MPU_GetAccTempGyro( void );
uint16_t Send_integers_over_UART( uint8_t argCount );
int32_t median3(int32_t a, int32_t b, int32_t c);
uint8_t table_crc8( void );
uint8_t compute_crc8( void );

bool     CSFR_Extract_Data( void );
void     CSFR_Parse_Data( void );
uint16_t CSFR_Reverse11( uint16_t arg_data_for_rev );

void    WS2812B_LED_Col(uint8_t led, uint8_t R, uint8_t G, uint8_t B);
void    WS2812B_All_LED(uint8_t ArgColor, uint8_t ArgBrightness);
void    WS2812B_Half_LED(uint8_t argRGB1, uint8_t argRGB2, uint8_t argBrg1, uint8_t argBrg2);

void    Wheel_GetSpeed_SG(void);
float   Robot_GetSpeed_SG(void);
float   Robot_GetYawRate_SG(void);

void    BatteryGauge_8LED(float voltage);
void    TiltGauge_8LED(float tilt_deg);

// *****************************************  *****************************************  *****************************************
// *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****
// *****************************************  *****************************************  *****************************************

int main( void ) {
    MCU_Init();

    gPosition_Gain = 0.1f;
    gSpeed_Gain    = 0.1f;

    gPosition_setPoint_m = 0.0f;   // position set point 0 m
    gSpeed_setPoint_msec = 0.0f;   // speed set point 0 m/sec

    WS2812B_Half_LED( 2 , 0 , 0x20 , 0x40 );

// *****************************************  *****************************************  *****************************************
// *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****
// *****************************************  *****************************************  *****************************************
    while ( 1 ) {

        BatteryGauge_8LED( gBatt_Volt_EMA );

        TiltGauge_8LED( gAngle_PITCH_Deg_CF);

        // ----- 10. QUAD -----
        Update_Quad_0_RH();
        Update_Quad_1_LH();
        
        // ----- 20. READ MPU -----        
        MPU_check();
        
        // ----- 30. (A) OUTER LOOP -----
        PID_Outer_Loop();
        
        // ----- 40. (B) INNER LOOP -----
        PID_Inner_Loop();
        
        // ----- 50. SEND DATA OVER UART -----
        UART_check();
        
        // ----- 60. FREE -----
        // ----- 70. FREE -----
        // ----- 80. FREE -----
        // nothing
        
        // ----- 90. ANALOG MEASUREMENT -----
        ADC_check();
        // ----- 100. CROSS FIRE DETECTOR -----
        CSFR_check();
    }
}

// *****************************************  *****************************************  *****************************************
// *****     INTERRUPT HANDLERS        *****  *****     INTERRUPT HANDLERS        *****  *****     INTERRUPT HANDLERS        *****
// *****************************************  *****************************************  *****************************************

/****************************************
*****   TIMER 0 for 1ms timing      *****
****************************************/
void TIMER_0_INST_IRQHandler()
{
    // ----- Heart beat -----
    if ( --cnt_Heart_LED == 0 )
    {
        if ( gLED_Seq_Now & ( 1 << cnt_Heart_Tick ) )
        {
            DL_GPIO_setPins( GPIO_PORT , GPIO_GRN4_PIN );
            cnt_Heart_LED = LED_HEART_ON_MS;
        }
        else {
            DL_GPIO_clearPins( GPIO_PORT , GPIO_GRN4_PIN );
            cnt_Heart_LED = LED_HEART_OFF_MS;
        }        

        cnt_Heart_Tick++;
        cnt_Heart_Tick %= LED_TICK_COUNT;
    }
    // ----- MOTOR SPEED/OMEGA MEASUREMENT -----
    if ( --cnt_OuterLoop == 0 )
    {
        cnt_OuterLoop = TIME_MOTOR_OMEGA_MS;
        flg_RUN_OuterLoop = true;
    }
    // ----- MPU DELAY -----
    if ( cnt_Delay_MPU > 0 )
        --cnt_Delay_MPU;
    // ----- READ DATA FROM MPU -----
    if ( --cnt_Read_MPU == 0 )
    {
        cnt_Read_MPU = TIME_MPU_READ_MS;
        flg_Read_MPU = true;
    }
    // ----- SEND DATA OVER UART -----
    if ( --cnt_Send_UART == 0 )
    {
        cnt_Send_UART = TIME_SEND_DATA_MS;
        flg_Send_UART = true;
    }
    if ( --cnt_Read_CRSF == 0 )
    {
        cnt_Read_CRSF = TIME_CRSF_MS;
        flg_Read_CRSF = true;
    }
    // COMMUNICATION TIME OUT - motors STOP
    if ( cnt_Fail_Safe > 0 )
        --cnt_Fail_Safe;
}

// Chan 0: VS1(Battery)
// Chan 1: Current M0
// Chan 2: Current M1
// Chan 3: Temp M0
// Chan 4: Temp M1   

/****************************************
*****   ADC 0 - MOTOR 1 - TEMP+CURR *****
****************************************/
void ADC12_0_INST_IRQHandler( void )
{
    switch  ( DL_ADC12_getPendingInterrupt( ADC12_0_INST ) )
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            flg_ADC[ 4 ] = true;
            ADC_LSB[ 4 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_TEMP_M1 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            flg_ADC[ 2 ] = true;
            ADC_LSB[ 2 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_CURR_M1 );
            break;    
        default:
            break;
    }
}

/****************************************
***** ADC 1 - MOTOR 0 - TEMP+CURR+VS ****
****************************************/
void ADC12_1_INST_IRQHandler( void )
{
    switch  ( DL_ADC12_getPendingInterrupt( ADC12_1_INST ) )
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            flg_ADC[ 3 ] = true;
            ADC_LSB[ 3 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_TEMP_M0 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            flg_ADC[ 1 ] = true;
            ADC_LSB[ 1 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_CURR_M0 );
            break;    
        case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            flg_ADC[ 0 ] = true;
            ADC_LSB[ 0 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_VS1 );
            break; 
        default:
            break;
    }
}

/****************************************
*****   UART 0 TRANSCEIVER          *****
****************************************/
void UART_0_INST_IRQHandler( void )
{
    switch ( DL_UART_Main_getPendingInterrupt( UART_0_INST ) )
    {
        case DL_UART_MAIN_IIDX_RX:
            UART0_RXbytes &= UART0_RX_BUFFER_SIZE - 1U;  // mask to prevent overflow
            UART0_RXbuffer[ UART0_RXbytes ] = DL_UART_Main_receiveData( UART_0_INST );
            // Check terminator
            if( UART0_RXbuffer[ UART0_RXbytes ] == '\n' )
                UART0_terminator_detected = true;
            UART0_RXbytes++;
        break;

        case DL_UART_MAIN_IIDX_TX:
            //transmit byte
            DL_UART_Main_transmitData ( UART_0_INST , UART0_message[ UART0_TXbytes++ ] );
            // Disable transmit interrupt if no more bytes
            if ( UART0_TXbytes == UART0_MessageLength )
            {
                DL_UART_Main_disableInterrupt( UART_0_INST , DL_UART_MAIN_INTERRUPT_TX );
                UART0_TXbytes = 0;
            }
        break;

        default:
            break;
    }
}

/****************************************
*****   UART 1 TRANSCEIVER          *****
****************************************/
void UART_1_INST_IRQHandler( void )
{
    switch ( DL_UART_Main_getPendingInterrupt( UART_1_INST ) )
    {
        case DL_UART_MAIN_IIDX_RX:
            UART1_RXbuffer[ UART1_RXbuffer_index ] = DL_UART_Main_receiveData( UART_1_INST );
            UART1_RXbuffer_index = ( UART1_RXbuffer_index + 1 ) % UART1_RX_BUFFER_SIZE;
                // UART1_RXbuffer[ UART1_RXbuffer_index++ % UART1_RX_BUFFER_SIZE ] = DL_UART_Main_receiveData( UART_1_INST );  
        break;

        case DL_UART_MAIN_IIDX_TX:

        break;

        default:
            break;
    }
}

/****************************************
*****   I2C 1 TRANSMITTER          *****
****************************************/
void I2C1_INST_IRQHandler ( void )
{
    switch ( DL_I2C_getPendingInterrupt( I2C1_INST ) )
    {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            
            DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            /* Receive all bytes from target */
            while ( DL_I2C_isControllerRXFIFOEmpty( I2C1_INST ) != true )
            {
                if ( gRxCount < gRxLen )
                {
                    gRxPacket[ gRxCount++ ] = DL_I2C_receiveControllerData( I2C1_INST );
                } else {
                    /* Ignore and remove from FIFO if the buffer is full */
                    DL_I2C_receiveControllerData( I2C1_INST );
                }
            }

            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            /* Fill TX FIFO with next bytes to send */
            if ( gTxCount < gTxLen )
            {
                gTxCount += DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ gTxCount ] , gTxLen - gTxCount );
            }
            break;
            /* Not used for this example */
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ( ( gI2cControllerStatus == I2C_STATUS_RX_STARTED ) ||
                 ( gI2cControllerStatus == I2C_STATUS_TX_STARTED ) )
            {
                /* NACK interrupt if I2C Target is disconnected */
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
        case DL_I2C_IIDX_CONTROLLER_START:
        case DL_I2C_IIDX_CONTROLLER_STOP:
        case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
        case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
        default:
            break;
    }
}

// *****************************************  *****************************************  *****************************************
// **********      FUNCTIONS      **********  **********      FUNCTIONS      **********  **********      FUNCTIONS      **********
// *****************************************  *****************************************  *****************************************

/**************************
*****   MCU INIT      *****
**************************/
void MCU_Init( void )
{
    SYSCFG_DL_init();
    // ***** TIMER 0 - enable interrupt *****
    NVIC_EnableIRQ( TIMER_0_INST_INT_IRQN) ;  // 1ms
    // ***** ADC - enable interrupt *****
    NVIC_EnableIRQ( ADC12_0_INST_INT_IRQN );
    NVIC_EnableIRQ( ADC12_1_INST_INT_IRQN );
    DL_ADC12_startConversion( ADC12_0_INST );
    DL_ADC12_startConversion( ADC12_1_INST );
    // ***** UART 0 - enable interrupt *****
    NVIC_ClearPendingIRQ( UART_0_INST_INT_IRQN );
    NVIC_EnableIRQ( UART_0_INST_INT_IRQN );
    NVIC_ClearPendingIRQ( UART_1_INST_INT_IRQN );
    NVIC_EnableIRQ( UART_1_INST_INT_IRQN );
    // ***** I2C 1 - enable interrupt *****
    NVIC_EnableIRQ( I2C1_INST_INT_IRQN);
    // ***** DMA - TIMG7 - WS2812B RGB LED *****
    DL_DMA_setSrcAddr     ( DMA , DMA_CH0_CHAN_ID , (uint32_t) &gLEDColors_8b_GRB[0][0] );
    DL_DMA_setDestAddr    ( DMA , DMA_CH0_CHAN_ID , TIMG7_BASE + 0x1810 );   // CC_0
    DL_DMA_setTransferSize( DMA , DMA_CH0_CHAN_ID , WS2812B_BIT_COUNT );
    DL_DMA_enableChannel  ( DMA , DMA_CH0_CHAN_ID );
    // ***** default motor value - motor stop *****
    SET_PWM_DUTY( PHASE_A , 0U );       // MOTOR 0 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_B , 0U );       // MOTOR 0 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
    SET_PWM_DUTY( PHASE_C , 0U );       // MOTOR 1 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_D , 0U );       // MOTOR 1 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
}

/*****************************
*****   Controller CONTROLLER   *****
*****************************/
void PID_Inner_Loop( void )
{
    if ( flg_RUN_InnerLoop )
    {
STORE_TIMER6(4);
        flg_RUN_InnerLoop = false;

        // ------------------------------
        // ----- 40. (B) INNER LOOP -----
        // ------------------------------
        // --- 40.5 Calculate Feed Forward (no error) ---
        gActVal_FeedF_Volt   = gK0_FeedForward * gRC_CMD_Pitch;
        // --- 41. Calculate DUMPING (no error) ---
        gActVal_Rates_Volt   = gK1_DynamicDamp * gRate_Y_Degs;         // Damping for preventing flip over
        // --- 43. Calculate Gain from Angle (error) ---
        gPID_Error_Angle_Deg = gAngle_setPoint_deg - gAngle_PITCH_Deg_CF;
        gActVal_Angle_Volt   = gK2_BalanceAngle * gPID_Error_Angle_Deg;  // Angle gain for upright position
        // --- 44. Combine ALL Motor Action Values ---
        gActVal_Robot_Volt = gActVal_Angle_Volt + gActVal_Rates_Volt + gActVal_FeedF_Volt;
        // --- 45. Motor Voltage Limit ---
        if ( gActVal_Robot_Volt > VOLT_MOTOR_LIM_HI ) gActVal_Robot_Volt = VOLT_MOTOR_LIM_HI;
        if ( gActVal_Robot_Volt < -VOLT_MOTOR_LIM_HI ) gActVal_Robot_Volt = -VOLT_MOTOR_LIM_HI;

        // --------------------------
        // ---- 46. MOTOR CONTROL ---
        // --------------------------
        if ( flg_Motor_EN && ( gBatt_Volt_EMA > VOLT_BATTERY_MIN ) )
        {
            // --- Positive Angle with Hysteresis ---
            if ( gAngle_PITCH_Deg_CF > +ANGLE_LIM_BODY_DEG + ANGLE_LIM_HYST_DEG )
            {
                // Motors OFF - Robot falling
                Motor_M0_volt( 0.0f );
                Motor_M1_volt( 0.0f );
            } else
                // --- Negative Angle with Hysteresis ---
                if ( gAngle_PITCH_Deg_CF < -ANGLE_LIM_BODY_DEG - ANGLE_LIM_HYST_DEG )
                {
                    // Motors OFF - Robot falling
                    Motor_M0_volt( 0.0f );
                    Motor_M1_volt( 0.0f );
                } else
                    // --- Angle in Range ---
                    if ( ( gAngle_PITCH_Deg_CF < +ANGLE_LIM_BODY_DEG ) &&
                         ( gAngle_PITCH_Deg_CF > -ANGLE_LIM_BODY_DEG )
                        )            
                    {
                        // --- Standard mixing ---
                        gMotor_RH_Volt = gActVal_Robot_Volt + gActVal_TurnRH_Volt;
                        gMotor_LH_Volt = gActVal_Robot_Volt + gActVal_TurnLH_Volt;
                        // --- Output ---
                        Motor_M0_volt(gMotor_LH_Volt);
                        Motor_M1_volt(gMotor_RH_Volt);
                    }
        } else {
            Motor_M0_volt( 0.0f );
            Motor_M1_volt( 0.0f );
        }
GET_DURATION(4);    
    }
}

// ----- OMEGA CALCULATION -----
void PID_Outer_Loop( void )
{
    if ( flg_RUN_OuterLoop )
    {
STORE_TIMER6(3);
        flg_RUN_OuterLoop = false;
        
        // -------------------------
        // ----- (B) OUTER LOOP ----
        // -------------------------
        // --- 310. Calculate Wheel Angle ---
        gTheta_1_LH_rad = 2.0f * 3.14159f * gTheta_1_LH_pos / MOTOR_GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        gTheta_0_RH_rad = 2.0f * 3.14159f * gTheta_0_RH_pos / MOTOR_GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        // --- 311. Calculate Wheel distance ---
        gWheel_LH_m     = gTheta_1_LH_rad * WHEEL_RADIUS_M;
        gWheel_RH_m     = gTheta_0_RH_rad * WHEEL_RADIUS_M;
        // --- 312. Calculate Wheel speed ---
        Wheel_GetSpeed_SG();    // return value in variables: gWheel_LH_msec and gWheel_RH_msec
        // --- 313. Calculate ROBOT Position ---
        gPosition_m     = 0.5f * ( gWheel_LH_m + gWheel_RH_m );
        // --- 314. Calculate ROBOT Speed ---
        gSpeed_msec     = Robot_GetSpeed_SG();      // Savitzky-Golay Derivative Filter
        // --- 315. Calculate ROBOT Angle ---
        gTurn_rad = (gWheel_RH_m - gWheel_LH_m) / WHEEL_TRACK_M;
        // --- 316. Calculate ROBOT Angle rate ---
        gTurnR_rads = Robot_GetYawRate_SG();    // Savitzky-Golay Derivative Filter
        // --- 320. Position Controller ---
        gPosition_error_m    = gPosition_setPoint_m - gPosition_m;
        gPosition_ActVal_deg = gPosition_error_m * gPosition_Gain;  // [deg] = [m] * [deg/m]
        // --- 330. Speed Controller ---
        gSpeed_error_msec    = gSpeed_setPoint_msec - gSpeed_msec;
        gSpeed_ActVal_deg    = gSpeed_error_msec * gSpeed_Gain;     // [deg] = [m/sec] * [deg/m/sec]
        // --- 340. POSITION CONTROLLER ---
        // gAngle_setPoint_deg  = gPosition_ActVal_deg + gSpeed_ActVal_deg;
        // --- 350. Angle Set Point Limit ---
        // #define     ANGLE_LIMIT     ( 2.0f )
        // if ( gAngle_setPoint_deg > ANGLE_LIMIT ) gAngle_setPoint_deg = ANGLE_LIMIT;
        // if ( gAngle_setPoint_deg < -ANGLE_LIMIT ) gAngle_setPoint_deg = -ANGLE_LIMIT;
        
        // -------------------------
        // ----- (C) SIDE LOOP -----
        // -------------------------
        // --- 360. TURN CONTROLLER Error---
        // gTurn_Error_RH_Rad  = gTurn_SetPoint_Rad - gTheta_0_RH_rad;
        // gTurn_Error_LH_Rad  = gTurn_SetPoint_Rad - gTheta_1_LH_rad;
        // --- 361. Action Value ---
        // gTurn_AV_RH_Volt    = gK3_Turn_Gain_RH * gTurn_Error_RH_Rad;
        // gTurn_AV_LH_Volt    = gK4_Turn_Gain_LH * gTurn_Error_LH_Rad;
        // --- 362. Motor Voltage Limit ---
        // if ( gTurn_AV_RH_Volt > VOLT_MOTOR_LIM_LO ) gTurn_AV_RH_Volt = VOLT_MOTOR_LIM_LO;
        // if ( gTurn_AV_RH_Volt < -VOLT_MOTOR_LIM_LO ) gTurn_AV_RH_Volt = -VOLT_MOTOR_LIM_LO;
        // if ( gTurn_AV_LH_Volt > VOLT_MOTOR_LIM_LO ) gTurn_AV_LH_Volt = VOLT_MOTOR_LIM_LO;
        // if ( gTurn_AV_LH_Volt < -VOLT_MOTOR_LIM_LO ) gTurn_AV_LH_Volt = -VOLT_MOTOR_LIM_LO;

GET_DURATION(3);    
    }
}

void    CSFR_check ( void )
{
    if ( flg_Read_CRSF )
    {
        flg_Read_CRSF = false;
        if ( CSFR_Extract_Data() && cnt_Fail_Safe > 0 )
        {
            CSFR_Parse_Data();
            //run motors
            gRC_CMD_Roll  = (int16_t)( ( (float)RC_channels[ 0 ] - 992.0f ) / 8.19f ); // scale to -100 0 +100
            gRC_CMD_Pitch = (int16_t)( ( (float)RC_channels[ 1 ] - 992.0f ) / 8.19f ); // scale to -100 0 +100
            // manipulate gains based on RC
            // Input range (from your RC receiver)
            const float x1 = 191.0f;    // min RC value
            const float x2 = 1792.0f;   // max RC value
            // Output range (your desired scaling)
            const float y1 = 1.00f;
            const float y2 = 1.50f;
            // Compute linear coefficients
            float a = (y2 - y1) / (x2 - x1);
            float b = y1 - (a * x1);
            // --- apply new gains ---
            gK0_FeedForward  = K0_FEEDFOR_DEF * ( ( a * (float)RC_channels[ 4 ] ) + b ) * ( ( a * (float)RC_channels[ 7 ] ) + b );
            gK2_BalanceAngle = K2_BALANCE_DEF * ( ( a * (float)RC_channels[ 5 ] ) + b );
            gK1_DynamicDamp  = K1_DAMPIMG_DEF * ( ( a * (float)RC_channels[ 6 ] ) + b );

            // ROLL is actually used as YAW/TURN command
            gActVal_TurnRH_Volt    = gK3_Turn_Gain_RH * gRC_CMD_Roll;
            gActVal_TurnLH_Volt    = gK4_Turn_Gain_LH * gRC_CMD_Roll;
            
            if ( gActVal_TurnRH_Volt > VOLT_MOTOR_LIM_LO ) gActVal_TurnRH_Volt = VOLT_MOTOR_LIM_LO;
            if ( gActVal_TurnRH_Volt < -VOLT_MOTOR_LIM_LO ) gActVal_TurnRH_Volt = -VOLT_MOTOR_LIM_LO;
            if ( gActVal_TurnLH_Volt > VOLT_MOTOR_LIM_LO ) gActVal_TurnLH_Volt = VOLT_MOTOR_LIM_LO;
            if ( gActVal_TurnLH_Volt < -VOLT_MOTOR_LIM_LO ) gActVal_TurnLH_Volt = -VOLT_MOTOR_LIM_LO;
            // gTurn_SetPoint_Rad += 0.015f*RC_roll; // integrate

            // PITCH
            gAngle_setPoint_deg = -0.50f*gRC_CMD_Pitch;
            #define PITCH_ANGLE_LIMIT (25.0f)
            if ( gAngle_setPoint_deg >  PITCH_ANGLE_LIMIT ) { gAngle_setPoint_deg =  PITCH_ANGLE_LIMIT; }
            if ( gAngle_setPoint_deg < -PITCH_ANGLE_LIMIT ) { gAngle_setPoint_deg = -PITCH_ANGLE_LIMIT; }
            
        }
        else
        {
            // default
            gAngle_setPoint_deg = gAngle_setPoint_deg * 0.9f; // exp decay
            gRC_CMD_Roll = 0;
            gRC_CMD_Pitch = 0;
        }
    }
}

/*****************************
*****   MOTOR VOLTAGE   *****
*****************************/

void Motor_M0_volt( float argVoltM0 )
{
    static int8_t m0_state = 0;   // -1 = reverse, 0 = stop, +1 = forward
    uint16_t pwm_uint_value_M0 = 0U;

    // --- Battery OK? ---
    if (gBatt_Volt_EMA > VOLT_BATTERY_MIN)
    {
        // --- Hysteresis decision ---
        // Forward RUN threshold
        if (argVoltM0 > (VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m0_state = +1;
        }
        // Reverse RUN threshold
        else if (argVoltM0 < -(VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m0_state = -1;
        }
        // STOP threshold (both directions)
        else if (argVoltM0 <  (VOLT_MOTOR_MIN - VOLT_MOTOR_HYST) &&
                 argVoltM0 > -(VOLT_MOTOR_MIN - VOLT_MOTOR_HYST))
        {
            m0_state = 0;
        }

        // --- Apply state ---
        if (m0_state == +1)
        {
            pwm_uint_value_M0 = (uint16_t)(100.0f * argVoltM0 / gBatt_Volt_EMA);
            SET_PWM_DUTY(PHASE_A, 0U);
            SET_PWM_DUTY(PHASE_B, pwm_uint_value_M0);
        }
        else if (m0_state == -1)
        {
            pwm_uint_value_M0 = (uint16_t)(-100.0f * argVoltM0 / gBatt_Volt_EMA);
            SET_PWM_DUTY(PHASE_A, pwm_uint_value_M0);
            SET_PWM_DUTY(PHASE_B, 0U);
        }
        else
        {
            SET_PWM_DUTY(PHASE_A, 0U);
            SET_PWM_DUTY(PHASE_B, 0U);
        }
    }
    else
    {
        // --- Battery LOW - STOP ---
        m0_state = 0;
        SET_PWM_DUTY(PHASE_A, 0U);
        SET_PWM_DUTY(PHASE_B, 0U);
    }
}

void Motor_M1_volt( float argVoltM1 )
{
    static int8_t m1_state = 0;   // -1 = reverse, 0 = stop, +1 = forward
    uint16_t pwm_uint_value_M1 = 0U;

    // --- Battery OK? ---
    if (gBatt_Volt_EMA > VOLT_BATTERY_MIN)
    {
        // --- Hysteresis decision ---
        // Forward RUN threshold
        if (argVoltM1 > (VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m1_state = +1;
        }
        // Reverse RUN threshold
        else if (argVoltM1 < -(VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m1_state = -1;
        }
        // STOP threshold (both directions)
        else if (argVoltM1 <  (VOLT_MOTOR_MIN - VOLT_MOTOR_HYST) &&
                 argVoltM1 > -(VOLT_MOTOR_MIN - VOLT_MOTOR_HYST))
        {
            m1_state = 0;
        }

        // --- Apply state ---
        if (m1_state == +1)
        {
            pwm_uint_value_M1 = (uint16_t)(100.0f * argVoltM1 / gBatt_Volt_EMA);
            SET_PWM_DUTY(PHASE_C, pwm_uint_value_M1);
            SET_PWM_DUTY(PHASE_D, 0U);
        }
        else if (m1_state == -1)
        {
            pwm_uint_value_M1 = (uint16_t)(-100.0f * argVoltM1 / gBatt_Volt_EMA);
            SET_PWM_DUTY(PHASE_C, 0U);
            SET_PWM_DUTY(PHASE_D, pwm_uint_value_M1);
        }
        else
        {
            SET_PWM_DUTY(PHASE_C, 0U);
            SET_PWM_DUTY(PHASE_D, 0U);
        }
    }
    else
    {
        // --- Battery LOW - STOP ---
        m1_state = 0;
        SET_PWM_DUTY(PHASE_C, 0U);
        SET_PWM_DUTY(PHASE_D, 0U);
    }
}

/**************************************
*****     SEND DATA OVER UART     *****
**************************************/
uint16_t Send_integers_over_UART( uint8_t argCount )
{
    // ten thousands, thousands, hundreds, tens, ones - store variables for each decimal order
    int16_t     A = 0 , B = 0 , C = 0 , D = 0 , E = 0;
    uint16_t    i = 0U;     // number counter
    uint16_t    j = 0U;     // char in string counter
    // MAIN FOR-CYCLE for conversion integers into strings
    for ( i = 0 ; i < argCount ; i++ )
    {
        // limiting -99 999 to +99 999
        if ( UART0_signals[ i ] > 99999 )
            UART0_signals[ i ] = 99999;
        if ( UART0_signals[ i ] < -99999 )
            UART0_signals[ i ] = -99999;
           
        // negative sign assigment
        if ( UART0_signals[ i ] < 0  )
        {
            // negative number will be inverted to positive
            UART0_signals[ i ] = -1 * UART0_signals[ i ];
            // write '-' symbol to signalize negative number
            UART0_message[ j++ ] = '-';
        }
       
        // ten thousands, thousands, hundreds, tens, ones variables are cleared for cycle run
        A = 0;
        B = 0;
        C = 0;
        D = 0;
        E = 0;
       
        // calculate a cipher for each decimal order
        // ten thousands
        if ( UART0_signals[ i ] >= 10000 )
            A = UART0_signals[ i ] / 10000;
        // thousands
        if ( UART0_signals[ i ] >= 1000 )    
            B = ( UART0_signals[ i ] - A * 10000 ) / 1000;
        // hundreds
        if ( UART0_signals[ i ] >= 100 )    
            C = ( UART0_signals[ i ] - A * 10000 - B * 1000 ) / 100;
        // tens
        if ( UART0_signals[ i ] >= 10 )        
            D = ( UART0_signals[ i ] - A * 10000 - B * 1000 - C * 100 ) / 10;
        // ones
        E = ( UART0_signals[ i ] - A * 10000 - B * 1000 - C * 100 - D * 10 ) % 10;
       
        // string creation, leading zeros are inhibited, base is '0'
        if ( A > 0 )
            UART0_message[ j++ ] = '0' + A;
        if ( ( A > 0 ) || ( B > 0 ) )
            UART0_message[ j++ ] = '0' + B;
        if ( ( A > 0 ) || ( B > 0 ) || ( C > 0 ) )
            UART0_message[ j++ ] = '0' + C;
        if ( ( A > 0 ) || ( B > 0 ) || ( C > 0 ) || ( D > 0 ) )
            UART0_message[ j++ ] = '0' + D;
        // ones will be always writte becase of 0
        UART0_message[ j++ ] = '0' + E;
        // comma separated data
        if ( i < argCount - 1 )
            UART0_message[ j++ ] = ',';
        else
            // last character as requested by Processing Grapher
            // https://wired.chillibasket.com/processing-grapher/
            UART0_message[ j++ ] = '\n';
    }
   
    UART0_MessageLength = j;
    DL_UART_Main_enableInterrupt( UART_0_INST, DL_UART_MAIN_INTERRUPT_TX );
   
    return j;
}

// ----- ANALOG MEASUREMENT -----
// 5 channel used: (0)VS1(Battery), (1)Current M0, (2)Current M1 , (3)Temp M0, (4)Temp M1     
void ADC_check( void )
{
    for ( int i = 0 ; i < ADC_CNT ; i++ )
    {
        if ( flg_ADC[ i ] )
        {
            flg_ADC[ i ] = false;

            ADC_Volt[ i ] = (float)ADC_LSB[ i ] * VOLT_ADC_REF / 4096.0f;   // LSB to volt 

            // >> BATTERY <<
            if ( i == 0 ) {
                gBatt_Volt       = VOLT_SCHTKY_DROP + ADC_Volt[ 0 ] * 11.0f;      // 11 => divider ( 4k7 + 47k ) / 4k7
                gBatt_Volt_EMA   = gBETA_EMA_Coef * gBatt_Volt + (1.0f-gBETA_EMA_Coef) * gBatt_Volt_EMA;   // Exponential Moving Average
            }

            // >> MOTOR 0 <<
            // Motor_current[A] = V_sns[V] / ( R_sns[R] * INA_GAIN[-] ) => Motor_cuureent = V_sns / 0.033R * 20
            if ( i == 1 ) { // Motor 0
                // MEDIAN of 3 - current spike elimination
                ADC_LSB_M0_X = ADC_LSB_M0_Y;
                ADC_LSB_M0_Y = ADC_LSB_M0_Z;
                ADC_LSB_M0_Z = ADC_LSB[ 1 ];
                ADC_LSB_M0_MED3 = median3( ADC_LSB_M0_X , ADC_LSB_M0_Y , ADC_LSB_M0_Z );
                gM0_RH_Curr_mA      = 1000.0f * ( ADC_Volt[ 1 ] ) / ( 0.033f * 20.0f );
                gM0_RH_Curr_mA_Med  = 1000.0f * ( (float)ADC_LSB_M0_MED3 * VOLT_ADC_REF / 4096.0f ) / ( 0.033f * 20.0f );
                gM0_RH_Curr_mA_EMA  = gZETA_EMA_Coef * gM0_RH_Curr_mA_Med + (1.0f-gZETA_EMA_Coef) * gM0_RH_Curr_mA_EMA;
            }
            if ( i == 2 ) { // Motor 1
                // MEDIAN of 3 - current spike elimination
                ADC_LSB_M1_X = ADC_LSB_M1_Y;
                ADC_LSB_M1_Y = ADC_LSB_M1_Z;
                ADC_LSB_M1_Z = ADC_LSB[ 2 ];
                ADC_LSB_M1_MED3 = median3( ADC_LSB_M1_X , ADC_LSB_M1_Y , ADC_LSB_M1_Z );
                gM1_LH_Curr_mA      = 1000.0f * ( ADC_Volt[ 2 ] ) / ( 0.033f * 20.0f );
                gM1_LH_Curr_mA_Med  = 1000.0f * ( (float)ADC_LSB_M1_MED3 * VOLT_ADC_REF / 4096.0f ) / ( 0.033f * 20.0f );
                gM1_LH_Curr_mA_EMA  = gZETA_EMA_Coef * gM1_LH_Curr_mA_Med + (1.0f-gZETA_EMA_Coef) * gM1_LH_Curr_mA_EMA;
                // gM1_LH_Curr_mA = 1000.0f * ADC_volt[ 2 ] / ( 0.033f * 20.0f );
            }
        }
    }
}

void UART_check( void )
{
    // ----- COMMAND RECEIVED CHECK -----
    if ( UART0_terminator_detected ) {
STORE_TIMER6(5);
            
        UART0_terminator_detected = false;

        // checkForGainCommand();   
        checkForCommand2();

        clearUART0_RXbuffer_Zero();   
GET_DURATION(5);
    }
    // ----- SEND DATA OVER UART -----
    if ( flg_Send_UART )
    {
STORE_TIMER6(6);
        flg_Send_UART = false;

        switch ( gRobot_State )
        {
            // ----- OBSERVATION ------  
            case ROBO_STATE_INIT:                  
                break;
            // ----- REGULATION ------    
            case ROBO_STATE_BALA:                    
                UART0_signals[  0 ] = 100.0f * gAngle_PITCH_Deg;         // raw Pitch FAST
                UART0_signals[  1 ] = 100.0f * gAngle_PITCH_Deg_CF;   // filtered Pitch
                // UART0_signals[  2 ] = 100.0f * PID_actVal;              // Total Action value volt
                // UART0_signals[  3 ] = 100.0f * PID_P_term_volt;         // P volt
                // UART0_signals[  4 ] = 100.0f * PID_I_term_volt;         // I volt
                // UART0_signals[  5 ] = 100.0f * PID_D_term_volt;         // D volt
                // UART0_signals[  6 ] = 100.0f * PID_C_LHRH_volt;         // C volt
                UART0_signals[  7 ] = 100.0f * gActVal_Rates_Volt ;         // G volt

                UART0_signals[  8 ] = 1.000f * gM0_RH_Curr_mA;          // raw Motor I
                UART0_signals[  9 ] = 1.000f * gM0_RH_Curr_mA_EMA;      // filtered Motor I
                UART0_signals[ 10 ] = 1.000f * gM1_LH_Curr_mA;          // raw Motor I
                UART0_signals[ 11 ] = 1.000f * gM1_LH_Curr_mA_EMA;      // filtered Motor I
                // UART0_signals[ 12 ] = 100.0f * Motor0_theta_rad;        // motor shaft angle
                // UART0_signals[ 13 ] = 100.0f * Motor1_theta_rad;        // motor shaft angle
                // UART0_signals[ 14 ] = 100.0f * Motor0_omega_rads;       // motor speed
                // UART0_signals[ 15 ] = 100.0f * Motor1_omega_rads;       // motor speed
                
                // UART0_signals[ 16 ] = 100.0f * P_0_gain;                  // P gain
                // UART0_signals[ 17 ] = 100.0f * I_0_gain;                  // I gain
                // UART0_signals[ 18 ] = 100.0f * D_0_gain;                  // D gain
                // UART0_signals[ 19 ] = 100.0f * C_gain_LH_RH;            // LH to RH compensation
                // UART0_signals[ 20 ] = 100.0f * N_0_coef;                  // N filter for D component
                UART0_signals[ 21 ] = 100.0f * gK1_DynamicDamp;                  // G damping - gyro
                UART0_signals[ 22 ] = 100.0f * gALPHA_CF_Coef;            // Alpha coef. for complementary filter Acc+Gyro
                UART0_signals[ 23 ] = 100.0f * gZETA_EMA_Coef;           // Zeta coef. for Exponencial

                UART0_signals[ 24 ] = 100.0f * gBatt_Volt_EMA;           // Filtered battery voltage
                UART0_signals[ 25 ] = 100.0f * gMPU_Temp_C;              // temperature of MPU
                UART0_signals[ 26 ] = 0.0f;                             // empty - for further use
                UART0_signals[ 27 ] = 0.0f;                             // empty - for further use

                Send_integers_over_UART ( SIGNAL_CNT );

                // UART0_signals[  9 ] = 1.0f * gM1_LH_Curr_mA;
                break;
            // ----- CALIBRATION ------
            case ROBO_STATE_CALIB:                  
                break;    
        } 
GET_DURATION(6);
    }
}

bool checkForGainCommand( void )
{
    char gainType = UART0_RXbuffer[0];

    // Check if it matches [P|I|D|C|N|G]GAIN=
    if ( (gainType == 'P' || gainType == 'I' || gainType == 'D' || gainType == 'C' || gainType == 'N' || gainType == 'G' || gainType == 'A' || gainType == 'Z') &&
        UART0_RXbuffer[1] == 'G' &&
        UART0_RXbuffer[2] == 'A' &&
        UART0_RXbuffer[3] == 'I' &&
        UART0_RXbuffer[4] == 'N' &&
        UART0_RXbuffer[5] == '=' )
    {
        // Look for '\n' terminator
        for ( uint8_t i = 6; i < UART0_RX_BUFFER_SIZE; ++i )
        {
            if ( UART0_RXbuffer[i] == '\n' )
            {
                // Extract value substring
                char valueStr[8] = {0};
                uint8_t len = i - 6;
                if ( len > 0 && len < sizeof( valueStr ) )
                {
                    for ( uint8_t j = 0; j < len; ++j )
                    {
                        valueStr[j] = UART0_RXbuffer[6 + j];
                    }

                    int16_t value = atoi( valueStr );

                    // Assign to correct variable
                    switch ( gainType )
                    {
                        case 'P': 
                            P_Gain_received = value;
                            if ( 100 >= P_Gain_received && P_Gain_received >= 0 ) { // P shall be small and positive
                                // P_0_gain = (float)P_Gain_received / 100.0f;
                            }
                            break;
                        case 'I': 
                            I_Gain_received = value;
                            if ( 100 >= I_Gain_received && I_Gain_received >= 0 ) { // I shall be small and positive
                                // I_0_gain = (float)I_Gain_received / 10000.0f;
                            }
                            break;
                        case 'D':
                            D_Gain_received = value;
                            if ( 100 >= D_Gain_received && D_Gain_received >= -100  ) { // D shall be small
                                // D_0_gain = (float)D_Gain_received / 100.0f;
                            }
                            break;
                        case 'C':
                            C_Gain_received = value;
                            if ( 10000 >= C_Gain_received && C_Gain_received >= 0 ) {   // C could be large and positive
                                // C_gain_LH_RH = (float)C_Gain_received / 100.0f;
                            }
                            break;
                        case 'N':
                            N_Gain_received = value;
                            if ( 2000 >= N_Gain_received && N_Gain_received >= 0 ) {    // N could be large and positive
                                // N_0_coef = (float)N_Gain_received / 100.0f;
                            }
                            break;
                        case 'G':
                            G_Gain_received = value;
                            if ( 2000 >= G_Gain_received && G_Gain_received >= 0 ) {    // G could be large and positive
                                gK1_DynamicDamp = (float)G_Gain_received / 100.0f;
                            }
                            break;    
                        case 'A':
                            A_Filt_received = value;
                            if ( 100 >= A_Filt_received && A_Filt_received >= 0 ) {    // A shall be 0-100 (0-1.0)
                                gALPHA_CF_Coef = (float)A_Filt_received / 100.0f;
                            }
                            break;
                        case 'Z':
                            Z_Filt_received = value;
                            if ( 100 >= Z_Filt_received && Z_Filt_received >= 0 ) {    // Z shall be 0-100 (0-1.0)
                                gZETA_EMA_Coef = (float)Z_Filt_received / 100.0f;
                            }
                            break; 
                        default: return false;
                    }
                    return true;
                }
            }
        }
    } 
    return false;
}

bool checkForCommand2( void )
{
    char typeChar = UART0_RXbuffer[0];

    // -------- Branch 1: xGAIN= --------
    if (UART0_RXbuffer[1]=='G' && UART0_RXbuffer[2]=='A' &&
        UART0_RXbuffer[3]=='I' && UART0_RXbuffer[4]=='N' &&
        UART0_RXbuffer[5]=='=')
    {
        // find terminator
        for (uint8_t i=6; i<UART0_RX_BUFFER_SIZE; i++)
        {
            if (UART0_RXbuffer[i]=='\n')
            {
                char valueStr[8] = {0};
                uint8_t len = i-6;
                if (len>0 && len<sizeof(valueStr))
                {
                    for (uint8_t j=0; j<len; j++)
                        valueStr[j] = UART0_RXbuffer[6+j];

                    int16_t value = atoi(valueStr);

                    switch(typeChar)
                    {
                        case 'P': P_Gain_received=value;
                                  if(value>=0 && value<=100)
                                    //   P_0_gain=(float)value/100.0f;
                                  break;
                        case 'I': I_Gain_received=value;
                                  if(value>=0 && value<=100)
                                    //   I_0_gain=(float)value/10000.0f;
                                  break;
                        case 'D': D_Gain_received=value;
                                  if(value>=-100 && value<=100)
                                    //   D_0_gain=(float)value/100.0f;
                                  break;
                        case 'C': C_Gain_received=value;
                                  if(value>=0 && value<=10000)
                                    //   C_gain_LH_RH=(float)value/100.0f;
                                  break;
                        case 'N': N_Gain_received=value;
                                  if(value>=0 && value<=2000)
                                    //   N_0_coef=(float)value/100.0f;
                                  break;
                        case 'G': G_Gain_received=value;
                                  if(value>=0 && value<=2000)
                                      gK1_DynamicDamp=(float)value/100.0f;
                                  break;
                        case 'A': A_Filt_received=value;
                                  if(value>=0 && value<=100)
                                      gALPHA_CF_Coef=(float)value/100.0f;
                                  break;
                        case 'Z': Z_Filt_received=value;
                                  if(value>=0 && value<=100)
                                      gZETA_EMA_Coef=(float)value/100.0f;
                                  break;
                        default: return false;
                    }
                    return true;
                }
            }
        }
    }

    // -------- Branch 2: yPARA= --------
    if (UART0_RXbuffer[1]=='P' && UART0_RXbuffer[2]=='A' &&
        UART0_RXbuffer[3]=='R' && UART0_RXbuffer[4]=='A' &&
        UART0_RXbuffer[5]=='=')
    {
        for (uint8_t i=6; i<UART0_RX_BUFFER_SIZE; i++)
        {
            if (UART0_RXbuffer[i]=='\n')
            {
                char valueStr[8]={0};
                uint8_t len=i-6;
                if(len>0 && len<sizeof(valueStr))
                {
                    for(uint8_t j=0;j<len;j++)
                        valueStr[j]=UART0_RXbuffer[6+j];
                    int16_t value=atoi(valueStr);

                    // Example: assign to parameter variable
                    switch(typeChar)
                    {
                        // case 'K': Kpara_received=value;
                        //           Kpara_flag=(value!=0);
                        //           break;
                        // add other PARA parameters here
                        default: return false;
                    }
                    return true;
                }
            }
        }
    }

    // -------- Branch 3: zOFFS= --------
    if (UART0_RXbuffer[1]=='O' && UART0_RXbuffer[2]=='F' &&
        UART0_RXbuffer[3]=='F' && UART0_RXbuffer[4]=='S' &&
        UART0_RXbuffer[5]=='=')
    {
        for (uint8_t i=6; i<UART0_RX_BUFFER_SIZE; i++)
        {
            if (UART0_RXbuffer[i]=='\n')
            {
                char valueStr[8]={0};
                uint8_t len=i-6;
                if(len>0 && len<sizeof(valueStr))
                {
                    for(uint8_t j=0;j<len;j++)
                        valueStr[j]=UART0_RXbuffer[6+j];
                    int16_t value=atoi(valueStr);

                    // Example: assign to offset variable
                    switch( typeChar )
                    {
                        // case 'X': X_offset=value; break;
                        case 'Y':
                            Y_Offs_received = value;
                            if( 1 == value || value == -1 )
                                // PID_0_angle_ref += value;
                            // else if ( value == 0 )
                                // PID_0_angle_ref = ANGLE_UPRIGHT_DEG;
                        break;
                        // case 'Z': Z_offset=value; break;
                        // add other OFFS parameters here
                        default: return false;
                    }
                    return true;
                }
            }
        }
    }

    return false;
}

void clearUART0_RXbuffer_Zero( void )
{
    for ( int i = 0; i < UART0_RX_BUFFER_SIZE; ++i )
    {
        UART0_RXbuffer[i] = 0x00;
    }
    UART0_RXbytes = 0;
}

uint8_t table_crc8( void )
{
    uint8_t crc = 0;

    crc = crc8tab[ crc ^ 0x16 ];

    for ( uint8_t i = 0 ; i < CRSF_MSG_DATA_SIZE ; i++ )
        crc = crc8tab[ crc ^ CSFR_Data_RX[ i ] ];
    return crc;
}    

uint8_t compute_crc8( void )
{
    uint8_t crc = 0;

    crc ^= 0x16; // XOR with fitst byte

    for ( uint8_t j = 0 ; j < 8 ; j++ )
    {
        if ( crc & 0x80 )
        {
            crc = ( crc << 1 ) ^ CRSF_CRC_POLY; // Apply polynomial if MSB is set
        } else {
            crc <<= 1;
        }
    }

    for ( uint8_t i = 0 ; i < CRSF_MSG_DATA_SIZE ; i++ )
    {
        crc ^= CSFR_Data_RX[ i ]; // XOR with current byte

        for ( uint8_t j = 0 ; j < 8 ; j++ )
        {
            if ( crc & 0x80 )
            {
                crc = ( crc << 1 ) ^ CRSF_CRC_POLY; // Apply polynomial if MSB is set
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Function to extract data following the header
bool CSFR_Extract_Data( void ) 
{
    // Define header sequence - Sync Byte, Bytes count, 16 RC channels
    uint8_t header[ CRSF_MSG_HEAD_SIZE ] = { 0xC8 , 0x18 , 0x16 };

    // Search for header in the buffer
    for ( int i = 0 ; i < UART1_RX_BUFFER_SIZE ; i++ )
    {
        // Check for header match
        if ( UART1_RXbuffer[  i ] == header[ 0 ] &&
            UART1_RXbuffer[ ( i + 1 ) % UART1_RX_BUFFER_SIZE ] == header[ 1 ] &&
            UART1_RXbuffer[ ( i + 2 ) % UART1_RX_BUFFER_SIZE ] == header[ 2 ] )
        {
            CSFR_Header_match++;
            // run Fail Safe Timer
            cnt_Fail_Safe = TIME_FAIL_SAFE_MS;
            // Extract the 22 bytes following the header
            for ( int j = 0 ; j < CRSF_MSG_DATA_SIZE + CRSF_MSG_CRC_SIZE ; j++ )
            {
                CSFR_Data_RX[ j ] = UART1_RXbuffer[ ( i + CRSF_MSG_HEAD_SIZE + j ) % UART1_RX_BUFFER_SIZE ];
            }
            // clear UART buffer
            for ( int k = 0 ; k < UART1_RX_BUFFER_SIZE ; k++ )
                UART1_RXbuffer[ k ] = 0x00;

            CSFR_CRC_Calculated = table_crc8();

            if ( CSFR_CRC_Calculated == CSFR_Data_RX[ 22 ] )   // CRC match?
            {
                CSFR_CRC_match++;
                return true; // Data extraction successful and CRC is OK
            }
            else
            {
                CSFR_CRC_error++;
                return false; // Header found but CRC error
            }            
        }
    }
    return false; // Header not found or CRC error
}

uint16_t CSFR_Reverse11( uint16_t arg_data_for_rev )
{
    uint16_t reversed11 = 0;
    // reverse bits in each channel
    for ( uint8_t j = 0 ; j < 11 ; j++ )
    {
        if ( ( arg_data_for_rev >> j ) & 0x001 )
            reversed11 |= 1 << ( 10 - j );
    }
    return reversed11;
}

void CSFR_Parse_Data( void )
{
    uint8_t reversed = 0;
    
    // reverse bits in each byte
    for ( uint8_t i = 0 ; i < CRSF_MSG_DATA_SIZE ; i++ )
    {
        reversed = 0;
        
        for ( uint8_t j = 0 ; j < 8 ; j++ )
        {
            if ( ( CSFR_Data_RX[ i ] >> j ) & 0x01 )
                reversed |= 1 << ( 7 - j );
        }
        CSFR_Data_Reversed[ i ] = reversed;
        
    }
    
    RC_channels[ 0 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 0 ] & 0xFF ) << 3 ) | ( CSFR_Data_Reversed[  1 ] >> 5 ) );
    RC_channels[ 1 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 1 ] & 0x1F ) << 6 ) | ( CSFR_Data_Reversed[  2 ] >> 2 ) );
    RC_channels[ 2 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 2 ] & 0x3F ) << 9 ) | ( CSFR_Data_Reversed[  3 ] << 1 ) | ( CSFR_Data_Reversed[ 4 ] >> 7 ) );
    RC_channels[ 3 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 4 ] & 0x7F ) << 4 ) | ( CSFR_Data_Reversed[  5 ] >> 4 ) );
    
    RC_channels[ 4 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 5 ] & 0x0F ) << 7 ) | ( CSFR_Data_Reversed[  6 ] >> 1 ) );
    RC_channels[ 5 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 6 ] & 0x01 ) << 10 )| ( CSFR_Data_Reversed[  7 ] << 2 ) | ( CSFR_Data_Reversed[ 8 ] >> 6 ) );
    RC_channels[ 6 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 8 ] & 0x3F ) << 5 ) | ( CSFR_Data_Reversed[  9 ] >> 3 ) );
    RC_channels[ 7 ] = CSFR_Reverse11( ( ( CSFR_Data_Reversed[ 9 ] & 0x07 ) << 8 ) | ( CSFR_Data_Reversed[ 10 ] >> 0 ) );
    
}

// ----- MOTOR PWM -----
void SET_PWM_DUTY( uint8_t argPhase , uint16_t argDuty )
{
    // PWM limiter 0-100%
    if ( argDuty > PWM_MAX_DUTY )
        argDuty = PWM_MAX_DUTY;
    else if ( argDuty < PWM_MIN_DUTY )
        argDuty = 0;

    if ( argPhase > (PHASE_COUNT-1U) )     // if greather than 3 (4 phases)
        argPhase %= (PHASE_COUNT-1U) ;

    switch ( argPhase )
    {
        case 0:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty , GPIO_PWM_0_C0_IDX );
            break;
        case 1:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty , GPIO_PWM_0_C1_IDX );
            break;
        case 2:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty , GPIO_PWM_0_C2_IDX );
            break;       
        case 3:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty , GPIO_PWM_0_C3_IDX );
            break;    
    }
    
}

// *****************************************  *****************************************  *****************************************
// *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****
// *****************************************  *****************************************  *****************************************

void MPU_check ( void )
{
    // ----- READ MPU DATA -----        
    if ( flg_Read_MPU && ( cnt_Delay_MPU==0 ) )
    {
STORE_TIMER6(2);
        flg_Read_MPU = false;     
    
        switch ( MPU_switch )
        {
            case 0:
                MPU_ExitSleep_Return = MPU_ExitSleep();
                if (  MPU_ExitSleep_Return == 0 ) {
                    MPU_switch = 1;
                    gLED_Seq_Now = LED_SEQUENCE_2;
                    MPU_switch_transition_cnt++;
                } else {
                    MPU_switch = 0;
                    flg_Motor_EN = false;
                    gLED_Seq_Now = LED_SEQUENCE_1;
                    er_ct[0]++;
                }
                break;
            case 1:
                MPU_WhoAmI_data = 0x00;
                MPU_WhoAmI_Return = MPU_WhoAmI();   // takes ~105us @400kHz
                if ( MPU_WhoAmI_Return == 0 ) {
                    if ( MPU_WhoAmI_data == 104 ) {
                        MPU_switch = 2;
                        flg_Motor_EN = true;
                        gLED_Seq_Now = LED_SEQUENCE_3;
                        MPU_switch_transition_cnt++;
                    } else {
                        MPU_switch = 0;
                        flg_Motor_EN = false;
                        gLED_Seq_Now = LED_SEQUENCE_1;
                        er_ct[1]++;// ----- >>>>> ERROR OFTEN HERE <<<<< ----- 2025-11-25 17:22
                    }
                } else {
                    MPU_switch = 0;
                    flg_Motor_EN = false;
                    gLED_Seq_Now = LED_SEQUENCE_1;
                    er_ct[2]++;
                }
                break;
            case 2:      
                MPU_GetAccTempGyro_Return = MPU_GetAccTempGyro();   // takes ~410us @400kHz
                if ( MPU_GetAccTempGyro_Return != 0 ) { 
                    MPU_switch = 0;
                    flg_Motor_EN = false;
                    gLED_Seq_Now = LED_SEQUENCE_1;
                    er_ct[3]++;// ----- >>>>> ERROR OFTEN HERE <<<<< ----- 2025-11-25 17:22
                }

                break;
            default:
                MPU_switch = 0;
                gLED_Seq_Now = LED_SEQUENCE_1;
                er_ct[4]++;
                break;
        }
GET_DURATION(2);
    }
}

uint16_t MPU_ExitSleep( void )
{
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 2;
    gTxPacket[ 0 ] = 0x6B;  // PWM_MGMT1 register
    gTxPacket[ 1 ] = 0x00;  // Exit Sleep    

    I2C_Flag_Block = 0;

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );

    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gTxCount < gTxLen )
    {
        DL_I2C_enableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    } else {
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    }

    // Send the packet to the controller. This function will send Start + Stop automatically.
    gI2cControllerStatus = I2C_STATUS_TX_STARTED;

    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 5 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[5]++;
    }
    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
 
    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 6 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[6]++;
    }

    return I2C_Flag_Block;
}

uint16_t MPU_WhoAmI( void )
{
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 1;
    gTxPacket[ 0 ] = 0x75;  // WHO_AM_I register 

    I2C_Flag_Block = 0;

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );
 
    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gTxCount < gTxLen )
    {
        DL_I2C_enableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    } else {
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    }

    // Send the packet to the controller. This function will send Start + Stop automatically.
    gI2cControllerStatus = I2C_STATUS_TX_STARTED;
 
    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 7 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[7]++;
    }
  
    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    
    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 8 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[8]++;
    }

    // ANTIBLOCK - count down to 994
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 9 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[9]++;
    }
    
    // ANTIBLOCK - countdown to 768
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 10 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[10]++;
    }

    //----- REPEAT START -----

    // Send a read request to Target
    gRxLen               = 1;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );

    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 11 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[11]++;
    }
    
    MPU_WhoAmI_data = gRxPacket[ 0 ];	// read one byte

    return I2C_Flag_Block;
}

uint16_t MPU_GetAccTempGyro( void )
{
// ACCEL_XOUT_H
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 1;
    gTxPacket[ 0 ] = 0x3B;  // ACCEL_XOUT_H

    I2C_Flag_Block = 0;

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );
 
    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gTxCount < gTxLen )
    {
        DL_I2C_enableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    } else {
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    }
    
    // Send the packet to the controller. This function will send Start + Stop automatically.
    gI2cControllerStatus = I2C_STATUS_TX_STARTED;

    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 13 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[13]++;
    }

    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    
    // ANTIBLOCK - stays at 1000
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 14 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[14]++;
    }

    // ANTIBLOCK - count down to 994
    for ( I2C_Anti_Block = 8000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 15 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[15]++;// ----- >>>>> ERROR OFTEN HERE <<<<< ----- 2025-11-25 17:22
    }

    // ANTIBLOCK - count down to 768
    for ( I2C_Anti_Block = 1000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 16 ] = I2C_Anti_Block;
    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[16]++;
    }

    //----- REPEAT START -----
    // Send a read request to Target
    gRxLen               = 14;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (14 bytes - 2*(ACCx+ACCy+ACCz+GYROx+GYROy+GYROz+TEMP))
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );

// t0_us[0] = (uint16_t)DL_TimerG_getTimerCount(TIMER_6_INST);
    // ANTIBLOCK - count down to 8549 -> this takes 366us
    for ( I2C_Anti_Block = 10000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;
    ab_ct[ 17 ] = I2C_Anti_Block;
// dt_us[0] = (uint16_t)DL_TimerG_getTimerCount(TIMER_6_INST) - t0_us[0];

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        er_ct[17]++;
    }

    MPU_accX_raw = (int16_t)gRxPacket[ 0 ] << 8;	// read High byte
    MPU_accX_raw += (int16_t)gRxPacket[ 1 ];
    MPU_accX = MPU_accX_raw - gMPU_AccX_offset;

    MPU_accY_raw = (uint16_t)gRxPacket[ 2 ] << 8;	// read High byte
    MPU_accY_raw += gRxPacket[ 3 ];
    MPU_accY = MPU_accY_raw - gMPU_AccY_offset;

    MPU_accZ_raw = (uint16_t)gRxPacket[ 4 ] << 8;	// read High byte
    MPU_accZ_raw += gRxPacket[ 5 ];
    MPU_accZ = MPU_accZ_raw - gMPU_AccZ_offset;

    MPU_temp = (uint16_t)gRxPacket[ 6 ] << 8;	// read High byte
    MPU_temp += gRxPacket[ 7 ];

    gMPU_AccX_g = MPU_accX / 16384.0f; //
    gMPU_AccY_g = MPU_accY / 16384.0f; //
    gMPU_AccZ_g = MPU_accZ / 16384.0f; // this three lines takes 50us

// TRIGINOMETRY USED TO DETERMINE ANGLES - theta, psi, phi
// https://www.analog.com/en/resources/app-notes/an-1057.html

    gAngle_PITCH_Deg = RAD_TO_DEG * atan2( gMPU_AccX_g , ( sqrt( gMPU_AccY_g*gMPU_AccY_g + gMPU_AccZ_g*gMPU_AccZ_g ) ) ); //
    gAngle_ROLL_Deg  = RAD_TO_DEG * atan2( gMPU_AccY_g , ( sqrt( gMPU_AccX_g*gMPU_AccX_g + gMPU_AccZ_g*gMPU_AccZ_g ) ) ); //
    gAngle_YAW_Deg   = RAD_TO_DEG * atan2( ( sqrt( gMPU_AccX_g*gMPU_AccX_g + gMPU_AccY_g*gMPU_AccY_g ) ) , gMPU_AccZ_g ); //  this three lines takes 1.1ms@32MHz !!! 450us@80MHz

    gMPU_Temp_C = MPU_temp / 340.0f + 36.53f;  // 40us

    MPU_gyroX_raw = (uint16_t)gRxPacket[ 8 ] << 8;	// read High byte
    MPU_gyroX_raw += gRxPacket[ 9 ];
    MPU_gyroX = MPU_gyroX_raw - gMPU_GyroX_offset;

    MPU_gyroY_raw = (uint16_t)gRxPacket[ 10 ] << 8;	// read High byte
    MPU_gyroY_raw += gRxPacket[ 11 ];
    MPU_gyroY = MPU_gyroY_raw - gMPU_GyroY_offset;

    MPU_gyroZ_raw = (uint16_t)gRxPacket[ 12 ] << 8;	// read High byte
    MPU_gyroZ_raw += gRxPacket[ 13 ];
    MPU_gyroZ = MPU_gyroZ_raw - gMPU_GyroZ_offset;

// ******************************************
// ********** COMPLEMENTARY FILTER **********
// ******************************************    
// https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d
// angle = (1 - α) * (angle + gyroscope * dt) + α * accelerometer
// pitch = 0.98 * (pitch + gyroscope_x * dt) + 0.02* accelerometer_x

    // T_sampling, sensitivity 131LSB / deg/s
	gRate_X_Degs = ( ( MPU_gyroX * TIME_MPU_READ_SEC ) / 131.0f );
    gRate_Y_Degs = ( ( MPU_gyroY * TIME_MPU_READ_SEC ) / -131.0f );       // it seems this axis is reversed
    gRate_Z_Degs = ( ( MPU_gyroZ * TIME_MPU_READ_SEC ) / 131.0f );       // 3 lines 125us	
	
    gAngle_PITCH_Deg_CF = ( 1 - gALPHA_CF_Coef ) * ( gAngle_PITCH_Deg_0 + gRate_Y_Degs ) + gALPHA_CF_Coef * gAngle_PITCH_Deg;
	gAngle_ROLL_Deg_CF  = ( 1 - gALPHA_CF_Coef ) * ( gAngle_ROLL_Deg_0  + gRate_X_Degs ) + gALPHA_CF_Coef * gAngle_ROLL_Deg;
    gAngle_YAW_Deg_CF   = ( 1 - gALPHA_CF_Coef ) * ( gAngle_YAW_Deg_0   + gRate_Z_Degs ) + gALPHA_CF_Coef * gAngle_YAW_Deg;  // 3 lines 120us

    // re-shift
    gAngle_PITCH_Deg_0 = gAngle_PITCH_Deg_CF;
    gAngle_ROLL_Deg_0  = gAngle_ROLL_Deg_CF;
    gAngle_YAW_Deg_0   = gAngle_YAW_Deg_CF;

    // RUN Controller
    flg_RUN_InnerLoop = true;

    return I2C_Flag_Block;

}

/*******************************
*****   QUADRATURE FUNCT   *****
********************************/
void    Update_Quad_0_RH( void )   // RH MOTOR
{
STORE_TIMER6(1);
    // ----- MOTOR 0 quad channels -----
    bool A = DL_GPIO_readPins( QUAD_M0_PORT,QUAD_M0_A_PIN );
    bool B = DL_GPIO_readPins( QUAD_M0_PORT,QUAD_M0_B_PIN );

    // --- A channel edge ---
    if ( A != A_0[0] )
    {
        A_0[0] = A;
        gTheta_0_RH_pos += ( A ^ B ) ? -1 : +1;
    }

    // --- B channel edge ---
    if ( B != B_0[0] )
    {
        B_0[0] = B;
        gTheta_0_RH_pos += ( B ^ A ) ? +1 : -1;
    }
}
void    Update_Quad_1_LH( void )   // LH MOTOR inversed due to motor is inverted
{
    // ----- MOTOR 0 quad channels -----
    bool A = DL_GPIO_readPins( QUAD_M1_PORT,QUAD_M1_C_PIN );
    bool B = DL_GPIO_readPins( QUAD_M1_PORT,QUAD_M1_D_PIN );

    // --- A channel edge ---
    if ( A != A_0[1] )
    {
        A_0[1] = A;
        gTheta_1_LH_pos += ( A ^ B ) ? +1 : -1;
    }

    // --- B channel edge ---
    if ( B != B_0[1] )
    {
        B_0[1] = B;
        gTheta_1_LH_pos += ( B ^ A ) ? -1 : +1;
    }
GET_DURATION(1);   
}

void Wheel_GetSpeed_SG(void)
{
    // ----- SHIFT LEFT WHEEL HISTORY -----
    gWheel_LH_hist_m[0] = gWheel_LH_hist_m[1];
    gWheel_LH_hist_m[1] = gWheel_LH_hist_m[2];
    gWheel_LH_hist_m[2] = gWheel_LH_hist_m[3];
    gWheel_LH_hist_m[3] = gWheel_LH_hist_m[4];
    gWheel_LH_hist_m[4] = gWheel_LH_m;      // newest sample

    // ----- SHIFT RIGHT WHEEL HISTORY -----
    gWheel_RH_hist_m[0] = gWheel_RH_hist_m[1];
    gWheel_RH_hist_m[1] = gWheel_RH_hist_m[2];
    gWheel_RH_hist_m[2] = gWheel_RH_hist_m[3];
    gWheel_RH_hist_m[3] = gWheel_RH_hist_m[4];
    gWheel_RH_hist_m[4] = gWheel_RH_m;      // newest sample

    // ----- SG DERIVATIVE FOR LEFT WHEEL -----
    gWheel_LH_msec =
        SG_K * ( gWheel_LH_hist_m[0]
                - 8.0f * gWheel_LH_hist_m[1]
                + 8.0f * gWheel_LH_hist_m[3]
                - gWheel_LH_hist_m[4] );

    // ----- SG DERIVATIVE FOR RIGHT WHEEL -----
    gWheel_RH_msec =
        SG_K * ( gWheel_RH_hist_m[0]
                - 8.0f * gWheel_RH_hist_m[1]
                + 8.0f * gWheel_RH_hist_m[3]
                - gWheel_RH_hist_m[4] );
}

// Call this every 50 ms after updating gPosition_m
float Robot_GetSpeed_SG(void)
{
    // Shift history: [0]=k-2, [1]=k-1, [2]=k, [3]=k+1, [4]=k+2
    gPosition_hist_m[0] = gPosition_hist_m[1];
    gPosition_hist_m[1] = gPosition_hist_m[2];
    gPosition_hist_m[2] = gPosition_hist_m[3];
    gPosition_hist_m[3] = gPosition_hist_m[4];
    gPosition_hist_m[4] = gPosition_m;   // newest sample

    // Only valid after initial fill; you can gate this if needed
    float v = SG_K * ( gPosition_hist_m[0]
                     - 8.0f * gPosition_hist_m[1]
                     + 8.0f * gPosition_hist_m[3]
                     - gPosition_hist_m[4] );

    return v;   // [m/s]
}
float Robot_GetYawRate_SG(void)
{
    gYaw_hist_rad[0] = gYaw_hist_rad[1];
    gYaw_hist_rad[1] = gYaw_hist_rad[2];
    gYaw_hist_rad[2] = gYaw_hist_rad[3];
    gYaw_hist_rad[3] = gYaw_hist_rad[4];
    gYaw_hist_rad[4] = gTurn_rad;

    return (1.0f / (12.0f * TIME_MOTOR_OMEGA_S)) *
           ( gYaw_hist_rad[0]
           - 8.0f * gYaw_hist_rad[1]
           + 8.0f * gYaw_hist_rad[3]
           - gYaw_hist_rad[4] );
}

// Median of 3 values
int32_t median3(int32_t a, int32_t b, int32_t c)
{
// #define SWAP(a,b) if ((a) > (b)) { int32_t tmp = (a); (a) = (b); (b) = tmp; }
    int32_t tmp = 0;

    if ( a > b ) {
        tmp = a;
        a = b;
        b = tmp;
    } 

    if ( b > c ) {
        tmp = b;
        b = c;
        c = tmp;
    } 

    if ( a > b ) {
        tmp = a;
        a = b;
        b = tmp;
    } 

    return b; // middle value
}

// Converts 8-bit RGB color into WS2812 PWM pattern (0x1C = 0, 0x38 = 1)
void WS2812B_LED_Col(uint8_t led, uint8_t R, uint8_t G, uint8_t B)
{
    volatile uint8_t *p = gLEDColors_8b_GRB[led];

    // GREEN
    for (uint8_t i = 0; i < 8; i++)
        p[i] = (G & (0x80 >> i)) ? 0x38 : 0x1C;

    // RED
    for (uint8_t i = 0; i < 8; i++)
        p[8 + i] = (R & (0x80 >> i)) ? 0x38 : 0x1C;

    // BLUE
    for (uint8_t i = 0; i < 8; i++)
        p[16 + i] = (B & (0x80 >> i)) ? 0x38 : 0x1C;
}

void WS2812B_All_LED(uint8_t ArgColor, uint8_t ArgBrightness)
{
    uint8_t R = 0, G = 0, B = 0;

    if (ArgColor == 0)      { R = ArgBrightness; }
    else if (ArgColor == 1) { G = ArgBrightness; }
    else if (ArgColor == 2) { B = ArgBrightness; }

    for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++)
        WS2812B_LED_Col(i, R, G, B);
}

void WS2812B_Half_LED(uint8_t argRGB1, uint8_t argRGB2,
                      uint8_t argBrg1, uint8_t argBrg2)
{
    uint8_t R1=0, G1=0, B1=0;
    uint8_t R2=0, G2=0, B2=0;

    // ----- FIRST HALF COLOR -----
    if (argRGB1 == 0)      R1 = argBrg1;
    else if (argRGB1 == 1) G1 = argBrg1;
    else if (argRGB1 == 2) B1 = argBrg1;

    // ----- SECOND HALF COLOR -----
    if (argRGB2 == 0)      R2 = argBrg2;
    else if (argRGB2 == 1) G2 = argBrg2;
    else if (argRGB2 == 2) B2 = argBrg2;

    uint8_t half = WS2812B_LED_COUNT >> 1;

    // ----- FIRST HALF -----
    for (uint8_t i = 0; i < half; i++)
        WS2812B_LED_Col(i, R1, G1, B1);

    // ----- SECOND HALF -----
    for (uint8_t i = half; i < WS2812B_LED_COUNT; i++)
        WS2812B_LED_Col(i, R2, G2, B2);
}

void BatteryGauge_8LED(float voltage)
{
    const float Vmax = VOLT_BATTERY_DEF + 0.7f; // 12.0V
    const float Vmin = VOLT_BATTERY_MIN + 0.1f; // 11.3V
    const uint8_t LEDcount = 8;
    const uint8_t LEDoffset = 0;

    // --- Normalize voltage into 0..1 range ---
    float norm = (voltage - Vmin) / (Vmax - Vmin);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    // --- Convert to LED steps ---
    uint8_t active = (uint8_t)(norm * (LEDcount+LEDoffset));
    if (active == 0) active = 1;   // always show at least 1 LED

    // --- Draw the gauge ---
    for (uint8_t i = LEDoffset; i < (LEDcount+LEDoffset); i++)
    {
        if (i < active - 1)
        {
            // Full green LEDs
            WS2812B_LED_Col(i, 0, 0x0F, 0);   // R,G,B
        }
        else if (i == active - 1)
        {
            // Last LED: color depends on battery level
            if (active == 1)
                WS2812B_LED_Col(i, 0x10, 0, 0);   // RED (low battery)
            else
                WS2812B_LED_Col(i, 0x10, 0x04, 0); // Yellow (transition)
        }
        else
        {
            // LEDs above active level = OFF
            WS2812B_LED_Col(i, 0, 0, 0);
        }
    }
}

void TiltGauge_8LED(float tilt_deg)
{
    const float TILT_MIN = -10.0f;
    const float TILT_MAX =  10.0f;
    const uint8_t LEDcount  = 8;
    const uint8_t LEDoffset = 8;   // tilt LEDs start at index 8

    // Clamp tilt
    if (tilt_deg < TILT_MIN) tilt_deg = TILT_MIN;
    if (tilt_deg > TILT_MAX) tilt_deg = TILT_MAX;

    // Normalize tilt into 0..1
    float norm = (tilt_deg - TILT_MIN) / (TILT_MAX - TILT_MIN);

    // Map to LED index 0..6 (left LED of the pair)
    float pos = norm * 6.0f;

    // Apply offset correctly
    uint8_t idxA = (uint8_t)pos + LEDoffset;  // left LED
    uint8_t idxB = idxA + 1;                  // right LED (adjacent!)

    // Draw LEDs
    for (uint8_t i = LEDoffset; i < (LEDoffset + LEDcount); i++)
    {
        if (i == idxA || i == idxB)
        {
            // Active pair = RED
            WS2812B_LED_Col(i, 0x0F, 0, 0);   // R,G,B
        }
        else
        {
            // Off
            WS2812B_LED_Col(i, 0, 0, 0);
        }
    }
}


