/**********************************************************************
 * PROGRAM  NAME: PEND-2025-CLANG_0G3507.c
 * PROGRAM  FUNCTION: Balancing Robot
 * HARDWARE SPECIFICATIONS:
 *   - 2xPOLOLU 4753 (50:1 Metal Gearmotor 37Dx70Lmm/12V/64CPR)
 *   - MSPM0G3507@80MHz/12.5ns
 * BAUD RATE:   0.46 Mbps (8bit, 1stp, no parity) - UART0 - PC communication
 *              0.42 Mbps (8bit, 1stp, no parity) - UART1 - CrossFire communication
 * COMPILER/SDK/SysConfig VERSION: Clang v4.0.4 -O2 / 2.10.0.04 / 1.27
 * PROGRAMMER: Ing. Tomas Solarski
 * LAST MODIFIED: 2026-11-13 23:03:17
 **********************************************************************/

/*******************************************************************************
 * JUGGERNAUT SYSTEM TIMING & CONTROL ARCHITECTURE
 *******************************************************************************
 * Domain        | Loop Function       | Period  | Rate     | Target / Method
 * --------------|---------------------|---------|----------|-------------------
 * Current Loop  | Motor Current       |  400 us | 2.50 kHz | Field/Torque (Unused)
 * Inner Loop    | Tilt/Balance Ctrl   |  800 us | 1.25 kHz | Angle & Gyro Rate
 * Command       | CRSF/ELRS Receiver  |    4 ms |  250 Hz  | Direct Event ISR
 * Outer Loop    | Position / Traject  |   10 ms |  100 Hz  | Encoder & Velocity
 * Telemetry     | PC Diagnostic       |   10 ms |  100 Hz  | PC/SW Grapher or GUI
 * Telemetry     | CRSF Diagnostic     |   50 ms |   20 Hz  | ATT/VBAT/GPS telemetry
 ******************************************************************************/
/*******************************************************************************
 * MSPM0G3507 MCU PIN MAPPING: PORT A & B COMPACT VIEW
  *******************************************************************************
 * PORT A (LEFT)                           | PORT B (RIGHT)
 * ----------------------------------------|-----------------------------------------
 * PA0  | N.C.      | Open Drain           | PB0  | UART0_TX  | PC Comm TX
 * PA1  | N.C.      | Open Drain           | PB1  | UART0_RX  | PC Comm RX
 * PA2  | N.C.      |                      | PB2  | TIMA0_C3  | PWM3 Phase D
 * PA3  | TIMA0_C1  | PWM1 Phase A         | PB3  | TIMA0_C3N | nPWM3 (Comp)
 * PA4  | TIMA0_C1N | nPWM1 (Comp)         | PB4  | UART1_TX  | CrossFire TX
 * PA5  | HFXIN     | Crystal              | PB5  | UART1_RX  | CrossFire RX
 * PA6  | HFXOUT    | Crystal              | PB6  | N.C.      |
 * PA7  | GRN4      | Status LED           | PB7  | N.C.      |
 * PA8  | QUAD-C    | Motor 1 Enc          | PB8  | N.C.      |
 * PA9  | QUAD-D    | Motor 1 Enc          | PB9  | N.C.      |
 * PA10 | TIMA0_C2  | PWM2 Phase C         | PB10 | N.C.      |
 * PA11 | TIMA0_C2N | nPWM2 (Comp)         | PB11 | QUAD-B    | Motor 0 Enc
 * PA12 | SPI0_SCK  | Radio Clock (S0)     | PB12 | QUAD-A    | Motor 0 Enc
 * PA13 | SPI0_POCI | Radio MISO  (S0)     | PB13 | N.C.      |
 * PA14 | SPI0_PICO | Radio MOSI  (S0)     | PB14 | SPI1_POCI | MISO (S1)
 * PA15 | ADC1_0    | TEMP-M0              | PB15 | SPI1_PICO | MOSI (S1)
 * PA16 | ADC1_1    | CURRENT-M0           | PB16 | SPI1_SCK  | Clock (S1)
 * PA17 | ADC1_2    | Battery Volt         | PB17 | N.C.      |
 * PA18 | N.C.      | BSL Invoke           | PB18 | N.C.      |
 * PA19 | SWDIO     | Debug Data           | PB19 | N.C.      |
 * PA20 | SWCLK     | Debug Clock          | PB20 | IRQ0      | Radio IRQ
 * PA21 | TIMA0_C0  | PWM0 Phase B         | PB21 | CS0       | Radio CS
 * PA22 | TIMA0_C0N | nPWM0 (Comp)         | PB22 | CE0       | Radio CE
 * PA23 | N.C.      |                      | PB23 | N.C.      |
 * PA24 | ADC0_3    | CURRENT-M1           | PB24 | N.C.      |
 * PA25 | N.C.      |                      | PB25 | N.C.      |
 * PA26 | ADC0_1    | TEMP-M1              | PB26 | N.C.      |
 * PA27 | N.C.      |                      | PB27 | N.C.      |
 * PA28 | TIMG7_C0  | RGB LED WS2812B      |
 * PA29 | SCL1      | I2C Clock            |
 * PA30 | SDA1      | I2C Data             |
 * PA31 | TIMG7_C1  | RC-PWM1              |
 * -------------------------------------------------------------------------------- */

// ** LAST UPDATE **
// I2C1 global variable was not volatile - i2c state machine was not working - bug fixed

#include    <stdbool.h>
#include    <string.h>
#include    <stdlib.h>
#include    <stdio.h>

#include    "ti/driverlib/dl_gpio.h"
#include    "ti_msp_dl_config.h"

#define     GLOBAL_IQ 16
#include    <ti/iqmath/include/IQmathLib.h>
/* IQ TYPE  | INT BITS | FRAC BITS | MIN RANGE         | MAX RANGE         | RESOLUTION    */
/* _iq30    | 2        | 30        | -2                | 1.999999999       | 0.000000001   */
/* _iq24    | 8        | 24        | -128              | 127.999999940     | 0.000000060   */
/* _iq16    | 16       | 16        | -32768            | 32767.999984741   | 0.000015259   */
/* _iq8     | 24       | 8         | -8388608          | 8388607.996093750 | 0.003906250   */
/* _iq1     | 31       | 1         | -1073741824       | 1073741823.500000 | 0.500000000   */
// https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/middleware/iqmath/doc_guide/doc_guide-srcs/Users_Guide.html

// ----- LED Sequence ----
#define     LED_SEQ_DEF             (0b001111111111)    // one long  blink
#define     LED_SEQ_ERROR           (0b010101010101)    // fast blinking
#define     LED_SEQ_PD_CTRL         (0b000000000011)    // one short blink
#define     LED_SEQ_SMC_CTRL        (0b000000011011)    // two short blink
#define     LED_TICK_COUNT          (12U)               // LED heartbeat period
// #define     LED_SEQUENCE_3          (0b000011011011)    // three blink
// #define     LED_SEQUENCE_4          (0b000001010101)    // four blink
// #define     LED_SEQUENCE_5          (0b000101010101)    // five blink
// ----- Timebase ----
// #define     TIME_MPU_READ_SEC       (0.0008f)       // reading rate of MPU in seconds - Main Sample Period - Inner Loop
#define     SAMPLE_TIME_iq          _IQ(0.0008f)       // 800us in seconds - actual compiler value = 52/65536 = 0.00079346s
#define     TIME_MOTOR_OMEGA_MS     (10U)           // rate to calculate impulses to determine motor speed
#define     TIME_MOTOR_OMEGA_S      (0.01f)         // in seconds
// co-prime numbers are used to avoid harmonics in timebased routines
// #define     TIME_SEND_DATA_MS       (19U)           // 50Hz send data rate - NOW send data is in Position Calculations
#define     TIME_TILT_GAU_MS        (41U)           // 25Hz update data rate for tilt gauge WS2812B display
#define     TIME_LED_MS             (51U)           // LED heartbeat period
#define     TIME_BATT_GAU_MS        (249U)          // 4Hz update data rate for battery gauge WS2812B display
#define     TIME_MPU_DELAY_MS       (251U)          // delay before MPU is initiated
#define     TIME_SEND_TELE_MS       (103U)          // Battery Telemetry send back to Radio (upstream)
#define     TIME_FAIL_SAFE_MS       (1501U)         // No command received
// ----- Angles Limits ----
#define     ANGLE_LIM_BODY_RAD      (0.50f)         // Angle limit - beyond Controller is OFF   - 0.50rad ~ 30deg
#define     ANGLE_LIM_HYST_RAD      (0.10f)         // Angle hysteresis - preventing oscilations- 0.10rad ~  6deg
#define     ANGLE_P_ZONE_RAD        (0.20f)         // Region to boost P terms                  - 0.20rad = 12deg
#define     ANGLE_SP_PITCH_RAD      (0.15f)         // Set Point limit for position controller  - 0.15rad =  9deg
// ----- Voltages ----
#define     VOLT_MOTOR_LIM_HI       (8.00f)         // Action value limit for Inner Loop
#define     VOLT_MOTOR_LIM_LO       (4.00f)         // Action value limit for Side Loop
#define     VOLT_MOTOR_MIN          (0.25f)         // Motor voltage minimum value
#define     VOLT_MOTOR_HYST         (0.10f)         // Small hysteresis to prevent oscillations
#define     VOLT_BATTERY_MAX        (12.6f)         // Li-Pol High voltage - 4.2V per cell
#define     VOLT_BATTERY_MIN        (10.2f)         // Li-Pol low voltage - 3.4V per cell
#define     VOLT_BATTERY_DEF        (11.3f)         // Li-Pol default voltage - 3.75V per cell
#define     VOLT_ADC_REF            (3.60f)         // ADC reference voltage
#define     VOLT_SCHTKY_DROP        (0.15f)         // Input rectifier drop
#define     VOLT_DIVIDER_RATIO      (11.0f)         // (47k + 4.7k) / 4.7k - Batt Voltage divider ratio
// ----- Currents Limits ----
#define     MAX_CURRENT_STEP_A      (0.050f)        // Max allowed change per 400us step (50mA) during Motor Current Measurement
// ----- POLOLU MOTOR 4753 -----
// 50:1 Metal Gearmotor 37Dx70L mm 12V 200RPM with 64 CPR Encoder
#define     MOTOR_GEAR_RATIO        (50.0f)         // Motor gear ratio (-)
#define     MOTOR_CPR               (64.0f)         // Motor encoder 64 Counts Per Revolution (quadrature) (-)
#define     MOTOR_RAD_PER_TICK      (2.0f*3.14159f/(MOTOR_CPR*MOTOR_GEAR_RATIO)) // Motor Radian per Encoder Tick (rad/tick)
#define     ROBOT_WHEEL_DIA_M       (0.125f)        // Juggernaut Wheel Diameter - 125mm (m)
#define     ROBOT_WHEEL_RAD_M       (0.0625f)       // Juggernaut Wheel Radius - 62.5mm (m)
#define     ROBOT_WHEEL_SOD_M       (0.21f)         // 210mm - wheels separation distance

#define     ROBO_STATE_INIT         (0U)            // MPU not initiated or found
#define     ROBO_STATE_BALA         (1U)            //
#define     ROBO_STATE_CALIB        (2U)            //
#define     ROBO_STATE_IDENT        (3U)            //
#define     ROBO_STATE_NOK          (4U)            //

#define     CONTROLLER_TYPE_PID     (0U)            // Generic (Original) Controller
#define     CONTROLLER_TYPE_SMC     (1U)            // Current (under test) Controller
#define     CONTROLLER_TYPE_LQR     (2U)            // Future Expansion Controller

#define     PWM_MAX_DUTY            (90U)       // [ % ] -> 0.90*12V => 10.8V
#define     PWM_MIN_DUTY            (2U)        // [ % ] -> 0.02*12V => 0.24V

// TIMER CHANNELS ASSIGNED TO OUTPUTS
#define     PHASE_A                 (1U)    // Motor 0 - Positive/Negative terminal
#define     PHASE_B                 (0U)    // Motor 0 - Positive/Negative terminal
#define     PHASE_C                 (2U)    // Motor 1 - Positive/Negative terminal
#define     PHASE_D                 (3U)    // Motor 1 - Positive/Negative terminal
#define     PHASE_COUNT             (4U)

// ****** Savitzky-Golay Derivative Filter ******
// used to filter and calculate the speed
#define     DT_S        TIME_MOTOR_OMEGA_S              // Wheel Sample Period
#define     SG_K        (1.0f / (12.0f * DT_S))         // For 1st derivative (Velocity)
#define     SG_K2       (1.0f / (7.0f * DT_S * DT_S))   // For 2nd derivative (Acceleration)

// ----- WS2812B LED -----
#define     WS2812B_COLOR_CNT       (3U)    // RGB - 3 colors BUT on WS2812B GREEN is first, RED, last BLUE
#define     WS2812B_PWM_PATTR       (8U)    // 8 bits of PWM CC to map one bit of WS2818
#define     WS2812B_LED_COUNT       (16U)   // number of physical LED (8+8) to control 
#define     WS2812B_DUMY_BYTE       (4U)    // +4 dummy patterns for latch
#define     WS2812B_BIT_COUNT       (WS2812B_COLOR_CNT*WS2812B_PWM_PATTR*(WS2812B_LED_COUNT+WS2812B_DUMY_BYTE))

// ----- RGB Digi LED WS WS2812B -----
// 8bit PWM pattern: zero pulse - 0x1C (28*12.5ns=0.35us), one pulse - 0x38 (56*12.5ns=0.7us)
static    uint8_t     gDMA_LED_array[ (WS2812B_LED_COUNT+WS2812B_DUMY_BYTE) ][ WS2812B_COLOR_CNT*WS2812B_PWM_PATTR ] = // WAS volatile
{   
    // GREEN color (MSB first),                RED color (MSB first),                  BLUE color (MSB first)
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED0 - FRONT RH
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED1
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED2
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED3 - FRONT MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED4 - FRONT MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED5
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED6
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED7 - FRONT LH

    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED8 - REAR LH
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED9
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED10
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED11 - REAR MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED12 - REAR MID
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED13
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED14
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C, 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C } , // LED15 - REAR RH

    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } , // DUMMY
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } , // DUMMY
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } , // DUMMY
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }   // DUMMY  
};

static  const   uint8_t gCRC8_LUT[256] = {  // WAS volatile
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

volatile    uint16_t    gLED_seq_now        = LED_SEQ_DEF;
volatile    uint8_t     gRobot_state        = ROBO_STATE_BALA;
volatile    uint8_t     gRobot_state_0      = ROBO_STATE_BALA;
volatile    uint8_t     gController_type    = CONTROLLER_TYPE_PID;  // Default state use PID

volatile    uint8_t     gRobot_ident_state = 0U;

// ----- FLAGS (often used in ISR) -----
volatile    bool        gFlg_run_outer_loop = false;            // Flag signal to run Position Calculation
volatile    bool        gFlg_run_inner_loop = false;            // Flag signal to run Balance Control
volatile    bool        gFlg_motor_enable   = false;            // Flag signal to enable motors (MPU fail safe can disable motors)
volatile    bool        gFlg_batt_gauge     = false;            // Flag signal to update WS2812 battery gauge display
volatile    bool        gFlg_tilt_angle     = false;            // Flag signal to update WS2812 tilt angle display

// ----- COUNTERS (often used in ISR) -----
volatile    uint32_t    gCnt_LED_period     = TIME_LED_MS;      // LED ON/OFF period
volatile    uint8_t     gCnt_LED_seq_idx    = 0U;               // LED sequence index
volatile    uint32_t    gCnt_outer_loop     = TIME_MOTOR_OMEGA_MS;  //Position Calculation period
volatile    uint32_t    gCnt_fail_safe      = TIME_FAIL_SAFE_MS;    // CRSF fail safe period - signal lost
volatile    uint32_t    gCnt_wait_ms        = 0U;               // Wait time in ms for Identification state machine
volatile    uint32_t    gCnt_batt_gauge     = TIME_BATT_GAU_MS; // Battery gauge display update period
volatile    uint32_t    gCnt_tilt_angle     = TIME_TILT_GAU_MS; // Tilt angle display update period

// ----- DEBUG ERROR COUNTERS -----
volatile    uint32_t    debug_i2c_write_reg_cnt[ 4 ];       // I2C duration in CPU_tics only for MPU_Write_Reg();
volatile    uint32_t    debug_i2c_write_reg_us[  4 ];       // I2C duration in usec. only for MPU_Write_Reg();
volatile    uint32_t    debug_i2c_read_regs_cnt[  9 ];       // I2C duration in CPU_tics only for mpu_read_regs();
volatile    uint32_t    debug_i2c_read_regs_us[   9 ];       // I2C duration in usec. only for mpu_read_regs();

// ----- usec DURATION TIMERS -----
#define     DURATION_TIMERS  (12U)                          // number of timers WAS 10
typedef struct duration_timer_t{
    uint32_t quad_isr;      // dt 0: quadrature ISR
    uint32_t mpu_read;      // dt 1: I2C/SPI overhead
    uint32_t uav_physics;   // dt 2 & 3: Scaling, G-calc, Rad/s
    uint32_t trigonometry;  // dt 4: atan2f, sqrt
    uint32_t comp_filter;   // dt 5: The Alpha blend
    uint32_t uart_comms;    // dt 6: Buffer filling and TX
    uint32_t inner_loop;    // dt 7: Tilt control
    uint32_t outer_loop;    // dt 8: Position control
    uint32_t crsf;          // dt 9: Commands
    uint32_t adc_m0;        // dt 10: ADC M0 sample time
    uint32_t adc_m1;        // dt 11: ADC M1 sample time
    uint32_t total_loop;    // total sum - The heartbeat of your UAV

    uint16_t delta_time[ DURATION_TIMERS ];
    uint16_t time_stamp[ DURATION_TIMERS ];
} duration_timer_t;
// Global or module-level instance
volatile duration_timer_t gDuration_timer_us;

#define     GET_TICK_US (uint16_t)DL_TimerG_getTimerCount( TIMER_6_INST )
#define     STORE_TIMER6(i) ( gDuration_timer_us.time_stamp[i] = (uint16_t)DL_TimerG_getTimerCount( TIMER_6_INST ) )
#define     GET_DURATION(i) ( gDuration_timer_us.delta_time[i] = (uint16_t)(DL_TimerG_getTimerCount( TIMER_6_INST ) - gDuration_timer_us.time_stamp[i]) )

// volatile    uint16_t    gDelta_time[ DURATION_TIMERS ];     // delta time in us
// volatile    uint16_t    gTime_stamp[ DURATION_TIMERS ];     // save timer value
// #define     STORE_TIMER6(i) ( gTime_stamp[i] = (uint16_t)DL_TimerG_getTimerCount( TIMER_6_INST ) )
// #define     GET_DURATION(i) ( gDelta_time[i] = (uint16_t)(DL_TimerG_getTimerCount( TIMER_6_INST ) - gTime_stamp[i]) )

// ----- ANALOG - All channels -----
#define     ADC_CNT                 (5U)        // 5 channel used: VS1(Battery), Current M0, Current M1 , Temp M0, Temp M1     
typedef struct adc_t {
    bool        flag[ ADC_CNT ];                // Flag signal to read ADCs
    uint32_t    LSB[  ADC_CNT ];                // [ LSB ]
    float       volt[ ADC_CNT ];                // [ V ]
} adc_t;
volatile    adc_t       gADC = { .flag = { false }, .LSB = { 0U }, .volt = { 0.0f } };
// original ADC section
volatile    bool        gADC_flg[  ADC_CNT ]   = { false };    // Flag signal to read ADCs
volatile    uint32_t    gADC_LSB[  ADC_CNT ];       // [ LSB ]
volatile    float       gADC_Volt[ ADC_CNT ];       // [ V ]

// ============================================================================
// ----- SLIDING MODE CONTROLLER (SMC) PARAMETERS & INITIALIZATION -----------
// ============================================================================

// --- Sliding Surface Hyperplane Weights (s = sum(lambda_i * error_i)) ---
#define SMC_LAMBDA_ANGLE_DEF      (1.0f)   // Pitch stiffness (primary restoring authority)
#define SMC_LAMBDA_RATE_DEF       (0.2f)   // Pitch damping (phase lead to counter 50:1 gear lag)
#define SMC_LAMBDA_POS_DEF        (0.0f)   // Position weight (0.0f for bench; -0.05f for floor drift)
#define SMC_LAMBDA_SPEED_DEF      (0.0f)   // Speed weight (0.0f for bench; -0.08f to damp runaway)

// --- Robust Control & Boundary Parameters ---
#define SMC_K_DEF                 (10.0f)  // "The Hammer"  - Max switching gain / motor drive effort (V)
#define SMC_PHI_DEF               (0.75f)  // "The Cushion" - Boundary layer thickness (eliminates 5-10Hz gear chatter)

// ----- Sliding Mode Controller Structure -----
typedef struct smc_controller_t {
    // Hyperplane weights (surface slope vector)
    float lambda_angle;       // Pitch error gain
    float lambda_angle_rate;  // Pitch rate error gain
    float lambda_pos;         // Body position error gain
    float lambda_speed;       // Body speed error gain

    // Robust switching law parameters
    float K;                  // Switching gain (max drive effort)
    float phi;                // Saturation boundary layer (chatter filter)

    // Action value of Motor voltages Va (unclamped and clamped)
    float CM_term_unclamped_V;
    float CM_term_V;
    float TI_term_V;
} smc_controller_t;

// ----- Global Controller Instance -----
volatile smc_controller_t smc_controller = {
    .lambda_angle           = SMC_LAMBDA_ANGLE_DEF,
    .lambda_angle_rate      = SMC_LAMBDA_RATE_DEF,
    .lambda_pos             = SMC_LAMBDA_POS_DEF,
    .lambda_speed           = SMC_LAMBDA_SPEED_DEF,
    .K                      = SMC_K_DEF,
    .phi                    = SMC_PHI_DEF,
    .CM_term_unclamped_V    = 0.0f,
    .CM_term_V              = 0.0f,
    .TI_term_V              = 0.0f
};

// ----- Define what a "WHEEL" is -----
typedef struct wheel_t {
    float       pos_m;              // Wheel Linear position (m)            - can go negative
    float       pos_odo_m;          // Wheel Absolute position (m)          - absolute rolled distance
    float       pos_hist_m[5];      // Wheel Linear position history (m)    - for SG derivative filter
    float       speed_ms;           // Wheel Linear speed (m/s)             - raw derivative (no delay)
    float       speed_sg_ms;        // Wheel Linear speed (m/s)             - filtered by SG derivative filter (two sample delay)
    float       accel_sg_ms2;       // Wheel Linear acceleration (m/s^2)    - filtered by SG derivative filter (two sample delay)
} wheel_t;
// ----- Define what a "MOTOR" is -----
typedef struct motor_t {
    int32_t     imp;                // Motor Encoder impulses (-)
    float       theta_rad;          // Motor Angle (rad)
    float       theta_hist_rad[5];  // Motor Angle history (rad)            - for SG derivative filter
    float       omega_sg_rads;      // Motor Angular velocity (rad/s)       - filtered by SG filter
    float       alpha_sg_rads2;     // Motor Angular acceleration (rad/s^2) - filtered by SG filter
    float       curr_hist_A[5];     // Motor Current history (A)            - for Median filter
    float       curr_med5_A;        // Motor Current (A)                    - Median 5 filter - Almost raw data
    float       curr_ema_A;         // Motor Current (A)                    - IIR filter
    float       volt_V;             // Motor Voltage (V)                    - Commanded voltage
    // float       curr_med3_A;        // Motor Current (A)                 - Median 3 filter - Almost raw data
} motor_t;
// ----- Define what a "POWER" is -----
typedef struct power_t {
    float       batt_volt_V;        // Battery filtered voltage (V)
    float       batt_curr_A;        // Battery current (A)
    uint32_t    batt_cap_mAh;       // Battery fuel used (mAh)
    float       batt_gauge_ptc;     // Battery gauge in percentage (%)
    bool        is_low_power;       // Critical low voltage flag
} power_t;

// *****************************************  *****************************************  *****************************************
// *****    Define what a "ROBOT" is   *****  *****   Define what a "ROBOT" is    *****  *****   Define what a "ROBOT" is    *****
// *****************************************  *****************************************  *****************************************
typedef struct Robot_t {
    // --- 10. GAINS (The "DNA") ---
    struct {
        float   FF;             // Angle Feed Forward gain     - DIRECT DRIVE
        float   KP;             // Angle Proportional gain     - SPRING
        float   KD;             // Angle Gyro derivative gain  - DAMPER
        float   N;              // Derivative gain Filter      - DAMPER^2
        float   TI;             // Angle Combined turning gain - YAW
        float   LH_RH;          // Robot Wheel Position LHxRH  - L/R
        float   KP_Pos;         // Robot Position Proportional gain
        float   KD_Pos;         // Robot Speed Proportional gain
    } gain;
    // --- 20. POWER (The "Health") ---
    power_t power;

    // --- 30. WHEELS (The "Legs") ---
    struct {
        wheel_t left;
        wheel_t right;
    } wheel;

    // --- 40. MOTORS (The "Engine") ---
    struct {
        motor_t left;
        motor_t right;
    } motor;
    
    // --- 50. BODY (The "Robot") ---
    struct {
        float   accX_g;             // Robot Body Acceleration (g)  - IMU measured
        float   accY_g;             // Robot Body Acceleration (g)  - IMU measured
        float   accZ_g;             // Robot Body Acceleration (g)  - IMU measured
        float   temp_C;             // Robot Body Temperature (C)   - IMU measured
        float   pitch_rad;          // Robot Body Tilt Angle (rad)  - IMU measured
        float   rateX_rads;         // Robot Body X Rate (rad/s)    - IMU measured
        float   roll_rad;           // Robot Body Roll Angle (rad)  - IMU measured
        float   rateY_rads;         // Robot Body Y Rate (rad/s)    - IMU measured
        float   yaw_rad;            // Robot Body Turn Angle (rad)  - IMU measured
        float   rateZ_rads;         // Robot Body Y Rate (rad/s)    - IMU measured
        float   turn_rad;           // Robot Body Turn Angle (rad)  - calculated from wheels
        float   turn_hist_rad[5];   // Robot Body Turn Angle History (rad)
        float   turn_rate_rads;     // Robot Body Turn Rate (rad/s) - calculated from wheels
        float   pos_m;              // Robot Body Position (m) - how far the robot has moved
        float   pos_abs_m;          // Robot Body Absolute position (m) - absolute rolled distance
        float   pos_hist_m[5];      // Robot Body Position History (m)
        float   speed_ms;           // Robot Body Speed (m/s) - how fast the robot is moving
    } body;
    // --- 60. CONTROLLER (The "Action") ---
    struct { 
        float   FFterm_V;           // Feed Forward Term [V]
        float   KPterm_V;           // Proportional term [V]
        float   KDterm_V;           // Derivative term - Damping from gyro [V]
        float   CMterm_V;           // Combined term - common - all action terms [V]
        float   TIterm_V;           // Turning term [V]
        float   SP_Pitch_rad;       // Balancing/Tilt Setpoint [rad] - used from command
        float   SP_Pos_m;           // Robot Position Setpoint [m] - used when no command received
        float   SP_Speed_ms;        // Robot Speed Setpoint [m/s] - used when no command received
        float   Err_Pitch_rad;      // Balancing/Tilt Error [rad] - used for controller
        float   Err_Pos_m;          // Robot Position Error [m] - used for controller
        float   Err_Speed_ms;       // Robot Speed Error [m/s] - used for controller
        float   Err_lh_rh_m;        // Robot Wheel Position Mismatch (m) - left - right
        float   Correction_Pos_rad; // Angle to correct position error [rad]
        float   Booster_P_gain;     // Angle correction proportional gain
    } control;
    // --- 70. RC (The "Remote Control") ---
    struct {
        // --- Raw RC Data ---
        uint16_t ch[16U];           // [0 .. 2047]   - Raw 11-bit values
        // --- Primary Sticks (Normalized) ---
        float roll;                 // [-1.00, 1.00] - Roll stick input
        float pitch;                // [-1.00, 1.00] - Pitch stick input
        float throttle;             // [-1.00, 1.00] - Throttle stick input
        float yaw;                  // [-1.00, 1.00] - Yaw stick input
        // --- Auxiliary Switches (Normalized) ---
        float latch_L;              // [-1.00, 1.00] - Latch switch input
        float state3_L;             // [-1.00, 1.00] - State 3 switch input
        float state3_R;             // [-1.00, 1.00] - State 3 switch input
        float latch_R;              // [-1.00, 1.00] - Latch switch input
        float button_L;             // [-1.00, 1.00] - Momentary switch input
        bool is_connected;          // Failsafe flag
    } rc_cmd;
} Robot_t;
// ----- Robot Defaults -----
#define PID_FF_GAIN_DEF         (5.00f)     // Tilt - Feed Forward Default Gain             (WAS 2.5)
#define PID_KP_GAIN_DEF         (8.00f)     // Tilt - Proportional Default Gain INNER LOOP  (WAS 10.0)
#define PID_KD_GAIN_DEF         (1.00f)     // Tilt - Derivative Default Gain   INNER LOOP  (WAS 1.5)
#define PID_N_FIL_HZ_DEF        (35.0f)     // Derivative Filter Default Value  INNER LOOP  (WAS 10.0)
#define PID_TI_GAIN_DEF         (2.50f)     // Turn - Proportional Default Gain SIDE LOOP   (WAS 1.0)
#define PID_LR_GAIN_DEF         (2.00f)     // LHxRH- Proportional Default Gain SIDE LOOP   (new)
#define PID_KP_POS_GAIN_DEF     (0.25f)     // Pos  - Proportional Default Gain OUTER LOOP  (WAS 0.4)
#define PID_KD_POS_GAIN_DEF     (0.25f)     // Pos  - Derivative Default Gain OUTER LOOP    (WAS 0.4)

volatile Robot_t gRobot = {
    .gain = {
        .FF = PID_FF_GAIN_DEF,              // [V]
        .KP = PID_KP_GAIN_DEF,              // [V/rad]
        .KD = PID_KD_GAIN_DEF,              // [Vs/rad]
        .N  = PID_N_FIL_HZ_DEF,             // [Hz]
        .TI = PID_TI_GAIN_DEF,              // [V]
        .LH_RH = PID_LR_GAIN_DEF,           // [V/m]
        .KP_Pos = PID_KP_POS_GAIN_DEF,      // [V/m]
        .KD_Pos = PID_KD_POS_GAIN_DEF       // [V/m/s]
    },
    .power = {
        .batt_volt_V = VOLT_BATTERY_DEF,    // strong filter used for battery voltage
        .is_low_power = false
    },
    .control = {
        .SP_Pitch_rad   = 0.0f,             // Default Upright Tilt Angle [rad]
        .SP_Pos_m       = 0.0f,             // Default Stay Where You Are [m]
        .SP_Speed_ms    = 0.0f,             // Default Dont Move [m/s]
        .Booster_P_gain = 1.0f              // Default Booster Gain
    }
};
// ----- Define what a "ENCODER" is -----
typedef struct quad_memory_t {
    volatile bool A;
    volatile bool B;
    volatile bool C;
    volatile bool D;
} quad_memory_t;
// ----- Quad Encoder Defaults -----
static quad_memory_t sQuad_memory = {
    .A = false,
    .B = false,
    .C = false,
    .D = false
};

// ***** UART0 - Communication to/from PC *****
// volatile    uint32_t        gCnt_UART0_TXD      = TIME_SEND_DATA_MS;
volatile    bool            gFlg_UART0_TX_En    = false;    // Flag signal to enable sending data via UART
volatile    bool            gFlg_UART0_TXD      = false;    // Flag signal to send data via UART
// --- RX - Used for TEXT commands ---
#define     RX0_BUFF_N      (4U)                            // 4 -> 2^4 = 16
#define     RX0_BUFF_SIZE   (1U<<RX0_BUFF_N)                // Buffer size in 2^N size (1,2,4,8,16,32,64,...)
volatile    uint8_t         gRX0_buffer[ RX0_BUFF_SIZE ];   // RX Buffer
volatile    uint8_t         gRX0_idx = 0U;                  // RX Buffer counter (index)
volatile    bool            gRX0_flg_eol = false;           // RX Message End of Line flag (CR+LF) (data ready)
// --- TX - Used for Serial Monitor APP or Custom GUI ---
#define     TX0_SIG_CNT     (28U)                           // Count of Signals that will be sended over UART
#define     TX0_SIG_RNG     (7U)                            // Signals -99999 to +99999 range (5 characters + sign) + coma separator
volatile    int32_t         gTX0_signal[ TX0_SIG_CNT ];     // Signals are stored in field to use for-cycle
volatile    uint8_t         gTX0_buffer[ TX0_SIG_CNT * TX0_SIG_RNG + 1 ];   // TX Buffer (Signals + separators + terminator) - CSV
volatile    uint8_t         gTX0_size       = 0U;           // TX Buffer size - store number of actual bytes
volatile    uint8_t         gTX0_idx        = 0U;           // TX Buffer counter (index)
volatile    uint32_t        gTX0_sample_cnt = 0U;           // TX Sample index - to see sample count info in log file

// ***** UART1 - Radio Control via CRSF *****
volatile    uint32_t        gCnt_UART1_TXD      = TIME_SEND_TELE_MS;
volatile    bool            gFlg_UART1_TXD      = false;    // Flag signal to send CRSF telemetry
// --- RX - 16ch CRSF ---
#define     RX1_BUFF_N      (5U)                            // 5 -> 2^5 -> 32 (as CRSF message is 26bytes long for 16channels)
#define     RX1_BUFF_SIZE   (1U<<RX1_BUFF_N)                // Buffer size in 2^N size (1,2,4,8,16,32,64,...)
volatile    uint8_t         gRX1_buffer[ RX1_BUFF_SIZE ];
volatile    uint8_t         gRX1_idx = 0;
// --- TX - Telemetry CRSF ---
volatile    uint8_t         gTX1_buffer[ RX1_BUFF_SIZE ];
volatile    uint8_t         gTX1_size = 0;
volatile    uint8_t         gTX1_idx = 0;

// ***** Cross Fire Radio System *****
// CRSF Protocol - https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#broadcast-frame-types
#define     CRSF_SYNC_BYTE          (0xC8)          // Serial Sync Byte - #1 byte of the message
#define     CRSF_FR_TYPE_GPS        (0x02)          // Broadcast Frame Type - GPS
#define     CRSF_FR_TYPE_BATT       (0x08)          // Broadcast Frame Type - Battery Sensor
#define     CRSF_FR_TYPE_16CH       (0x16)          // Broadcast Frame Type - RC Channels Packed Payload
#define     CRSF_FR_TYPE_ATTD       (0x1E)          // Broadcast Frame Type - Attitude

#define     CRSF_MSG_HEAD_SIZE      (3U)            // 0xC8 SYNC, 0x18 LEN (TYPE+PAYLOAD+CRC=24B), 0x16 TYPE (16 RC chans)
#define     CRSF_MSG_16CH_SIZE      (22U)           // PAYLOAD: 22*8bit = 176bit => 176bit/16chan = 11bit resolution per RC channel
#define     CRSF_MSG_CRC_SIZE       (1U)            // CRC-8 0xD5
#define     CRSF_MESSAGE_SIZE       (CRSF_MSG_HEAD_SIZE+CRSF_MSG_16CH_SIZE+CRSF_MSG_CRC_SIZE)
#define     CRSF_CHANNEL_COUNT      (16U)           // Standart 16-chan, but Radiomaster Pocket use 10chans
#define     CRSF_CRC_POLY           (0xD5)          // 0xD5 gives better burst‑error detection for short frames

volatile    bool            gCRSF_RX_flg_eol        = false;    // CRSF Frame is received and ready to be processed
volatile    uint32_t        gCRSF_Header_match_cnt  = 0U;       // Count of header matches
volatile    uint32_t        gCRSF_CRC_match_cnt     = 0U;       // Count of CRC matches
volatile    uint32_t        gCRSF_CRC_error_cnt     = 0U;       // Count of CRC errors
volatile    uint8_t         gCRSF_RX_DataCRC[ CRSF_MSG_16CH_SIZE + CRSF_MSG_CRC_SIZE ]; // Data (22B) + CRC (1B)
volatile    uint8_t         gCRSF_Frame_idx         = 0U;       // CRSF Frame type index used for telemetry

// ***** I2C1 - GYRO ACCELEROMETER *****
#define     I2C_TX_MAX_PACKET_SIZE (16U)
#define     I2C_RX_MAX_PACKET_SIZE (16U)
uint8_t     gI2C1_tx_buffer[ I2C_TX_MAX_PACKET_SIZE ] = { 0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F };
uint8_t     gI2C1_rx_buffer[ I2C_RX_MAX_PACKET_SIZE ] = { 0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F };
uint32_t    gI2C1_tx_size = 0U, gI2C1_tx_idx = 0U;      // Counters for TX length and bytes sent
uint32_t    gI2C1_rx_size = 0U, gI2C1_rx_idx = 0U;      // Counters for RX length and bytes sent
volatile    uint32_t    gI2C1_anti_block_cnt = 0U;      // I2C Anti Block System, counter for MCU loops
volatile    uint16_t    gI2C_anti_block_error = 0U;     // I2C Anti Block System, error memory 

// Indicates status of I2C
typedef enum
{
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR
} I2cControllerStatus_t;
// Declare the actual global variable as volatile!
volatile I2cControllerStatus_t gI2C_controller_status = I2C_STATUS_IDLE;

// enum I2cControllerStatus
// {
//     I2C_STATUS_IDLE = 0,
//     I2C_STATUS_TX_STARTED,
//     I2C_STATUS_TX_INPROGRESS,
//     I2C_STATUS_TX_COMPLETE,
//     I2C_STATUS_RX_STARTED,
//     I2C_STATUS_RX_INPROGRESS,
//     I2C_STATUS_RX_COMPLETE,
//     I2C_STATUS_ERROR,
// } gI2cControllerStatus;

// ***** IMU - GYRO ACCELEROMETER - MPU-6050 *****
//MPU physical AD0 = 0
#define     MPU_ADDRESS ( 0b1101000 )  // MPU slave address in 7bit format

// --- MPU structure ---
typedef struct mpu_t {
    // --- 1. CONFIG (Acc and Gyro Fullscale Range setup) ---
    struct {
        uint8_t     accel_afs_sel;      // 0=2g , 1=4g , 2=8g , 3=16g
        uint8_t     gyro_fs_sel;        // 0=250dps , 1=500dps , 2=1000dps , 3=2000dps
        float       accel_afs_lsb;      // 0=16384LSB/g , 1=8192LSB/g , 2=4096LSB/g , 3=2048LSB/g
        float       gyro_fs_lsb;        // 0=131LSB/dps , 1=65.5LSB/dps , 2=32.8LSB/dps , 3=16.4LSB/dps
    } cfg;
    // --- 2. STATUS & HEALTH ---
    struct {
        uint8_t     irq_latchs;         // reg INT_PIN_CFG = 0x20
        uint8_t     irq_enable;         // reg INT_ENABLE = 0x01
        uint8_t     exit_sleep;         // reg PWR_MGMT_1 = 0x00
        uint8_t     who_am_i;           // reg WHO_AM_I = 0x68
        uint8_t     irq_status;         // reg INT_STATUS (0x3A)
        uint16_t    error;              // Bitmask of your gMPU_return[10]
        bool        is_initialized;     // <--- THE FAILSAFE
        bool        is_calibrated;      // after Gyro calibration
        uint32_t    delay_cnt_ms;       // initialization delay
        bool        read_flg;           // read MPU flag
    } status;
    struct {
        uint32_t    irq_set_cnt;
        uint32_t    irq_not_set_cnt;
        uint32_t    reset_cnt;
    } diag;
    // --- 3. Calibration Offsets ---
    struct {
        int16_t accX_lsb;  // Initialized to 500
        int16_t accY_lsb;  // Initialized to -250
        int16_t accZ_lsb;
        int16_t gyroX_lsb; // Initialized to -250
        int16_t gyroY_lsb;
        int16_t gyroZ_lsb;
    } offset;
    // --- 4. Raw Data (Direct from I2C Buffer) ---
    struct {
        int16_t accX_lsb;
        int16_t accY_lsb;
        int16_t accZ_lsb;
        int16_t temp_lsb;
        int16_t gyroX_lsb;
        int16_t gyroY_lsb;
        int16_t gyroZ_lsb;
    } raw;
    // --- 5. Processed (Compensated) LSB Integers ---
    struct {
        int16_t accX_lsb;       // Accelerometer X Compensated Values in LSB
        int16_t accY_lsb;       // Accelerometer Y Compensated Values in LSB
        int16_t accZ_lsb;       // Accelerometer Z Compensated Values in LSB
        int16_t gyroX_lsb;      // Gyroscope X Compensated Values in LSB
        int16_t gyroY_lsb;      // Gyroscope Y Compensated Values in LSB
        int16_t gyroZ_lsb;      // Gyroscope Z Compensated Values in LSB
    } val;
    // --- 6. IQ Fixed-Point Physical Data ---
    struct {
        _iq     iqAccX_g;
        _iq     iqAccY_g;
        _iq     iqAccZ_g;
        _iq     iqGyroX_rads;
        _iq     iqGyroY_rads;
        _iq     iqGyroZ_rads;
        _iq     iqTemp_C;
        
        _iq     iqPitch_rad;
        _iq     iqRoll_rad;
        _iq     iqYaw_rad;
        
        _iq     iqPitch_fil_rad;
        _iq     iqRoll_fil_rad;
        _iq     iqYaw_fil_rad;
        
        _iq     iqPitch_N_1;
        _iq     iqRoll_N_1;
        _iq     iqYaw_N_1;
    } iq;
} mpu_t;

volatile mpu_t gMpu = {
    .cfg = {
        .accel_afs_sel = 0,         // 2g
        .gyro_fs_sel = 0,           // 250 dps
        .accel_afs_lsb = 16384.0f,
        .gyro_fs_lsb = 131.0f
    },
    .offset = {
        .accX_lsb  = 500,  .accY_lsb  = -250, .accZ_lsb  = 0,
        .gyroX_lsb = -250, .gyroY_lsb = 0,    .gyroZ_lsb = 0
    },
    .status = {
        .irq_latchs = 0U,
        .irq_enable = 0U,
        .exit_sleep = 0xFF,
        .who_am_i = 0U,
        .error = 0U,
        .is_initialized = false,
        .is_calibrated = false,
        .delay_cnt_ms = TIME_MPU_DELAY_MS,
        .read_flg = false
    }
};
// *****************************************  *****************************************  *****************************************
// *****    FUNCTIONS DECLARATIONS     *****  *****    FUNCTIONS DECLARATIONS     *****  *****    FUNCTIONS DECLARATIONS     *****
// *****************************************  *****************************************  *****************************************
void        mcu_initialize  ( void );

void        mpu_initialize  ( void );
void        mpu_calibrate   ( void );
void        mpu_process_data ( void );

void        pd_outer_loop( void );
void        pd_inner_loop( void );

void        smc_inner_loop( void );
float       smc_saturate(float s, float arg_boundary_layer);

void        adc12_update( void );

void        motor_rh_set_voltage( float arg_volt );
void        motor_lh_set_voltage( float arg_volt );
void        motor_set_voltage( float arg_volt_lh, float arg_volt_rh );
void        motor_set_duty(uint8_t arg_phase,uint16_t arg_duty);

void        uart0_update( void );
uint16_t    uart0_build_csv_frame(uint8_t arg_count );
void        uart0_clear_buff(void);

void        crsf_update ( void );
bool        crsf_rx_xxtract_16ch_payload( void );
uint8_t     crsf_get_crc8( uint8_t arg_frame_type, volatile uint8_t *arg_data, uint8_t arg_size );
void        crsf_parse_data( void );
uint16_t    crsf_reverse11( uint16_t arg_data_for_rev );
void        crsf_tx_telemetry( void );

void        wheel_get_speed_sg(void);

void        robot_ident( void );
float       robot_get_speed_sg(void);
float       robot_get_turn_rate_sg(void);
void        robot_get_wheel_physics(volatile motor_t *arg_motor , volatile wheel_t *arg_wheel);
void        robot_get_motor_current( volatile motor_t *arg_motor, uint32_t arg_raw_adc_lsb );
void        robot_get_battery_voltage( volatile power_t *arg_power , uint32_t raw_adc_lsb );

bool        mpu_uav_calculation ( volatile mpu_t *arg_mpu );
uint16_t    mpu_write_reg( uint8_t arg_reg , uint8_t arg_data );
uint16_t    mpu_read_regs( uint8_t arg_reg , uint8_t arg_size );
void        mpu_update_physics(volatile mpu_t *arg_mpu, uint8_t *arg_buffer);

void        ws2812b_batt_gauge(float arg_volt);
void        ws2812b_tilt_gauge(float tilt_rad);
void        ws2812b_set_led_color(uint8_t arg_led_idx, uint8_t R, uint8_t G, uint8_t B);
// void        ws2812b_all_led(uint8_t arg_color, uint8_t arg_brightness);
// void        WS2812B_Half_LED(uint8_t argRGB1, uint8_t argRGB2, uint8_t argBrg1, uint8_t argBrg2);

static inline float filter_median_3_f(float a, float b, float c);
static inline float filter_median_5_f(float p0, float p1, float p2, float p3, float p4);

// *****************************************  *****************************************  *****************************************
// *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****
// *****************************************  *****************************************  *****************************************
int main( void ) {
    
    mcu_initialize();   // enable IRQs, set DMA, set Motors to 0V
    
    mpu_initialize();   // Latch MPU interrupts, Exit sleep mode, read WHO_AM_I

    mpu_calibrate();    // Robot must be in upright position - gather 1250 samples to calibrate MPU

// *****************************************  *****************************************  *****************************************
// *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****
// *****************************************  *****************************************  *****************************************
    while ( 1 ) {

        // ----- 2. READ MPU - get actual acc and gyro data -----        
        mpu_process_data();
        // ----- 3. (A) OUTER LOOP -----
        pd_outer_loop();
        // ----- 4. (B) INNER LOOP -----
        switch ( gController_type ) {
            case CONTROLLER_TYPE_PID: pd_inner_loop();  break;
            case CONTROLLER_TYPE_SMC: smc_inner_loop(); break;
                             default: pd_inner_loop();  break; }
        // ----- 5. UART UPDATE -----
        uart0_update();
        // ----- 6. BATTERY GAUGE -----
        ws2812b_batt_gauge( gRobot.power.batt_volt_V );
        // ----- 7. TILT DISPLAY -----
        ws2812b_tilt_gauge( gRobot.body.pitch_rad );
        // ----- 9. ANALOG MEASUREMENT -----
        adc12_update();
        // ----- 10. CROSS FIRE DETECTOR -----
        crsf_update();
        // ----- 11. SEND BATT TELEMETRY -----
        crsf_tx_telemetry();
        // ----- 0. IDENTIFICATION (STEP RESPONSE) - not used during ballancing -----
        robot_ident();
        // ----- 8. DURATION -----
        // gDuration_timer_us.total_loop = gDuration_timer_us.inner_loop + gDuration_timer_us.outer_loop;
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
    // ----- 1ms COUNTER for IDENT -----
    gCnt_wait_ms++;
    
    // ----- Heart LED beat -----
    if ( --gCnt_LED_period == 0 )
    {
        if ( gLED_seq_now & ( 1 << gCnt_LED_seq_idx ) )
        {
            DL_GPIO_setPins( GPIO_PORT , GPIO_GRN4_PIN );
            gCnt_LED_period = TIME_LED_MS;
        }
        else {
            DL_GPIO_clearPins( GPIO_PORT , GPIO_GRN4_PIN );
            gCnt_LED_period = TIME_LED_MS;
        }        

        gCnt_LED_seq_idx++;
        gCnt_LED_seq_idx %= LED_TICK_COUNT;
    }

    // ----- MOTOR SPEED/OMEGA MEASUREMENT -----
    if ( --gCnt_outer_loop == 0 )
    {
        gCnt_outer_loop = TIME_MOTOR_OMEGA_MS;
        gFlg_run_outer_loop = true;
    }

    // ----- MPU - DELAY -----
    if ( gMpu.status.delay_cnt_ms > 0 )
        --gMpu.status.delay_cnt_ms;
    
    // ----- UART0 TX -----
    // if ( --gCnt_UART0_TXD == 0 )
    // {
    //     gCnt_UART0_TXD = TIME_SEND_DATA_MS;
    //     gFlg_UART0_TXD = true;
    // }

    // ----- UART1 TX -----    
    if ( --gCnt_UART1_TXD == 0 )
    {
        gCnt_UART1_TXD = TIME_SEND_TELE_MS;
        gFlg_UART1_TXD = true;
    }

    // ----- COMMUNICATION TIME OUT - motors STOP
    if ( gCnt_fail_safe > 0 )
    {
        if ( --gCnt_fail_safe == 0 ) {
            gRobot.rc_cmd.is_connected = false;
        }
    }

    // ----- BATT and TILT GAUGE -----
    if ( --gCnt_batt_gauge == 0 )
    {
        gCnt_batt_gauge = TIME_BATT_GAU_MS;
        gFlg_batt_gauge = true;
    }
    if ( --gCnt_tilt_angle == 0 )
    {
        gCnt_tilt_angle = TIME_TILT_GAU_MS;
        gFlg_tilt_angle = true;
    }
}

/****************************************
*****   TIMER 8 for 0.8ms timing    *****
****************************************/
void TIMER_8_INST_IRQHandler(void)
{
    // READ DATA FROM MPU
    gMpu.status.read_flg = true; 
}

/****************************************
*****   ADC 0 - MOTOR 1 - TEMP+CURR *****
****************************************/
// Chan 2: Current M1
// Chan 4: Temp M1   
void ADC12_0_INST_IRQHandler( void )
{
    switch  ( DL_ADC12_getPendingInterrupt( ADC12_0_INST ) )
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gADC_flg[ 4 ] = true;
            gADC_LSB[ 4 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_TEMP_M1 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            gADC_flg[ 2 ] = true;
            gADC_LSB[ 2 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_CURR_M1 );
GET_DURATION(11);
gDuration_timer_us.adc_m1 = gDuration_timer_us.delta_time[11];
// gDuration_timer_us.adc_m1 = gDelta_time[11];
STORE_TIMER6(11);
            break;    
        default:
            break;
    }
}
/****************************************
***** ADC 1 - MOTOR 0 - TEMP+CURR+VS ****
****************************************/
// Chan 0: VS1(Battery)
// Chan 1: Current M0
// Chan 3: Temp M0
void ADC12_1_INST_IRQHandler( void )
{
    switch  ( DL_ADC12_getPendingInterrupt( ADC12_1_INST ) )
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gADC_flg[ 3 ] = true;
            gADC_LSB[ 3 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_TEMP_M0 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            gADC_flg[ 1 ] = true;
            gADC_LSB[ 1 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_CURR_M0 );
GET_DURATION(10);
gDuration_timer_us.adc_m0 = gDuration_timer_us.delta_time[10];
// gDuration_timer_us.adc_m0 = gDelta_time[10];
STORE_TIMER6(10);
            break;    
        case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            gADC_flg[ 0 ] = true;
            gADC_LSB[ 0 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_VS1 );
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
            gRX0_idx &= RX0_BUFF_SIZE - 1U;  // mask to prevent overflow
            gRX0_buffer[ gRX0_idx ] = DL_UART_Main_receiveData( UART_0_INST );
            // Check terminator
            if( gRX0_buffer[ gRX0_idx ] == '\n' )
                gRX0_flg_eol = true;
            gRX0_idx++;
        break;

        case DL_UART_MAIN_IIDX_TX:
            //transmit byte
            DL_UART_Main_transmitData ( UART_0_INST , gTX0_buffer[ gTX0_idx++ ] );
            // Disable transmit interrupt if no more bytes
            if ( gTX0_idx == gTX0_size )
            {
                DL_UART_Main_disableInterrupt( UART_0_INST , DL_UART_MAIN_INTERRUPT_TX );
                gTX0_idx = 0;
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
    uint8_t rx_byte = 0;

    switch ( DL_UART_Main_getPendingInterrupt( UART_1_INST ) )
    {
        case DL_UART_MAIN_IIDX_RX:
            // Read hardware FIFO directly into local variable
            rx_byte = (uint8_t)DL_UART_Main_receiveData( UART_1_INST );

            // --- BYTE 0: Must be SYNC (0xC8) ---
            if ( gRX1_idx == 0 )
            {
                if ( rx_byte == 0xC8 )
                {
                    gRX1_buffer[0] = rx_byte;
                    gRX1_idx = 1;
                }
                break;
            }

            // --- BYTE 1: Must be LENGTH (0x18) ---
            if ( gRX1_idx == 1 )
            {
                if ( rx_byte == 0x18 )
                {
                    gRX1_buffer[1] = rx_byte;
                    gRX1_idx = 2;
                }
                else
                {
                    gRX1_idx = 0; // Alignment lost, reset
                }
                break;
            }

            // --- BYTE 2: Must be TYPE (0x16) ---
            if ( gRX1_idx == 2 )
            {
                if ( rx_byte == 0x16 )
                {
                    gRX1_buffer[2] = rx_byte;
                    gRX1_idx = 3;
                }
                else
                {
                    gRX1_idx = 0; // Alignment lost, reset
                }
                break;
            }

            // --- PAYLOAD & CRC (Bytes 3 to 25) ---
            gRX1_buffer[gRX1_idx++] = rx_byte;

            // Full 26-byte packet collected
            if ( gRX1_idx >= 26 )
            {
                gCRSF_RX_flg_eol = true;    // Signal main loop to run CRC & decode channels
                gRX1_idx = 0;               // Reset index for next incoming frame
            }
            break;

        case DL_UART_MAIN_IIDX_TX:
            //transmit byte
            DL_UART_Main_transmitData ( UART_1_INST , gTX1_buffer[ gTX1_idx++ ] );
            // Disable transmit interrupt if no more bytes
            if ( gTX1_idx == gTX1_size )
            {
                DL_UART_Main_disableInterrupt( UART_1_INST , DL_UART_MAIN_INTERRUPT_TX );
                gTX1_idx = 0;
                // gCnt_Tele_sended++;
            }
            break;

        default:
            break;
    }
}

// *********************************
// *****   I2C 1 TRANSMITTER   *****
// *********************************
void I2C1_INST_IRQHandler ( void )
{
    switch ( DL_I2C_getPendingInterrupt( I2C1_INST ) )
    {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            
            gI2C_controller_status = I2C_STATUS_RX_COMPLETE;
            
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            
            DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
            gI2C_controller_status = I2C_STATUS_TX_COMPLETE;
            
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            
            gI2C_controller_status = I2C_STATUS_RX_INPROGRESS;
            // Receive all bytes from target
            while ( DL_I2C_isControllerRXFIFOEmpty( I2C1_INST ) != true )
            {
                if ( gI2C1_rx_idx < gI2C1_rx_size ) {
                    gI2C1_rx_buffer[ gI2C1_rx_idx++ ] = DL_I2C_receiveControllerData( I2C1_INST );
                } else {
                    // Ignore and remove from FIFO if the buffer is full
                    DL_I2C_receiveControllerData( I2C1_INST );
                }
            }

            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            
            gI2C_controller_status = I2C_STATUS_TX_INPROGRESS;
            // Fill TX FIFO with next bytes to send
            if ( gI2C1_tx_idx < gI2C1_tx_size )
            {
                gI2C1_tx_idx += DL_I2C_fillControllerTXFIFO(
                    I2C1_INST , 
                    &gI2C1_tx_buffer[ gI2C1_tx_idx ] , 
                    gI2C1_tx_size - gI2C1_tx_idx );
            }
            break;
            // Not used for this example
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ( ( gI2C_controller_status == I2C_STATUS_RX_STARTED ) ||
                 ( gI2C_controller_status == I2C_STATUS_TX_STARTED ) )
            {
                // NACK interrupt if I2C Target is disconnected
                gI2C_controller_status = I2C_STATUS_ERROR;
            }
            break;

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

/******************************
*****   GPIO QUAD ABCD    *****
*******************************/
void GROUP1_IRQHandler(void)
{
    STORE_TIMER6( 0 );
    // used to select correct ports and pins
    static uint32_t PortIndex;
    static uint32_t PinIndex_M0;
    static uint32_t PinIndex_M1;
    // Read all quadrature pins once
    bool A = DL_GPIO_readPins(QUAD_M0_PORT, QUAD_M0_A_PIN);
    bool B = DL_GPIO_readPins(QUAD_M0_PORT, QUAD_M0_B_PIN);
    bool C = DL_GPIO_readPins(QUAD_M1_PORT, QUAD_M1_C_PIN);
    bool D = DL_GPIO_readPins(QUAD_M1_PORT, QUAD_M1_D_PIN);
    // 
    PortIndex = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
    PinIndex_M0 = DL_GPIO_getPendingInterrupt(QUAD_M0_PORT);
    PinIndex_M1 = DL_GPIO_getPendingInterrupt(QUAD_M1_PORT);
    
    switch(PortIndex)
    {
    case QUAD_M0_INT_IIDX:      // RH MOTOR
        switch (PinIndex_M0)
        {
        
        case QUAD_M0_A_IIDX:    // A - channel
            if (A != sQuad_memory.A)
            {
                sQuad_memory.A = A;
                gRobot.motor.right.imp += (A ^ B) ? -1 : +1;
            }
            break;
        
        case QUAD_M0_B_IIDX:    // B - channel
            if (B != sQuad_memory.B)
            {
                sQuad_memory.B = B;
                gRobot.motor.right.imp += (B ^ A) ? +1 : -1;
            }
            break;
        }
    
    case QUAD_M1_INT_IIDX:      // LH MOTOR
        switch (PinIndex_M1)
        {
        
        case QUAD_M1_C_IIDX:    // C - channel
            if (C != sQuad_memory.C)
            {
                sQuad_memory.C = C;
                gRobot.motor.left.imp += (C ^ D) ? +1 : -1;
            }
            break;
        
        case QUAD_M1_D_IIDX:    // D - channel
            if (D != sQuad_memory.D)
            {
                sQuad_memory.D = D;
                gRobot.motor.left.imp += (D ^ C) ? -1 : +1;
            }
            break;
        }
    }
    GET_DURATION( 0 );
    gDuration_timer_us.quad_isr = gDuration_timer_us.delta_time[0];
    // gDuration_timer_us.quad_isr = gDelta_time[0];
}

// ----- ANALOG MEASUREMENT FUNCTION ------
// 5 channel used: (0)VS1(Battery), (1)Current M0, (2)Current M1 , (3)Temp M0, (4)Temp M1     
void adc12_update( void ) {
    for ( int i = 0 ; i < ADC_CNT ; i++ ) {
        if ( gADC_flg[ i ] ) {
            gADC_flg[ i ] = false;

            gADC_Volt[ i ] = (float)gADC_LSB[ i ] * VOLT_ADC_REF / 4096.0f;   // LSB to volt 

            // >> BATTERY <<
            if ( i == 0 ) {
                robot_get_battery_voltage( (power_t *)&gRobot.power , gADC_LSB[0] );
            }
            // >> MOTORS  <<            
            // Motor 0
            if ( i == 1 ) {
                robot_get_motor_current( &gRobot.motor.right , gADC_LSB[1] );
            }
            // Motor 1
            if ( i == 2 ) { 
                robot_get_motor_current( &gRobot.motor.left , gADC_LSB[2] );
            }
        }
    }
}

void crsf_update( void ) {
    if (!gCRSF_RX_flg_eol) return;
    gCRSF_RX_flg_eol = false;

GET_DURATION( 9 );
gDuration_timer_us.crsf = gDuration_timer_us.delta_time[9];  
// gDuration_timer_us.crsf = gDelta_time[9];  
STORE_TIMER6( 9 );

    // 1. SNAPSHOT: Extract and Parse into local/intermediate buffers
    if (crsf_rx_xxtract_16ch_payload() && gRobot.rc_cmd.is_connected) {
        crsf_parse_data();

        // 2. PROCESS: Use local variables for math to keep the pipeline clean
        
        // Calculation using 174-1811 range to map into -1.00f to +1.00f
        // a = (+1.00-(-1.00)) / (1811-174) = 0.00122175f
        // b = -1.00 - (a * 174) ≈ -1.21258f
        const float a1 = (+1.00f-(-1.00f))/(1811.0f-174.0f);
        const float b1 = -1.00f - (a1 * 174.0f);
        
        float loc_roll     = a1 * (float)gRobot.rc_cmd.ch[0] + b1;
        float loc_pitch    = a1 * (float)gRobot.rc_cmd.ch[1] + b1;
        float loc_yaw      = a1 * (float)gRobot.rc_cmd.ch[3] + b1;

        // Calculation using 191.0f-1792.0f range to map into +0.00f to +2.00f
        const float a2 = (+2.00f-(+0.00f)) / (1792.0f-191.0f); 
        const float b2 = (+0.00f) - (a2 * 191.0f);
        
        float loc_latch_L  = a2 * (float)gRobot.rc_cmd.ch[4] + b2;
        float loc_state3_L = a2 * (float)gRobot.rc_cmd.ch[5] + b2;
        float loc_state3_R = a2 * (float)gRobot.rc_cmd.ch[6] + b2;
        float loc_latch_R  = a2 * (float)gRobot.rc_cmd.ch[7] + b2;
        float loc_button_L = a2 * (float)gRobot.rc_cmd.ch[8] + b2;

        // Pre-calculated coefficients (using 174-1811 range) to map into +1.00f to +2.00f
        const float a3 = (+2.0f-(+1.0f)) / (1811.0f-174.0f);
        const float b3 = (+1.0f) - (a3 * 174.0f);

        float loc_throttle = a3 * (float)gRobot.rc_cmd.ch[2] + b3;

        // 3. COMMIT: Write processed data back to the global struct in one block
        gRobot.rc_cmd.roll     = loc_roll;
        gRobot.rc_cmd.pitch    = loc_pitch;
        gRobot.rc_cmd.throttle = loc_throttle;
        gRobot.rc_cmd.yaw      = loc_yaw;
        
        gRobot.rc_cmd.latch_L  = loc_latch_L;
        gRobot.rc_cmd.latch_R  = loc_latch_R;
        
        gRobot.rc_cmd.state3_L = loc_state3_L;
        gRobot.rc_cmd.state3_R = loc_state3_R;

        gRobot.rc_cmd.button_L = loc_button_L;

        // *** NEW Setpoint/Reference ***
        // As Forward movment need to lean forward, we need to have the PITCH angle negative
        gRobot.control.SP_Pitch_rad = -0.25f * (float)loc_pitch;    // Pitch Set point range -0.25 to 0.25 rad
        
        // Switch on/off logging
        if ( loc_latch_L > 1.20f )
            gFlg_UART0_TX_En = true;
        else {
            gFlg_UART0_TX_En = false;
            gTX0_sample_cnt = 0U;
        }
        
        // Switch between PID and SMC
        if ( loc_latch_R > 1.20f )
            gController_type = CONTROLLER_TYPE_SMC;
        else
            gController_type = CONTROLLER_TYPE_PID;

        // Apply boosted gains - PID
        if ( gController_type == CONTROLLER_TYPE_PID ) {
            gRobot.gain.KP = PID_KP_GAIN_DEF * loc_state3_L * loc_throttle;
            gRobot.gain.KD = PID_KD_GAIN_DEF * loc_state3_R;
        }
        // Apply boosted gains - SMC
        else if ( gController_type == CONTROLLER_TYPE_SMC ) {
            smc_controller.K    = SMC_K_DEF     * loc_state3_L;
            smc_controller.phi  = SMC_PHI_DEF   * loc_state3_R;
        }
    }
    else {
        // FAILSAFE COMMIT
        gRobot.control.SP_Pitch_rad  *= 0.9f;    // upright angle fade
        gRobot.rc_cmd.roll           *= 0.9f;    // turning fade
        gRobot.rc_cmd.pitch          *= 0.9f;    // pitching fade
    }
}

uint8_t crsf_get_crc8( uint8_t arg_frame_type, volatile uint8_t *arg_data, uint8_t arg_size ) {
    uint8_t loc_crc = 0;

    loc_crc = gCRC8_LUT[ loc_crc ^ arg_frame_type ];

    for ( uint8_t i = 0 ; i < arg_size ; i++ )
        loc_crc = gCRC8_LUT[ loc_crc ^ arg_data[ i ] ];
    return loc_crc;
}    

// Function to extract payload and CRC from PACKET -> following the header
bool crsf_rx_xxtract_16ch_payload( void ) {
    // Define header sequence:
    // 0xC8 - SYNC BYTE
    // 0x18 - LENGTH (FRAME_TYPE+PAYLOAD+CRC8 => 1B+22B+1B=24B -> 0x18)
    // 0x16 - FRAME TYPE (RC channels)
    
    uint8_t header[ CRSF_MSG_HEAD_SIZE ] = { 0xC8 , 0x18 , 0x16 };
    uint8_t crc_calculated = 0;

    // Search for header in the buffer
    for ( int i = 0 ; i < RX1_BUFF_SIZE ; i++ ) {
        // Check for header match
        if (    gRX1_buffer[ i ]                         == header[ 0 ] &&
                gRX1_buffer[ ( i + 1 ) % RX1_BUFF_SIZE ] == header[ 1 ] &&
                gRX1_buffer[ ( i + 2 ) % RX1_BUFF_SIZE ] == header[ 2 ] ) {
            gCRSF_Header_match_cnt++;
            // run Fail Safe Timer
            gCnt_fail_safe = TIME_FAIL_SAFE_MS;
            gRobot.rc_cmd.is_connected = true;
            // Extract the 22 bytes of 16ch payload + CRC following the header
            for ( int j = 0 ; j < CRSF_MSG_16CH_SIZE + CRSF_MSG_CRC_SIZE ; j++ ) {
                gCRSF_RX_DataCRC[ j ] = gRX1_buffer[ ( i + CRSF_MSG_HEAD_SIZE + j ) % RX1_BUFF_SIZE ];
            }
            // clear UART buffer
            for ( int k = 0 ; k < RX1_BUFF_SIZE ; k++ )
                gRX1_buffer[ k ] = 0x00;

            crc_calculated = crsf_get_crc8( CRSF_FR_TYPE_16CH, &gCRSF_RX_DataCRC[0], CRSF_MSG_16CH_SIZE );
            // gCSFR_CRC_Calculated = CSFR_Get_CRC8();

            if ( crc_calculated == gCRSF_RX_DataCRC[ 22 ] ) {  // CRC match?
                gCRSF_CRC_match_cnt++;
                return true; // Data extraction successful and CRC is OK
            }
            else {
                gCRSF_CRC_error_cnt++;
                return false; // Header found but CRC error
            }            
        }
    }
    return false; // Header not found or CRC error
}

// *********************************
// *****   CRSF TX Telemetry   *****
// *********************************
// parser and constructor of 16ch payload from 22 bytes
void crsf_parse_data( void ) {
    uint8_t reversed_byte = 0;
    uint8_t reversed_array[ CRSF_MSG_16CH_SIZE ];
    
    // reverse bits in each byte
    for ( uint8_t i = 0 ; i < CRSF_MSG_16CH_SIZE ; i++ ) {
        reversed_byte = 0;
        
        for ( uint8_t j = 0 ; j < 8 ; j++ ) {
            if ( ( gCRSF_RX_DataCRC[ i ] >> j ) & 0x01 )
                reversed_byte |= 1 << ( 7 - j );
        }
        reversed_array[ i ] = reversed_byte;
    }
    
    gRobot.rc_cmd.ch[ 0 ] = crsf_reverse11( ( ( reversed_array[  0 ] & 0xFF ) << 3 ) | ( reversed_array[  1 ] >> 5 ) );
    gRobot.rc_cmd.ch[ 1 ] = crsf_reverse11( ( ( reversed_array[  1 ] & 0x1F ) << 6 ) | ( reversed_array[  2 ] >> 2 ) );
    gRobot.rc_cmd.ch[ 2 ] = crsf_reverse11( ( ( reversed_array[  2 ] & 0x3F ) << 9 ) | ( reversed_array[  3 ] << 1 ) | ( reversed_array[ 4 ] >> 7 ) );
    gRobot.rc_cmd.ch[ 3 ] = crsf_reverse11( ( ( reversed_array[  4 ] & 0x7F ) << 4 ) | ( reversed_array[  5 ] >> 4 ) );
    
    gRobot.rc_cmd.ch[ 4 ] = crsf_reverse11( ( ( reversed_array[  5 ] & 0x0F ) << 7 ) | ( reversed_array[  6 ] >> 1 ) );
    gRobot.rc_cmd.ch[ 5 ] = crsf_reverse11( ( ( reversed_array[  6 ] & 0x01 ) << 10 )| ( reversed_array[  7 ] << 2 ) | ( reversed_array[ 8 ] >> 6 ) );
    gRobot.rc_cmd.ch[ 6 ] = crsf_reverse11( ( ( reversed_array[  8 ] & 0x3F ) << 5 ) | ( reversed_array[  9 ] >> 3 ) );
    gRobot.rc_cmd.ch[ 7 ] = crsf_reverse11( ( ( reversed_array[  9 ] & 0x07 ) << 8 ) | ( reversed_array[ 10 ] >> 0 ) );

    gRobot.rc_cmd.ch[ 8 ] = crsf_reverse11( ( ( reversed_array[ 11 ] & 0xFF ) << 3 ) | ( reversed_array[ 12 ] >> 5 ) );
    gRobot.rc_cmd.ch[ 9 ] = crsf_reverse11( ( ( reversed_array[ 12 ] & 0x1F ) << 6 ) | ( reversed_array[ 13 ] >> 2 ) );
}

uint16_t crsf_reverse11( uint16_t arg_data_for_rev ) {
    uint16_t reversed11 = 0;
    // reverse bits in each channel
    for ( uint8_t j = 0 ; j < 11 ; j++ )
    {
        if ( ( arg_data_for_rev >> j ) & 0x001 )
            reversed11 |= 1 << ( 10 - j );
    }
    return reversed11;
}

// *********************************
// *****   CRSF TX Telemetry   *****
// *********************************
void crsf_tx_telemetry( void ) {
    if ( !gFlg_UART1_TXD ) return;
    
    gFlg_UART1_TXD = false;

    // Recalculate battery voltage and current into CRSF format
    uint16_t volt_dec = (uint16_t)( gRobot.power.batt_volt_V * 10.0f );   // 12.6V -> 126
    uint16_t curr_dec = (uint16_t)( gRobot.power.batt_curr_A * 10.0f );   // 1.5A  -> 15
    // Recalculate speed into CRSF format (km/h * 10)
    uint16_t crsf_speed = (uint16_t)(gRobot.body.speed_ms * 3.60f * 10.0f);
    // Recalculate angles (rad) into CRSF format ( * 10000 )
    int16_t pitch_val = (int16_t)(gRobot.body.pitch_rad * 10000.0f);
    int16_t roll_val  = (int16_t)(gRobot.body.roll_rad  * 10000.0f);
    int16_t yaw_val   = (int16_t)(gRobot.body.yaw_rad   * 10000.0f);

    switch ( gCRSF_Frame_idx ) {
         // ***** Telemetry frame Battery Sensor *****
        case 0:
            {
                // Calculate and safely clamp battery gauge percentage
                float gauge = 100.0f * ( gRobot.power.batt_volt_V - VOLT_BATTERY_MIN ) / ( VOLT_BATTERY_MAX - VOLT_BATTERY_MIN );
                if ( gauge < 0.0f   ) gauge = 0.0f;
                if ( gauge > 100.0f ) gauge = 100.0f;
                gRobot.power.batt_gauge_ptc = gauge;
            }
            // robot.power.batt_gauge_ptc = 100.0f * ( robot.power.batt_volt_V - VOLT_BATTERY_MIN ) / ( VOLT_BATTERY_MAX - VOLT_BATTERY_MIN );
                    
            // CRSF header assembly
            gTX1_buffer[0] = CRSF_SYNC_BYTE;        // 0xC8
            gTX1_buffer[1] = 0x0A;                  // Length from TYPE to CRC (1 + 8 + 1)
            gTX1_buffer[2] = CRSF_FR_TYPE_BATT;     // 0x08

            // Big-Endian for values:
            // Voltage (uint16_t)
            gTX1_buffer[3] = ( volt_dec >> 8 ) & 0xFF;
            gTX1_buffer[4] =   volt_dec & 0xFF;
            // Current (uint16_t)
            gTX1_buffer[5] = ( curr_dec >> 8 ) & 0xFF;
            gTX1_buffer[6] =   curr_dec & 0xFF;
            // Capacity used mAh (uint24_t - 3bytes)
            gTX1_buffer[7] = ( gRobot.power.batt_cap_mAh >> 16 ) & 0xFF;
            gTX1_buffer[8] = ( gRobot.power.batt_cap_mAh >> 8  ) & 0xFF;
            gTX1_buffer[9] =   gRobot.power.batt_cap_mAh & 0xFF;
            // Remaining % (uint8_t)
            gTX1_buffer[10] = (uint8_t)gRobot.power.batt_gauge_ptc;
            // CRC8 calculated from FRAME TYPE and PAYLOAD
            gTX1_buffer[11] = crsf_get_crc8( CRSF_FR_TYPE_BATT, &gTX1_buffer[3], 8U );

            gTX1_size = 12U;
            DL_UART_Main_enableInterrupt( UART_1_INST, DL_UART_MAIN_INTERRUPT_TX );

            gCRSF_Frame_idx = 1U; // Next frame

            break;
        
        // ***** Telemetry frame Attitude Sensor *****
        case 1:
            gTX1_buffer[0] = CRSF_SYNC_BYTE;        // 0xC8
            gTX1_buffer[1] = 0x08;                  // Length from TYPE to CRC (1 + 6 + 1)
            gTX1_buffer[2] = CRSF_FR_TYPE_ATTD;     // TYPE (Attitude)

            // Big-Endian Pitch
            gTX1_buffer[3] = (pitch_val >> 8) & 0xFF;
            gTX1_buffer[4] = pitch_val & 0xFF;

            // Big-Endian Roll
            gTX1_buffer[5] = (roll_val >> 8) & 0xFF;
            gTX1_buffer[6] = roll_val & 0xFF;

            // Big-Endian Yaw
            gTX1_buffer[7] = (yaw_val >> 8) & 0xFF;
            gTX1_buffer[8] = yaw_val & 0xFF;

            // CRC8 calculated from FRAME TYPE and PAYLOAD
            gTX1_buffer[9] = crsf_get_crc8( CRSF_FR_TYPE_ATTD, &gTX1_buffer[3], 6U );

            gTX1_size = 10U;
            DL_UART_Main_enableInterrupt( UART_1_INST, DL_UART_MAIN_INTERRUPT_TX );

            gCRSF_Frame_idx = 2U; // Next frame
            
            break;

        // ***** Telemetry frame Attitude Sensor *****
        case 2:
            gTX1_buffer[0] = CRSF_SYNC_BYTE;        // 0xC8
            gTX1_buffer[1] = 0x11;                  // Length from TYPE to CRC (17B 1 + 15 + 1)
            gTX1_buffer[2] = CRSF_FR_TYPE_GPS;      // TYPE (Attitude)

            // Latitude (int32_t) -> 0
            gTX1_buffer[3]  = 0; gTX1_buffer[4]  = 0; gTX1_buffer[5]  = 0; gTX1_buffer[6]  = 0;
            
            // Longitude (int32_t) -> 0
            gTX1_buffer[7]  = 0; gTX1_buffer[8]  = 0; gTX1_buffer[9]  = 0; gTX1_buffer[10] = 0;

            // Ground Speed (uint16_t, Big-Endian)
            gTX1_buffer[11] = (crsf_speed >> 8) & 0xFF;
            gTX1_buffer[12] = crsf_speed & 0xFF;

            // Heading (uint16_t) - směr (zde 0)
            gTX1_buffer[13] = 0; gTX1_buffer[14] = 0;

            // Altitude (uint16_t) - výška s offsetem 1000m (zde posíláme 1000 -> reálně 0m)
            gTX1_buffer[15] = (1000 >> 8) & 0xFF;
            gTX1_buffer[16] = 1000 & 0xFF;

            // Satellites (uint8_t) - např. 0 satelitů
            gTX1_buffer[17] = 0;

            gTX1_buffer[18] = crsf_get_crc8( CRSF_FR_TYPE_GPS, &gTX1_buffer[3], 15U );

            gTX1_size = 19U;
            DL_UART_Main_enableInterrupt( UART_1_INST, DL_UART_MAIN_INTERRUPT_TX );

            gCRSF_Frame_idx = 0U; // Next frame (back to 0)

            break;

        default:
            gCRSF_Frame_idx = 0U; // Safety fallback
            break;
    }

}

// ----- PD INNER LOOP FUNCTION -----
// Tilt stabilization Controller
void pd_inner_loop( void ) {
    if (!gMpu.status.is_initialized) return; // DON'T DRIVE MOTORS IF SENSOR IS DEAD

    if ( gLED_seq_now != LED_SEQ_PD_CTRL ) gLED_seq_now = LED_SEQ_PD_CTRL;
    
    if ( (gFlg_run_inner_loop) && (gRobot_state == ROBO_STATE_BALA) )
    {
GET_DURATION(7);
gDuration_timer_us.inner_loop = gDuration_timer_us.delta_time[7];
// gDuration_timer_us.inner_loop = gDelta_time[7];
STORE_TIMER6(7);
        gFlg_run_inner_loop = false;

        // --- 1. FEED FORWARD / "The Direct Drive" ---
        gRobot.control.FFterm_V  = -(gRobot.gain.FF * gRobot.rc_cmd.pitch);   // Stick is leaning to fornt - forward

        // --- 2. SETPOINT / "The Balancing vortex" ---
        if (!gRobot.rc_cmd.is_connected) // if not connected use the Position control
            gRobot.control.SP_Pitch_rad = gRobot.control.Correction_Pos_rad;
        
        // --- 3. ERROR / "The Mission" ---
        gRobot.control.Err_Pitch_rad = gRobot.control.SP_Pitch_rad - gRobot.body.pitch_rad;
        
        // --- 4. BOOSTER / "The Hammer" ---
        float alpha_boost = 0.975f;

        if ( fabsf(gRobot.control.Err_Pitch_rad) < ANGLE_P_ZONE_RAD ) {
            // converge to default proportional gain
            gRobot.control.Booster_P_gain  = (alpha_boost * gRobot.control.Booster_P_gain) + ((1.0f-alpha_boost) * (1.00f));
        } else {
            // converge to maximum proportional gain
            gRobot.control.Booster_P_gain  = (alpha_boost * gRobot.control.Booster_P_gain) + ((1.0f-alpha_boost) * (1.50f));
        }

        // --- 5. PROPORTIONAL / "The Spring" ---
        gRobot.control.KPterm_V  = gRobot.gain.KP * gRobot.control.Booster_P_gain * gRobot.control.Err_Pitch_rad;

        // --- 6. DERIVATIVE / "The Damping" with Frequency-based Filter (N in Hz) ---
        // dt = 800 us = 0.0008 seconds
        // robot.gain.N = Cutoff frequency in Hz (e.g., set N to 10.0 - 12.0 Hz)

        const float dt = 0.0008f;
        const float PI = 3.14159265359f;

        // 1. Calculate the low-pass filter coefficient alpha from frequency N (Hz)
        float two_pi_N_dt = 2.0f * PI * gRobot.gain.N * dt;
        float N_alpha = two_pi_N_dt / (1.0f + two_pi_N_dt);

        // 2. Compute the raw derivative counter-force
        float raw_KDterm = -(gRobot.gain.KD * gRobot.body.rateY_rads);

        // 3. Apply the low-pass filter to smooth out the 16-20 Hz oscillations
        // (robot.control.KDterm_V acts as the state memory for the filter)
        gRobot.control.KDterm_V = gRobot.control.KDterm_V + N_alpha * (raw_KDterm - gRobot.control.KDterm_V);

        // --- 6. DERIVATIVE / "The Damping " or "The Counter-Force" ---
        // If Rate is (+), Result is (-). If Rate is (-), Result is (+).
        // robot.control.KDterm_V  = -(robot.gain.KD * robot.body.rateY_rads);
        
        // --- 7. Combine ALL Motor terms "The Summing Junction" ---
        gRobot.control.CMterm_V = gRobot.control.FFterm_V + 
                                 gRobot.control.KPterm_V + 
                                 gRobot.control.KDterm_V;
        
        // --- 8. Motor Voltage Limit ---
        if ( gRobot.control.CMterm_V >  VOLT_MOTOR_LIM_HI ) gRobot.control.CMterm_V =  VOLT_MOTOR_LIM_HI;
        if ( gRobot.control.CMterm_V < -VOLT_MOTOR_LIM_HI ) gRobot.control.CMterm_V = -VOLT_MOTOR_LIM_HI;
        
        // --- 9. Calculate TURN term ---
        gRobot.control.TIterm_V  = ( gRobot.gain.TI    * gRobot.rc_cmd.roll );
                                //   ( robot.gain.LH_RH * robot.control.Err_lh_rh_m );

        // --- 10. Motor Voltage Limit ---
        if ( gRobot.control.TIterm_V > VOLT_MOTOR_LIM_LO )  gRobot.control.TIterm_V = VOLT_MOTOR_LIM_LO;
        if ( gRobot.control.TIterm_V < -VOLT_MOTOR_LIM_LO ) gRobot.control.TIterm_V = -VOLT_MOTOR_LIM_LO;
        
        // --- 11. Motor mixing ---
        gRobot.motor.left.volt_V  = gRobot.control.CMterm_V + gRobot.control.TIterm_V;
        gRobot.motor.right.volt_V = gRobot.control.CMterm_V - gRobot.control.TIterm_V;
        
        // --- 12. APPLY / "The Make It Happen" ---
        motor_set_voltage( gRobot.motor.left.volt_V , gRobot.motor.right.volt_V );
    }
}

// ------------------------
// ----- PD OUTER LOOP ----
// ------------------------
void pd_outer_loop( void ) {
    if ( gFlg_run_outer_loop )
    {
GET_DURATION(8);    
gDuration_timer_us.outer_loop = gDuration_timer_us.delta_time[8];
// gDuration_timer_us.outer_loop = gDelta_time[8];
STORE_TIMER6(8);
        gFlg_run_outer_loop = false;

        // --- 1. Calculate Motor: Angle [rad], Rate [rad/s] and Wheel: Position [m] and Speed [m/s] ---
        robot_get_wheel_physics(&gRobot.motor.left  , &gRobot.wheel.left);
        robot_get_wheel_physics(&gRobot.motor.right , &gRobot.wheel.right);
        // --- 2. Calculate ROBOT BODY Position ---
        gRobot.body.pos_m = (gRobot.wheel.left.pos_m + gRobot.wheel.right.pos_m) * 0.5f;
        // --- 3. Calculate ROBOT BODY Speed --- 
        gRobot.body.speed_ms = robot_get_speed_sg();      // Savitzky-Golay Derivative Filter
        // --- 4. Calculate ROBOT TURN Angle ---
        gRobot.body.turn_rad = (gRobot.wheel.right.pos_m - gRobot.wheel.left.pos_m) / ROBOT_WHEEL_SOD_M;
        // --- 5. Calculate ROBOT TURN Angle rate ---
        gRobot.body.turn_rate_rads = robot_get_turn_rate_sg();    // Savitzky-Golay Derivative Filter
        
        // --- 6. Position Controller ---
        if (!gRobot.rc_cmd.is_connected) // run only when no RC communication is active
        {
            gRobot.control.Err_Pos_m    = gRobot.control.SP_Pos_m     - gRobot.body.pos_m;
            gRobot.control.Err_lh_rh_m  = gRobot.wheel.left.pos_odo_m - gRobot.wheel.right.pos_odo_m;
            gRobot.control.Err_Speed_ms = gRobot.control.SP_Speed_ms  - gRobot.body.speed_ms;
            // Angle as Action Value shall be POSITIVE (Nose Up) if Robot Position is positive (+) or Speed is positive (+)
            gRobot.control.Correction_Pos_rad =  -( ( gRobot.control.Err_Pos_m    * gRobot.gain.KP_Pos ) +
                                                   ( gRobot.control.Err_Speed_ms * gRobot.gain.KD_Pos ) );

            if ( gRobot.control.Correction_Pos_rad >  ANGLE_SP_PITCH_RAD ) gRobot.control.Correction_Pos_rad =  ANGLE_SP_PITCH_RAD;
            if ( gRobot.control.Correction_Pos_rad < -ANGLE_SP_PITCH_RAD ) gRobot.control.Correction_Pos_rad = -ANGLE_SP_PITCH_RAD;

        } else {
            gRobot.control.Err_Pos_m    = 0.0f;
            gRobot.control.Err_Speed_ms = 0.0f;
            gRobot.control.Correction_Pos_rad = 0.0f;
        }

        // send data over UART - NOW the same rate as Outer Loop
        gFlg_UART0_TXD = true;
    }
}


// *********************************************
// *****   BUILD AND SEND DATA OVER UART   *****
// *********************************************
uint16_t uart0_build_csv_frame( uint8_t arg_count )
{
    // ten thousands, thousands, hundreds, tens, ones - store variables for each decimal order
    int16_t     d10k = 0 , d1k = 0 , d100 = 0 , d10 = 0 , d1 = 0;
    uint16_t    i = 0U;         // number counter
    uint16_t    j = 0U;         // char in string counter
    
    // MAIN FOR-CYCLE for conversion integers into strings
    for ( i = 0 ; i < arg_count ; i++ )
    {
        // limiting -99 999 to +99 999
        if ( gTX0_signal[ i ] > 99999 )
            gTX0_signal[ i ] = 99999;
        if ( gTX0_signal[ i ] < -99999 )
            gTX0_signal[ i ] = -99999;
           
        // negative sign assigment
        if ( gTX0_signal[ i ] < 0  )
        {
            // negative number will be inverted to positive
            // although this modifys the original value - but after pushing to PC is not needed anyway
            gTX0_signal[ i ] = -1 * gTX0_signal[ i ];
            // write '-' symbol to signalize negative number
            gTX0_buffer[ j++ ] = '-';
        }
       
        // ten thousands, thousands, hundreds, tens, ones variables are cleared for cycle run
        d10k = 0; d1k = 0; d100 = 0; d10 = 0; d1 = 0;
       
        // calculate a cipher for each decimal order
        // ten thousands
        if ( gTX0_signal[ i ] >= 10000 )
            d10k = gTX0_signal[ i ] / 10000;
        // thousands
        if ( gTX0_signal[ i ] >= 1000 )    
            d1k = ( gTX0_signal[ i ] - d10k * 10000 ) / 1000;
        // hundreds
        if ( gTX0_signal[ i ] >= 100 )    
            d100 = ( gTX0_signal[ i ] - d10k * 10000 - d1k * 1000 ) / 100;
        // tens
        if ( gTX0_signal[ i ] >= 10 )        
            d10 = ( gTX0_signal[ i ] - d10k * 10000 - d1k * 1000 - d100 * 100 ) / 10;
        // ones
        d1 = ( gTX0_signal[ i ] - d10k * 10000 - d1k * 1000 - d100 * 100 - d10 * 10 ) % 10;
       
        // string creation, leading zeros are inhibited, base is '0'
        if ( d10k > 0 )
            gTX0_buffer[ j++ ] = '0' + d10k;
        if ( ( d10k > 0 ) || ( d1k > 0 ) )
            gTX0_buffer[ j++ ] = '0' + d1k;
        if ( ( d10k > 0 ) || ( d1k > 0 ) || ( d100 > 0 ) )
            gTX0_buffer[ j++ ] = '0' + d100;
        if ( ( d10k > 0 ) || ( d1k > 0 ) || ( d100 > 0 ) || ( d10 > 0 ) )
            gTX0_buffer[ j++ ] = '0' + d10;
        // ones will be always writte becase of 0
        gTX0_buffer[ j++ ] = '0' + d1;
        // comma separated data
        if ( i < arg_count - 1 )
            gTX0_buffer[ j++ ] = ',';
        else
            // last character as requested by Processing Grapher
            // https://wired.chillibasket.com/processing-grapher/
            gTX0_buffer[ j++ ] = '\n';
    }
   
    // store string length
    gTX0_size = j;
    // by enable TX interrupt data will be sent to PC
    DL_UART_Main_enableInterrupt( UART_0_INST, DL_UART_MAIN_INTERRUPT_TX );
   
    return j;   // return string length
}


void uart0_update( void )
{
    // ----- COMMAND RECEIVED CHECK -----
    if ( gRX0_flg_eol ) {
            
        gRX0_flg_eol = false;

        // checkForGainCommand();   
        // UART0_Update_Cmd();

        uart0_clear_buff();   
    }
    // ----- SEND DATA OVER UART -----
    if ( gFlg_UART0_TXD && gFlg_UART0_TX_En )
    {
STORE_TIMER6( 6 );
        gFlg_UART0_TXD = false;

        switch ( gRobot_state )
        {
            // ----- OBSERVATION ------  
            case ROBO_STATE_INIT:                  
                break;

            // ----- REGULATION ------    
            case ROBO_STATE_BALA:
                // signals for Kalman Filter state estimation
                gTX0_signal[ 0 ] = gTX0_sample_cnt++;                           // sample counter

                gTX0_signal[ 1 ] = 1000.0f * gRobot.wheel.left.pos_m;           // mm
                gTX0_signal[ 2 ] = 1000.0f * gRobot.wheel.left.speed_ms;        // mm/s
                gTX0_signal[ 3 ] = 1000.0f * gRobot.motor.left.curr_med5_A;     // mA

                gTX0_signal[ 4 ] = 1000.0f * gRobot.wheel.right.pos_m;          // mm
                gTX0_signal[ 5 ] = 1000.0f * gRobot.wheel.right.speed_ms;       // mm/s
                gTX0_signal[ 6 ] = 1000.0f * gRobot.motor.right.curr_med5_A;    // mA

                gTX0_signal[ 7 ] = 1000.0f * gRobot.body.accX_g;                // mg
                gTX0_signal[ 8 ] = 1000.0f * gRobot.body.accY_g;                // mg
                gTX0_signal[ 9 ] = 1000.0f * gRobot.body.accZ_g;                // mg

                gTX0_signal[ 10 ] = 1000.0f * gRobot.body.rateY_rads;           // mrad/s

                gTX0_signal[ 11 ] = 1000.0f * gRobot.control.CMterm_V;          // mV - control signal
                
                uart0_build_csv_frame( 12U ); 
    
                break;

            // ----- CALIBRATION ------
            case ROBO_STATE_CALIB:                  
                break;    

            // ----- IDENTIFICATION ------
            case ROBO_STATE_IDENT:
                gTX0_signal[ 0 ] = gTX0_sample_cnt++;
                gTX0_signal[ 1 ] = gRobot.control.CMterm_V;
                gTX0_signal[ 2 ] = gRobot.motor.left.imp;
                gTX0_signal[ 3 ] = gRobot.motor.right.imp;
                gTX0_signal[ 4 ] = 1000.0f * gRobot.motor.left.curr_ema_A;
                gTX0_signal[ 5 ] = 1000.0f * gRobot.motor.right.curr_ema_A;
                uart0_build_csv_frame ( 6U );
                break;

            // ----- ERROR ------
            case ROBO_STATE_NOK:
                break;
        } 
GET_DURATION( 6 );  // duration of the loop 31-32us
gDuration_timer_us.uart_comms = gDuration_timer_us.delta_time[6];
    }
}

void uart0_clear_buff( void ) {
    for ( int i = 0; i < RX0_BUFF_SIZE; ++i )
    {
        gRX0_buffer[i] = 0x00;
    }
    gRX0_idx = 0;
}
// bool UART0_Update_Gain( void )
// {
//     return false;
// }

// bool UART0_Update_Cmd( void )
// {
//     return false;
// }



/******************************
*****   MOTOR FUNCTIONS   *****
******************************/
void motor_set_voltage( float arg_volt_lh, float arg_volt_rh )
{
    // --------------------------
    // ---- 499. MOTOR CONTROL ---
    // --------------------------
    if ( gFlg_motor_enable && !(gRobot.power.is_low_power) )
    {
        // --- Positive Angle with Hysteresis ---
        if ( gRobot.body.pitch_rad > +ANGLE_LIM_BODY_RAD + ANGLE_LIM_HYST_RAD )
        {
            // Motors OFF - Robot falling
            motor_rh_set_voltage( 0.0f );
            motor_lh_set_voltage( 0.0f );
        } else
            // --- Negative Angle with Hysteresis ---
            if ( gRobot.body.pitch_rad < -ANGLE_LIM_BODY_RAD - ANGLE_LIM_HYST_RAD )
            {
                // Motors OFF - Robot falling
                motor_rh_set_voltage( 0.0f );
                motor_lh_set_voltage( 0.0f );
            } else
                // --- Angle in Range ---
                if ( ( gRobot.body.pitch_rad < +ANGLE_LIM_BODY_RAD ) &&
                        ( gRobot.body.pitch_rad > -ANGLE_LIM_BODY_RAD )
                    )            
                {
                    // --- Output ---
                    motor_rh_set_voltage( arg_volt_rh );
                    motor_lh_set_voltage( arg_volt_lh );
                    // Motor0_RH_SetVolt( robot.motor.right.volt_V );
                    // Motor1_LH_SetVolt( robot.motor.left.volt_V );
                }
    } else {
        motor_rh_set_voltage( 0.0f );
        motor_lh_set_voltage( 0.0f );
    }
}
// RH motor is motor0
void motor_rh_set_voltage( float arg_volt )
{
    static int8_t m0_state = 0;   // -1 = reverse, 0 = stop, +1 = forward
    uint16_t pwm_uint_value_M0 = 0U;

    // --- Battery OK? ---
    // if (robot.power.batt_volt > VOLT_BATTERY_MIN)
    if (!gRobot.power.is_low_power)
    {
        // --- Hysteresis decision ---
        // Forward RUN threshold
        if (arg_volt > (VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m0_state = +1;
        }
        // Reverse RUN threshold
        else if (arg_volt < -(VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m0_state = -1;
        }
        // STOP threshold (both directions)
        else if (arg_volt <  (VOLT_MOTOR_MIN - VOLT_MOTOR_HYST) &&
                 arg_volt > -(VOLT_MOTOR_MIN - VOLT_MOTOR_HYST))
        {
            m0_state = 0;
        }

        // --- Apply state ---
        if (m0_state == +1)
        {
            pwm_uint_value_M0 = (uint16_t)(100.0f * arg_volt / gRobot.power.batt_volt_V);
            motor_set_duty(PHASE_A, 0U);
            motor_set_duty(PHASE_B, pwm_uint_value_M0);
        }
        else if (m0_state == -1)
        {
            pwm_uint_value_M0 = (uint16_t)(-100.0f * arg_volt / gRobot.power.batt_volt_V);
            motor_set_duty(PHASE_A, pwm_uint_value_M0);
            motor_set_duty(PHASE_B, 0U);
        }
        else
        {
            motor_set_duty(PHASE_A, 0U);
            motor_set_duty(PHASE_B, 0U);
        }
    }
    else
    {
        // --- Battery LOW - STOP ---
        m0_state = 0;
        motor_set_duty(PHASE_A, 0U);
        motor_set_duty(PHASE_B, 0U);
    }
}

// LH motor is motor1
void motor_lh_set_voltage( float arg_volt )
{
    static int8_t m1_state = 0;   // -1 = reverse, 0 = stop, +1 = forward
    uint16_t pwm_uint_value_M1 = 0U;

    // --- Battery OK? ---
    if (gRobot.power.batt_volt_V > VOLT_BATTERY_MIN)
    {
        // --- Hysteresis decision ---
        // Forward RUN threshold
        if (arg_volt > (VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m1_state = +1;
        }
        // Reverse RUN threshold
        else if (arg_volt < -(VOLT_MOTOR_MIN + VOLT_MOTOR_HYST))
        {
            m1_state = -1;
        }
        // STOP threshold (both directions)
        else if (arg_volt <  (VOLT_MOTOR_MIN - VOLT_MOTOR_HYST) &&
                 arg_volt > -(VOLT_MOTOR_MIN - VOLT_MOTOR_HYST))
        {
            m1_state = 0;
        }

        // --- Apply state ---
        if (m1_state == +1)
        {
            pwm_uint_value_M1 = (uint16_t)(100.0f * arg_volt / gRobot.power.batt_volt_V);
            motor_set_duty(PHASE_C, pwm_uint_value_M1);
            motor_set_duty(PHASE_D, 0U);
        }
        else if (m1_state == -1)
        {
            pwm_uint_value_M1 = (uint16_t)(-100.0f * arg_volt / gRobot.power.batt_volt_V);
            motor_set_duty(PHASE_C, 0U);
            motor_set_duty(PHASE_D, pwm_uint_value_M1);
        }
        else
        {
            motor_set_duty(PHASE_C, 0U);
            motor_set_duty(PHASE_D, 0U);
        }
    }
    else
    {
        // --- Battery LOW - STOP ---
        m1_state = 0;
        motor_set_duty(PHASE_C, 0U);
        motor_set_duty(PHASE_D, 0U);
    }
}
/*****************************
*****   MOTOR PWM DUTY   *****
*****************************/
void motor_set_duty( uint8_t arg_phase , uint16_t arg_duty )
{
    // PWM limiter 2-90%
    if ( arg_duty > PWM_MAX_DUTY )
        arg_duty = PWM_MAX_DUTY;
    else if ( arg_duty < PWM_MIN_DUTY )
        arg_duty = 0;

    if ( arg_phase > (PHASE_COUNT-1U) )     // if greather than 3 (4 phases)
        arg_phase %= (PHASE_COUNT);
        // arg_phase %= (PHASE_COUNT-1U) ;

    switch ( arg_phase )
    {
        case 0:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-arg_duty , GPIO_PWM_0_C0_IDX );
            break;
        case 1:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-arg_duty , GPIO_PWM_0_C1_IDX );
            break;
        case 2:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-arg_duty , GPIO_PWM_0_C2_IDX );
            break;       
        case 3:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-arg_duty , GPIO_PWM_0_C3_IDX );
            break;    
    }
    
}

// ************************
// *****   MCU INIT   *****
// ************************
void mcu_initialize( void ) {
    SYSCFG_DL_init();
    // ***** TIMER 0 and 8 - enable interrupt *****
    NVIC_EnableIRQ( TIMER_0_INST_INT_IRQN) ;  // 1ms
    NVIC_EnableIRQ( TIMER_8_INST_INT_IRQN) ;  // 800us
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
    // ***** QUAD ABCD - enable interrupt *****
    NVIC_EnableIRQ( QUAD_M0_INT_IRQN );
    NVIC_EnableIRQ( QUAD_M1_INT_IRQN );
    // ***** DMA - TIMG7 - WS2812B RGB LED *****
    DL_DMA_setSrcAddr     ( DMA , DMA_CH0_CHAN_ID , (uint32_t) &gDMA_LED_array[0][0] );
    DL_DMA_setDestAddr    ( DMA , DMA_CH0_CHAN_ID , TIMG7_BASE + 0x1810 );   // CC_0
    DL_DMA_setTransferSize( DMA , DMA_CH0_CHAN_ID , WS2812B_BIT_COUNT );
    DL_DMA_enableChannel  ( DMA , DMA_CH0_CHAN_ID );
    // ***** default motor value - motor stop *****
    motor_set_duty( PHASE_A , 0U );       // MOTOR 0 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    motor_set_duty( PHASE_B , 0U );       // MOTOR 0 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
    motor_set_duty( PHASE_C , 0U );       // MOTOR 1 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    motor_set_duty( PHASE_D , 0U );       // MOTOR 1 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
}
// *****************************************  *****************************************  *****************************************
// *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****
// *****************************************  *****************************************  *****************************************


void mpu_initialize(void) {
    // 1. Initial Delay for MPU Power-up stabilization
    gMpu.status.delay_cnt_ms = TIME_MPU_DELAY_MS;
    while (gMpu.status.delay_cnt_ms > 0U);
    
    // Reset error mask and initialization status
    gMpu.status.error = 0;
    gMpu.status.is_initialized = false;

    // 2. Sequential Writes with Bitmask Error Tracking
    // Bits 0, 1, 2 track the success of the I2C Write transaction itself
    if (mpu_write_reg(0x37, 0x20) != 0) gMpu.status.error |= ( 1U << 0 ); // Latch En
    if (mpu_write_reg(0x38, 0x01) != 0) gMpu.status.error |= ( 1U << 1 ); // Data Rdy En
    if (mpu_write_reg(0x6B, 0x00) != 0) gMpu.status.error |= ( 1U << 2 ); // Exit Sleep

    // Second Delay to allow MPU internal clock PLL to stabilize after waking up
    gMpu.status.delay_cnt_ms = TIME_MPU_DELAY_MS;
    while (gMpu.status.delay_cnt_ms > 0U);
    
    // 3. Read-Back and Validation logic
    // We check if the data on the chip actually matches what we just wrote
    
    // Validate INT_PIN_CFG (0x37)
    if (mpu_read_regs(0x37, 1) != 0) {
        gMpu.status.error |= (1 << 3);
    } else {
        gMpu.status.irq_latchs = gI2C1_rx_buffer[0];
        if (gMpu.status.irq_latchs != 0x20) gMpu.status.error |= ( 1U << 3 ); 
    }

    // Validate INT_ENABLE (0x38)
    if (mpu_read_regs(0x38, 1) != 0) {
        gMpu.status.error |= (1 << 4);
    } else {
        gMpu.status.irq_enable = gI2C1_rx_buffer[0];
        if (gMpu.status.irq_enable != 0x01) gMpu.status.error |= ( 1U << 4 );
    }

    // Validate PWR_MGMT_1 (0x6B)
    if (mpu_read_regs(0x6B, 1) != 0) {
        gMpu.status.error |= (1 << 5);
    } else {
        gMpu.status.exit_sleep = gI2C1_rx_buffer[0];
        if (gMpu.status.exit_sleep != 0x00) gMpu.status.error |= ( 1U << 5 );
    }

    // 4. Identity Check (WHO_AM_I)
    if (mpu_read_regs(0x75, 1) != 0) {
        gMpu.status.error |= (1 << 6);
    } else {
        gMpu.status.who_am_i = gI2C1_rx_buffer[0];
        // MPU-6050 should always return 0x68
        if (gMpu.status.who_am_i != 0x68) gMpu.status.error |= ( 1U << 6 );
    }

    // 5. Final Failsafe Check
    // If there are no bits set in the error mask, initialization is successful
    if ( gMpu.status.error == 0)  {
        gMpu.status.is_initialized = true;
        // gLED_seq_now = LED_SEQ_DEF;
    } else {
        gMpu.status.is_initialized = false;
        gLED_seq_now = LED_SEQ_ERROR;
    }
}

// ***********************************
// *****   IMU/MPU CALIBRATION   *****
// ***********************************
void mpu_calibrate(void)
{
    const   uint16_t    loc_target_samples = 1250U;  // shall take about 1 sec (800us * 1250)
            uint16_t    loc_sample_count = 0U;
            int32_t     loc_accX_sum = 0;
            int32_t     loc_accY_sum = 0;
            int32_t     loc_accZ_sum = 0;
            int32_t     loc_gyroX_sum = 0;
            int32_t     loc_gyroY_sum = 0;
            int32_t     loc_gyroZ_sum = 0;

    const   uint8_t     loc_LED_count   = 8U;
    const   uint8_t     loc_LED_offset  = 8U; // Offset for Other side of
            uint8_t     loc_LED_active  = 0U;
            float       loc_LED_norm    = 0.0f;
    
    // Delay to allow Robot to stop shaking from powering up
    gMpu.status.delay_cnt_ms = TIME_MPU_DELAY_MS;
    while ( gMpu.status.delay_cnt_ms > 0U );

    gMpu.status.is_calibrated = false;

    // Collect Series of samples synchronized with the 800us inner loop tick
    while ( loc_sample_count < loc_target_samples ) {
        // ----- READ MPU -----        
        mpu_process_data();

        if ( gFlg_run_inner_loop ) {
            gFlg_run_inner_loop = false; // Clear flag for the next cycle

            // --- FEATURE: Progressbar ---
            // Calculate the level based on current sample count
            loc_LED_norm = (float)(loc_sample_count) / (float)loc_target_samples;
            if (loc_LED_norm < 0.0f) loc_LED_norm = 0.0f;
            if (loc_LED_norm > 1.0f) loc_LED_norm = 1.0f;
            loc_LED_active = (uint8_t)(loc_LED_norm * (float)loc_LED_count);

            // --- Draw the Progressbar - One side ---
            for (uint8_t i = 0; i < (loc_LED_count); i++) {
                if ( i <= (loc_LED_active) ) {
                    ws2812b_set_led_color( i, 0, 0, 0x10 ); // Blue
                } else {
                    ws2812b_set_led_color( i, 0, 0, 0 );    // Off
                }
            }

            // --- Draw the Progressbar - Other side ---
            for ( uint8_t i = loc_LED_offset; i < ( loc_LED_count + loc_LED_offset ); i++ ) {
                if ( i <= ( loc_LED_active + loc_LED_offset ) ) {
                    ws2812b_set_led_color( i, 0, 0, 0x10 ); // Blue
                } else {
                    ws2812b_set_led_color( i, 0, 0, 0 );    // Off
                }
            }

            // Accumulate raw values for acc and gyro
            loc_accX_sum  += gMpu.raw.accX_lsb;
            loc_accY_sum  += gMpu.raw.accY_lsb;
            // loc_accZ_sum  += gMpu.raw.accZ_lsb;
            
            loc_gyroX_sum += gMpu.raw.gyroX_lsb;
            loc_gyroY_sum += gMpu.raw.gyroY_lsb;
            // loc_gyroZ_sum += gMpu.raw.gyroZ_lsb;

            loc_sample_count++;
        }
    }

    // Calculate average offsets and store them directly in the offset structure
    gMpu.offset.accX_lsb  = (int16_t)( loc_accX_sum / loc_target_samples );
    gMpu.offset.accY_lsb  = (int16_t)( loc_accY_sum / loc_target_samples );
    gMpu.offset.accZ_lsb  = (int16_t)( loc_accZ_sum / loc_target_samples );
    gMpu.offset.gyroX_lsb = (int16_t)( loc_gyroX_sum / loc_target_samples );
    gMpu.offset.gyroY_lsb = (int16_t)( loc_gyroY_sum / loc_target_samples );
    gMpu.offset.gyroZ_lsb = (int16_t)( loc_gyroZ_sum / loc_target_samples );

    gMpu.status.is_calibrated = true;
}

// **********************************
// *****   MPU WRITE REGISTER   *****
// **********************************
// based on TI MSPM0 Academy example, updated 2026-11-13 23:02
// https://dev.ti.com/tirex/explore/node?node=A__Adk.xJzQkkC7nuidYK5bXg__MSPM0-ACADEMY__2f1Egw1__LATEST
uint16_t mpu_write_reg( uint8_t arg_reg , uint8_t arg_data )
{
    uint32_t loc_timeout        = 2000U;    // Cycle-based (tick-based) timeout
    uint16_t loc_time_stamp_us  = 0U;       // time stamp in us
    
    gI2C_anti_block_error = 0U;  // reset ANTIBLOCK error memory
    
    // Get-In GUARD
    // ANTIBLOCK LOOP #0 - wait for controller to be idle (ready for write), observation: 0 cycles, 0-1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        ( gI2C1_anti_block_cnt < loc_timeout ) ;
        gI2C1_anti_block_cnt++)
        ;
    // store the anti block counter to see if value has changed
    debug_i2c_write_reg_cnt[ 0 ] = gI2C1_anti_block_cnt;
    debug_i2c_write_reg_us[  0 ] = GET_TICK_US - loc_time_stamp_us;
    // if anti block counter reach loc_timeout - I2C halted here
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 0 );  // latch the error bit
    
    gI2C_controller_status = I2C_STATUS_IDLE;
    
    gI2C1_tx_size = 2U;
    gI2C1_tx_buffer[ 0 ] = arg_reg;
    gI2C1_tx_buffer[ 1 ] = arg_data;

    // CODE FROM MSPM0 ACADEMY
    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gI2C1_tx_idx = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gI2C1_tx_buffer[ 0 ] , gI2C1_tx_size );

    // CODE FROM MSPM0 ACADEMY
    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gI2C1_tx_idx < gI2C1_tx_size )
        DL_I2C_enableInterrupt ( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    else
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    
    // CODE FROM MSPM0 ACADEMY
    gI2C_controller_status = I2C_STATUS_TX_STARTED;
    
    // CODE FROM MSPM0 ACADEMY
    // SEND DATA TO SLAVE - WRITE (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gI2C1_tx_size );
    
    // CODE FROM MSPM0 ACADEMY - updated for Anti Block system
    // ANTIBLOCK LOOP #1 - wait until data has been sent, or error has been set, observation: count to 312 iterations, 76us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( gI2C_controller_status != I2C_STATUS_TX_COMPLETE ) &&
        ( gI2C_controller_status != I2C_STATUS_ERROR ) &&
        ( gI2C1_anti_block_cnt < loc_timeout ) ;
        gI2C1_anti_block_cnt++ ) 
	    ;
    // store the anti block counter to see if value has changed
    debug_i2c_write_reg_cnt[ 1 ] = gI2C1_anti_block_cnt;
    debug_i2c_write_reg_us[  1 ] = GET_TICK_US - loc_time_stamp_us;
    // if anti block counter reach loc_timeout - I2C halted here
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 1 );  // latch the error bit
    
    // CODE FROM MSPM0 ACADEMY - updated for Anti Block system
    // ANTIBLOCK LOOP #2 - wait to bus bussy flag to clear, observation: count to 7 iterations, 2us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) ) &&
        ( gI2C1_anti_block_cnt < loc_timeout ) ;
        gI2C1_anti_block_cnt++ )
	    ;
    // store the anti block counter to see if value has changed    
    debug_i2c_write_reg_cnt[ 2 ] = gI2C1_anti_block_cnt;
    debug_i2c_write_reg_us[  2 ] = GET_TICK_US - loc_time_stamp_us;
    // if anti block counter reach loc_timeout - I2C halted here
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 2 );  // latch the error bit
    
    // Get-Out GUARD - leaves I2C in a known state and idle
    // ANTIBLOCK LOOP #3 - wait for controller to be idle, observation: count to 3-4 iterations, 1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        ( gI2C1_anti_block_cnt < loc_timeout ); 
        gI2C1_anti_block_cnt++ )
	    ;
    // store the anti block counter to see if value has changed
    debug_i2c_write_reg_cnt[ 3 ] = gI2C1_anti_block_cnt;    
    debug_i2c_write_reg_us[  3 ] = GET_TICK_US - loc_time_stamp_us;
    // if anti block counter reach zero - I2C halted here
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 3 );  // latch the error bit
    
    return gI2C_anti_block_error;
}

// ***********************************
// ***** MPU BURST REGISTER READ *****
// ***********************************
uint16_t mpu_read_regs( uint8_t arg_reg , uint8_t arg_size )
{
    uint32_t loc_timeout        = 2000U;    // Cycle-based (tick-based) timeout
    uint16_t loc_time_stamp_us  = 0U;       // time stamp in us
    
    gI2C_anti_block_error = 0U;     // reset anti block error counter

    // Get-In GUARD
    // ANTIBLOCK LOOP #0 - , observation: count to 0 iterations, 0-1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 0 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  0 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 0 );    // latch the error bit
  
    gI2C_controller_status = I2C_STATUS_IDLE;
    gI2C1_tx_size = 1;
    gI2C1_tx_buffer[ 0 ] = arg_reg;

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gI2C1_tx_idx = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gI2C1_tx_buffer[ 0 ] , gI2C1_tx_size );
 
    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gI2C1_tx_idx < gI2C1_tx_size )
        DL_I2C_enableInterrupt ( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    else
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );

    gI2C_controller_status = I2C_STATUS_TX_STARTED;
 
    // SEND DATA TO SLAVE - WRITE (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gI2C1_tx_size );

    // ANTIBLOCK LOOP #1 - observation:count to 208-213 iterations, 52-53us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        // ( gI2C_controller_status == I2C_STATUS_TX_COMPLETE ) &&
        (gI2C_controller_status != I2C_STATUS_TX_COMPLETE) &&
        (gI2C_controller_status != I2C_STATUS_ERROR) &&
        ( gI2C1_anti_block_cnt < loc_timeout ) ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 1 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  1 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 1 );    // latch the error bit

    // CODE FROM MSPM0 ACADEMY - updated for Anti Block system
    // ANTIBLOCK LOOP #2 - observation: count to 7-8 iterations, 1-2us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 2 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  2 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 2 );    // latch the error bit
    
    // ANTIBLOCK LOOP #3 - observation: count to 3 iterations, 1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 3 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  3 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 3 );    // latch the error bit

    // ANTIBLOCK LOOP #4 - observation: count to 0 iterations, 0-1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 4 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  4 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 4 );    // latch the error bit
    
    // ANTIBLOCK LOOP #5 - observation: count to 0 iterations, 0-1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 5 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  5 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 5 );    // latch the error bit

    //----- REPEAT START -----

    // Send a read request to Target
    gI2C1_rx_size          = arg_size;
    gI2C1_rx_idx           = 0U;
    gI2C_controller_status = I2C_STATUS_RX_STARTED;

    // SEND DATA TO SLAVE - REQUEST TO READ (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gI2C1_rx_size );
    
    // ANTIBLOCK LOOP #6 - observation: count to 0 iterations, 0-1us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ; 
        ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 6 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  6 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 6 );    // latch the error bit

    // ANTIBLOCK LOOP #7 - observation: count to 1455-1468 iterations, 385-386us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        // ( gI2C_controller_status==I2C_STATUS_RX_COMPLETE ) &&
        (gI2C_controller_status != I2C_STATUS_RX_COMPLETE) &&
        (gI2C_controller_status != I2C_STATUS_ERROR) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 7 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  7 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 7 );    // latch the error bit

    // Get-Out GUARD - leaves I2C in a known state and idle
    // ANTIBLOCK LOOP #8 - observation: count to 13-14 iterations, 3-4us
    loc_time_stamp_us = GET_TICK_US;
    for ( gI2C1_anti_block_cnt = 0U ;
        ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) &&
        gI2C1_anti_block_cnt < loc_timeout ;
        gI2C1_anti_block_cnt++ )
	    ;
    debug_i2c_read_regs_cnt[ 8 ] = gI2C1_anti_block_cnt;
    debug_i2c_read_regs_us[  8 ] = GET_TICK_US - loc_time_stamp_us;
    if ( gI2C1_anti_block_cnt == loc_timeout ) gI2C_anti_block_error |= ( 1U << 8 );    // latch the error bit

    return gI2C_anti_block_error;
}

void mpu_update_physics( volatile mpu_t *arg_mpu , uint8_t *arg_buffer )
{
    // 1. Snapshot: Local copy for register-based math
    mpu_t loc_temp = *arg_mpu;

STORE_TIMER6( 2 );
    // 2. Combine Big-Endian bytes into Raw int16_t
    loc_temp.raw.accX_lsb  = (int16_t)((arg_buffer[1]  << 8) | arg_buffer[2]);
    loc_temp.raw.accY_lsb  = (int16_t)((arg_buffer[3]  << 8) | arg_buffer[4]);
    loc_temp.raw.accZ_lsb  = (int16_t)((arg_buffer[5]  << 8) | arg_buffer[6]);
    loc_temp.raw.temp_lsb  = (int16_t)((arg_buffer[7]  << 8) | arg_buffer[8]);
    loc_temp.raw.gyroX_lsb = (int16_t)((arg_buffer[9]  << 8) | arg_buffer[10]);
    loc_temp.raw.gyroY_lsb = (int16_t)((arg_buffer[11] << 8) | arg_buffer[12]);
    loc_temp.raw.gyroZ_lsb = (int16_t)((arg_buffer[13] << 8) | arg_buffer[14]);
    // 3. Apply Calibration Offsets
    loc_temp.val.accX_lsb  = loc_temp.raw.accX_lsb  - loc_temp.offset.accX_lsb;
    loc_temp.val.accY_lsb  = loc_temp.raw.accY_lsb  - loc_temp.offset.accY_lsb;
    loc_temp.val.accZ_lsb  = loc_temp.raw.accZ_lsb  - loc_temp.offset.accZ_lsb;
    loc_temp.val.gyroX_lsb = loc_temp.raw.gyroX_lsb - loc_temp.offset.gyroX_lsb;
    loc_temp.val.gyroY_lsb = loc_temp.raw.gyroY_lsb - loc_temp.offset.gyroY_lsb;
    loc_temp.val.gyroZ_lsb = loc_temp.raw.gyroZ_lsb - loc_temp.offset.gyroZ_lsb;
GET_DURATION( 2 );  // duration 1-2us

STORE_TIMER6( 3 );
    // 4. Convert to Physical Units but in Fixed Point IQ format
    // *** Acceleration [g] and temperature [C] ***
    loc_temp.iq.iqAccX_g = _IQdiv(_IQ(loc_temp.val.accX_lsb), _IQ(16384.0f));
    loc_temp.iq.iqAccY_g = _IQdiv(_IQ(loc_temp.val.accY_lsb), _IQ(16384.0f));
    loc_temp.iq.iqAccZ_g = _IQdiv(_IQ(loc_temp.val.accZ_lsb), _IQ(16384.0f));
    loc_temp.iq.iqTemp_C = _IQdiv(_IQ(loc_temp.raw.temp_lsb), _IQ(340.0f)) + _IQ(36.53f);
    // convert to float and comit to main Robot struct
    gRobot.body.accX_g = _IQtoF( loc_temp.iq.iqAccX_g );
    gRobot.body.accY_g = _IQtoF( loc_temp.iq.iqAccY_g );
    gRobot.body.accZ_g = _IQtoF( loc_temp.iq.iqAccZ_g );
    gRobot.body.temp_C = _IQtoF( loc_temp.iq.iqTemp_C );

    // *** Gyroscope [rad/s] ***
    loc_temp.iq.iqGyroX_rads = _IQdiv(_IQdiv(_IQ(loc_temp.val.gyroX_lsb), _IQ(131.0f)), _IQ(57.2958f));
    loc_temp.iq.iqGyroY_rads = _IQdiv(_IQdiv(_IQ(loc_temp.val.gyroY_lsb), _IQ(-131.0f)), _IQ(57.2958f));
    loc_temp.iq.iqGyroZ_rads = _IQdiv(_IQdiv(_IQ(loc_temp.val.gyroZ_lsb), _IQ(131.0f)), _IQ(57.2958f));
    // convert to float and comit to main Robot struct
    gRobot.body.rateX_rads = _IQtoF( loc_temp.iq.iqGyroX_rads );
    gRobot.body.rateY_rads = _IQtoF( loc_temp.iq.iqGyroY_rads );
    gRobot.body.rateZ_rads = _IQtoF( loc_temp.iq.iqGyroZ_rads );
GET_DURATION( 3 );  // duration 18-19us
gDuration_timer_us.uav_physics = gDuration_timer_us.delta_time[2]+gDuration_timer_us.delta_time[3];
// gDuration_timer_us.uav_physics = gDelta_time[2]+gDelta_time[3];
    // 6. Commit: Write back to RAM
    *arg_mpu = loc_temp;
}

bool mpu_uav_calculation ( volatile mpu_t *arg_mpu )
{
    // 1. Snapshot: Local copy for register-based math
    mpu_t temp = *arg_mpu;

STORE_TIMER6( 4 );
    // ********** TRIGINOMETRY **********
    // https://www.analog.com/en/resources/app-notes/an-1057.html
    _iq iq_accX_sq = _IQmpy(temp.iq.iqAccX_g, temp.iq.iqAccX_g);
    _iq iq_accY_sq = _IQmpy(temp.iq.iqAccY_g, temp.iq.iqAccY_g);
    _iq iq_accZ_sq = _IQmpy(temp.iq.iqAccZ_g, temp.iq.iqAccZ_g);

    temp.iq.iqPitch_rad = _IQatan2( temp.iq.iqAccX_g , _IQsqrt( iq_accY_sq + iq_accZ_sq ) );
    temp.iq.iqRoll_rad  = _IQatan2( temp.iq.iqAccY_g , _IQsqrt( iq_accX_sq + iq_accZ_sq ) );
    // Heading is integral of Yaw
    temp.iq.iqYaw_rad  += _IQmpy  ( temp.iq.iqGyroZ_rads , SAMPLE_TIME_iq );
GET_DURATION( 4 );  // duration 11-12us
gDuration_timer_us.trigonometry = gDuration_timer_us.delta_time[4];
// gDuration_timer_us.trigonometry = gDelta_time[4];

STORE_TIMER6( 5 );	
    // ********** COMPLEMENTARY FILTER **********
    // Trust the Gyro 98%, Trust the Accel 2%
    const _iq iq_alpha = _IQ(0.98f);
    const _iq iq_one_minus_alpha = _IQ(0.02f);

    // 1. Calculate the 'Short Term' change using Gyro (Rate * Time)
    _iq iq_delta_X_angle = _IQmpy(temp.iq.iqGyroX_rads, SAMPLE_TIME_iq);
    _iq iq_delta_Y_angle = _IQmpy(temp.iq.iqGyroY_rads, SAMPLE_TIME_iq);
    
    // 2. Combine with 'Long Term' stability from Accel
    // Formula: Angle = Alpha * (Angle + Gyro_Delta) + (1 - Alpha) * Accel_Angle
    temp.iq.iqPitch_fil_rad = _IQmpy(iq_alpha, temp.iq.iqPitch_N_1 + iq_delta_Y_angle) + _IQmpy(iq_one_minus_alpha, temp.iq.iqPitch_rad);
    temp.iq.iqRoll_fil_rad  = _IQmpy(iq_alpha, temp.iq.iqRoll_N_1  + iq_delta_X_angle) + _IQmpy(iq_one_minus_alpha, temp.iq.iqRoll_rad);
    
    // 3. Update the 'Long Term' angles
    temp.iq.iqPitch_N_1 = temp.iq.iqPitch_fil_rad;
    temp.iq.iqRoll_N_1  = temp.iq.iqRoll_fil_rad;
    
    // 4. Convert to float
    gRobot.body.pitch_rad = _IQtoF( temp.iq.iqPitch_fil_rad );
    gRobot.body.roll_rad  = _IQtoF( temp.iq.iqRoll_fil_rad  );
GET_DURATION( 5 );  // duration 6-7us
gDuration_timer_us.comp_filter = gDuration_timer_us.delta_time[5];
// gDuration_timer_us.comp_filter = gDelta_time[5];
    
    // 6. Commit: Write back to RAM
    *arg_mpu = temp;
    
    // NOW RUN PID
    return true;
}

void mpu_process_data(void)
{
    if (!gMpu.status.is_initialized) return;

    // if (gFlg_Read_MPU && (gCnt_MPU_Delay == 0))
    if ( (gMpu.status.read_flg) && (gMpu.status.delay_cnt_ms == 0U) )
    {
        // gFlg_Read_MPU = false; 
        gMpu.status.read_flg = false;

STORE_TIMER6(1);

        // Read 15 bytes starting at 0x3A (INT_STATUS)
        if (mpu_read_regs(0x3A, 15) != 0) 
        {      
            gMpu.status.error |= (1 << 7);
            gMpu.status.irq_status = 0U;
        } 
        else 
        {
            gMpu.status.irq_status = gI2C1_rx_buffer[0];

            if ((gMpu.status.irq_status & 0x01) == 0x01) 
            {
                gMpu.diag.irq_set_cnt++;    
                // NEW: Update Physics (Raw -> Compensated) before calculation
                mpu_update_physics( &gMpu , (uint8_t *)gI2C1_rx_buffer);
                // Now Calculation uses the freshly updated mpu.val data
                if (mpu_uav_calculation( &gMpu ) == true) 
                {
                    gFlg_run_inner_loop = true;
                    gFlg_motor_enable      = true;
                } 
                else 
                {
                    gFlg_run_inner_loop = false;
                    gFlg_motor_enable      = false;
                }
            } 
            else { gMpu.diag.irq_not_set_cnt++; }
        }
GET_DURATION(1);
gDuration_timer_us.mpu_read = gDuration_timer_us.delta_time[1];
// gDuration_timer_us.mpu_read = gDelta_time[1];
    }
}

void robot_ident( void )
{
    if ( gRobot_state == ROBO_STATE_IDENT )
    {
        switch ( gRobot_ident_state ) {
        
            // --- MOTOR OFF (0.0V) ---
            case 0U:
                gRobot.control.CMterm_V = 0.0f;
                motor_rh_set_voltage( gRobot.control.CMterm_V );
                motor_lh_set_voltage( gRobot.control.CMterm_V );
                // clear timer counter and go to next state
                gCnt_wait_ms = 0U;
                gRobot_ident_state++;
            break;

            // --- WAIT 0.5s ---
            case 1U:
            if ( gCnt_wait_ms >= 500U )
                gRobot_ident_state++;
            break;

            // --- SET LOW LEVEL ---
            case 2U:
                // robot.control.CMterm_V = 2.0f;
                gRobot.control.CMterm_V = 3.0f;
                motor_rh_set_voltage( gRobot.control.CMterm_V );
                motor_lh_set_voltage( gRobot.control.CMterm_V );                    
                // clear timer counter and go to next state
                gCnt_wait_ms = 0U;
                gRobot_ident_state++;
            break;

            // --- WAIT 0.5s ---
            case 3U:
            if ( gCnt_wait_ms >= 500U )
                gRobot_ident_state++;
            break;

            // --- Start logging ---
            case 4U:
                gTX0_sample_cnt = 0U;
                gFlg_UART0_TX_En = true;
                // clear timer counter and go to next state
                gCnt_wait_ms = 0U;
                gRobot_ident_state++;
            break;

            // --- WAIT 0.5s ---
            case 5U:
            if ( gCnt_wait_ms >= 500U )
                gRobot_ident_state++;
            break;

            // --- SET HI LEVEL ---
            case 6U:
                // robot.control.CMterm_V = 4.0f;
                gRobot.control.CMterm_V = 6.0f;
                motor_rh_set_voltage( gRobot.control.CMterm_V );
                motor_lh_set_voltage( gRobot.control.CMterm_V );
                // clear timer counter and go to next state
                gCnt_wait_ms = 0U;
                gRobot_ident_state++;
            break;
            
            // --- WAIT 0.5s ---
            case 7U:
            if ( gCnt_wait_ms >= 500U )
                gRobot_ident_state++;
            break;

            // --- SET LOW LEVEL ---
            case 8U:
                // robot.control.CMterm_V = 2.0f;
                gRobot.control.CMterm_V = 3.0f;
                motor_rh_set_voltage( gRobot.control.CMterm_V );
                motor_lh_set_voltage( gRobot.control.CMterm_V );
                // clear timer counter and go to next state
                gCnt_wait_ms = 0U;
                gRobot_ident_state++;
            break;

            // --- WAIT 0.5s ---
            case 9U:
                if ( gCnt_wait_ms >= 500U )
                    gRobot_ident_state++;
            break;

            // --- MOTOR OFF (0.0V) ---
            case 10U:
                // stop logging
                gFlg_UART0_TX_En = false;
                // motors OFF
                gRobot.control.CMterm_V = 0.0f;
                motor_rh_set_voltage( gRobot.control.CMterm_V );
                motor_lh_set_voltage( gRobot.control.CMterm_V );
            // --- END ---
                // reinitialization
                gRobot_state = ROBO_STATE_BALA;
                gRobot_ident_state = 0U;
            break;                
        }                    
    }
}

// Snapshot-Process-Commit
void robot_get_wheel_physics(volatile motor_t *arg_motor, volatile wheel_t *arg_wheel)
{
    // 1. Snapshot: Move volatile RAM to CPU Registers/Stack
    motor_t M_temp = *arg_motor; 
    wheel_t W_temp = *arg_wheel; 

    // 2. Kinematics: Update Current Positions
    M_temp.theta_rad = (float)M_temp.imp * MOTOR_RAD_PER_TICK;
    W_temp.pos_m     = M_temp.theta_rad  * ROBOT_WHEEL_RAD_M;
    // 3. Odometry: Update Absolute Position
    if ( W_temp.pos_m > W_temp.pos_hist_m[4] )
        W_temp.pos_odo_m += W_temp.pos_m - W_temp.pos_hist_m[4];
    else
        W_temp.pos_odo_m += W_temp.pos_hist_m[4] - W_temp.pos_m;

    // 4. History Shift: Update both Linear and Angular buffers
    for( int i = 0; i < 4; i++ ) {
        W_temp.pos_hist_m[i]     = W_temp.pos_hist_m[i+1];
        M_temp.theta_hist_rad[i] = M_temp.theta_hist_rad[i+1];
    }
    W_temp.pos_hist_m[4]     = W_temp.pos_m;
    M_temp.theta_hist_rad[4] = M_temp.theta_rad;

    // 5. Pure Derivative: Linear Speed (m/s)
    W_temp.speed_ms = ( W_temp.pos_m - W_temp.pos_hist_m[3] ) / TIME_MOTOR_OMEGA_S;
    
    // 6. SG Derivative: Linear Speed (m/s)
    // Coeffs: [1, -8, 0, 8, -1] / (12 * dt)
    W_temp.speed_sg_ms = SG_K * ( 1.0f * W_temp.pos_hist_m[0] 
                                - 8.0f * W_temp.pos_hist_m[1] 
                                + 8.0f * W_temp.pos_hist_m[3] 
                                - 1.0f * W_temp.pos_hist_m[4] );

    // 7. SG 2nd Derivative: Linear Acceleration (m/s^2) [NEW]
    // Coeffs: [2, -1, -2, -1, 2] / (7 * dt^2)
    W_temp.accel_sg_ms2 = SG_K2 * ( + 2.0f * W_temp.pos_hist_m[0] 
                                    - 1.0f * W_temp.pos_hist_m[1] 
                                    - 2.0f * W_temp.pos_hist_m[2] 
                                    - 1.0f * W_temp.pos_hist_m[3] 
                                    + 2.0f * W_temp.pos_hist_m[4] );

    // 8. SG Derivative: Motor Angular Velocity (rad/s)
    // Using the same SG kernel but on the theta history
    M_temp.omega_sg_rads = SG_K * ( 1.0f * M_temp.theta_hist_rad[0] 
                               - 8.0f * M_temp.theta_hist_rad[1] 
                               + 8.0f * M_temp.theta_hist_rad[3] 
                               - 1.0f * M_temp.theta_hist_rad[4] );

    // 9. SG 2nd Derivative: Motor Angular Acceleration (rad/s^2) [NEW]
    M_temp.alpha_sg_rads2 = SG_K2 * (  2.0f * M_temp.theta_hist_rad[0] 
                                  - 1.0f * M_temp.theta_hist_rad[1] 
                                  - 2.0f * M_temp.theta_hist_rad[2] 
                                  - 1.0f * M_temp.theta_hist_rad[3] 
                                  + 2.0f * M_temp.theta_hist_rad[4] );

    // 10. Commit: Atomic-style write back to RAM
    *arg_motor = M_temp;
    *arg_wheel = W_temp;
}

// Robot Get Speed calculates robot speed using derivative SG filter
float   robot_get_speed_sg(void)
{
    // Shift history: [0]=k-2, [1]=k-1, [2]=k, [3]=k+1, [4]=k+2
    gRobot.body.pos_hist_m[0] = gRobot.body.pos_hist_m[1];
    gRobot.body.pos_hist_m[1] = gRobot.body.pos_hist_m[2];
    gRobot.body.pos_hist_m[2] = gRobot.body.pos_hist_m[3];
    gRobot.body.pos_hist_m[3] = gRobot.body.pos_hist_m[4];
    gRobot.body.pos_hist_m[4] = gRobot.body.pos_m;   // newest sample

    // Savitzky-Golay Filter 5-point derivative kernel
    float v = SG_K * ( gRobot.body.pos_hist_m[0]
                     - 8.0f * gRobot.body.pos_hist_m[1]
                     + 8.0f * gRobot.body.pos_hist_m[3]
                     - gRobot.body.pos_hist_m[4] );

    return v;   // [m/s]
}

float   robot_get_turn_rate_sg(void)
{
    gRobot.body.turn_hist_rad[0] = gRobot.body.turn_hist_rad[1];
    gRobot.body.turn_hist_rad[1] = gRobot.body.turn_hist_rad[2];
    gRobot.body.turn_hist_rad[2] = gRobot.body.turn_hist_rad[3];
    gRobot.body.turn_hist_rad[3] = gRobot.body.turn_hist_rad[4];

    gRobot.body.turn_hist_rad[4] = gRobot.body.turn_rad;

    return (1.0f / (12.0f * TIME_MOTOR_OMEGA_S)) *
           ( gRobot.body.turn_hist_rad[0]
           - 8.0f * gRobot.body.turn_hist_rad[1]
           + 8.0f * gRobot.body.turn_hist_rad[3]
           - gRobot.body.turn_hist_rad[4] );
}

// Snapshot-Process-Commit
void    robot_get_motor_current( volatile motor_t *arg_motor, uint32_t arg_raw_adc_slb )
// Motor_current[A] = V_sns[V] / ( R_sns[R] * INA_GAIN[-] ) => Motor_cuureent = V_sns / 0.033R * 20
{
    float delta = 0.0f;
    // 1. Snapshot
    motor_t temp = *arg_motor;

    // 2. Conversion: LSB to Amps (V_ref/4096 / (R_shunt * Gain))
    // Current [A] = (V_adc) / (0.033 ohms * 20 gain)
    float currentSample = ((float)arg_raw_adc_slb * VOLT_ADC_REF / 4096.0f) / (0.033f * 20.0f);

    // 3. Shift Median History
    // temp.curr_hist_A[0] = temp.curr_hist_A[1];
    // temp.curr_hist_A[1] = temp.curr_hist_A[2];
    // temp.curr_hist_A[2] = currentSample;

    // 3. Shift 5-sample history
    temp.curr_hist_A[0] = temp.curr_hist_A[1];
    temp.curr_hist_A[1] = temp.curr_hist_A[2];
    temp.curr_hist_A[2] = temp.curr_hist_A[3];
    temp.curr_hist_A[3] = temp.curr_hist_A[4];
    temp.curr_hist_A[4] = currentSample;

    // 4. Calculate Median
    // float medCurrent = Filter_Median3_f(temp.curr_hist_A[0], 
    //                                    temp.curr_hist_A[1], 
    //                                    temp.curr_hist_A[2]);
    // temp.curr_med3_A = medCurrent;

    // 4. Calculate Median5 (Inlined, completely ignores up to 2 corrupt samples)
    float medCurrent = filter_median_5_f(temp.curr_hist_A[0], 
                                        temp.curr_hist_A[1], 
                                        temp.curr_hist_A[2],
                                        temp.curr_hist_A[3],
                                        temp.curr_hist_A[4]);
    
    // 41. Adding Delta step to increase or decrease current in case of spikes
    delta = medCurrent - temp.curr_med5_A;

    if (delta > MAX_CURRENT_STEP_A)         temp.curr_med5_A += MAX_CURRENT_STEP_A;
    else if (delta < -MAX_CURRENT_STEP_A)   temp.curr_med5_A -= MAX_CURRENT_STEP_A;
    else                                    temp.curr_med5_A = medCurrent;

    // 5. Exponential Moving Average (EMA)
    // We apply EMA on top of the Median to smooth out the remaining ripple
    temp.curr_ema_A =   ( 0.01f * medCurrent ) + 
                        ( 0.99f * temp.curr_ema_A );
    // temp.curr_ema_A = (gCURR_EMA_Coef * medCurrent) + 
    //               ((1.0f - gCURR_EMA_Coef) * temp.curr_ema_A);

    // 6. Commit
    *arg_motor = temp;
}

// Snapshot-Process-Commit
void    robot_get_battery_voltage( volatile power_t *arg_power , uint32_t arg_raw_adc_lsb )
{
    // 1. Snapshot: Copy volatile structure to stack
    power_t temp = *arg_power;

    // 2. Conversion: LSB to Real Volts
    // (Raw -> Voltage at Pin) * Divider + Diode Drop
    float pinVoltage = (float)arg_raw_adc_lsb * (VOLT_ADC_REF / 4096.0f);
    float instBattVolt = (pinVoltage * VOLT_DIVIDER_RATIO) + VOLT_SCHTKY_DROP;

    // 3. Process: Exponential Moving Average (EMA)
    // Smooths out voltage ripple during motor transients
    temp.batt_volt_V =  ( 0.01f * instBattVolt ) + 
                        ( 0.99f* temp.batt_volt_V);
    // temp.batt_volt_V = (gBATT_EMA_Coef * instBattVolt) + 
    //                  ((1.0f - gBATT_EMA_Coef) * temp.batt_volt_V);

    // 4. Protection: Low Power Detection with Hysteresis
    // Prevents "flickering" near the threshold
    float thresholdHigh = VOLT_BATTERY_MIN + 0.15f;
    float thresholdLow  = VOLT_BATTERY_MIN;

    if (temp.batt_volt_V < thresholdLow) {
        temp.is_low_power = true;
    } 
    else if (temp.batt_volt_V > thresholdHigh) {
        temp.is_low_power = false;
    }
    // Note: If voltage is between Low and High, it keeps its previous state

    // 5. Commit: Write back to global RAM
    *arg_power = temp;
}

// ***** CALCULATION OF MEDIAN OF THREE FLOAT VALUES *****
static inline float filter_median_3_f(float a, float b, float c)
// Clock Cycles (Worst Case): approx 8 to 12 cycles
// Noise rejection: medium to small as 400-800mA spikes can easily walk through during 25mA motor current (no load/small Va)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// ***** CALCULATION OF MEDIAN OF FIVE FLOAT VALUES *****
static inline float filter_median_5_f(float a, float b, float c, float d, float e) {
// Clock Cycles (Worst Case): approx 25 to 35 cycles
// Noise rejection: good as 200-300mA spikes can walk through during 25mA motor current (no load/small Va)

    float t;
    if (a > b) { t = a; a = b; b = t; }
    if (c > d) { t = c; c = d; d = t; }
    if (a > c) { t = a; a = c; c = t; t = b; b = d; d = t; }
    if (a > e) { a = e; }
    if (b > e) { t = b; b = e; e = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (c > d) { t = c; c = d; d = t; }
    if (c > e) { t = c; c = e; e = t; }
    return c; // 'c' holds the true median
}

void smc_inner_loop( void )
{
    if (!gMpu.status.is_initialized) return; // DON'T DRIVE MOTORS IF SENSOR IS DEAD

    if ( gLED_seq_now != LED_SEQ_SMC_CTRL ) gLED_seq_now = LED_SEQ_SMC_CTRL;
    
    if ( (gFlg_run_inner_loop) && (gRobot_state == ROBO_STATE_BALA) )
    {
GET_DURATION(7);
gDuration_timer_us.inner_loop = gDuration_timer_us.delta_time[7];
// gDuration_timer_us.inner_loop = gDelta_time[7];
STORE_TIMER6(7);
        gFlg_run_inner_loop = false;

        // --- 1. Determine the Setpoint ---
        if (!gRobot.rc_cmd.is_connected)
            gRobot.control.SP_Pitch_rad = gRobot.control.Correction_Pos_rad;

        // --- 1. Calculate errors for all 4 states ---
        float err_angle      = gRobot.control.SP_Pitch_rad - gRobot.body.pitch_rad;
        // float err_angle      = 0.0f - robot.body.pitch_rad;
        float err_angle_rate = 0.0f - gRobot.body.rateY_rads;
        float err_pos        = 0.0f - gRobot.body.pos_m;
        float err_speed      = 0.0f - gRobot.body.speed_ms;

        // --- 2. Define the composite sliding surface (s) using dot notation ---
        float s =   (smc_controller.lambda_angle        * err_angle) + 
                    (smc_controller.lambda_angle_rate   * err_angle_rate) + 
                    (smc_controller.lambda_pos          * err_pos) + 
                    (smc_controller.lambda_speed        * err_speed);

        // --- 3. Calculate the switching control law (u_sw) ---
        float u_total = smc_controller.K * smc_saturate(s, smc_controller.phi);

        // --- 4. Feed Forward Term "Direct Drive" (FFgain * CMD) ---
        gRobot.control.FFterm_V  = -(gRobot.gain.FF * gRobot.rc_cmd.pitch);

        u_total = u_total + gRobot.control.FFterm_V;

        smc_controller.CM_term_unclamped_V = u_total;

        // --- 4. Motor Voltage Limit ---
        if ( u_total > VOLT_MOTOR_LIM_HI ) u_total = VOLT_MOTOR_LIM_HI;
        if ( u_total < -VOLT_MOTOR_LIM_HI ) u_total = -VOLT_MOTOR_LIM_HI;

        smc_controller.CM_term_V = u_total; // just to see it in Watch window

        // --- 48. Calculate TURN term ---
        smc_controller.TI_term_V  = gRobot.gain.TI * gRobot.rc_cmd.roll;
        
        // --- 49. Motor mixing ---
        // robot.motor.left.volt_V  = robot.control.CMterm_V + robot.control.TIterm_V;
        // robot.motor.right.volt_V = robot.control.CMterm_V - robot.control.TIterm_V;
        
        motor_set_voltage( smc_controller.CM_term_V + smc_controller.TI_term_V ,
                       smc_controller.CM_term_V - smc_controller.TI_term_V );
        
    }
}

// Saturation function to smooth out the control signal near the sliding surface
float smc_saturate(float arg_s, float arg_boundary_layer)
{
    if (arg_s > arg_boundary_layer) return 1.0;
    if (arg_s < -arg_boundary_layer) return -1.0;
    return arg_s / arg_boundary_layer;
}

// Converts 8-bit RGB color into WS2812 PWM pattern (0x1C = 0, 0x38 = 1)
void ws2812b_set_led_color(uint8_t arg_led_idx, uint8_t R, uint8_t G, uint8_t B)
{
    volatile uint8_t *p = gDMA_LED_array[arg_led_idx];

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

// void ws2812b_all_led(uint8_t arg_color, uint8_t arg_brightness)
// {
//     uint8_t R = 0, G = 0, B = 0;

//     if (arg_color == 0)      { R = arg_brightness; }
//     else if (arg_color == 1) { G = arg_brightness; }
//     else if (arg_color == 2) { B = arg_brightness; }

//     for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++)
//         ws2812b_set_led_color(i, R, G, B);
// }

// void WS2812B_Half_LED(uint8_t argRGB1, uint8_t argRGB2,
//                       uint8_t argBrg1, uint8_t argBrg2)
// {
//     uint8_t R1=0, G1=0, B1=0;
//     uint8_t R2=0, G2=0, B2=0;

//     // ----- FIRST HALF COLOR -----
//     if (argRGB1 == 0)      R1 = argBrg1;
//     else if (argRGB1 == 1) G1 = argBrg1;
//     else if (argRGB1 == 2) B1 = argBrg1;

//     // ----- SECOND HALF COLOR -----
//     if (argRGB2 == 0)      R2 = argBrg2;
//     else if (argRGB2 == 1) G2 = argBrg2;
//     else if (argRGB2 == 2) B2 = argBrg2;

//     uint8_t half = WS2812B_LED_COUNT >> 1;

//     // ----- FIRST HALF -----
//     for (uint8_t i = 0; i < half; i++)
//         ws2812b_set_led_color(i, R1, G1, B1);

//     // ----- SECOND HALF -----
//     for (uint8_t i = half; i < WS2812B_LED_COUNT; i++)
//         ws2812b_set_led_color(i, R2, G2, B2);
// }

// --- TIBATTERY GAUGE FEATURE ---
void ws2812b_batt_gauge(float arg_volt)
{
    if ( gFlg_batt_gauge) {
        gFlg_batt_gauge = false;

        static uint8_t last_active = 8; // Persistent memory
        const float Vmax = VOLT_BATTERY_DEF + 0.7f; // 12.0V
        const float Vmin = VOLT_BATTERY_MIN + 0.1f; // 11.3V
        const float Hysteresis_V = 0.05f;           // 50mV gap
        const uint8_t LEDcount = 8;

        // --- FEATURE: Flash-To-Pass Override ---
        // Checks if left button/trigger is pressed beyond threshold (> 1.2f)
        if (gRobot.rc_cmd.button_L > 1.2f)
        {
            // Flash all 8 LEDs high-intensity White/Blue headlight pulse
            for (uint8_t i = 0; i < LEDcount; i++)
            {
                ws2812b_set_led_color(i, 0xFF, 0xFF, 0xFF); // High-beam White
                // ws2812b_set_led_color(i, 0x00, 0x10, 0xFF); // POLICE
            }
            return; // Skip normal calculation while flashing
        }

        // 1. Calculate the 'Ideal' active level based on current voltage
        float norm = (arg_volt - Vmin) / (Vmax - Vmin);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        
        uint8_t current_target = (uint8_t)(norm * LEDcount);
        if (current_target == 0) current_target = 1;

        // 2. APPLY HYSTERESIS
        // We only change 'last_active' if the voltage has moved significantly
        // enough to clear the threshold for a different LED level.
        
        // Calculate the voltage threshold for the current LED level
        float volts_per_led = (Vmax - Vmin) / (float)LEDcount;
        float threshold_low  = Vmin + (volts_per_led * (last_active - 1)) - Hysteresis_V;
        float threshold_high = Vmin + (volts_per_led * last_active) + Hysteresis_V;

        if (arg_volt < threshold_low || arg_volt > threshold_high) {
            last_active = current_target;
        }

        // Use our 'debounced' value for drawing
        uint8_t active = last_active;

        // --- Draw the gauge (Rest of your logic remains the same) ---
        for (uint8_t i = 0; i < LEDcount; i++)
        {
            if (i < active - 1) {
                ws2812b_set_led_color(i, 0, 0x0F, 0);   // Green
            }
            else if (i == active - 1) {
                if (active == 1)
                    ws2812b_set_led_color(i, 0xF0, 0, 0);   // Red - strong
                else
                    ws2812b_set_led_color(i, 0x0F, 0, 0); // Red - weak
            }
            else {
                ws2812b_set_led_color(i, 0, 0, 0); // Off
            }
        }
    }
}

// --- TILT GAUGE FEATURE ---
void ws2812b_tilt_gauge(float arg_tilt_rad)
{
    if (gFlg_tilt_angle) {
        gFlg_tilt_angle = false;
        
        const float TILT_LIMIT_RAD = 0.4f; // 0.4 rad is 22deg
        const uint8_t LED_COUNT    = 8;
        const uint8_t LED_OFFSET   = 8; 

        // 1. Clamp tilt using the symmetric limit
        if (arg_tilt_rad < -TILT_LIMIT_RAD) arg_tilt_rad = -TILT_LIMIT_RAD;
        if (arg_tilt_rad >  TILT_LIMIT_RAD) arg_tilt_rad =  TILT_LIMIT_RAD;

        // 2. Normalize tilt into 0.0 .. 1.0 range
        float norm = (arg_tilt_rad + TILT_LIMIT_RAD) / (2.0f * TILT_LIMIT_RAD);

        // 3. Map to "Left LED" index of the pair (0 to 6)
        uint8_t idxA = (uint8_t)(norm * 6.0f) + LED_OFFSET;
        uint8_t idxB = idxA + 1;

        // 4. Update the LED strip
        for (uint8_t i = LED_OFFSET; i < (LED_OFFSET + LED_COUNT); i++)
        {
            if (i == idxA || i == idxB)
            {
                // Active Tilt "Needle" - Red
                ws2812b_set_led_color(i, 0x0F, 0x00, 0x00); 
            }
            else
            {
                // Background / Off
                ws2812b_set_led_color(i, 0x00, 0x00, 0x00);
            }
        }
    }
}

