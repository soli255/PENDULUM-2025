/****************************************
*****   PROGRAM: PENDULUM-ROBOT     *****
*****   VER 2025 2x12V 10W motors   *****
*****   MSPM0G3507@80MHz/12.5ns     *****
*****   Compiler CLANG -xx          *****
*****   Ing. TOMAS SOLARSKI         *****
*****   2025-11-18 1030             *****
****************************************/

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"

#define     LED_HEART_ON_MS         150
#define     LED_HEART_OFF_MS        150
#define     LED_TICK_COUNT          12
#define     LED_SEQUENCE_1          0b1
#define     LED_SEQUENCE_2          0b101
#define     LED_SEQUENCE_3          0b10101
#define     LED_SEQUENCE_4          0b1010101

#define     TIME_SEND_DATA_MS       100         // 10Hz send data rate
#define     TIME_MOTOR_OMEGA_MS     50          // rate to calculate impulses to determine motor speed
#define     TIME_MOTOR_OMEGA_S      0.05f       // in seconds
#define     TIME_MPU_DELAY_MS       250         // delay before MPU is initiated
#define     TIME_MPU_READ_MS        2           // reading rate of MPU
#define     TIME_MPU_READ_SEC       0.002f      // in seconds
#define     TIME_PID_CONTROLLER_MS  4           // MAIN PID loop control sample time

#define     ANGLE_DEG_LIMIT_NEG     -45.0f      // negative angle limit - beyond PID controll os OFF    
#define     ANGLE_DEG_LIMIT_POS     45.0f       // positive limit
#define     ANGLE_DEG_HYST          9.9f        // hysteresis
#define     ANGLE_DEG_DEFAULT       0.01f       // default upright angle for ballancing

#define     VOLT_MOTOR_LIMIT_NEG    -8.5f       // Action value limit - motor maximum voltage
#define     VOLT_MOTOR_LIMIT_POS    8.5f
#define     VOLT_BATTERY_MIN        10.5f       // Li-Pol low voltage
#define     VOLT_BATTERY_DEF        12.0f       // supply default voltage
#define     VOLT_ADC_REF            3.6f        // ADC ref voltage
#define     VOLT_SCHTKY_DROP        0.15f       // input rectifier

#define     MOTOR_COUNT             2           // M0 and M1

#define     ROBOT_STATE_INIT        0           // MPU not initiated or found
#define     ROBOT_STATE_BALANCING   1
#define     ROBOT_STATE_CALIBRATE   2           //

#define     ADC_CHANNELS            5           // 5 channel used: VS1, Current M0, Current M1 , Temp M0, Temp M1     
#define     MOV_AVG_ADC             16

#define     ALPHA_DEFAULT           0.02f      // Complementary filter coefficient

#define     P_GAIN_def              0.10f
#define     I_GAIN_def              0.00f
#define     D_GAIN_def              0.02f
#define     C_GAIN_LH_RH_def        1.0f 
#define     N_FILTER_def            2.0f        // smaller N means stronger filtering.
#define     GYRO_DAMP_def           15.0f       // Use angular velocity (from gyroscope) as a damping term
// #define     P_GAIN_def              0.35f
// #define     I_GAIN_def              0.01f
// #define     D_GAIN_def              -15.0f

// POLOLU MOTOR 4753 ( 50:1 Metal Gearmotor 37Dx70L mm 12V 200RPM with 64 CPR Encoder )
#define     GEAR_RATIO              50.0f
#define     IMPULSES_PER_REVOLUTION 64.0f

#define     WHEEL_DIAMETER_M        0.125f
#define     WHEEL_RADIUS_M          WHEEL_DIAMETER_M / 2.0f

#define     PI_NUMBER               3.1415926535f

#define     RAD_TO_DEG              57.2958f

#define     PWM_MAX_DUTY            90  // [ % ]
#define     PWM_MIN_DUTY            2   // [ % ]

// TIMER CHANNELS ASSIGNED TO OUTPUTS
#define     PHASE_A                 1
#define     PHASE_B                 0
#define     PHASE_C                 2
#define     PHASE_D                 3
#define     PHASE_COUNT             4

volatile    uint16_t    LED_Sequence_Actual    = LED_SEQUENCE_1;
volatile    uint8_t     Robot_State     = ROBOT_STATE_BALANCING;
volatile    uint8_t     Robot_State_1   = ROBOT_STATE_BALANCING;

// ----- FLAGS -----
volatile    bool        fHeartLED       = false;
volatile    bool        fMPUread        = false;
volatile    bool        fADC[ ADC_CHANNELS ];
volatile    bool        fSendData       = false;
volatile    bool        fQuadA          = false , fQuadA_1 = false;
volatile    bool        fQuadB          = false , fQuadB_1 = false;
volatile    bool        fQuadC          = false , fQuadC_1 = false;
volatile    bool        fQuadD          = false , fQuadD_1 = false;
volatile    bool        fMotorOmega     = false;
volatile    bool        fPIDcontroller  = false;
volatile    bool        fMotorEnable    = false;

// ----- COUNTERS -----
volatile    uint32_t    cHeartLED       = LED_HEART_ON_MS;
volatile    uint8_t     cHeartTick      = 0;
volatile    uint32_t    cMPUread        = TIME_MPU_READ_MS;
volatile    uint32_t    cMPUdelay       = TIME_MPU_DELAY_MS;
volatile    uint32_t    cSendData       = TIME_SEND_DATA_MS;
volatile    uint32_t    cMotorOmega     = TIME_MOTOR_OMEGA_MS;
volatile    uint32_t    cPIDcontroller  = TIME_PID_CONTROLLER_MS;
volatile    uint32_t    errorCnt[ 10 ];

// ----- MOTOR QUADRATURE ENCODERS -----
volatile    int32_t     cQuadM0 = 0 , cQuadM0_1 = 0;
volatile    int32_t     cQuadM1 = 0 , cQuadM1_1 = 0;

// ----- ANALOG -----
volatile    uint32_t    ADC_LSB[ ADC_CHANNELS ];    // [ LSB ]
volatile    float       volt[ ADC_CHANNELS ];   // [ V ]
volatile    float       voltBAT = VOLT_BATTERY_DEF;
volatile    float       Motor0_curr_mA = 0.0f , Motor1_curr_mA = 0.0f;
volatile    float       Drive0_temp_C  = 0.0f , Drive1_temp_C  = 0.0f;
// AVG filter
volatile    float       volt_AVG[ ADC_CHANNELS ];
volatile    float       volt_Tmp[ ADC_CHANNELS ];

// ----- PID -----
volatile    float       T_sample        = TIME_PID_CONTROLLER_MS / 1000.0f;     // sample time [sec]
volatile    float       P_gain          = P_GAIN_def;
volatile    float       I_gain          = I_GAIN_def;
volatile    float       D_gain          = D_GAIN_def;
volatile    float       C_gain_LH_RH    = C_GAIN_LH_RH_def;
volatile    float       N_coef          = N_FILTER_def;    // smaller N means stronger filtering.
volatile    float       G_damp          = GYRO_DAMP_def;   // Use angular velocity (from gyroscope) as a damping term
volatile    float       PID_angle_ref   = ANGLE_DEG_DEFAULT;
volatile    float       PID_error       = 0.0f;
volatile    float       PID_error_0     = 0.0f;
volatile    float       PID_actVal      = 0.0f;
volatile    float       PID_P_term_volt = 0.0f;
volatile    float       PID_I_term_volt = 0.0f;
volatile    float       PID_I_integral  = 0.0f;
volatile    float       PID_D_raw       = 0.0f;
volatile    float       PID_D_alpha     = 0.0f;
volatile    float       PID_D_fil       = 0.0f;
volatile    float       PID_D_term_volt = 0.0f;
volatile    float       PID_G_damp_volt = 0.0f;
volatile    float       PID_C_LHRH_volt = 0.0f;

// ----- USART0 -----
// RX
#define     RX0_BUFF_N  (4U) 
#define     UART0_RX_BUFFER_SIZE ( 1 << RX0_BUFF_N )
volatile    uint8_t     UART0_RXbuffer[ UART0_RX_BUFFER_SIZE ];         // Buffer size in 2^N size (1,2,4,8,16,32,64,...)
volatile    uint8_t     UART0_RXbytes = 0;                              // Buffer counter
volatile    bool        UART0_terminator_detected = false;
volatile    bool        UART0_command_detected = false;
volatile    int16_t     P_Gain_received = 0;
volatile    int16_t     I_Gain_received = 0;
volatile    int16_t     D_Gain_received = 0;
volatile    int16_t     C_Gain_received = 0;
volatile    int16_t     N_Gain_received = 0;
volatile    int16_t     G_Gain_received = 0;
// TX
#define     SIGNAL_CNT  (24U)                                           // count of Signals that will be sended over UART
#define     SIGNAL_RNG  (7U)                                            // Signals -99999 to +99999 range (6 characters max) + coma separator
volatile    int32_t     UART0_signals[ SIGNAL_CNT ];                    // Signals are stored in fiel to use for-cycle - now global
volatile    uint8_t     UART0_message[ SIGNAL_CNT * SIGNAL_RNG + 1 ];   // array with message (Signals + separators + terminator) - max. size
volatile    uint8_t     UART0_MessageLength = 0;
volatile    uint8_t     UART0_TXbytes = 0;

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
volatile	uint8_t		MPU_WhoAmI_data = 0;
volatile	uint8_t		MPU_switch = 1;
volatile    uint16_t    MPU_WhoAmI_Return = 0;
volatile    uint16_t    MPU_ExitSleep_Return = 0;
volatile    uint16_t    MPU_GetAccTempGyro_Return = 0;
// raw data registers
volatile	int16_t	    MPU_temp = 0;
volatile	int16_t	    MPU_accX = 0, MPU_accY = 0, MPU_accZ = 0;
volatile	int16_t	    MPU_accX_raw = 0, MPU_accY_raw = 0, MPU_accZ_raw = 0;
volatile	int16_t	    MPU_gyroX = 0, MPU_gyroY = 0, MPU_gyroZ = 0;
volatile	int16_t	    MPU_gyroX_raw = 0, MPU_gyroY_raw = 0, MPU_gyroZ_raw = 0;
// ----- OFFSET COMPENSATION -----
volatile	 int16_t	MPU_accX_offset  =  400 , MPU_accY_offset  = -250 , MPU_accZ_offset  = 800; // 2025-09-13
volatile	 int16_t	MPU_gyroX_offset =  100 , MPU_gyroY_offset =  -50 , MPU_gyroZ_offset =  75; // 2025-09-13
// calculated data
volatile    float       MPU_temp_C = 0.0f , MPU_accX_g = 0.0f , MPU_accY_g = 0.0f , MPU_accZ_g = 0.0f;
volatile	float		Gyro_X_RateDegs = 0.0f , Gyro_Y_RateDegs = 0.0f , Gyro_Z_RateDegs = 0.0f;
volatile    float       Angle_Pitch_Deg = 0.0f;
volatile    float       Angle_Roll_Deg = 0.0f;
volatile    float       Angle_Yaw_Deg = 0.0f;
// ----- COMPLEMENTARY FILTER -----
volatile    float       Alpha = ALPHA_DEFAULT;
volatile    float       Angle_Pitch_Deg_CoFil = 0.0f , Angle_Pitch_DegN_1 = 0.0f;
volatile    float       Angle_Roll_Deg_CoFil  = 0.0f , Angle_Roll_DegN_1  = 0.0f;
volatile    float       Angle_Yaw_Deg_CoFil   = 0.0f , Yaw_AngleDegN_1    = 0.0f;

// ----- MOTOR/ROBOT -----
volatile    float       Motor0_omega_rads;   // [rad/sec]    motor Angular Velocity
volatile    float       Motor1_omega_rads;   // [rad/sec]     

volatile    float       Motor0_theta_rad;    // [rad]        motor Angle
volatile    float       Motor1_theta_rad;    // [rad]     

volatile    float       robot_pos_m;     // [m]
volatile    float       robot_pos_m_1;
volatile    float       robot_speed_ms;  // [m/sec]
volatile    float       robot_angle_rad; // [rad] 
volatile    float       robot_rate_rads; // [rad/s]

// ----- FUNCTIONS -----
void    MCU_Init( void );
void    PID_check( void );
void    ADC_check( void );
void    UART_check( void );
void    QUAD_check( void );
void    OMEGA_check( void );
void    MPU_check ( void );
void    SET_PWM_DUTY( uint8_t argPhase , uint16_t argDuty );
void    Motor_M0_volt( float argVoltM0 );
void    Motor_M1_volt( float argVoltM1 );
bool    checkForGainCommand( void );
void    clearUART0_RXbuffer_Zero(void);
uint16_t MPU_ExitSleep( void );
uint16_t MPU_WhoAmI( void );
uint16_t MPU_GetAccTempGyro( void );
uint16_t Send_integers_over_UART( uint8_t argCount );

// *****************************************  *****************************************  *****************************************
// *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****
// *****************************************  *****************************************  *****************************************

int main( void ) {
    MCU_Init();

// *****************************************  *****************************************  *****************************************
// *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****
// *****************************************  *****************************************  *****************************************
    while ( 1 ) {

        MPU_check();    // read IMS
        
        PID_check();    // Regulate
        
        ADC_check();    // read Vs and Motor
        
        UART_check();   // send diagnostic data or receive command
        
        QUAD_check();   // calculate motor RPM
        
        OMEGA_check();  // calculate speed

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
    if ( --cHeartLED == 0 )
    {
        if ( LED_Sequence_Actual & ( 1 << cHeartTick ) )
        {
            DL_GPIO_setPins( GPIO_PORT , GPIO_GRN4_PIN );
            cHeartLED = LED_HEART_ON_MS;
        }
        else {
            DL_GPIO_clearPins( GPIO_PORT , GPIO_GRN4_PIN );
            cHeartLED = LED_HEART_OFF_MS;
        }        

        cHeartTick++;
        cHeartTick %= LED_TICK_COUNT;
    }
    // ----- PID REGULATION -----
    if ( --cPIDcontroller == 0 )
    {
        cPIDcontroller = TIME_PID_CONTROLLER_MS;
        fPIDcontroller = true;
    }
    // ----- MOTOR SPEED/OMEGA MEASUREMENT -----
    if ( --cMotorOmega == 0 )
    {
        cMotorOmega = TIME_MOTOR_OMEGA_MS;
        fMotorOmega = true;
    }
    // ----- MPU DELAY -----
    if ( cMPUdelay > 0 )
        --cMPUdelay;
    // ----- READ DATA FROM MPU -----
    if ( --cMPUread == 0 )
    {
        cMPUread = TIME_MPU_READ_MS;
        fMPUread = true;
    }
    // ----- SEND DATA OVER UART -----
    if ( --cSendData == 0 )
    {
        cSendData = TIME_SEND_DATA_MS;
        fSendData = true;
    }
}

// Chan 0: VS1
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
            fADC[ 4 ] = true;
            ADC_LSB[ 4 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_TEMP_M1 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            fADC[ 2 ] = true;
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
            fADC[ 3 ] = true;
            ADC_LSB[ 3 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_TEMP_M0 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            fADC[ 1 ] = true;
            ADC_LSB[ 1 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_CURR_M0 );
            break;    
        case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            fADC[ 0 ] = true;
            ADC_LSB[ 0 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_VS1 );
            break; 
        default:
            break;
    }
}

/****************************************
*****   UART 3 TRANSMITTER          *****
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
DL_GPIO_setPins( GPIO_PORT , GPIO_RC_PWM1_PIN );
            //transmit byte
            DL_UART_Main_transmitData ( UART_0_INST , UART0_message[ UART0_TXbytes++ ] );
            // Disable transmit interrupt if no more bytes
            if ( UART0_TXbytes == UART0_MessageLength )
            {
                DL_UART_Main_disableInterrupt( UART_0_INST , DL_UART_MAIN_INTERRUPT_TX );
                UART0_TXbytes = 0;
DL_GPIO_clearPins( GPIO_PORT , GPIO_RC_PWM1_PIN );
            }
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
    // ***** I2C 1 - enable interrupt *****
    NVIC_EnableIRQ( I2C1_INST_INT_IRQN);
    // ***** default motor value - motor stop *****
    SET_PWM_DUTY( PHASE_A , 0U );       // MOTOR 0 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_B , 0U );       // MOTOR 0 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
    SET_PWM_DUTY( PHASE_C , 0U );       // MOTOR 1 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_D , 0U );       // MOTOR 1 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
}

/*****************************
*****   PID CONTROLLER   *****
*****************************/
void PID_check( void )
{
    // uint16_t pwm_uint_value = 0;

    if ( fPIDcontroller )
    {
        fPIDcontroller = false;

        PID_error = PID_angle_ref - Angle_Pitch_Deg_CoFil;

        // P term
        PID_P_term_volt = PID_error * P_gain;

        // I term
        PID_I_integral += PID_error * T_sample;
        PID_I_term_volt = PID_I_integral * I_gain;

        // D term with filtered derivative
        PID_D_raw = (PID_error - PID_error_0) / T_sample;
        PID_D_alpha = T_sample / (T_sample + (1.0f / N_coef));  // Filter coefficient
        PID_D_fil = PID_D_alpha * PID_D_raw + (1.0f - PID_D_alpha) * PID_D_fil;
        PID_D_term_volt = PID_D_fil * D_gain;

        //
        PID_G_damp_volt = G_damp * Gyro_Y_RateDegs;

        // Final output
        PID_error_0 = PID_error;
        PID_actVal = PID_P_term_volt + PID_I_term_volt + PID_D_term_volt + PID_G_damp_volt;

        PID_C_LHRH_volt = ( Motor0_theta_rad - Motor1_theta_rad ) * C_gain_LH_RH; 

        // PID_D_term_volt  = Gyro_Y_RateDegs         * D_gain;   // D
        
        if ( PID_actVal > VOLT_MOTOR_LIMIT_POS )
            PID_actVal = VOLT_MOTOR_LIMIT_POS;

        if ( PID_actVal < VOLT_MOTOR_LIMIT_NEG )
            PID_actVal = VOLT_MOTOR_LIMIT_NEG;

        if ( fMotorEnable && ( voltBAT > VOLT_BATTERY_MIN ) )
        {
            if ( Angle_Pitch_Deg_CoFil > ANGLE_DEG_LIMIT_POS + ANGLE_DEG_HYST )
            {
                Motor_M0_volt( 0.0f );
                Motor_M1_volt( 0.0f );
            } else
                if ( Angle_Pitch_Deg_CoFil < ANGLE_DEG_LIMIT_NEG - ANGLE_DEG_HYST )
                {
                    Motor_M0_volt( 0.0f );
                    Motor_M1_volt( 0.0f );
                } else
                    if ( ( Angle_Pitch_Deg_CoFil < ANGLE_DEG_LIMIT_POS ) &&
                         ( Angle_Pitch_Deg_CoFil > ANGLE_DEG_LIMIT_NEG )
                        )            
                    {
                        Motor_M0_volt( PID_actVal - PID_C_LHRH_volt );
                        Motor_M1_volt( PID_actVal + PID_C_LHRH_volt );
                    }
        } else {
            Motor_M0_volt( 0.0f );
            Motor_M1_volt( 0.0f );
        }
    }
}

/*****************************
*****   MOTOR VOLTAGE   *****
*****************************/

void Motor_M0_volt( float argVoltM0 )
{
    uint16_t pwm_uint_value_M0 = 0;

    if ( argVoltM0 > 0.15f ) {
        pwm_uint_value_M0 = (uint16_t)( 100.0f * argVoltM0 / 12.0f );
        // run ROBOT FORWARD
        SET_PWM_DUTY( PHASE_A , 0U );
        SET_PWM_DUTY( PHASE_B , pwm_uint_value_M0 );
    } else if ( argVoltM0 < -0.15f ) {
        pwm_uint_value_M0 = (uint16_t)( -100.0f * argVoltM0 / 12.0f );
        // run ROBOT REVERSE
        SET_PWM_DUTY( PHASE_A , pwm_uint_value_M0 );
        SET_PWM_DUTY( PHASE_B , 0U );
    } else {
        // STOP
        SET_PWM_DUTY( PHASE_A , 0U );
        SET_PWM_DUTY( PHASE_B , 0U );
    } 
}

void Motor_M1_volt( float argVoltM1 )
{
    uint16_t pwm_uint_value_M1 = 0;

    if ( argVoltM1 > 0.15f ) {
        pwm_uint_value_M1 = (uint16_t)( 100.0f * argVoltM1 / 12.0f );
        // run ROBOT FORWARD
        SET_PWM_DUTY( PHASE_C , pwm_uint_value_M1 );
        SET_PWM_DUTY( PHASE_D , 0U );
    } else if ( argVoltM1 < -0.15f ) {
        pwm_uint_value_M1 = (uint16_t)( -100.0f * argVoltM1 / 12.0f );
        // run ROBOT REVERSE
        SET_PWM_DUTY( PHASE_C , 0U );
        SET_PWM_DUTY( PHASE_D , pwm_uint_value_M1 );
    } else {
        // STOP
        SET_PWM_DUTY( PHASE_C , 0U );
        SET_PWM_DUTY( PHASE_D , 0U );
    } 
}

/**************************************
*****     SEND DATA OVER UART     *****
**************************************/
uint16_t Send_integers_over_UART( uint8_t argCount )
{
    // ten thousands, thousands, hundreds, tens, ones - store variables for each decimal order
    int16_t     A = 0 , B = 0 , C = 0 , D = 0 , E = 0;
    // number counter
    uint16_t    i = 0;
    // char in string counter
    uint16_t    j = 0;
    // put numbers into field to better proccess later
    // UART0_signals[ 0 ] = D0;
    // UART0_signals[ 1 ] = D1;
    // UART0_signals[ 2 ] = D2;
    // UART0_signals[ 3 ] = D3;
    // UART0_signals[ 4 ] = D4;
    // UART0_signals[ 5 ] = D5;
    // MAIN FOR-CYCLE for conversion integers into strings
    // for ( i = 0 ; i < SIGNAL_CNT ; i++ )
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
        // if ( i < SIGNAL_CNT - 1 )
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
// 5 channel used: VS1, Current M0, Current M1 , Temp M0, Temp M1     
void ADC_check( void )
{
    for ( int i = 0 ; i < ADC_CHANNELS ; i++ )
    {
        if ( fADC[ i ] )
        {
            fADC[ i ] = false;

            if ( i == 0 )
                volt[ i ] = (float)ADC_LSB[ i ] * VOLT_ADC_REF * 11.0f / 4096.0f;   // 11 => divider ( 4k7 + 47k ) / 4k7
            else
                volt[ i ] = (float)ADC_LSB[ i ] * VOLT_ADC_REF / 4096.0f;
            
            volt_Tmp[ i ] = volt_Tmp[ i ] - ( volt_Tmp[ i ] / MOV_AVG_ADC ) + volt[ i ];
            volt_AVG[ i ] = volt_Tmp[ i ] / MOV_AVG_ADC;

            if ( i == 1 )
                Motor0_curr_mA = 1000.0f * volt_AVG[ 1 ] / ( 0.033f * 20.0f );    // Motor_current[A] = V_sns[V] / R_sns[R] * INA_GAIN[-] -> Motor_cuureent = V_sns / 0.033R * 20
            if ( i == 2 )
                Motor1_curr_mA = 1000.0f * volt_AVG[ 2 ] / ( 0.033f * 20.0f );
        }
    }
    voltBAT = volt_AVG[ 0 ] + VOLT_SCHTKY_DROP;
}

void UART_check( void )
{
    // ----- COMMAND RECEIVED CHECK -----
    if ( UART0_terminator_detected ) {
        UART0_terminator_detected = false;

        checkForGainCommand();   

        clearUART0_RXbuffer_Zero();   
    }
    // ----- SEND DATA OVER UART -----
    if ( fSendData )
    {
        fSendData = false;

        switch ( Robot_State )
        {
            // ----- OBSERVATION ------  
            case ROBOT_STATE_INIT:                  
                break;
            // ----- REGULATION ------    
            case ROBOT_STATE_BALANCING:                    
                UART0_signals[  0 ] = 100.0f * PID_angle_ref;
                UART0_signals[  1 ] = 100.0f * Angle_Pitch_Deg_CoFil;
                UART0_signals[  2 ] = 100.0f * PID_actVal;
                UART0_signals[  3 ] = 100.0f * PID_P_term_volt;         // P
                UART0_signals[  4 ] = 100.0f * PID_I_term_volt;         // I
                UART0_signals[  5 ] = 100.0f * PID_D_term_volt;         // D
                UART0_signals[  6 ] = 100.0f * N_coef;                  // N
                UART0_signals[  7 ] = 100.0f * PID_C_LHRH_volt;         // C

                UART0_signals[  8 ] = 1.0f * Motor0_curr_mA;
                UART0_signals[  9 ] = 1.0f * Motor1_curr_mA;
                UART0_signals[ 10 ] = 100.0f * Motor0_theta_rad;
                UART0_signals[ 11 ] = 100.0f * Motor1_theta_rad;
                UART0_signals[ 12 ] = 100.0f * Motor0_omega_rads;
                UART0_signals[ 13 ] = 100.0f * Motor1_omega_rads;
                UART0_signals[ 14 ] = 100.0f * PID_G_damp_volt;
                UART0_signals[ 15 ] = 100.0f * G_damp;
                
                UART0_signals[ 16 ] = 100.0f * P_gain;
                UART0_signals[ 17 ] = 100.0f * I_gain;
                UART0_signals[ 18 ] = 100.0f * D_gain;
                UART0_signals[ 19 ] = 100.0f * C_gain_LH_RH;
                UART0_signals[ 20 ] = 100.0f * N_coef;
                UART0_signals[ 21 ] = 100.0f * voltBAT;
                UART0_signals[ 22 ] = 100.0f * MPU_temp_C;
                UART0_signals[ 23 ] = 0.0f; // empty

                Send_integers_over_UART ( SIGNAL_CNT );

                break;
            // ----- CALIBRATION ------
            case ROBOT_STATE_CALIBRATE:                  
                break;    
        } 
    }
}

bool checkForGainCommand(void)
{
    char gainType = UART0_RXbuffer[0];

    // Check if it matches [P|D|C]GAIN=
    if ((gainType == 'P' || gainType == 'I' || gainType == 'D' || gainType == 'C' || gainType == 'N' || gainType == 'G') &&
        UART0_RXbuffer[1] == 'G' &&
        UART0_RXbuffer[2] == 'A' &&
        UART0_RXbuffer[3] == 'I' &&
        UART0_RXbuffer[4] == 'N' &&
        UART0_RXbuffer[5] == '=')
    {
        // Look for '\n' terminator
        for (uint8_t i = 6; i < UART0_RX_BUFFER_SIZE; ++i)
        {
            if (UART0_RXbuffer[i] == '\n')
            {
                // Extract value substring
                char valueStr[8] = {0};
                uint8_t len = i - 6;
                if (len > 0 && len < sizeof(valueStr))
                {
                    for (uint8_t j = 0; j < len; ++j)
                    {
                        valueStr[j] = UART0_RXbuffer[6 + j];
                    }

                    int16_t value = atoi(valueStr);

                    // Assign to correct variable
                    switch (gainType)
                    {
                        case 'P': 
                            P_Gain_received = value;
                            if ( 100 >= P_Gain_received && P_Gain_received >= 0 ) { // P shall be small and positive
                                P_gain = (float)P_Gain_received / 100.0f;
                            }
                            break;
                        case 'I': 
                            I_Gain_received = value;
                            if ( 100 >= I_Gain_received && I_Gain_received >= 0 ) { // I shall be small and positive
                                I_gain = (float)I_Gain_received / 10000.0f;
                            }
                            break;
                        case 'D':
                            D_Gain_received = value;
                            if ( 100 >= D_Gain_received && D_Gain_received >= -100  ) { // D shall be small
                                D_gain = (float)D_Gain_received / 100.0f;
                            }
                            break;
                        case 'C':
                            C_Gain_received = value;
                            if ( 10000 >= C_Gain_received && C_Gain_received >= 0 ) {   // C could be large and positive
                                C_gain_LH_RH = (float)C_Gain_received / 100.0f;
                            }
                            break;
                        case 'N':
                            N_Gain_received = value;
                            if ( 2000 >= N_Gain_received && N_Gain_received >= 0 ) {    // N could be large and positive
                                N_coef = (float)N_Gain_received / 100.0f;
                            }
                            break;
                        case 'G':
                            G_Gain_received = value;
                            if ( 2000 >= G_Gain_received && G_Gain_received >= 0 ) {    // G could be large and positive
                                G_damp = (float)G_Gain_received / 100.0f;
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

void clearUART0_RXbuffer_Zero(void)
{
    for (int i = 0; i < UART0_RX_BUFFER_SIZE; ++i)
    {
        UART0_RXbuffer[i] = 0x00;
    }
    UART0_RXbytes = 0;
}

// ----- MOTOR PWM -----
void SET_PWM_DUTY( uint8_t argPhase , uint16_t argDuty )
{
    // PWM limiter 0-100%
    if ( argDuty > PWM_MAX_DUTY )
        argDuty = PWM_MAX_DUTY;
    else if ( argDuty < PWM_MIN_DUTY )
        argDuty = 0;

    if ( argPhase > PHASE_COUNT-1 )     // if greather than 3 (4 phases)
        argPhase %= PHASE_COUNT-1 ;

    switch ( argPhase )
    {
        case 0:
            // DL_Timer_setCaptureCompareOutCtl( PWM_0_INST , DL_TIMER_CC_OCTL_INIT_VAL_LOW , DL_TIMER_CC_OCTL_INV_OUT_DISABLED , DL_TIMER_CC_OCTL_SRC_FUNCVAL , GPIO_PWM_0_C0_IDX );
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty ,                                                                                    GPIO_PWM_0_C0_IDX );
            // DL_Timer_overrideCCPOut(          PWM_0_INST , DL_TIMER_FORCE_OUT_DISABLED , DL_TIMER_FORCE_CMPL_OUT_DISABLED ,                                   GPIO_PWM_0_C0_IDX );
            break;
        case 1:
            // DL_Timer_setCaptureCompareOutCtl( PWM_0_INST , DL_TIMER_CC_OCTL_INIT_VAL_LOW , DL_TIMER_CC_OCTL_INV_OUT_DISABLED , DL_TIMER_CC_OCTL_SRC_FUNCVAL , GPIO_PWM_0_C1_IDX );
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty ,                                                                                    GPIO_PWM_0_C1_IDX );
            // DL_Timer_overrideCCPOut(          PWM_0_INST , DL_TIMER_FORCE_OUT_DISABLED , DL_TIMER_FORCE_CMPL_OUT_DISABLED ,                                   GPIO_PWM_0_C1_IDX );
            break;
        case 2:
            // DL_Timer_setCaptureCompareOutCtl( PWM_0_INST , DL_TIMER_CC_OCTL_INIT_VAL_LOW , DL_TIMER_CC_OCTL_INV_OUT_DISABLED , DL_TIMER_CC_OCTL_SRC_FUNCVAL , GPIO_PWM_0_C2_IDX );
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty ,                                                                                    GPIO_PWM_0_C2_IDX );
            // DL_Timer_overrideCCPOut(          PWM_0_INST , DL_TIMER_FORCE_OUT_DISABLED , DL_TIMER_FORCE_CMPL_OUT_DISABLED ,                                   GPIO_PWM_0_C2_IDX );
            break;       
        case 3:
            DL_TimerA_setCaptureCompareValue( PWM_0_INST , 100U-argDuty ,                                                                                    GPIO_PWM_0_C3_IDX );
            break;    
    }
    
}

// *****************************************  *****************************************  *****************************************
// *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****  *****     MPU GYRO ACCELEROMETER    *****
// *****************************************  *****************************************  *****************************************

void MPU_check ( void )
{
    // ----- READ MPU DATA -----        
    if ( fMPUread && ( cMPUdelay==0 ) )
    {
        fMPUread = false;     

        switch ( MPU_switch )
        {
            case 1:
                MPU_ExitSleep_Return = MPU_ExitSleep();
                if (  MPU_ExitSleep_Return == 0 ) {
                    MPU_switch = 2;
                    LED_Sequence_Actual = LED_SEQUENCE_2;
                } else {
                    errorCnt[6]++;
                    MPU_switch = 1;
                    fMotorEnable = false;
                    LED_Sequence_Actual = LED_SEQUENCE_1;
                }
                break;
            case 2:
                MPU_WhoAmI_data = 0x00;
                MPU_WhoAmI_Return = MPU_WhoAmI();   // takes ~105us @400kHz
                if ( MPU_WhoAmI_Return == 0 ) {
                    if ( MPU_WhoAmI_data == 104 ) {
                        MPU_switch = 3;
                        fMotorEnable = true;
                        LED_Sequence_Actual = LED_SEQUENCE_3;
                    } else {
                        MPU_switch = 1;
                        fMotorEnable = false;
                        LED_Sequence_Actual = LED_SEQUENCE_1;
                    }
                } else {
                    MPU_switch = 1;
                    fMotorEnable = false;
                    LED_Sequence_Actual = LED_SEQUENCE_1;
                }
                break;
            case 3:      
        DL_GPIO_setPins( GPIO_PORT , GPIO_RC_PWM0_PIN );
                MPU_GetAccTempGyro_Return = MPU_GetAccTempGyro();   // takes ~410us @400kHz
        DL_GPIO_clearPins( GPIO_PORT , GPIO_RC_PWM0_PIN );
                if ( MPU_GetAccTempGyro_Return != 0 ) { 
                    MPU_switch = 1;
                    fMotorEnable = false;
                    LED_Sequence_Actual = LED_SEQUENCE_1;
                    errorCnt[0]++;
                }

                break;
            default:
                MPU_switch = 1;
                LED_Sequence_Actual = LED_SEQUENCE_1;
                break;
        }
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

    // ANTIBLOCK 
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;

    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
 
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;

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
 
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;
  
    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;
  
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;
    
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;

    //----- REPEAT START -----

    // Send a read request to Target
    gRxLen               = 1;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );

    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 )
        I2C_Flag_Block++;
    
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

    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        errorCnt[1]++;
    }

    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    
    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        errorCnt[2]++;
    }

    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        errorCnt[3]++;
    }

    // ANTIBLOCK
    for ( I2C_Anti_Block = 255 ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        errorCnt[4]++;
    }

    //----- REPEAT START -----

    // Send a read request to Target
    gRxLen               = 14;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );

    // ANTIBLOCK
    for ( I2C_Anti_Block = 15000U ; ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) ) && I2C_Anti_Block > 0 ; I2C_Anti_Block--)
	    ;

    if ( I2C_Anti_Block == 0 ) {
        I2C_Flag_Block++;
        errorCnt[5]++;
    }

    MPU_accX_raw = (int16_t)gRxPacket[ 0 ] << 8;	// read High byte
    MPU_accX_raw += (int16_t)gRxPacket[ 1 ];
    MPU_accX = MPU_accX_raw - MPU_accX_offset;

    MPU_accY_raw = (uint16_t)gRxPacket[ 2 ] << 8;	// read High byte
    MPU_accY_raw += gRxPacket[ 3 ];
    MPU_accY = MPU_accY_raw - MPU_accY_offset;

    MPU_accZ_raw = (uint16_t)gRxPacket[ 4 ] << 8;	// read High byte
    MPU_accZ_raw += gRxPacket[ 5 ];
    MPU_accZ = MPU_accZ_raw - MPU_accZ_offset;

    MPU_temp = (uint16_t)gRxPacket[ 6 ] << 8;	// read High byte
    MPU_temp += gRxPacket[ 7 ];

    MPU_accX_g = MPU_accX / 16384.0f; //
    MPU_accY_g = MPU_accY / 16384.0f; //
    MPU_accZ_g = MPU_accZ / 16384.0f; // this three lines takes 50us

// TRIGINOMETRY USED TO DETERMINE ANGLES - theta, psi, phi
// https://www.analog.com/en/resources/app-notes/an-1057.html
    Angle_Pitch_Deg = RAD_TO_DEG * atan2( MPU_accX_g , ( sqrt( MPU_accY_g*MPU_accY_g + MPU_accZ_g*MPU_accZ_g ) ) ); //
    Angle_Roll_Deg  = RAD_TO_DEG * atan2( MPU_accY_g , ( sqrt( MPU_accX_g*MPU_accX_g + MPU_accZ_g*MPU_accZ_g ) ) ); //
    Angle_Yaw_Deg   = RAD_TO_DEG * atan2( ( sqrt( MPU_accX_g*MPU_accX_g + MPU_accY_g*MPU_accY_g ) ) , MPU_accZ_g ); //  this three lines takes 1.1ms@32MHz !!! 450us@80MHz
    
    MPU_temp_C = MPU_temp / 340.0f + 36.53f;  // 40us

    MPU_gyroX_raw = (uint16_t)gRxPacket[ 8 ] << 8;	// read High byte
    MPU_gyroX_raw += gRxPacket[ 9 ];
    MPU_gyroX = MPU_gyroX_raw - MPU_gyroX_offset;

    MPU_gyroY_raw = (uint16_t)gRxPacket[ 10 ] << 8;	// read High byte
    MPU_gyroY_raw += gRxPacket[ 11 ];
    MPU_gyroY = MPU_gyroY_raw - MPU_gyroY_offset;

    MPU_gyroZ_raw = (uint16_t)gRxPacket[ 12 ] << 8;	// read High byte
    MPU_gyroZ_raw += gRxPacket[ 13 ];
    MPU_gyroZ = MPU_gyroZ_raw - MPU_gyroZ_offset;

// ******************************************
// ********** COMPLEMENTARY FILTER **********
// ******************************************    
// https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d
// angle = (1 - α) * (angle + gyroscope * dt) + α * accelerometer
// pitch = 0.98 * (pitch + gyroscope_x * dt) + 0.02* accelerometer_x

    // T_sampling, sensitivity 131LSB / deg/s
	Gyro_X_RateDegs = ( ( MPU_gyroX * TIME_MPU_READ_SEC ) / 131.0f );
    Gyro_Y_RateDegs = ( ( MPU_gyroY * TIME_MPU_READ_SEC ) / 131.0f );
    Gyro_Z_RateDegs = ( ( MPU_gyroZ * TIME_MPU_READ_SEC ) / 131.0f );   // 3 lines 125us	
	
    Angle_Pitch_Deg_CoFil = ( 1 - Alpha ) * ( Angle_Pitch_DegN_1 + Gyro_Y_RateDegs ) + Alpha * Angle_Pitch_Deg;
	Angle_Roll_Deg_CoFil  = ( 1 - Alpha ) * ( Angle_Roll_DegN_1  + Gyro_X_RateDegs ) + Alpha * Angle_Roll_Deg;
    Angle_Yaw_Deg_CoFil   = ( 1 - Alpha ) * ( Yaw_AngleDegN_1   + Gyro_Z_RateDegs ) + Alpha * Angle_Yaw_Deg;  // 3 lines 120us

    // re-shift
    Angle_Pitch_DegN_1 = Angle_Pitch_Deg_CoFil;
    Angle_Roll_DegN_1  = Angle_Roll_Deg_CoFil;
    Yaw_AngleDegN_1   = Angle_Yaw_Deg_CoFil;

// ********************************
// ********** AVG FILTER **********
// ******************************** 
    // Pitch_AngleDeg_Tmp = Pitch_AngleDeg_Tmp - ( Pitch_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Angle_Pitch_Deg;
	// Pitch_AngleDeg_AVG = Pitch_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    // Roll_AngleDeg_Tmp = Roll_AngleDeg_Tmp - ( Roll_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Angle_Roll_Deg;
	// Roll_AngleDeg_AVG = Roll_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    // Yaw_AngleDeg_Tmp = Yaw_AngleDeg_Tmp - ( Yaw_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Angle_Yaw_Deg;
	// Yaw_AngleDeg_AVG = Yaw_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    // robot_angle_rad = Pitch_AngleDeg_AVG / RAD_TO_DEG;
    // robot_rate_rads = Gyro_X_RateDegs / RAD_TO_DEG;

    return I2C_Flag_Block;

}

// ----- OMEGA CALCULATION -----
void OMEGA_check( void )
{
    if ( fMotorOmega )
    {
        fMotorOmega = false;

        Motor0_omega_rads = -2.0f * PI_NUMBER * ( (float)( cQuadM0 - cQuadM0_1 ) ) * 1/TIME_MOTOR_OMEGA_S / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        Motor0_theta_rad  = -2.0f * PI_NUMBER * (float)( cQuadM0 ) / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        cQuadM0_1 = cQuadM0;
        // cQuadM0_1 = cQuadM0;
        
        Motor1_omega_rads = 2.0f * PI_NUMBER * ( (float)( cQuadM1 - cQuadM1_1 ) ) * 1/TIME_MOTOR_OMEGA_S / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        Motor1_theta_rad  = 2.0f * PI_NUMBER * (float)( cQuadM1 ) / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        cQuadM1_1 = cQuadM1;
        // cQuadM1_1 = cQuadM1;

        // Robot speed and position calculation
        robot_pos_m = 0.5f * ( Motor0_theta_rad + Motor1_theta_rad ) * WHEEL_RADIUS_M;
        robot_speed_ms = ( robot_pos_m - robot_pos_m_1 ) * 1.0f/TIME_MOTOR_OMEGA_S;
        robot_pos_m_1 = robot_pos_m;
    }
}

/*******************************
*****   QUADRATURE FUNCT   *****
********************************/
void QUAD_check( void )
{
    // ----- MOTOR 0 quad channels -----
    fQuadA = DL_GPIO_readPins( QUAD_M0_PORT,QUAD_M0_A_PIN );
    fQuadB = DL_GPIO_readPins( QUAD_M0_PORT,QUAD_M0_B_PIN );
    // ----- A-channel -----
    if ( fQuadA != fQuadA_1 )
    {
        fQuadA_1 = fQuadA;
        // increment or decrement in depend of A channel EDGE and level on B channel
        // rising edge		
        if ( fQuadA )
        {
            if ( fQuadB )
                --cQuadM0;
            else
                ++cQuadM0;
        }
        else
        {
            if ( fQuadB )
                ++cQuadM0;
            else
                --cQuadM0;
        }
    }
    // ----- B-channel -----
    if ( fQuadB != fQuadB_1 )
    {
        fQuadB_1 = fQuadB;
        // increment or decrement in depend of B channel EDGE and level on A channel
        // rising edge
        if ( fQuadB )
        {
            if ( fQuadA )
                ++cQuadM0;
            else
                --cQuadM0;
        }
        else
        {
            if ( fQuadA )
                --cQuadM0;
            else
                ++cQuadM0;
        }	
    }  

    // ----- MOTOR 1 quad channels -----
    fQuadC = DL_GPIO_readPins( QUAD_M1_PORT,QUAD_M1_C_PIN );
    fQuadD = DL_GPIO_readPins( QUAD_M1_PORT,QUAD_M1_D_PIN );
    // ----- D-channel -----
    if ( fQuadC != fQuadC_1 )
    {
        fQuadC_1 = fQuadC;
        // increment or decrement in depend of C channel EDGE and level on D channel
        // rising edge		
        if ( fQuadC )
        {
            if ( fQuadD )
                --cQuadM1;
            else
                ++cQuadM1;
        }
        else
        {
            if ( fQuadD )
                ++cQuadM1;
            else
                --cQuadM1;
        }
    }
    // ----- D-channel -----
    if ( fQuadD != fQuadD_1 )
    {
        fQuadD_1 = fQuadD;
        // increment or decrement in depend of B channel EDGE and level on A channel
        // rising edge
        if ( fQuadD )
        {
            if ( fQuadC )
                ++cQuadM1;
            else
                --cQuadM1;
        }
        else
        {
            if ( fQuadC )
                --cQuadM1;
            else
                ++cQuadM1;
        }	
    } 
}