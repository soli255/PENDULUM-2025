/****************************************
*****   PROGRAM: PENDULUM-ROBOT     *****
*****   VER 2025 2x12V 10W motors   *****
*****   MSPM0G3507@80MHz/12.5ns     *****
*****   Compiler CLANG -xx          *****
*****   Ing. TOMAS SOLARSKI         *****
*****   2025-08-31 1900             *****
****************************************/

#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"

#define     LED_HEART_ON_MS         150
#define     LED_HEART_OFF_MS        150
#define     LED_TICK_COUNT          12
#define     LED_SEQUENCE            0b1010101

#define     TIME_SEND_DATA_MS       10          // 100Hz send data rate
#define     TIME_MOTOR_OMEGA_MS     50          // rate to calculate impulses to determine motor speed
#define     TIME_MOTOR_OMEGA_S      0.05f       // in seconds

#define     TIME_MPU_DELAY_MS       125         // delay before MPU is initiated

#define     TIME_MPU_READ_MS        2           // reading rate of MPU
#define     TIME_MPU_READ_SEC       0.002f      // in seconds
#define     TIME_PID_CONTROLLER_MS  TIME_MPU_READ_MS// MAIN PID loop control sample time

#define     ANGLE_DEG_LIMIT_NEG     -15.0f       // negative ange limit - beyond PID controll os OFF    
#define     ANGLE_DEG_LIMIT_POS      15.0f       // positive limit

#define     MOTOR_VOLT_LIMIT_NEG    -6.5f       // Action value limit - motor maximum voltage
#define     MOTOR_VOLT_LIMIT_POS     6.5f

#define     MOTOR_COUNT             2           // M0 and M1

#define     ROBOT_STATE_INIT        0
#define     ROBOT_STATE_BALANCING   1
#define     ROBOT_STATE_CALIBRATE   2

#define     VOLT_REF                3.6f        // ADC ref voltage
#define     VOLT_VS_DEFAULT         7.5f        // supply default voltage

#define     ADC_CHANNELS            5           // 5 channel used: VS1, Current M0, Current M1 , Temp M0, Temp M1     
#define     MOV_AVG_ADC             8

#define     ALPHA_DEFAULT            0.02f      // Complementary filter coefficient

// POLOLU MOTOR 4753 ( 50:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder )
#define     GEAR_RATIO              50.0f
#define     IMPULSES_PER_REVOLUTION 64.0f

#define     WHEEL_DIAMETER_M        0.125f
#define     WHEEL_RADIUS_M          WHEEL_DIAMETER_M / 2.0f

#define     PI_NUMBER               3.1415926535f

#define     RAD_TO_DEG              57.2958f

#define     PWM_MAX_DUTY            90  // [ % ]
#define     PWM_MIN_DUTY            5   // [ % ]

// TIMER CHANNELS ASSIGNED TO OUTPUTS
#define     PHASE_A                 1
#define     PHASE_B                 0
#define     PHASE_C                 2
#define     PHASE_D                 3
#define     PHASE_COUNT             4

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

// ----- COUNTERS -----
volatile    uint32_t    cHeartLED       = LED_HEART_ON_MS;
volatile    uint8_t     cHeartTick      = 0;
volatile    uint32_t    cMPUread        = TIME_MPU_READ_MS;
volatile    uint32_t    cMPUdelay       = TIME_MPU_DELAY_MS;
volatile    uint32_t    cSendData       = TIME_SEND_DATA_MS;
volatile    uint32_t    cMotorOmega     = TIME_MOTOR_OMEGA_MS;
volatile    uint32_t    cPIDcontroller  = TIME_PID_CONTROLLER_MS;

// ----- MOTOR QUADRATURE ENCODERS -----
volatile    int32_t     cQuadM0 = 0 , cQuadM0_1 = 0;
volatile    int32_t     cQuadM1 = 0 , cQuadM1_1 = 0;

// ----- ANALOG -----
volatile    uint32_t    adc[ ADC_CHANNELS ];    // [ LSB ]
volatile    float       volt[ ADC_CHANNELS ];   // [ V ]
volatile    float       MotorCurr_mA[ MOTOR_COUNT ];
volatile    float       MotorTemp_C[ MOTOR_COUNT ];
// AVG filter
volatile    float       volt_AVG[ ADC_CHANNELS ];
volatile    float       volt_Tmp[ ADC_CHANNELS ];

// ----- COMPLEMENTARY FILTER -----
volatile    float       Alpha = ALPHA_DEFAULT;

// ----- PID -----
volatile    float       T_sample            = TIME_PID_CONTROLLER_MS / 1000.0f;     // sample time [sec]

volatile    uint16_t    auxCounters[14];

// ----- I2C1 -----
#define I2C_TX_MAX_PACKET_SIZE (16)
// #define I2C_TX_PACKET_SIZE (16)
#define I2C_RX_MAX_PACKET_SIZE (16)
// #define I2C_RX_PACKET_SIZE (16)

// Data sent to the Target
uint8_t gTxPacket[ I2C_TX_MAX_PACKET_SIZE ] = { 0x00 , 0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0x0A , 0x0B , 0x0C , 0x0D , 0x0E , 0x0F };
// Counters for TX length and bytes sent
uint32_t gTxLen, gTxCount;
// Data received from Target
uint8_t gRxPacket[ I2C_RX_MAX_PACKET_SIZE ];
// Counters for TX length and bytes sent
uint32_t gRxLen, gRxCount;
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
volatile	uint8_t		MPU_switch = 0;
// raw data registers
volatile	int16_t	    MPU_temp = 0;
volatile	int16_t	    MPU_accX = 0, MPU_accY = 0, MPU_accZ = 0;
volatile	int16_t	    MPU_accX_raw = 0, MPU_accY_raw = 0, MPU_accZ_raw = 0;
volatile	int16_t	    MPU_gyroX = 0, MPU_gyroY = 0, MPU_gyroZ = 0;
volatile	int16_t	    MPU_gyroX_raw = 0, MPU_gyroY_raw = 0, MPU_gyroZ_raw = 0;
// ----- OFFSET COMPENSATION -----
volatile	 int16_t	MPU_accX_offset  =  450 , MPU_accY_offset  = -350 , MPU_accZ_offset  = 800; // 2025-03-15
volatile	 int16_t	MPU_gyroX_offset = -200 , MPU_gyroY_offset =    0 , MPU_gyroZ_offset =  75; // 2025-03-15
// calculated data
volatile    float       MPU_temp_C = 0.0f , MPU_accX_g = 0.0f , MPU_accY_g = 0.0f , MPU_accZ_g = 0.0f;
volatile    float		Acc_X_AngleDeg = 0.0f , Acc_Y_AngleDeg = 0.0f , Acc_Z_AngleDeg = 0.0f;
volatile	float		Gyro_X_RateDegs = 0.0f , Gyro_Y_RateDegs = 0.0f , Gyro_Z_RateDegs = 0.0f;
volatile    float       Pitch_AngleDeg = 0.0f , Pitch_AngleDegN_1 = 0.0f;
volatile    float       Roll_AngleDeg = 0.0f , Roll_AngleDegN_1 = 0.0f;
volatile    float       Yaw_AngleDeg = 0.0f , Yaw_AngleDegN_1 = 0.0f;
// AVG filter
#define     MOV_AVG_ANGLE_DEG 64
volatile    float       Pitch_AngleDeg_AVG = 0.0f , Pitch_AngleDeg_Tmp = 0.0f;
volatile    float       Roll_AngleDeg_AVG = 0.0f , Roll_AngleDeg_Tmp = 0.0f;
volatile    float       Yaw_AngleDeg_AVG = 0.0f , Yaw_AngleDeg_Tmp = 0.0f;

// ----- MOTOR/ROBOT -----
float   omega_M0_rads;   // [rad/sec]    motor Angular Velocity
float   omega_M1_rads;   // [rad/sec]     

float   theta_M0_rad;    // [rad]        motor Angule
float   theta_M1_rad;    // [rad]     

float   robot_pos_m;     // [m]
float   robot_pos_m_1;
float   robot_speed_ms;  // [m/sec]
float   robot_angle_rad; // [rad] 
float   robot_rate_rads; // [rad/s]

// ----- FUNCTIONS -----
void    MCU_Init( void );
void    ADC_check( void );
void    QUAD_check( void );
void    OMEGA_check( void );
void    MPU_check ( void );
void    MPU_ExitSleep( void );
void    MPU_GetAccTempGyro( void );
void    MPU_WhoAmI( void );
void    SET_PWM_DUTY( uint8_t argPhase , uint16_t argDuty );
// int     Send_6_integers_over_UART( int D0 , int D1 , int D2 , int D3 , int D4 , int D5 );
// float   PID0_classic( float arg_measurement );
// float   PID1_classic( float arg_measurement );
// float   PID2_classic( float arg_measurement );
// float   Kalman( float newAngle, float newRate, float dt );

// *****************************************  *****************************************  *****************************************
// *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****  *****   MAIN MAIN MAIN MAIN MAIN    *****
// *****************************************  *****************************************  *****************************************

int main( void )
{
    MCU_Init();

// *****************************************  *****************************************  *****************************************
// *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****  *****   INFINITE LOOP INFINITE LOOP *****
// *****************************************  *****************************************  *****************************************
    while ( 1 )
    {
        MPU_check();
        ADC_check();
        QUAD_check();
        OMEGA_check();
        DL_GPIO_togglePins( GPIO_PORT , GPIO_RC_PWM1_PIN );
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
        if ( LED_SEQUENCE & ( 1 << cHeartTick ) )
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
            adc[ 4 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_TEMP_M1 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            fADC[ 2 ] = true;
            adc[ 2 ] = DL_ADC12_getMemResult( ADC12_0_INST , ADC12_0_ADCMEM_CURR_M1 );
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
            adc[ 3 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_TEMP_M0 );
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            fADC[ 1 ] = true;
            adc[ 1 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_CURR_M0 );
            break;    
        case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            fADC[ 0 ] = true;
            adc[ 0 ] = DL_ADC12_getMemResult( ADC12_1_INST , ADC12_1_ADCMEM_VS1 );
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
    // ***** UART 3 - enable interrupt *****
    // NVIC_ClearPendingIRQ( UART_3_INST_INT_IRQN );
    // NVIC_EnableIRQ( UART_3_INST_INT_IRQN );
    // ***** I2C 1 - enable interrupt *****
    NVIC_EnableIRQ( I2C1_INST_INT_IRQN);
    // ***** default motor value - motor stop *****
    SET_PWM_DUTY( PHASE_A , 0U );       // MOTOR 0 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_B , 0U );       // MOTOR 0 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
    SET_PWM_DUTY( PHASE_C , 25U );       // MOTOR 1 FORWARDING ( QUAD COUNTER INCREMENT ) - RED  LED ACTIVE
    SET_PWM_DUTY( PHASE_D , 0U );       // MOTOR 1 REVERSING  ( QUAD COUNTER DECREMENT ) - BLUE LED ACTIVE
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
                volt[ i ] = (float)adc[ i ] * VOLT_REF * 11.0f / 4096.0f;   // 11 => divider ( 4k7 + 47k ) / 4k7
            else
                volt[ i ] = (float)adc[ i ] * VOLT_REF / 4096.0f;
            
            volt_Tmp[ i ] = volt_Tmp[ i ] - ( volt_Tmp[ i ] / MOV_AVG_ADC ) + volt[ i ];
            volt_AVG[ i ] = volt_Tmp[ i ] / MOV_AVG_ADC;

            if ( i == 1 )
                MotorCurr_mA[ 0 ] = 1000.0f * volt_AVG[ 1 ] / ( 0.033f * 20.0f );    // Motor_current[A] = V_sns[V] / R_sns[R] * INA_GAIN[-] -> Motor_cuureent = V_sns / 0.033R * 20
            if ( i == 2 )
                MotorCurr_mA[ 1 ] = 1000.0f * volt_AVG[ 2 ] / ( 0.033f * 20.0f );
        }
    }
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
            default:
                MPU_switch = 1;
                break;
            case 1:
                MPU_ExitSleep();
                MPU_switch = 2;
                break;
            case 2:
                MPU_WhoAmI(); // takes ~105us @400kHz
                if ( MPU_WhoAmI_data == 104 )
                    MPU_switch = 3;
                break;
            case 3:
                DL_GPIO_setPins( GPIO_PORT , GPIO_RC_PWM0_PIN );
                MPU_GetAccTempGyro(); // takes ~410us @400kHz
                DL_GPIO_clearPins( GPIO_PORT , GPIO_RC_PWM0_PIN );
                break;
        }
    }
}

void MPU_ExitSleep( void )
{
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 2;
    gTxPacket[ 0 ] = 0x6B;  // PWM_MGMT1 register
    gTxPacket[ 1 ] = 0x00;  // Exit Sleep    

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );
    // Send the packet to the controller. This function will send Start + Stop automatically.
    gI2cControllerStatus = I2C_STATUS_TX_STARTED;
    //
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;
    
    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    //
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;   
}

void MPU_WhoAmI( void )
{
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 1;
    gTxPacket[ 0 ] = 0x75;  // WHO_AM_I register 

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
 
    //
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;
  
    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    //
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ; 
  
    /* Wait until the Controller sends all bytes */
    // while ( ( gI2cControllerStatus != I2C_STATUS_TX_COMPLETE ) && ( gI2cControllerStatus != I2C_STATUS_ERROR ) )
    //     ;

    while ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS )
        ;

    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;

    //----- REPEAT START -----

    // Send a read request to Target
    gRxLen               = 1;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );

    // Wait for all bytes to be received in interrupt
    // z nejakeho duvodu se data prijmou ale sekne se to tady
    // while ( gI2cControllerStatus != I2C_STATUS_RX_COMPLETE )
    //     ;

    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ; 
    
    MPU_WhoAmI_data = gRxPacket[ 0 ];	// read one byte
}

void MPU_GetAccTempGyro( void )
{
// ACCEL_XOUT_H
    gI2cControllerStatus = I2C_STATUS_IDLE;
    gTxLen = 1;
    gTxPacket[ 0 ] = 0x3B;  // ACCEL_XOUT_H

    // Fill the FIFO. The FIFO is 8-bytes deep, and this function will return number of bytes written to FIFO
    gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );
 
    // Enable TXFIFO trigger interrupt if there are more bytes to send
    if ( gTxCount < gTxLen )
    {
        DL_I2C_enableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    } else {
        DL_I2C_disableInterrupt( I2C1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER );
    }

    //gTxCount = DL_I2C_fillControllerTXFIFO( I2C1_INST , &gTxPacket[ 0 ] , gTxLen );
    
    // Send the packet to the controller. This function will send Start + Stop automatically.
    gI2cControllerStatus = I2C_STATUS_TX_STARTED;

    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;

    // Send data to Slave - Write (0)
    DL_I2C_startControllerTransfer( I2C1_INST , MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen );
    //
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ; 

    /* Wait until the Controller sends all bytes */
    // while ( ( gI2cControllerStatus != I2C_STATUS_TX_COMPLETE ) && ( gI2cControllerStatus != I2C_STATUS_ERROR ) )
    //     ;

    while ( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS )
        ;

    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;

    //----- REPEAT START -----

    // Send a read request to Target
    gRxLen               = 14;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;
    // Send data to Slave - Request to read (1)
    DL_I2C_startControllerTransfer( I2C1_INST, MPU_ADDRESS , DL_I2C_CONTROLLER_DIRECTION_RX , gRxLen );
    
    // Wait for all bytes to be received in interrupt
    // z nejakeho duvodu se data prijmou ale sekne se to tady
    // while ( gI2cControllerStatus != I2C_STATUS_RX_COMPLETE )
    //     ;
    
    while ( !( DL_I2C_getControllerStatus( I2C1_INST ) & DL_I2C_CONTROLLER_STATUS_IDLE ) )
        ;

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
    Acc_X_AngleDeg = RAD_TO_DEG * atan2( MPU_accX_g , ( sqrt( MPU_accY_g*MPU_accY_g + MPU_accZ_g*MPU_accZ_g ) ) ); //
    Acc_Y_AngleDeg = RAD_TO_DEG * atan2( MPU_accY_g , ( sqrt( MPU_accX_g*MPU_accX_g + MPU_accZ_g*MPU_accZ_g ) ) ); //
    Acc_Z_AngleDeg = RAD_TO_DEG * atan2( ( sqrt( MPU_accX_g*MPU_accX_g + MPU_accY_g*MPU_accY_g ) ) , MPU_accZ_g ); //  this three lines takes 1.1ms@32MHz !!! 450us@80MHz
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
	
	Roll_AngleDeg  = ( 1 - Alpha ) * ( Roll_AngleDegN_1  + Gyro_X_RateDegs ) + Alpha * Acc_X_AngleDeg;
    Pitch_AngleDeg = ( 1 - Alpha ) * ( Pitch_AngleDegN_1 + Gyro_Y_RateDegs ) + Alpha * Acc_Y_AngleDeg;
    Yaw_AngleDeg   = ( 1 - Alpha ) * ( Yaw_AngleDegN_1   + Gyro_Z_RateDegs ) + Alpha * Acc_Z_AngleDeg;  // 3 lines 120us

    // re-shift
    Pitch_AngleDegN_1 = Pitch_AngleDeg;
    Roll_AngleDegN_1 = Roll_AngleDeg;
    Yaw_AngleDegN_1 = Yaw_AngleDeg;

// ***********************************
// ********** KALMAN FILTER **********
// *********************************** 
    // Pitch_AngleDeg_kalman = Kalman( Acc_X_AngleDeg , Gyro_X_RateDegs , TIME_MPU_READ_SEC );

// ********************************
// ********** AVG FILTER **********
// ******************************** 
    Pitch_AngleDeg_Tmp = Pitch_AngleDeg_Tmp - ( Pitch_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Pitch_AngleDeg;
	Pitch_AngleDeg_AVG = Pitch_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    Roll_AngleDeg_Tmp = Roll_AngleDeg_Tmp - ( Roll_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Roll_AngleDeg;
	Roll_AngleDeg_AVG = Roll_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    Yaw_AngleDeg_Tmp = Yaw_AngleDeg_Tmp - ( Yaw_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG ) + Yaw_AngleDeg;
	Yaw_AngleDeg_AVG = Yaw_AngleDeg_Tmp / MOV_AVG_ANGLE_DEG;

    robot_angle_rad = Pitch_AngleDeg_AVG / RAD_TO_DEG;
    robot_rate_rads = Gyro_X_RateDegs / RAD_TO_DEG;
}

// ----- OMEGA CALCULATION -----
void OMEGA_check( void )
{
    if ( fMotorOmega )
    {
        fMotorOmega = false;

        omega_M0_rads = -2.0f * PI_NUMBER * ( (float)( cQuadM0 - cQuadM0_1 ) ) * 1/TIME_MOTOR_OMEGA_S / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        theta_M0_rad  = -2.0f * PI_NUMBER * (float)( cQuadM0 ) / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        cQuadM0_1 = cQuadM0;
        cQuadM0_1 = cQuadM0;
        
        omega_M1_rads = 2.0f * PI_NUMBER * ( (float)( cQuadM1 - cQuadM1_1 ) ) * 1/TIME_MOTOR_OMEGA_S / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        theta_M1_rad  = 2.0f * PI_NUMBER * (float)( cQuadM1 ) / GEAR_RATIO / IMPULSES_PER_REVOLUTION;
        cQuadM1_1 = cQuadM1;
        cQuadM1_1 = cQuadM1;

        // Robot speed and position calculation
        robot_pos_m = 0.5f * ( theta_M0_rad + theta_M1_rad ) * WHEEL_RADIUS_M;
        robot_speed_ms = ( robot_pos_m - robot_pos_m_1 ) * 1/TIME_MOTOR_OMEGA_S;
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
   