/**
 * @file main.c
 * @author Andrew Powell
 * @date 8 July 2016
 * @brief This is the main source for Prototype 1 Version 1 Sac Sumo 2017 Robot
 *
 * The link to the project's main page should added here at a later time.
 */
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard C includes. */
#include <plib.h>
#include <stdio.h>
#include <stdbool.h>

/* User-Defined includes. */
#include "ConfigPerformance.h"
#include "UARTExt.h"
#include "I2CExt.h"
#include "MB85RC256V.h"
#include "TCA9548A.h"
#include "VCNL4010.h"
#include "L293DNE.h"
#include "DSP.h"
#include "linear_transform_1.h"

/* Core configuration fuse settings */
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2
#pragma config CP = OFF, BWP = OFF, PWP = OFF

/* Additional config fuse settings for other supported processors */
#if defined(__32MX460F512L__)
	#pragma config UPLLEN = OFF
#elif defined(__32MX795F512L__)
	#pragma config UPLLEN = OFF
	#pragma config FSRSSEL = PRIORITY_7
#endif

#define LD4                         IOPORT_G, BIT_6                     /* Status LED. */
#define LD5                         IOPORT_F, BIT_0                     /* Status LED. */
#define CKPIN4                      IOPORT_F, BIT_1                     /* Pushbutton. */
#define CKPIN38                     IOPORT_F, BIT_6                     /* Interrupt 0. */
#define IR_I2C_CHANNEL              ( I2C1 )                            /* IR: I2C Channel. */
#define IR_I2C_CLK_FREQ             ( 400000 )                          /* IR: I2C Clock Frequency. */
#define IR_TOTAL                    ( CONTROL_I )                       /* IR: Total number IR Sensors used in the Tracking Algorithm. */
#define IR_EWMA_ALPHA               ( 0.9f )                            /* IR: Exponentially Weighted Moving Average Filter coefficient. */
#define IR_PROX_INVERT( x )         ( 0xffff - ( x ) )                  /* IR: Macro for inverting the raw value from IR Sensor. */
#define FRAM_I2C_CHANNEL            ( I2C2 )                            /* FRAM: I2C Channel. */
#define FRAM_I2C_CLK_FREQ           ( 400000 )                          /* FRAM: I2C Clock Frequency. */
#define FRAM_IR_OFFSET_ADDR         ( 0x0000 )                          /* FRAM: Address for IR Offset values. */
#define FRAM_DATA_MAX               ( 0x10000 )                         /* FRAM: Maximum size of FRAM. */
#define MOTORS_A1RIGHT              IOPORT_E, BIT_1                     /* MOTORS: Direction pin for right motor. */
#define MOTORS_A2RIGHT              IOPORT_E, BIT_0                     /* MOTORS: Direction pin for right motor. */
#define MOTORS_A1LEFT               IOPORT_E, BIT_3                     /* MOTORS: Direction pin for left motor. */
#define MOTORS_A2LEFT               IOPORT_E, BIT_2                     /* MOTORS: Direction pin for left motor. */
#define MOTORS_ANGLE_VELO_MAG       ( M_PI / 2.0f )                     /* MOTORS: Maximum magnitude of angular velocity. */
#define MOTORS_COMPUTE_FOR( x )     \
( ( fabs( ( x ) ) < ( MOTORS_ANGLE_VELO_MAG / 5.0f ) ) ? 1.0f : 0.0f )  /* MOTORS: This macro computers the forward velocity. */                
#define MOTORS_LEFT_MIN             ( 175.0f )                          /* MOTORS: Minimum raw PWM value to left motor. */
#define MOTORS_RIGHT_MIN            ( 175.0f )                          /* MOTORS: Minimum raw PWM value to right motor. */
#define MOTORS_LEFT_MAX             ( 255.0f )                          /* MOTORS: Maximum raw PWM value to left motor. */
#define MOTORS_RIGHT_MAX            ( 255.0f )                          /* MOTORS: Maximum raw PWM value to right motor. */
#define MOTORS_LEFT_SCALAR          \
( MOTORS_LEFT_MAX - MOTORS_LEFT_MIN )                                   /* MOTORS: Normalizing scalar value needed for left motor. */
#define MOTORS_RIGHT_SCALAR         \
( MOTORS_RIGHT_MAX - MOTORS_RIGHT_MIN )                                 /* MOTORS: Normalizing scalar value needed for right motor. */
#define MOTORS_PER_FOR              ( 0.5f )                            /* MOTORS: How much forward velocity impacts speed of the motors as a percentage. */
#define MOTORS_PER_ANG              ( 0.5f )                            /* MOTORS: How much angular velocity impacts speed of the motors as a percentage. */
#define CONTROL_ITERS_MAX           ( 40 )                              /* CONTROL: Sets the maximum amount of iterations the gradient descent algorithm can execute. */
#define CONTROL_R                   ( 100.0f )                          /* CONTROL: Sets the radius of the robot in reference to the IR Sensors. */
#define CONTROL_CHECK               ( 0.00000000000001f )               /* CONTROL: Sets the break condition of the gradient descent algorithm. */
#define CONTROL_DESC                ( 0.000000000001f )                 /* CONTROL: Rate of update for the gradient descent algorithm. */
#define CONTROL_D                   ( 150.0f )                          /* CONTROL: Distance between each IR Sensor. */
#define CONTROL_E_TOTAL_MAX         ( MOTORS_ANGLE_VELO_MAG )           /* CONTROL: Maxmimum magnitude of the PID controller's total error. */
#define CONTROL_KP                  ( 7.00000f )                        /* CONTROL: Proportional constant of PID controller. */
#define CONTROL_KI                  ( 0.10000f )                        /* CONTROL: Integral constant of PID controller. */
#define CONTROL_KD                  ( 3.00000f )                        /* CONTROL: Derivative constant of PID controller. */
#define DEBUG_0_ENABLE              ( true )                            /* DEBUG: Enables debugging statements. */
#define DEBUG_0_ITERS_TOTAL         ( 20 )                              /* DEBUG: Number of samples necessary for a debug print to occur. */

/** @brief Extends the VCNL4010 type. */
typedef struct IRSensor
{
    VCNL4010 obj;               /**< VNCL4010 object. */
    EWMA filt;                  /**< An Exponentially Weighted Moving Average filter is included to ensure noise is mitigated. */
    int port;                   /**< Distinguishes each IR Sensor with the TCA9548A I2C Mux. */
    unsigned int timer_val;     /**< Timer value that is recorded as a way of measuring the number of core timer cycles between each sample. */
    uint16_t offset;            /**< Offset value is needed to ensure all the IR Sensors reflect similar values if an object is the same distance away from each sensor. */
    QueueHandle_t xDQueue;      /**< Queue is needed to synchronize the received value with the process thread. */
}
IRSensor;

/** @brief Prepares the PIC32. */
static void prvSetupHardware( void );

/** @brief Defines the task that services interrupt 0. */
static void prvTaskServiceInt( void* );

/** @brief Defines the task that samples pushbuttons. */
static void prvTaskSampleButton( void* );

/** @brief Defines the task that sends the request to the IR Sensors @ 20 Hz. */
static void prvTaskSendRequest( void* param );

/** @brief Defines the task that performs the processing. */
static void prvTaskProcess( void* param );

/** @brief Delays in milliseconds. Rely on FreeRTOS task delays if scheduler is running. */
static void DelayMs(unsigned int msec);

/** @brief TCA9548A: I2C Write method definition. */
static bool bTCA9548AI2CMethodDef( uint8_t slave_addr, uint8_t data, void* param );

/** @brief VCNL4010: I2C Write method definition. */
static bool bVCNL4010I2CWriteDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char data, void* param );

/** @brief VCNL4010: I2C Read method definition. */
static bool bVCNL4010I2CReadDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, void* param );

/** @brief MB85RC256V: I2C Write method definition. */
static bool bMB85RC256VI2WriteDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param );

/** @brief MB85RC256V: I2C Read method definition. */
static bool bMB85RC256VI2CReadDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param );

/** @brief L293DN: Set pin definition. */
static void vL293DNSetPinDef( L293DNEPin pin, bool state, void* param );

/** @brief L293DN: Set motor speed definition. */
static void vL293DNSetMotorDef( L293DNEMotor motor, uint8_t speed, void* param );

/** @brief L293DN: Updates motors based on angular and forward velocity. */
static void vMotorsUpdate( L293DNE* ptr, float angle, float forward );

/*-----------------------------------------------------------*/

static TCA9548A i2cmux;
static L293DNE motors;
static MB85RC256V fram;
static Control controller;
static SemaphoreHandle_t xIRSemphr;
static IRSensor ir_sensors[ IR_TOTAL ] = 
{
    { .obj = { 0 }, .port = 0, .timer_val = 0, .offset = 45467, .xDQueue = NULL },  
    { .obj = { 0 }, .port = 1, .timer_val = 0, .offset = 45642, .xDQueue = NULL } 
};
static volatile portBASE_TYPE xBtnState;

/*-----------------------------------------------------------*/

int main( void )
{
	uint32_t xReturned;

	/* Before starting anything, let's wait until all peripheral
	 * devices start before configuring anything.  */
	DelayMs(500);

	/* Prepare the hardware to run. */
	prvSetupHardware();

	/* Configure standard I/O via UART according to the chipKIT uC32. */
	vSTDIOSetup( UART1, configPERIPHERAL_CLOCK_HZ, 115200 );
    
    /* Configure digital I/O. */
	PORTSetPinsDigitalOut( LD4 );
	PORTSetBits( LD4 );
	PORTSetPinsDigitalOut( LD5 );
	PORTSetBits( LD5 );
	PORTSetPinsDigitalIn( CKPIN4 );
    PORTSetPinsDigitalIn( CKPIN38 );

	/* Create tasks. */
    xReturned = xTaskCreate( prvTaskServiceInt, "tsk0", configMINIMAL_STACK_SIZE,       
            NULL, tskIDLE_PRIORITY, NULL );                         
    xReturned = xTaskCreate( prvTaskProcess, "tsk1", configMINIMAL_STACK_SIZE,      
            NULL, tskIDLE_PRIORITY, NULL );                   
    xReturned = xTaskCreate( prvTaskSampleButton, "tsk2", configMINIMAL_STACK_SIZE,     
            NULL, tskIDLE_PRIORITY, NULL );                   
    xReturned = xTaskCreate( prvTaskSendRequest, "tsk3", configMINIMAL_STACK_SIZE,      
            NULL, tskIDLE_PRIORITY, NULL );                      
    
    /* Create semaphores. */
    xIRSemphr =  xSemaphoreCreateBinary();
    configASSERT( xIRSemphr != NULL );
    
    /* Initialize the state of the button. */
    xBtnState = PORTReadBits( CKPIN4 );
            
    /* Configure interrupt. */
	/* It is really important to note the symbol configMAX_SYSCALL_INTERRUPT_PRIORITY
	 * in FreeRTOSConfig.h must be equal to or higher than the priority of any
	 * interrupt. */
	ConfigINT0( EXT_INT_PRI_3 | FALLING_EDGE_INT | EXT_INT_ENABLE );
    
    /* Configure I2C Channels. */
    xReturned = bI2CSetup( IR_I2C_CHANNEL, configPERIPHERAL_CLOCK_HZ, IR_I2C_CLK_FREQ );
	configASSERT( xReturned == TRUE );
    xReturned = bI2CSetup( FRAM_I2C_CHANNEL, configPERIPHERAL_CLOCK_HZ, FRAM_I2C_CLK_FREQ );
	configASSERT( xReturned == TRUE );
    
    /* Configure I2C Mux. */
    vTCA9548ASetup( &i2cmux, false, false, false , 
        bTCA9548AI2CMethodDef, NULL );
    
    /* Configure FRAM. */
    vMB85RC256VSetup( &fram, false, false, false,
        bMB85RC256VI2WriteDef, bMB85RC256VI2CReadDef, NULL );
    
    /* Configure IR Sensors. */
    {
        int each_ir;
        for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
        {
            IRSensor* ir_sensor = &ir_sensors[ each_ir ];
            
            /* Configure IR Sensor. */
            vVCNL4010Setup( 
                    &ir_sensor->obj, 
                    bVCNL4010I2CWriteDef, 
                    bVCNL4010I2CReadDef, 
                    (void*)ir_sensor->port );
            xReturned = bVCNL4010SetIRCurrent( &ir_sensor->obj, VCNL4010_REG_IRLED_MAX_CURR );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010SetProxRate( &ir_sensor->obj, VCNL4010_250mps );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010SetProxFreq( &ir_sensor->obj, VCNL4010_390p625kHz );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010IntSetup( &ir_sensor->obj, true );
            configASSERT( xReturned == true );
            
            /* Configure the filter. */
            vEWMASetup( &ir_sensor->filt, IR_EWMA_ALPHA, 0 );
            
            /* Read offset values from memory. */
            xReturned = vMB85RC256Read( &fram, 
                    ( FRAM_IR_OFFSET_ADDR + ( each_ir * sizeof( float ) ) ),  
                    &ir_sensor->offset, sizeof( float ) );
            configASSERT( xReturned == true );
            
            /* Prepare queue for storing received data. */
            ir_sensor->xDQueue = xQueueCreate( 1, sizeof( float ) );
            configASSERT( ir_sensor->xDQueue != NULL );
            ir_sensor->timer_val = ReadCoreTimer();
        }
    }
    
    /* Configure Tracking Algorithm. */
    vControlSetup( &controller, CONTROL_ITERS_MAX, CONTROL_R,
            CONTROL_CHECK, CONTROL_D, CONTROL_DESC, 
            CONTROL_E_TOTAL_MAX, CONTROL_KP, CONTROL_KI, CONTROL_KD );
    
    /* Configure motors. */
    {
        /* Configure Digital I/O. */
        PORTSetPinsDigitalOut( MOTORS_A1LEFT );
        PORTClearBits( MOTORS_A1LEFT );
        PORTSetPinsDigitalOut( MOTORS_A2LEFT );
        PORTClearBits( MOTORS_A2LEFT );
        PORTSetPinsDigitalOut( MOTORS_A1RIGHT );
        PORTClearBits( MOTORS_A1RIGHT );
        PORTSetPinsDigitalOut( MOTORS_A2RIGHT );
        PORTClearBits( MOTORS_A2RIGHT );
        
        /* Configure Timers for PWM. */
        OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // PIN3 LEFT_MOTOR
        OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0x00FF);
        OpenOC2( OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // PIN5 RIGHT_MOTOR
        OpenTimer3( T3_ON | T3_PS_1_1 | T3_SOURCE_INT, 0x00FF);
        
        /* Configure motors object. */
        vL293DNESetup( &motors, vL293DNSetPinDef, vL293DNSetMotorDef, NULL );
    }  
     
#if ( DEBUG_0_ENABLE == true )
    printf( "Application is configured. Starting the scheduler...\n" );
#endif
    
	/* Start the scheduler. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void prvTaskSampleButton( void* param )
{
    ( void ) param;
    const portTickType xDelay = configTICK_RATE_HZ / 4;
    portTickType xLastWakeTime;
    
    xLastWakeTime = xTaskGetTickCount();
    while ( true )
    {
        vTaskDelayUntil( &xLastWakeTime, xDelay );
        xBtnState = PORTReadBits( CKPIN4 );
    }
}

void prvTaskSendRequest( void* param )
{
    ( void ) param;
    const portTickType xDelay = configTICK_RATE_HZ / 20;
    portTickType xLastWakeTime;
    
    /* Request new values from sensors at a rate of 20 Hz. */
    xLastWakeTime = xTaskGetTickCount();
    while ( true )
    {
        int each_ir;
        
        vTaskDelayUntil( &xLastWakeTime, xDelay );
        
        for ( each_ir = 0 ; each_ir < IR_TOTAL; each_ir++ )
        {
            bool success;
            IRSensor* ir_sensor = &ir_sensors[ each_ir ];
            
            success = bVCNL4010Request( &ir_sensor->obj );
            configASSERT( success == true );
        }
    }
}

void prvTaskServiceInt( void* param )
{
    ( void ) param;
    int each_ir;
    bool success;
    
    /* The following loop should execute indefinitely. */
    while ( true )
    {
        /* Suspend task until interrupt is triggered. */
        xSemaphoreTake( xIRSemphr, portMAX_DELAY  );
        
        /* Keep checking each sensor until interrupt is de-asserted. */
        while ( !PORTReadBits( CKPIN38 ) )
        { 
            for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
            {
                bool status;
                IRSensor* ir_sensor = &ir_sensors[ each_ir ];
                
                /* Check interrupt. */
                success = bVCNL4010IntHandler( &ir_sensor->obj, &status );
                configASSERT( success == true );
                
                /* If data is available, acquire it and print it to the console. */
                if ( status == true ) 
                {
                    uint16_t prox_value_ui;
                    float prox_value_fl;
                        
                    /* Read data from IR Sensor. */
                    success = bVCNL4010GetProx( &ir_sensor->obj, &prox_value_ui );
                    configASSERT( success == true );
                    
                    /* Transform data so that it changes positively linearly. */
                    prox_value_fl = linear_transform_1( ( float ) 
                            ( IR_PROX_INVERT( prox_value_ui ) - 
                            ( ir_sensor->offset ) ) ) ;
                    
                    /* Apply filter to mitigate noise. */
                    prox_value_fl = fEWMARun( &ir_sensor->filt, prox_value_fl );

                    /* Store selected IR sensor value in its respective queue. */
                    xQueueOverwrite( ir_sensor->xDQueue, &prox_value_fl );
                }
            }
        }
    }
}

void prvTaskProcess( void* param )
{
    ( void ) param;
    int print_iters = 0;
    unsigned int prev_process_core_time = 0,curr_process_core_time;
    
    prev_process_core_time = ReadCoreTimer();
    while ( true )
    {
        float D [ IR_TOTAL ];
        float theta_update, forward_update;
        unsigned int prev_control_core_time,curr_control_core_time;
        unsigned int prev_motor_update_time,curr_motor_update_time;
        int each_ir;
        
        /* Wait until new data is available. */
        for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
        {
            IRSensor* ir_sensor = &ir_sensors[ each_ir ];
            xQueueReceive( ir_sensor->xDQueue, 
                    ( void* ) &D[ ( ( IR_TOTAL-1 ) - each_ir ) ], 
                    portMAX_DELAY );
        }
        
        /* Sample from the core timer. */
        curr_process_core_time = ReadCoreTimer();
        
        /* Carry out the control algorithm. */
        prev_control_core_time = ReadCoreTimer();
        vControlUpdate( &controller, D, &theta_update );
        forward_update = MOTORS_COMPUTE_FOR( theta_update );
        curr_control_core_time = ReadCoreTimer();
        
        /* Update motors if enabled. */
        prev_motor_update_time = ReadCoreTimer();
        if ( xBtnState )
        {
            vMotorsUpdate( &motors, theta_update, forward_update );
        }
        else
        {
            vMotorsUpdate( &motors, 0.0f, 0.0f );
        }
        curr_motor_update_time = ReadCoreTimer();
        
#if ( DEBUG_0_ENABLE == true )
        /* Print useful debug message to console. */
        if ( print_iters == ( DEBUG_0_ITERS_TOTAL ) )
        {
            print_iters = 0;
            printf( "Process Cycles:\t%u\nControl Cycles:\t%u\n"
                    "Motor Cycles:\t%u\nTheta Degrees:\t%g\nForward Speed:\t%g\n",
                    ( curr_process_core_time - prev_process_core_time ),
                    ( curr_control_core_time - prev_control_core_time ),
                    ( curr_motor_update_time - prev_motor_update_time ),
                    ( theta_update * 180.0f / M_PI ),
                    forward_update );
            for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
            {
                printf( "IR:\t\t%d\nD:\t\t%g\n",
                        each_ir, D[ each_ir ] );
            }
            PORTToggleBits( LD4 );
        }
        else
        {
            print_iters++;
        }
#endif
        
        /* Store current process core timer value. */
        prev_process_core_time = curr_process_core_time;
    }
}

/*----------------------------------------------------------*/

void __ISR( _EXTERNAL_0_VECTOR, IPL3 ) INT0Interrupt()
{
	portBASE_TYPE xYieldRequired = pdFALSE;

    /* Clear interrupt flag. */
    mINT0ClearIntFlag();
    
	/* Service interrupt with task */
	xSemaphoreGiveFromISR( xIRSemphr, &xYieldRequired );
	portEND_SWITCHING_ISR( xYieldRequired );
}

bool bMB85RC256VI2WriteDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param )
{
    (void) param;
    return bI2CWriteTrans( FRAM_I2C_CHANNEL, (BYTE)slave_addr, (BYTE*)data, len );
}

bool bMB85RC256VI2CReadDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param )
{
    (void) param;
    return bI2CReadTrans( FRAM_I2C_CHANNEL, (BYTE)slave_addr, (BYTE*)data, len );
}

bool bTCA9548AI2CMethodDef( uint8_t slave_addr, uint8_t data, void* param )
{
    ( void ) param;
    
    return bI2CWriteTrans( IR_I2C_CHANNEL, (BYTE)slave_addr, &data, 1 );
}

bool bVCNL4010I2CWriteDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char data, void* param )
{
    int port  = (int)param;
    BYTE buffer[] = { reg_addr, data };
    
	return  bTCA9548ASetPort( &i2cmux, port ) && 
            bI2CWriteTrans( IR_I2C_CHANNEL, (BYTE)slave_addr, buffer, 2 );
}

bool bVCNL4010I2CReadDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, void* param )
{
    int port  = (int)param;
    
	return  bTCA9548ASetPort( &i2cmux, port ) && 
            bI2CWriteTrans( IR_I2C_CHANNEL, (BYTE)slave_addr, &reg_addr, 1 ) &&
            bI2CReadTrans( IR_I2C_CHANNEL, (BYTE)slave_addr, data, 1 );
}

void vL293DNSetPinDef( L293DNEPin pin, bool state, void* param )
{
    ( void ) param;
    
    if ( state )
    {
        switch( pin )
        {
        case L293DNE_A1Left:    PORTSetBits( MOTORS_A1LEFT ); break;
        case L293DNE_A2Left:    PORTSetBits( MOTORS_A2LEFT ); break;
        case L293DNE_A1Right:   PORTSetBits( MOTORS_A1RIGHT ); break;
        case L293DNE_A2Right:   PORTSetBits( MOTORS_A2RIGHT ); break;
        }
    }
    else
    {
        switch( pin )
        {
        case L293DNE_A1Left:    PORTClearBits( MOTORS_A1LEFT ); break;
        case L293DNE_A2Left:    PORTClearBits( MOTORS_A2LEFT ); break;
        case L293DNE_A1Right:   PORTClearBits( MOTORS_A1RIGHT ); break;
        case L293DNE_A2Right:   PORTClearBits( MOTORS_A2RIGHT ); break;
        }
    }
}

void vL293DNSetMotorDef( L293DNEMotor motor, uint8_t speed, void* param )
{
    ( void ) param;
    
    switch ( motor )
    {
    case L293DNE_Left:  SetDCOC1PWM( speed ); break;
    case L293DNE_Right: SetDCOC2PWM( speed ); break;
    }
}

void vMotorsUpdate( L293DNE* ptr, float angle, float forward )
{
    L293DNEMotorState* leftMtr;
    L293DNEMotorState* rightMtr;
    float left, right;
    
    /* Retrieve pointers to each motor. */
    leftMtr = ptrL293DNEGetMotorState( ptr, L293DNE_Left );
    rightMtr = ptrL293DNEGetMotorState( ptr, L293DNE_Right );
    
    /* Limit the angle. */
    if ( angle > MOTORS_ANGLE_VELO_MAG )
    {
        angle = MOTORS_ANGLE_VELO_MAG;
    }
    else if ( angle < ( -MOTORS_ANGLE_VELO_MAG ) )
    {
        angle = ( -MOTORS_ANGLE_VELO_MAG );
    }
    
    /* Compute values for motors. */
    forward *= MOTORS_PER_FOR;
    angle *= ( MOTORS_PER_ANG / MOTORS_ANGLE_VELO_MAG );
    left = forward - angle;
    right = forward + angle;
    
    /* Determine directions. */
    leftMtr->forward = ( left > 0.0f ) ? true : false;
    rightMtr->forward = ( right > 0.0f ) ? true : false;
    
    /* Set the magnitudes. */
    left = ( ( fabs( left ) * MOTORS_LEFT_SCALAR ) + MOTORS_LEFT_MIN );
    leftMtr->speed = ( uint8_t ) ( left > MOTORS_LEFT_MAX ) ? MOTORS_LEFT_MAX : left;
    right = ( ( fabs( right ) * MOTORS_RIGHT_SCALAR ) + MOTORS_RIGHT_MIN );
    rightMtr->speed = ( uint8_t ) ( right > MOTORS_RIGHT_MAX ) ? MOTORS_RIGHT_MAX : right;
    
    /* Finally, update the motors. */
    vL293DNEUpdateMotors( ptr );
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}

void DelayMs(unsigned int msec)
{
 unsigned int tWait, tStart;

 tWait=(configCPU_CLOCK_HZ/2000)*msec;        //    SYS_FREQ        (80000000)
 tStart=ReadCoreTimer();
 while((ReadCoreTimer()-tStart)<tWait);        // wait for the time to pass

}
