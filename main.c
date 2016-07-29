/**
 * @file main.c
 * @author Andrew Powell
 * @date 29 July 2016
 * @brief This is the main source for Prototype 2 Version 1 Sac Sumo 2017 Robot
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
#include <stdlib.h>

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
#define CKPIN30                     IOPORT_E, BIT_4                     /* Pushbutton. */
#define CKPIN31                     IOPORT_E, BIT_5                     /* Pushbutton. */
#define CKPIN32                     IOPORT_E, BIT_6                     /* Pushbutton. */
#define CKPIN38                     IOPORT_F, BIT_6                     /* Interrupt 0. */
#define PB_TOTAL                    ( 3 )                               /* PB: Number of pushbuttons. */
#define I2C_HP_CHANNEL              ( I2C2 )                            /* I2C: High Power Channel ( for Tracking ). */
#define I2C_HP_CLK_FREQ             ( 400000 )                          /* I2C: Clock Speed for High Power Channel. */
#define I2C_LP_CHANNEL              ( I2C1 )                            /* I2C: Low Power Channel ( for everything else ). */
#define I2C_LP_CLK_FREQ             ( 400000 )                          /* I2C: Clock Speed for Low Power Channel. */    
#define IR_TOTAL_TRACK              ( CONTROL_I )                       /* IR: Total number IR Sensors used in the Tracking Algorithm. */
#define IR_TOTAL_BORDER             ( 2 )                               /* IR: Total number IR Sensors used in the Border Algorithm. */
#define IR_TOTAL                    ( IR_TOTAL_TRACK + IR_TOTAL_BORDER )/* IR: Total number IR Sensors used in the Tracking Algorithm. */
#define IR_EWMA_ALPHA               ( 0.25f )                           /* IR: Exponentially Weighted Moving Average Filter coefficient. */
#define IR_PROX_INVERT( x )         ( 0xffff - ( x ) )                  /* IR: Macro for inverting the raw value from IR Sensor. */
#define IR_CALIBRATE_DES_VAL        ( ( uint16_t ) -17660 )             /* IR: Value needed for calibration ( Default -- 17679 ) */
#define FRAM_IR_OFFSET_START        ( 0x0000 )                          /* FRAM: Address for IR Offset values. */
#define FRAM_IR_OFFSET_END          \
( ( FRAM_IR_OFFSET_START + sizeof( uint16_t ) * IR_TOTAL ) )            /* FRAM: End address for IR Offset values. */
#define FRAM_IR_TRACK_MAX_START     ( FRAM_IR_OFFSET_END )              /* FRAM: Address for maximum track value ( should be removed later ). */
#define FRAM_IR_TRACK_MAX_END       \
( FRAM_IR_TRACK_MAX_START + sizeof( float ) )                           /* FRAM: End address for maximum track value ( should be removed later ). */
#define FRAM_DATA_MAX               ( 0x10000 )                         /* FRAM: Maximum size of FRAM. */
#define MOTORS_A1RIGHT              IOPORT_E, BIT_1                     /* MOTORS: Direction pin for right motor. */
#define MOTORS_A2RIGHT              IOPORT_E, BIT_0                     /* MOTORS: Direction pin for right motor. */
#define MOTORS_A1LEFT               IOPORT_E, BIT_3                     /* MOTORS: Direction pin for left motor. */
#define MOTORS_A2LEFT               IOPORT_E, BIT_2                     /* MOTORS: Direction pin for left motor. */
#define MOTORS_ANGLE_VELO_MAG       ( M_PI / 2.0f )                     /* MOTORS: Maximum magnitude of angular velocity. */
#define MOTORS_COMPUTE_FOR( x )     \
( ( fabs( ( x ) ) < ( MOTORS_ANGLE_VELO_MAG / 5.0f ) ) ? 1.0f : 0.0f )  /* MOTORS: This macro computers the forward velocity. */                
#define MOTORS_LEFT_MIN             ( 155.0f )                          /* MOTORS: Minimum raw PWM value to left motor. */
#define MOTORS_RIGHT_MIN            ( 155.0f )                          /* MOTORS: Minimum raw PWM value to right motor. */
#define MOTORS_LEFT_MAX             ( 255.0f )                          /* MOTORS: Maximum raw PWM value to left motor. */
#define MOTORS_RIGHT_MAX            ( 255.0f )                          /* MOTORS: Maximum raw PWM value to right motor. */
#define MOTORS_LEFT_SCALAR          \
( MOTORS_LEFT_MAX - MOTORS_LEFT_MIN )                                   /* MOTORS: Normalizing scalar value needed for left motor. */
#define MOTORS_RIGHT_SCALAR         \
( MOTORS_RIGHT_MAX - MOTORS_RIGHT_MIN )                                 /* MOTORS: Normalizing scalar value needed for right motor. */
#define MOTORS_PER_FOR              ( 0.75f )                           /* MOTORS: How much forward velocity impacts speed of the motors as a percentage. */
#define MOTORS_PER_ANG              ( 1.0f )                            /* MOTORS: How much angular velocity impacts speed of the motors as a percentage. */
#define CONTROL_ITERS_MAX           ( 40 )                              /* CONTROL: Sets the maximum amount of iterations the gradient descent algorithm can execute. */
#define CONTROL_R                   ( 100.0f )                          /* CONTROL: Sets the radius of the robot in reference to the IR Sensors. */
#define CONTROL_CHECK               ( 0.00000000000001f )               /* CONTROL: Sets the break condition of the gradient descent algorithm. */
#define CONTROL_DESC                ( 0.000000000001f )                 /* CONTROL: Rate of update for the gradient descent algorithm. */
#define CONTROL_D                   ( 150.0f )                          /* CONTROL: Distance between each IR Sensor. */
#define CONTROL_E_TOTAL_MAX         ( MOTORS_ANGLE_VELO_MAG )           /* CONTROL: Maxmimum magnitude of the PID controller's total error. */
#define CONTROL_KP                  ( 20.00000f )                       /* CONTROL: Proportional constant of PID controller. */
#define CONTROL_KI                  ( 0.04000f )                        /* CONTROL: Integral constant of PID controller. */
#define CONTROL_KD                  ( 10.00000f )                       /* CONTROL: Derivative constant of PID controller. */
#define CONTROL_BT                  ( 1000.0f )                         /* BORDER: Threshold needed for border detection. */
#define CONTROL_OX_DETECT           ( 0.75f )                           /* CONTROL: Percent threshold of current and max distant needed for object detection ( should be removed later ). */
#define ROBOT_STATE_WAIT_MIN        ( 10 )                              /* ROBOT: Minimum wait time in process cycles. */
#define ROBOT_STATE_WAIT_MAX        ( 40 )                              /* ROBOT: Maximum wait time in process cycles. */
#define ROBOT_STATE_GET_WAIT( )     \
( ( rand() % ( ROBOT_STATE_WAIT_MAX - ROBOT_STATE_WAIT_MIN ) ) + ROBOT_STATE_WAIT_MIN ) /* ROBOT: Macro for randomly determining wait time. */
#define DEBUG_0_ENABLE              ( true )                            /* DEBUG: Enables debugging statements. */
#define DEBUG_0_ITERS_TOTAL         ( 20 )                              /* DEBUG: Number of samples necessary for a debug print to occur. */

/** @brief Defines the state of the robot. */
typedef struct RobotState
{
    /** @brief This enumeration defines the possible states
     the robot can be in. Please see the state machine implemented in the
     process task for more detailed information. */
    enum 
    {
        RobotState_IDLE_0,
        RobotState_CALIBRATE_0,
        RobotState_CALIBRATE_1,
        RobotState_CALIBRATE_2,
        RobotState_FORWARD_0,
        RobotState_BACKWARD_0,
        RobotState_TURN_0
    }
    state;
    
    /** @brief Each state may have data associated with it. */
    union
    {
        /** @brief Was used to filter calibration values ( should be removed later ). */
        struct
        {
            EWMA filt;          
        }
        calibrate;
        
        /** @brief Contains waiting data for the backward state. */
        struct
        {
            uint16_t count;     
            uint16_t finish;
        }
        backward;
        
        /** @brief Contains waiting data for the turning state. */
        struct
        {
            uint16_t count;
            uint16_t finish;
            bool left;
        }
        turn;
    }
    data;
}
RobotState;

/** @brief Determines the role of the IR Sensor. */
typedef enum IRSensorRole
{
    IRSensorRole_TRACK,
    IRSensorRole_BORDER
}
IRSensorRole;

/** @brief Determines how the pushbutton is referenced in hardware. */
typedef struct PbRef
{
    IoPortId portId;    /**< Refers to the port to which the pushbutton belongs. */
    unsigned int bits;  /**< Refers to the bit that corresponds to the pushbutton. */
}
PbRef;

/** @brief Defines the state of a pushbutton. */
typedef struct PbState
{
    uint8_t curr_state;
    uint8_t prev_state;
    uint8_t npresses;
}
PbState;

/** @brief Represents a pushbutton. */
typedef struct Pb
{
    PbRef ref;      /**< How the software refers to a particular pushbutton. */
    PbState state;  /**< The state of the pushbutton. */
}
Pb;

/** @brief Represents a set of pushbuttons. */
typedef struct PbArry
{
    Pb* pbs;                    /**< The actual array. */
    SemaphoreHandle_t mutex;    /**< Mutex is necessary to prevent user from reading incorrect data. */
}
PbArry;

/** @brief Represents a I2C Master Interface. */
typedef struct I2CChannel
{
    TCA9548A i2cmux;            /**< Each I2C Master Interface is connected to a I2C Mux. */
    I2C_MODULE channel;         /**< Represents the channel in the PIC32 I2C driver. */
    SemaphoreHandle_t mutex;    /**< Mutex is necessary to for thread-protection. */
}
I2CChannel;

/** @brief Represents a I2C Device. */
typedef struct I2CDevice
{
    I2CChannel* i2cchannel;     /**< The channel to which the I2C Device belongs. */
    int port;                   /**< The port on the I2C Mux to which the I2C Device belongs. */
}
I2CDevice;

/** @brief Extends the VCNL4010 and I2CDevice types. */
typedef struct IRSensor
{
    VCNL4010 obj;               /**< VNCL4010 object. */
    EWMA filt;                  /**< An Exponentially Weighted Moving Average filter is included to ensure noise is mitigated. */
    unsigned int timer_val;     /**< Timer value that is recorded as a way of measuring the number of core timer cycles between each sample. */
    uint16_t offset;            /**< Offset value is needed to ensure all the IR Sensors reflect similar values if an object is the same distance away from each sensor. */
    QueueHandle_t xDQueue;      /**< Queue is needed to synchronize the received value with the process thread. */
    IRSensorRole role;          /**< Specifies the role the sensor plays. */  
    int roleid;                 /**< An identifier needed for the role. */
    I2CDevice i2c;              /**< I2CDevice object. */
}
IRSensor;

/** @brief Extends the MB85RC256V and I2CDevice types. */
typedef struct FRAM
{
    MB85RC256V obj;             
    I2CDevice i2c;
}
FRAM;

/** @brief Prepares the PIC32. */
static void prvSetupHardware( void );

/** @brief Defines the task whose job is to directly sample the pushbuttons. */
static void prvTaskSamplePbs( void* );

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

/** @brief PbArry: Thread safe way by which the pushbuttons can be accessed. */
static PbState PbArryGetPbState( PbArry* ptr, int pbindex );

/*-----------------------------------------------------------*/

static SemaphoreHandle_t xIRSemphr;
static L293DNE motors;
static Control controller;
static Border bcontrol;
static RobotState robotstate = 
{
    .state = RobotState_IDLE_0,
    .data = { 0 }
};
static Pb pbs[ PB_TOTAL ] =
{
    { .ref = { CKPIN30 }, .state = { 0 } },
    { .ref = { CKPIN31 }, .state = { 0 } },
    { .ref = { CKPIN32 }, .state = { 0 } }
};
static PbArry pbarry =
{
    .pbs = pbs,
    .mutex = NULL
};
static I2CChannel i2cmux_hp =
{
    .i2cmux = { 0 },
    .channel = I2C_HP_CHANNEL,
    .mutex = NULL
};
static I2CChannel i2cmux_lp = 
{
    .i2cmux = { 0 },
    .channel = I2C_LP_CHANNEL,
    .mutex = NULL    
};
static FRAM fram =
{
    .obj = { 0 },
    .i2c = { .i2cchannel = &i2cmux_lp, .port = 7 }
};
static IRSensor ir_sensors[ IR_TOTAL ] = 
{
    { 
        .obj = { 0 }, 
        .filt = { 0 }, 
        .timer_val = 0, 
        .offset = 0, 
        .xDQueue = NULL, 
        .role = IRSensorRole_BORDER, 
        .roleid = 0, 
        .i2c = { .i2cchannel = &i2cmux_lp, .port = 2 } 
    },
    { 
        .obj = { 0 }, 
        .filt = { 0 }, 
        .timer_val = 0, 
        .offset = 0, 
        .xDQueue = NULL, 
        .role = IRSensorRole_BORDER, 
        .roleid = 1, 
        .i2c = { .i2cchannel = &i2cmux_lp, .port = 3 } 
    },
    { 
        .obj = { 0 }, 
        .filt = { 0 }, 
        .timer_val = 0, 
        .offset = 0, 
        .xDQueue = NULL, 
        .role = IRSensorRole_TRACK, 
        .roleid = 0, 
        .i2c = { .i2cchannel = &i2cmux_hp, .port = 2 } 
    },
    { 
        .obj = { 0 }, 
        .filt = { 0 }, 
        .timer_val = 0, 
        .offset = 0, 
        .xDQueue = NULL, 
        .role = IRSensorRole_TRACK, 
        .roleid = 1, 
        .i2c = { .i2cchannel = &i2cmux_hp, .port = 3 } 
    }
};

/*-----------------------------------------------------------*/

int main( void )
{
	uint32_t xReturned;

	/* Before starting anything, let's wait until all peripheral
	 * devices start before configuring anything.  */
	DelayMs(500);
    
    //while ( true );

	/* Prepare the hardware to run. */
	prvSetupHardware();

	/* Configure standard I/O via UART according to the chipKIT uC32. */
	vSTDIOSetup( UART1, configPERIPHERAL_CLOCK_HZ, 115200 );
    
    /* Configure digital I/O. */
	PORTSetPinsDigitalOut( LD4 );
	PORTSetBits( LD4 );
	PORTSetPinsDigitalOut( LD5 );
	PORTSetBits( LD5 );
    PORTSetPinsDigitalIn( CKPIN38 );

	/* Create tasks. */
    xReturned = xTaskCreate( prvTaskSendRequest, "tsk0", ( configMINIMAL_STACK_SIZE / 2 ),      
            NULL, tskIDLE_PRIORITY+1, NULL ); 
    xReturned = xTaskCreate( prvTaskServiceInt, "tsk1", ( configMINIMAL_STACK_SIZE / 2 ),       
            NULL, tskIDLE_PRIORITY, NULL );                         
    xReturned = xTaskCreate( prvTaskProcess, "tsk2", ( configMINIMAL_STACK_SIZE * 2 ),      
            NULL, tskIDLE_PRIORITY, NULL );    
    xReturned = xTaskCreate( prvTaskSamplePbs, "tsk3", configMINIMAL_STACK_SIZE,      
            NULL, tskIDLE_PRIORITY, NULL ); 
                         
    /* Create semaphores. */
    xIRSemphr =  xSemaphoreCreateBinary();
    xSemaphoreGive( xIRSemphr ); /* This is needed! */
    configASSERT( xIRSemphr != NULL );
    i2cmux_hp.mutex = xSemaphoreCreateMutex();
    configASSERT( i2cmux_hp.mutex != NULL );
    i2cmux_lp.mutex = xSemaphoreCreateMutex();
    configASSERT( i2cmux_lp.mutex != NULL );
    pbarry.mutex = xSemaphoreCreateMutex();
    configASSERT( pbarry.mutex != NULL );
            
    /* Configure interrupt for sensors. */
	/* It is really important to note the symbol configMAX_SYSCALL_INTERRUPT_PRIORITY
	 * in FreeRTOSConfig.h must be equal to or higher than the priority of any
	 * interrupt. */
	ConfigINT0( EXT_INT_PRI_3 | FALLING_EDGE_INT | EXT_INT_ENABLE );
    
    /* Configure I2C Channels. */
    xReturned = bI2CSetup( I2C_HP_CHANNEL, configPERIPHERAL_CLOCK_HZ, I2C_HP_CLK_FREQ );
	configASSERT( xReturned == TRUE );
    xReturned = bI2CSetup( I2C_LP_CHANNEL, configPERIPHERAL_CLOCK_HZ, I2C_LP_CLK_FREQ );
	configASSERT( xReturned == TRUE );
    
    /* Configure I2C Mux. */
    vTCA9548ASetup( &i2cmux_hp.i2cmux, false, false, false , 
        bTCA9548AI2CMethodDef, (void*) i2cmux_hp.channel );
    vTCA9548ASetup( &i2cmux_lp.i2cmux, false, false, false , 
        bTCA9548AI2CMethodDef, (void*) i2cmux_lp.channel );
    
    /* Configure FRAM. */
    {
        const size_t SIZE = 32;
        int each;
        uint8_t buff_0[ SIZE ];
        uint8_t buff_1[ SIZE ];
        
        /* Prepare the driver. */
        vMB85RC256VSetup( &fram.obj, false, false, false,
            bMB85RC256VI2WriteDef, bMB85RC256VI2CReadDef, (void*) &fram );
        
        /* Run quick test on hardware. */
        xReturned = vMB85RC256VRead( &fram.obj, 0x0000, buff_0, SIZE );
        configASSERT( xReturned == TRUE );
        xReturned = vMB85RC256VWrite( &fram.obj, 0x0000, buff_0, SIZE );
        configASSERT( xReturned == TRUE );
        xReturned = vMB85RC256VRead( &fram.obj, 0x0000, buff_1, SIZE );
        configASSERT( xReturned == TRUE );
        
        /* Confirm the data transmitted is the same as the data received. */
        for ( each=0; each < SIZE; each++ )
        {
            if ( buff_0[ each ] != buff_1[ each ] )
            {
                configASSERT( 0 );
            }
        }
    }
    
    /* Configure IR Sensors. */
    {
        int each_ir;
        for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
        {
            int current;
            bool status;
            IRSensor* ir_sensor = &ir_sensors[ each_ir ];
            
            /* Configure IR Sensor based on role. */
            switch ( ir_sensor->role )
            {
                case IRSensorRole_TRACK:
                {
                    current = VCNL4010_REG_IRLED_MAX_CURR;
                }
                break;
                case IRSensorRole_BORDER:
                {
                    current = VCNL4010_REG_IRLED_DEF_CURR;
                }
                break;
            }
            
            /* Configure IR Sensor. */
            vVCNL4010Setup( 
                    &ir_sensor->obj, 
                    bVCNL4010I2CWriteDef, 
                    bVCNL4010I2CReadDef, 
                    ( void* ) ir_sensor );
            xReturned = bVCNL4010SetIRCurrent( &ir_sensor->obj, current );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010SetProxRate( &ir_sensor->obj, VCNL4010_250mps );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010SetProxFreq( &ir_sensor->obj, VCNL4010_390p625kHz );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010IntSetup( &ir_sensor->obj, true );
            configASSERT( xReturned == true );
            xReturned = bVCNL4010IntHandler( &ir_sensor->obj, &status );
            configASSERT( xReturned == true );
            
            /* Configure the filter. */
            vEWMASetup( &ir_sensor->filt, IR_EWMA_ALPHA, 0 );
            
            /* Read offset values from memory. */
            xReturned = vMB85RC256VRead( &fram.obj, 
                    ( FRAM_IR_OFFSET_START + ( each_ir * sizeof( uint16_t ) ) ),  
                    &ir_sensor->offset, sizeof( uint16_t ) );
            configASSERT( xReturned == true );
            
            /* Prepare queue for storing received data. */
            ir_sensor->xDQueue = xQueueCreate( 1, sizeof( uint16_t ) );
            configASSERT( ir_sensor->xDQueue != NULL );
            ir_sensor->timer_val = ReadCoreTimer();
        }
    }
    
    /* Configure Tracking Algorithm. */
    {
        float Ox_max; /* Oh how was this useless. May not be worth to fix? */
        
        /* Load current maximum distance from non-volatile memory. */
        xReturned =  vMB85RC256VRead( &fram, 
            FRAM_IR_TRACK_MAX_START , 
            ( uint8_t*) &Ox_max, sizeof( float ) );
        configASSERT( xReturned == true );
        
        /* Configure the controller object itself. */
        vControlSetup( &controller, CONTROL_ITERS_MAX, CONTROL_R,
                CONTROL_CHECK, CONTROL_D, CONTROL_DESC, 
                CONTROL_E_TOTAL_MAX, CONTROL_KP, CONTROL_KI, CONTROL_KD,
                Ox_max, CONTROL_OX_DETECT );
        vBorderSetup( &bcontrol, CONTROL_BT );
    }
        
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
        
        /* Make sure motors are stopped. */
        vMotorsUpdate( &motors, 0.0f, 0.0f );
    }
    
    /* Configure pushbuttons. */
    {
        Pb* start = pbarry.pbs;
        Pb* end = start + PB_TOTAL;
        
        for ( ; start != end; start++ )
        {
            PbRef* ref = &start->ref;
            PbState* state = &start->state;
            
            PORTSetPinsDigitalIn( ref->portId, ref->bits );
            state->curr_state = ( PORTReadBits( ref->portId, ref->bits ) != 0 );
            state->prev_state = state->curr_state;
        }
    }
     
#if ( DEBUG_0_ENABLE == true )
    printf( "Application is configured. Starting the scheduler...\n" );
#endif
    
	/* Start the scheduler. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void prvTaskSamplePbs( void* param )
{
    ( void ) param;
    const portTickType xDelay = configTICK_RATE_HZ / 4;
    portTickType xLastWakeTime;
    
    /* Sample the pushbuttons. */
    xLastWakeTime = xTaskGetTickCount();
    while ( true )
    {
        /* Only sample four times a second. */
        vTaskDelayUntil( &xLastWakeTime, xDelay );
        {
            SemaphoreHandle_t mutex = pbarry.mutex;
            Pb* start = pbarry.pbs;
            Pb* end = pbarry.pbs + PB_TOTAL; 
            
            /* Other tasks shouldn't read the states of each pusbutton until the
             sampling process is completed. */
            xSemaphoreTake( mutex, portMAX_DELAY );
    
            /* Start sampling each pushbutton and update corresponding states. */
            for ( ; start != end; start++ )
            {
                uint8_t prev_state, curr_state;
                PbRef* ref = &start->ref;
                PbState* state = &start->state;
                
                prev_state = state->curr_state;
                curr_state = ( PORTReadBits( ref->portId, ref->bits ) != 0 );
                if ( ( curr_state != prev_state ) && ( curr_state ) )
                {
                    state->npresses++;
                }
                
                state->prev_state = prev_state;
                state->curr_state = curr_state;
            }
            
            /* Allow other tasks to check the states. */
            xSemaphoreGive( mutex );
        }
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
                    uint16_t prox_value;
                        
                    /* Read data from IR Sensor. */
                    success = bVCNL4010GetProx( &ir_sensor->obj, &prox_value );
                    configASSERT( success == true );
                    
                    /* Store selected IR sensor value in its respective queue. */
                    xQueueOverwrite( ir_sensor->xDQueue, &prox_value );
                }
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////// Process Task --- Robot operation is best viewed here. ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void prvTaskProcess( void* param )
{
    ( void ) param;
    int print_iters = 0;
    unsigned int prev_process_core_time = 0,curr_process_core_time;
    
    prev_process_core_time = ReadCoreTimer();
    while ( true )
    {
        float D [ IR_TOTAL_TRACK ];
        float B [ IR_TOTAL_BORDER ];
        PbState pbstate[ PB_TOTAL ];
        float theta_update, forward_update, Ox_update;
        bool detect_update;
        ControlBorder border_update;
        unsigned int prev_control_core_time, curr_control_core_time;
        int each_ir, each_pb;
        
        /* Wait until new data is available. */
        for ( each_ir = 0; each_ir < IR_TOTAL; each_ir++ )
        {
            uint16_t prox_value_ui;
            float prox_value_fl;
            IRSensor* ir_sensor = &ir_sensors[ each_ir ];
            
            /* Acquire new value. */
            xQueueReceive( ir_sensor->xDQueue, ( void* ) &prox_value_ui, portMAX_DELAY );
            
            /* Invert proximity value so that it increases with more distance. */
            prox_value_ui = IR_PROX_INVERT( prox_value_ui );
            
            /* Apply offset. However, make sure the new value is floored to zero
              if the proximity value is smaller than offset. */
            prox_value_ui = ( prox_value_ui < ir_sensor->offset ) ?
                0 :
                prox_value_ui - ( ir_sensor->offset );
            
            /* The integral value needs to be converted into floating-point for processing. */
            prox_value_fl = ( float ) prox_value_ui;
            
            /* If in calibration mode, set the offsets. */
            switch ( robotstate.state )
            {
                case RobotState_CALIBRATE_1:
                {
                    /* Why in the world did I think this was going to work? */
//                    /* Let's perform some filtration so a better offset can be determined.*/
//                    prox_value_ui = ( uint16_t ) fEWMARun( 
//                            &robotstate.data.calibrate.filt, 
//                            prox_value_fl );
                    
                    /* Determine offset. */
                    ir_sensor->offset += ( prox_value_ui + IR_CALIBRATE_DES_VAL );
                }
                break;
                case RobotState_CALIBRATE_2:
                {
                    bool success;
                    
                    /* Store offset in memory. */
                    success =  vMB85RC256VWrite( &fram, 
                            ( FRAM_IR_OFFSET_START + ( each_ir * sizeof( uint16_t ) ) ), 
                            ( uint8_t*) &ir_sensor->offset, sizeof( uint16_t ) );
                    configASSERT( success == true );
                }
                break;
                default: break;
            }
            
            /* Perform operation based on role of sensor. */
            switch ( ir_sensor->role )
            {
                case IRSensorRole_TRACK:
                {
                    /* Transform data so that it changes positively linearly. */
                    prox_value_fl = linear_transform_1( prox_value_fl ) ;
                    
                    /* Apply filter to mitigate noise. */
                    prox_value_fl = fEWMARun( &ir_sensor->filt, prox_value_fl );
                    
                    /* Store tracking value. */
                    D[ ir_sensor->roleid ] = prox_value_fl;
                }
                break;
                case IRSensorRole_BORDER:
                {
                    /* Apply filter to mitigate noise. */
                    prox_value_fl = fEWMARun( &ir_sensor->filt, prox_value_fl );
                    
                    /* Store border value. */
                    B[ ir_sensor->roleid ] = prox_value_fl;
                }
                break;
            }
        }
        
        /* Sample from the core timer. */
        curr_process_core_time = ReadCoreTimer();
        
        /* Carry out the control algorithm. */
        prev_control_core_time = ReadCoreTimer();
        vControlUpdate( &controller, D, &theta_update, &Ox_update, &detect_update );
        vBorderUpdate( &bcontrol, B, &border_update );
        forward_update = MOTORS_COMPUTE_FOR( theta_update );
        curr_control_core_time = ReadCoreTimer();
        
        /* Grab the state of each pushbutton. */
        for ( each_pb = 0; each_pb < PB_TOTAL; each_pb++ )
        {
            pbstate[ each_pb ] = PbArryGetPbState( &pbarry, each_pb );
        }
        
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        //////////// MOTHER OF ALL STATE MACHINES. WUHAHAH. ////////////
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        
        /* Perform operation based on the state of the robot. */
        switch ( robotstate.state )
        {
            /* The idle state is basically the "do-nothing" state. */
            case RobotState_IDLE_0:
            {
                /* Check for the calibrate condition. */
                if ( pbstate[ 1 ].curr_state && pbstate[ 2 ].curr_state )
                {
                    robotstate.state = RobotState_CALIBRATE_0;
                } 
                /* Check for start forward direction condition. */
                else if ( !pbstate[ 0 ].curr_state && pbstate[ 0 ].prev_state )
                {
                    robotstate.state = RobotState_FORWARD_0;
                }
                
                /* Motors should never move while idling. */
                vMotorsUpdate( &motors, 0.0f, 0.0f );
            }
            break;
            
            /* Due to the nature of the sensors, calibration is necessary. This
             is especially true if the robot needs to be used upon different surfaces. */
            case RobotState_CALIBRATE_0:
            {
                /* Don't start the calibration until the pushbuttons are deselected. */
                if ( !pbstate[ 1 ].curr_state && !pbstate[ 2 ].curr_state )
                {
                    /* Go to the state that performs the actual calibration. */
                    robotstate.state = RobotState_CALIBRATE_1;
                    
                    /* Initialize the filter for calibration. */
                    vEWMASetup( &robotstate.data.calibrate.filt, IR_EWMA_ALPHA, 0 );
                }
            }
            break;
            case RobotState_CALIBRATE_1:
            {
                /* Look for condition to end calibration mode. */
                if ( !pbstate[ 2 ].curr_state && pbstate[ 2 ].prev_state )
                {
                    robotstate.state = RobotState_CALIBRATE_2;
                }
            }
            break;
            case RobotState_CALIBRATE_2:
            {
                bool success;
                
                /* Update the maximum distance and store it in fram. */  
                success =  vMB85RC256VWrite( &fram, 
                    FRAM_IR_TRACK_MAX_START , 
                    ( uint8_t*) &Ox_update, sizeof( float ) );
                configASSERT( success == true );
                
                /* Go back to the idle state. */
                robotstate.state = RobotState_IDLE_0;
            }
            break;
            
            /* The forward state, for the current prototype, is the most critical state. It's
             the only state that consistently updates the state of the motors according to
             the Tracking Algorithm. */
            case RobotState_FORWARD_0:
            {
                /* Check for the idle condition. This condition has priority over
                 all other conditions. */
                if ( pbstate[ 2 ].curr_state )
                {
                    robotstate.state = RobotState_IDLE_0;
                }
                else
                {
                    switch ( border_update )
                    {
                        /* If the border is detected in the front, the robot needs
                         to begin moving backwards immediately and go into its backward state. */
                        case ControlBorder_BORDER_DETECTED_FRONT:
                        {
                            robotstate.state = RobotState_BACKWARD_0;
                            robotstate.data.backward.finish = ROBOT_STATE_GET_WAIT() - 1;
                            robotstate.data.backward.count = 0;
                            vMotorsUpdate( &motors, 0.0f, -1.0f );
                        }
                        break;
                        
                        /* If the border is either not detected or detected at
                         the back of the robot, simply perform the update to the
                         motors as normal. */
                        case ControlBorder_BORDER_DETECTED_BACK:
                        case ControlBorder_NO_BORDER_DETECTED:
                        {
                            vMotorsUpdate( &motors, theta_update, forward_update );
                        }
                        break;
                    }
                }
            }
            break;
            
            /* This state defines the robot's actions as it's moving backwards. */
            case RobotState_BACKWARD_0:
            {
                /* In this state, the conditions for idling still have priority
                 over any other condition. */
                if ( pbstate[ 2 ].curr_state )
                {
                    robotstate.state = RobotState_IDLE_0;
                }
                else
                {
                    switch ( border_update )
                    {
                        /* If the robot tries to cross the back, it should return to
                         its forward state. */
                        case ControlBorder_BORDER_DETECTED_BACK:
                        {
                            robotstate.state = RobotState_FORWARD_0;
                            vMotorsUpdate( &motors, theta_update, forward_update );
                        }
                        break;
                        
                        /* The robot should continue to go backwards until either the counter finishes
                         or something was detected. */
                        case ControlBorder_BORDER_DETECTED_FRONT:
                        case ControlBorder_NO_BORDER_DETECTED:
                        {
                            if ( detect_update )
                            {
                                robotstate.state = RobotState_FORWARD_0;
                                vMotorsUpdate( &motors, theta_update, forward_update );
                            }
                            else if ( robotstate.data.backward.count == robotstate.data.backward.finish )
                            {
                                robotstate.state = RobotState_TURN_0;
                                robotstate.data.turn.finish = ROBOT_STATE_GET_WAIT() - 1;
                                robotstate.data.turn.count = 0;
                                robotstate.data.turn.left = ( rand() % 2 ) ? true : false;
                                vMotorsUpdate( &motors, ( ( robotstate.data.turn.left ) ?
                                    MOTORS_ANGLE_VELO_MAG : ( -MOTORS_ANGLE_VELO_MAG ) ), 0.0f );
                            }
                            else
                            {
                                robotstate.data.backward.count++;
                                vMotorsUpdate( &motors, 0.0f, -1.0f );
                            }
                        }
                        break;
                    }
                }
            }   
            break;
            
            /* */
            case RobotState_TURN_0:
            {
                /* In this state, the conditions for idling still have priority
                 over any other condition. */
                if ( pbstate[ 2 ].curr_state )
                {
                    robotstate.state = RobotState_IDLE_0;
                }
                else
                {
                    switch ( border_update )
                    {
                        case ControlBorder_BORDER_DETECTED_BACK:
                        {
                            robotstate.state = RobotState_FORWARD_0;
                            vMotorsUpdate( &motors, theta_update, forward_update );
                        }
                        break;
                        case ControlBorder_BORDER_DETECTED_FRONT:
                        {
                            robotstate.state = RobotState_BACKWARD_0;
                            robotstate.data.backward.finish = ROBOT_STATE_GET_WAIT() - 1;
                            robotstate.data.backward.count = 0;
                            vMotorsUpdate( &motors, 0.0f, -1.0f );
                        }
                        break;
                        case ControlBorder_NO_BORDER_DETECTED:
                        {
                            if ( detect_update )
                            {
                                robotstate.state = RobotState_FORWARD_0;
                                vMotorsUpdate( &motors, theta_update, forward_update );
                            }
                            else if ( robotstate.data.turn.count == robotstate.data.turn.finish )
                            {
                                robotstate.state = RobotState_FORWARD_0;
                                vMotorsUpdate( &motors, theta_update, forward_update );
                            }
                            else
                            {
                                robotstate.data.backward.count++;
                                vMotorsUpdate( &motors, ( ( robotstate.data.turn.left ) ?
                                    MOTORS_ANGLE_VELO_MAG : ( -MOTORS_ANGLE_VELO_MAG ) ), 0.0f );
                            }
                        }
                        break;
                    }
                }
            }
            break;
        }
        
#if ( DEBUG_0_ENABLE == true )
        /* Print useful debug message to console. */
        if ( print_iters == ( DEBUG_0_ITERS_TOTAL ) )
        {
            print_iters = 0;
            printf( "Process Cycles:\t%u\n" 
                    "Control Cycles:\t%u\n"
                    "Theta Update:\t%g\n"
                    "Forward Update:\t%g\n"
                    "Ox_update:\t%g\n"
                    "detect_update:\t%d\n"
                    "border_update:\t%s\n",
                    ( curr_process_core_time - prev_process_core_time ),
                    ( curr_control_core_time - prev_control_core_time ),
                    ( theta_update / M_PI * 180.0 ), forward_update,
                    Ox_update, detect_update, 
                    ( ( border_update == ControlBorder_NO_BORDER_DETECTED ) ? "no border" :  
                        ( border_update == ControlBorder_BORDER_DETECTED_FRONT ) ? "front" : 
                            "back" ) );
            for ( each_ir = 0; each_ir < IR_TOTAL_TRACK; each_ir++ )
            {
                printf( "IR:\t\t%d\nD:\t\t%g\n",
                        each_ir, D[ each_ir ] );
            }
            for ( each_ir = 0; each_ir < IR_TOTAL_BORDER; each_ir++ )
            {
                printf( "IR:\t\t%d\nB:\t\t%g\n",
                        each_ir, B[ each_ir ] );
            }
            for ( each_pb = 0; each_pb < PB_TOTAL; each_pb++ )
            {
                PbState state = PbArryGetPbState( &pbarry, each_pb );
                printf( "PB:\t\t%d\ncs:\t\t%u\nnpresses:\t%u\n",
                        each_pb, state.curr_state, state.npresses );
            }
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
    FRAM* fram = ( FRAM* ) param;
    I2CDevice* i2c = &fram->i2c;
    I2CChannel* i2cchannel = i2c->i2cchannel;
    SemaphoreHandle_t mutex = i2cchannel->mutex;
    bool status;
    
    xSemaphoreTake( mutex, portMAX_DELAY );
    
    status = bTCA9548ASetPort( &i2cchannel->i2cmux, i2c->port ) && 
            bI2CWriteTrans( i2cchannel->channel, (BYTE)slave_addr, (BYTE*)data, len );
    
    xSemaphoreGive( mutex );
    
    return status;
}

bool bMB85RC256VI2CReadDef( uint8_t slave_addr, uint8_t* data, size_t len, void* param )
{
    FRAM* fram = ( FRAM* ) param;
    I2CDevice* i2c = &fram->i2c;
    I2CChannel* i2cchannel = i2c->i2cchannel;
    SemaphoreHandle_t mutex = i2cchannel->mutex;
    bool status;
    
    xSemaphoreTake( mutex, portMAX_DELAY );
    
    status =  bTCA9548ASetPort( &i2cchannel->i2cmux, i2c->port ) && 
            bI2CReadTrans( i2cchannel->channel, ( BYTE )slave_addr, ( BYTE* ) data, len );
    
    xSemaphoreGive( mutex );
    
    return status;
}

bool bTCA9548AI2CMethodDef( uint8_t slave_addr, uint8_t data, void* param )
{
    return bI2CWriteTrans( ( I2C_MODULE ) param, (BYTE) slave_addr, &data, 1 );
}

bool bVCNL4010I2CWriteDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char data, void* param )
{   
    IRSensor* sensor  = ( IRSensor* ) param;
    I2CDevice* i2c = &sensor->i2c;
    I2CChannel* i2cchannel = i2c->i2cchannel;
    SemaphoreHandle_t mutex = i2cchannel->mutex;
    BYTE buffer[] = { reg_addr, data };
    bool status;
    
    xSemaphoreTake( mutex, portMAX_DELAY );
    
    status = bTCA9548ASetPort( &i2cchannel->i2cmux, i2c->port ) && 
            bI2CWriteTrans( i2cchannel->channel, ( BYTE ) slave_addr, buffer, 2 );
    
    xSemaphoreGive( mutex );
    
	return status;
}

bool bVCNL4010I2CReadDef( unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, void* param )
{
    IRSensor* sensor  = ( IRSensor* ) param;
    I2CDevice* i2c = &sensor->i2c;
    I2CChannel* i2cchannel = i2c->i2cchannel;
    SemaphoreHandle_t mutex = i2cchannel->mutex;
    bool status;
    
    xSemaphoreTake( mutex, portMAX_DELAY );
    
	status = bTCA9548ASetPort( &i2cchannel->i2cmux, i2c->port ) && 
            bI2CWriteTrans( i2cchannel->channel, (BYTE)slave_addr, &reg_addr, 1 ) &&
            bI2CReadTrans( i2cchannel->channel, (BYTE)slave_addr, data, 1 );
    
    xSemaphoreGive( mutex );
    
    return status;
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

PbState PbArryGetPbState( PbArry* ptr, int pbindex )
{
    SemaphoreHandle_t mutex = ptr->mutex;
    PbState state;
    
    xSemaphoreTake( mutex, portMAX_DELAY );
    state = ptr->pbs[ pbindex ].state;
    xSemaphoreGive( mutex );
    
    return state;
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
