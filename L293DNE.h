/**
 * @file L293DNE.h
 * @author Andrew Powell
 * @date 12 July 2016
 * @brief Platform independent library for the L293DNE. 
 *
 * This library can be used to control the L293DNE motor controller for the purpose
 * of driving two DC motors.
 * 
 * @see http://www.ti.com/lit/ds/symlink/l293.pdf
 */

#ifndef L293DNE_H_   
#define L293DNE_H_

/* Standard C includes. */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    
    /** @brief Pins related to controlling the polarity of the two DC motors. */
    typedef enum L293DNEPin
    {
        L293DNE_A1Left,
        L293DNE_A2Left,
        L293DNE_A1Right,
        L293DNE_A2Right
    }
    L293DNEPin;
    
    /** @brief Distinguishes between the two motors. */
    typedef enum L293DNEMotor
    {
        L293DNE_Left,
        L293DNE_Right
    }
    L293DNEMotor;
    
    /** @brief Holds the current state of a motor. */
    typedef struct L293DNEMotorState
    {
        uint8_t speed;      /**< Sets the speed of the motor. */
        bool forward;       /**< Sets the polarity of the motor. */
    }
    L293DNEMotorState;
    
    /**
     * @brief Undefined function for setting the polarity pins ( i.e. A ).
     * 
     * The user must define this function according to the platform. The 
     * definition should somewhat follow the pseudo code below. 
     * 
     * @code{.c}
     * void vL293DNSetPinDef( L293DNEPin pin, bool state, void* param )
     * {
     * int pin_val;
     * switch ( pin )
     * {
     * case L293DNE_A1Right:
     * pin_val = MOTOR_A1RIGHT;
     * break;
     * case L293DNE_A2Right:
     * pin_val = MOTOR_A2RIGHT;
     * break;
     * case L293DNE_A1Left:
     * pin_val = MOTOR_A1LEFT;
     * break;
     * case L293DNE_A2Left:
     * pin_val = MOTOR_A2LEFT;
     * break;
     * }
     * SET_PIN_STATE( pin_val, ( state ) ? HIGH : LOW );
     * }
     * @endcode
     * 
     * @param pin The polarity pin.
     * @param state The new state of the selected polarity pin.
     * @param param A user-define value specified in vL293DNESetup.
     */
    typedef void ( *vL293DNSetPin )( L293DNEPin pin, bool state, void* param );
    
    /**
     * @brief Undefined function for setting the speed of the motors via the EN pins.
     * 
     * The user must define this function according to the platform. The 
     * definition should somewhat follow the pseudo code below. 
     * 
     * @code{.c}
     * void vL293DNSetMotorDef( L293DNEMotor motor, uint8_t speed, void* param )
     * {
     * switch ( motor )
     * {
     * case L293DNE_Left:
     * PWM( MOTOR_LEFT_EN, CONVERT_TO_DUTY( speed ) );
     * break;
     * case L293DNE_right:
     * PWM( MOTOR_RIGHT_EN, CONVERT_TO_DUTY( speed ) );
     * break;
     * }
     * }
     * @endcode
     * 
     * @param motor The selected motor.
     * @param speed Value presenting the duty cycle. Ranges from 0 to 255.
     * @param param A user-define value specified in vL293DNESetup.
     */
    typedef void ( *vL293DNSetMotor )( L293DNEMotor motor, uint8_t speed, void* param );
    
    /** @brief L293DNE object. */
    typedef struct L293DNE
    {
        vL293DNSetPin setpin_ptr;       /**< Pointed to user-defined vL293DNSetPin. */
        vL293DNSetMotor setmotor_ptr;   /**< Pointed to user-defined vL293DNSetMotor. */
        void* param;                    /**< User specified value that is passed into the functions related to setting the A and EN pins. */
        L293DNEMotorState left, right;  /**< Objects that hold the state of each motor. */
    }
    L293DNE;

    /**
     * @brief Configures L293DNE object.
     * 
     * @param ptr The L293DNE object.
     * @param setpin_ptr Pointer to user-defined vL293DNSetPin. This will be platform dependent. 
     * @param setmotor_ptr Pointer to user-defined vL293DNSetMotor. This will be platform dependent. 
     * @param param A user-define value that is passed into the user-defined vL293DNSetPin and 
     * vL293DNSetMotor functions. 
     * @see See the type definitions for vL293DNSetPin and vL293DNSetMotor to learn 
     * the structure of those undefined functions. 
     */
    void vL293DNESetup( L293DNE* ptr, vL293DNSetPin setpin_ptr, vL293DNSetMotor setmotor_ptr, void* param );
    
    /**
     * @brief Update motors based on their current states.
     * @param ptr The L293DNE object.
     */
    void vL293DNEUpdateMotors( L293DNE* ptr );
    
    /**
     * @brief Returns the selected state of a motor.
     * 
     * Each motor, left and right, has a state that is represented with the 
     * L293DNEMotorState structure. ptrL293DNEGetMotorState returns a state which
     * can be modified by the user. When vL293DNEUpdateMotors is called, the
     * L293DNE is updated based on these states. 
     * 
     * @param ptr The L293DNE object.
     * @param motor Selects the motor whose state is returned. 
     * @return A pointer to the L293DNEMotorState object that represents the state
     * of the selected motor. 
     */
    L293DNEMotorState* ptrL293DNEGetMotorState( L293DNE* ptr, L293DNEMotor motor );
    
#ifdef __cplusplus
}
#endif

#endif /* L293DNE_H_ */

