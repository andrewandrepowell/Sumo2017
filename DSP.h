/**
 * @file DSP.h
 * @author Andrew Powell
 * @date 12 July 2016
 * @brief Contains the DSP and other algorithms related to the Sumo Robot.
 *
 * So far, this library only contains the operations related to the 
 * Exponentially Weighted Moving Average filter and the Tracking Algorithm 
 * implementations. 
 * 
 * For more information on the Tracking Algorithm, see the following link:
 * https://sites.google.com/site/powellsshowcase/home/25sumoprojectcontrol
 * 
 */
#ifndef DSP_H_    
#define DSP_H_

/* Standard C includes. */
#include <stdbool.h>
#include <stdio.h>

/* MATLAB includes. */
#include "grad_desc_4.h"

#define CONTROL_I                       ( 2 )       /* Number of possible tracking sensors. */
#define CONTROL_BORDER_TOTAL            ( 2 )       /* Number of possible border detection sensors. */
#define CONTROL_BORDER_FRONT_INDEX      ( 0 )
#define CONTROL_BORDER_BACK_INDEX       ( 1 )
#define CONTROL_OX_INIT_MULT            ( 10.0f )   /* Multiplier needed to initialize the assumed distance between robot and opponent. Needed for gradient descent. */      

#ifdef __cplusplus
extern "C" {
#endif
    
    typedef enum ControlBorder
    {
        ControlBorder_NO_BORDER_DETECTED,
        ControlBorder_BORDER_DETECTED_FRONT,
        ControlBorder_BORDER_DETECTED_BACK
    }
    ControlBorder;

    /** @brief Exponentially Weighted Moving Average filter object. */
    typedef struct EWMA
    {
        float alpha;        /**< Constant that controls the effectiveness of the filter. Should range between 0 and 1. */
        float prev_out;     /**< Previous average value. */
    }
    EWMA;
    
    /** @brief Control object ( i.e. Tracking Algorithm object ) */
    typedef struct Control
    {
        int iter_max;               /**< Maximum iterations in gradient descent algorithm. */
        float R;                    /**< Radius of self. */
        float Ox_prev;              /**< Last calculated distance between self and opponent. */
        float theta0_prev;          /**< Last calculated orientation between self and opponent. */
        float check;                /**< Gradient Descent break condition. */
        float thetad;               /**< Degrees between each sensor. */
        float desc;                 /**< Step rate of gradient descent algorithm. */
        float theta_offset;         /**< Offset value added to theta0 to determine average angle. */
        float e_total;              /**< PID: Total error. */
        float e_total_max;          /**< PID: Maximum magnitude of total error. */
        float e_prev;               /**< PID: Previous error. */
        float Kp, Ki, Kd;           /**< PID: Parameters of controller. */
        float Ox_max;               
        float Ox_detect;
    }
    Control;
    
    typedef struct Border
    {
        float Bt;
    }
    Border;
    
    /**
     * @brief Configure an EWMA object.
     * 
     * @param ptr The EWMA object.
     * @param alpha The desired alpha value. This value controls the effectiveness
     * of the filter; the lower the value, the more effective the filter will be.
     * However, if alpha is set to its lowest value of 0, then the output will 
     * never change. Conversely, if alpha is set to its highest value of 1, then 
     * no filtration will occur. 
     * @param init_out The initial value of the filter's average value.
     */
    static inline  __attribute__ ((always_inline))
    void vEWMASetup( EWMA* ptr, float alpha, float init_out )
    {
        ptr->alpha = alpha;
        ptr->prev_out = init_out;
    }
    
    /**
     * @brief Executes the EWMA filter given an input value.
     * 
     * @param ptr The EWMA object.
     * @param input The input value.
     * @return The current average value.
     */
    static inline  __attribute__ ((always_inline))
    float fEWMARun( EWMA* ptr, float input )
    {
        return ( ptr->prev_out = ( ptr->alpha * ( input - ptr->prev_out ) + ptr->prev_out ) );
    }
    
    /**
     * @brief Configures a Control object. 
     * 
     * This function configures the Tracking Algorithm which is represented by
     * a Control object. 
     * 
     * @param ptr Control object.
     * @param iter_max Maximum iterations in gradient descent algorithm.
     * @param R Radius of self.
     * @param check Gradient Descent break condition.
     * @param d Distance between each sensor. 
     * @param desc Step rate of gradient descent algorithm.
     * @param e_total_max PID: Maximum magnitude of total error.
     * @param Kp PID: Parameter of controller.
     * @param Ki PID: Parameter of controller.
     * @param Kd PID: Parameter of controller.
     */
    void vControlSetup( Control* ptr, int iter_max, float R,
            float check, float d, 
            float desc, float e_total_max, float Kp, float Ki, float Kd,
            float Ox_max, float Ox_detect );
    
    /**
     * @brief Runs an iteration of the Tracking Algorithm. 
     * @param ptr Control object.
     * @param D Sensor values.
     * @param theta_update The angle needed to update the robot's current orientation.
     */
    void vControlUpdate( Control* ptr, 
            const float* D, float* theta_update, float* Ox_update, bool* detect_update  );
    
    
    static inline  __attribute__ ((always_inline))
    void vControlSetOxMax( Control* ptr, float Ox_max )
    {
        ptr->Ox_max = Ox_max;
    }
    
    static inline  __attribute__ ((always_inline))
    void vBorderSetup( Border* ptr, float Bt )
    {
        ptr->Bt = Bt;
    }
    
    void vBorderUpdate( Border* ptr, const float* B, ControlBorder* border_update );
#ifdef __cplusplus
}
#endif

#endif /* DSP_H_ */
