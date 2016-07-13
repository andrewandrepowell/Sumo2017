#include "DSP.h"

void vControlSetup( Control* ptr, int iter_max, float R, 
        float check, float d, float desc,
        float e_total_max, float Kp, float Ki, float Kd )
{
    float Rsqaure = R*R;
    float dsquare = d*d;
    
    /* Store data into structure. */
    ptr->iter_max = iter_max;
    ptr->R = R;
    ptr->check = check;
    ptr->desc = desc;
    
    /* Prepare PID controller. */
    ptr->e_prev = 0;
    ptr->e_total = 0;
    ptr->e_total_max = e_total_max;
    ptr->Kp = Kp;
    ptr->Ki = Ki;
    ptr->Kd = Kd;
    
    /* Initialize other values based on mathematical relationships. */
    ptr->thetad = acos( ( 2*Rsqaure-dsquare ) / ( 2*Rsqaure ) );  
    ptr->theta_offset = ( float )( CONTROL_I-1 )*( ptr->thetad ) / 2;
    ptr->theta0_prev = -ptr->theta_offset;
    ptr->Ox_prev = CONTROL_OX_INIT_MULT*R;
}

void vControlUpdate( Control* ptr, const float* D, float* theta_update )
{
    float Ox_curr, theta0_curr, theta, e, de;
    float Ox_prev = ptr->Ox_prev;
    float theta0_prev = ptr->theta0_prev;
    float* e_total = &ptr->e_total;
    float* e_prev = &ptr->e_prev;
    float e_total_max = ptr->e_total_max;
    
    /* Run MATLAB-generated function to compute distance and orientation. */
    grad_desc_4( 
            ptr->R, 
            D, 
            ptr->thetad, 
            ptr->desc, 
            ptr->check, 
            ptr->iter_max, 
            Ox_prev, theta0_prev, 
            &Ox_curr, &theta0_curr );
    
    /* Determine the average angle as the output of the system. */
    theta = theta0_curr + ptr->theta_offset;
    
    /* Determine input with PID controller. Note e = r-y, where r = 0. */
    e = -theta; 
    *e_total += e;
    if ( *e_total > e_total_max )
    {
        *e_total = e_total_max;
    } 
    else if ( *e_total < ( -e_total_max ) )
    {
        *e_total = -e_total_max;
    }
    de = e-(*e_prev);
    *theta_update = (ptr->Kp)*e+(ptr->Ki)*(*e_total)+(ptr->Kd)*de;
    
    /* Store information to initialize next iteration. */
    *e_prev = e;
}