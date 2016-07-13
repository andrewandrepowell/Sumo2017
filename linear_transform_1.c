/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linear_transform_1.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 02-Jul-2016 15:55:10
 */

/* Include Files */
#include "linear_transform_1.h"

/* Function Definitions */

/*
 * Arguments    : float x
 * Return Type  : float
 */
float linear_transform_1(float x)
{
  float y;
  float b_y;
  float c_y;
  float d_y;
  float e_y;
  float f_y;
  y = (x - 17680.0F) / 6.315F;
  b_y = (x - 17760.0F) / 98.05F;
  c_y = (x - 17540.0F) / 176.2F;
  d_y = (x - 17510.0F) / 523.6F;
  e_y = (x - 16560.0F) / 228.6F;
  f_y = (x - 309500.0F) / 54790.0F;
  return ((((380.3F * (real32_T)exp(-(y * y)) + 627.7F * (real32_T)exp(-(b_y *
    b_y))) + 66.38F * (real32_T)exp(-(c_y * c_y))) + 30.41F * (real32_T)exp
           (-(d_y * d_y))) + 0.2203F * (real32_T)exp(-(e_y * e_y))) + 1.527E+14F
    * (real32_T)exp(-(f_y * f_y));
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void linear_transform_1_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void linear_transform_1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for linear_transform_1.c
 *
 * [EOF]
 */
