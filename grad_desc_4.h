/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: grad_desc_4.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 28-Jun-2016 00:23:38
 */

#ifndef __GRAD_DESC_4_H__
#define __GRAD_DESC_4_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "grad_desc_4_types.h"

/* Function Declarations */
extern void grad_desc_4(float R, const float D[2], float thetad, float desc,
  float check, int iters_max, float Ox, float theta0, float *Ox_next, float
  *theta0_next);
extern void grad_desc_4_initialize(void);
extern void grad_desc_4_terminate(void);

#endif

/*
 * File trailer for grad_desc_4.h
 *
 * [EOF]
 */
