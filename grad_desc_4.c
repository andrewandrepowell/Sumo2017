/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: grad_desc_4.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 28-Jun-2016 00:23:38
 */

/* Include Files */
#include "grad_desc_4.h"

/* Named Constants */
#define b_R_res                        (2.0F)
#define b_thetad_res                   (0.71514219F)
#define b_Ox_res                       (4.17118931F)
#define b_theta0_res                   (-0.357571155F)

/* Variable Definitions */
static float R_res;
static float thetad_res;
static float Ox_res;
static float theta0_res;
static float D_res[2];
static float thetamat_res[2];

/* Function Definitions */

/*
 * Arguments    : float R
 *                const float D[2]
 *                float thetad
 *                float desc
 *                float check
 *                int iters_max
 *                float Ox
 *                float theta0
 *                float *Ox_next
 *                float *theta0_next
 * Return Type  : void
 */
void grad_desc_4(float R, const float D[2], float thetad, float desc, float
                 check, int iters_max, float Ox, float theta0, float *Ox_next,
                 float *theta0_next)
{
  float curr[2];
  float res[4];
  float b_res[2];
  int i;
  float next[2];
  float f0;
  int i1;
  int iter;
  int32_T exitg1;

  /* I_res = I; */
  R_res = R;
  thetad_res = thetad;
  Ox_res = Ox;
  theta0_res = theta0;
  curr[0] = Ox_res;
  curr[1] = theta0_res;
  for (i = 0; i < 2; i++) {
    D_res[i] = D[i];
    thetamat_res[i] = 0.0F;
    thetamat_res[i] = theta0_res + (((float)i + 1.0F) - 1.0F) * thetad_res;
    res[i] = 2.0F * Ox_res - 2.0F * R_res * (real32_T)cos(thetamat_res[i]);
    res[2 + i] = 2.0F * R_res * Ox_res * (real32_T)sin(thetamat_res[i]);
    b_res[i] = ((R_res * R_res + Ox_res * Ox_res) - D_res[i] * D_res[i]) - 2.0F *
      R_res * Ox_res * (real32_T)cos(thetamat_res[i]);
  }

  for (i = 0; i < 2; i++) {
    f0 = 0.0F;
    for (i1 = 0; i1 < 2; i1++) {
      f0 += desc * res[i1 + (i << 1)] * b_res[i1];
    }

    next[i] = curr[i] - f0;
  }

  iter = 1;
  do {
    exitg1 = 0;
    for (i = 0; i < 2; i++) {
      b_res[i] = (real32_T)fabs(curr[i] - next[i]);
    }

    if (((b_res[0] + b_res[1]) / 2.0F > check) && (iter < iters_max)) {
      Ox_res = next[0];
      theta0_res = next[1];
      for (i = 0; i < 2; i++) {
        curr[i] = next[i];
        thetamat_res[i] = 0.0F;
        thetamat_res[i] = theta0_res + (((float)i + 1.0F) - 1.0F) * thetad_res;
        res[i] = 2.0F * Ox_res - 2.0F * R_res * (real32_T)cos(thetamat_res[i]);
        res[2 + i] = 2.0F * R_res * Ox_res * (real32_T)sin(thetamat_res[i]);
        b_res[i] = ((R_res * R_res + Ox_res * Ox_res) - D_res[i] * D_res[i]) -
          2.0F * R_res * Ox_res * (real32_T)cos(thetamat_res[i]);
      }

      for (i = 0; i < 2; i++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          f0 += desc * res[i1 + (i << 1)] * b_res[i1];
        }

        next[i] -= f0;
      }

      iter++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  *Ox_next = curr[0];
  *theta0_next = curr[1];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void grad_desc_4_initialize(void)
{
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    thetamat_res[i0] = -0.357571155F + 0.71514219F * (float)i0;
    D_res[i0] = 2.23875022F + 7.15255737E-7F * (float)i0;
  }

  theta0_res = b_theta0_res;
  Ox_res = b_Ox_res;
  thetad_res = b_thetad_res;
  R_res = b_R_res;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void grad_desc_4_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for grad_desc_4.c
 *
 * [EOF]
 */
