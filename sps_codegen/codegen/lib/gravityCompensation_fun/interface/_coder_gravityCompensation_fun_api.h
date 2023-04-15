//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_gravityCompensation_fun_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:36:44
//

#ifndef _CODER_GRAVITYCOMPENSATION_FUN_API_H
#define _CODER_GRAVITYCOMPENSATION_FUN_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void gravityCompensation_fun(real_T g, real_T in2[3], real_T in3[3],
                             real_T in4[3], real_T in5[18], real_T in6[3],
                             real_T u[3]);

void gravityCompensation_fun_api(const mxArray *const prhs[6],
                                 const mxArray **plhs);

void gravityCompensation_fun_atexit();

void gravityCompensation_fun_initialize();

void gravityCompensation_fun_terminate();

void gravityCompensation_fun_xil_shutdown();

void gravityCompensation_fun_xil_terminate();

#endif
//
// File trailer for _coder_gravityCompensation_fun_api.h
//
// [EOF]
//
