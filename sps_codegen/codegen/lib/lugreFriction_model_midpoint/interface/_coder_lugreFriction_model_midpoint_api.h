//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_lugreFriction_model_midpoint_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:37:10
//

#ifndef _CODER_LUGREFRICTION_MODEL_MIDPOINT_API_H
#define _CODER_LUGREFRICTION_MODEL_MIDPOINT_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct4_T {
  real_T sigma_0[3];
};

struct struct5_T {
  real_T k_theta[3];
  real_T k_omega[3];
  real_T k_domega[3];
  real_T k_ddomega[3];
  real_T cFilter;
};

struct struct1_T {
  real_T g;
  real_T i_g[3];
  real_T r_P1P2[3];
  real_T r_P2P3[3];
  real_T r_P3P4[3];
  real_T baseParam[18];
  real_T k_dataSheet[9];
  real_T c_k[6];
  real_T k_thetaDiff_lim[9];
};

struct struct2_T {
  real_T xLookup[2211];
  real_T yLookup[2211];
  real_T hasPError[3];
  real_T c_pError[18];
};

struct struct3_T {
  real_T c_c_pos[6];
  real_T c_c_neg[6];
  real_T c_load[3];
  real_T c_omega_pos[9];
  real_T c_omega_neg[9];
  real_T c_T_pos[6];
  real_T c_T_neg[6];
};

struct struct0_T {
  struct1_T robot;
  struct2_T kinError;
  struct3_T staticFriction;
  struct4_T lugreObserver;
  struct5_T controller;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void lugreFriction_model_midpoint(struct0_T *paramStruct, real_T h,
                                  real_T omega_k[3], real_T omega_pk[3],
                                  real_T tauFVW_pk[3], real_T tauLoad_k[3],
                                  real_T T_k[3], real_T z_pk[3],
                                  real_T dz_pk[3], real_T tauF_k[3],
                                  real_T tauFVW_k[3], real_T z_k[3],
                                  real_T dz_k[3]);

void lugreFriction_model_midpoint_api(const mxArray *const prhs[9],
                                      int32_T nlhs, const mxArray *plhs[4]);

void lugreFriction_model_midpoint_atexit();

void lugreFriction_model_midpoint_initialize();

void lugreFriction_model_midpoint_terminate();

void lugreFriction_model_midpoint_xil_shutdown();

void lugreFriction_model_midpoint_xil_terminate();

#endif
//
// File trailer for _coder_lugreFriction_model_midpoint_api.h
//
// [EOF]
//
