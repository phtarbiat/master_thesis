//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: disturbanceObserver_rigid_model_types.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:38:08
//

#ifndef DISTURBANCEOBSERVER_RIGID_MODEL_TYPES_H
#define DISTURBANCEOBSERVER_RIGID_MODEL_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct struct4_T {
  double sigma_0[3];
};

struct struct5_T {
  double k_theta[3];
  double k_omega[3];
  double k_domega[3];
  double k_ddomega[3];
  double cFilter;
};

struct struct1_T {
  double g;
  double i_g[3];
  double r_P1P2[3];
  double r_P2P3[3];
  double r_P3P4[3];
  double baseParam[18];
  double k_dataSheet[9];
  double c_k[6];
  double k_thetaDiff_lim[9];
};

struct struct2_T {
  double xLookup[2211];
  double yLookup[2211];
  double hasPError[3];
  double c_pError[18];
};

struct struct3_T {
  double c_c_pos[6];
  double c_c_neg[6];
  double c_load[3];
  double c_omega_pos[9];
  double c_omega_neg[9];
  double c_T_pos[6];
  double c_T_neg[6];
};

struct struct0_T {
  struct1_T robot;
  struct2_T kinError;
  struct3_T staticFriction;
  struct4_T lugreObserver;
  struct5_T controller;
};

#endif
//
// File trailer for disturbanceObserver_rigid_model_types.h
//
// [EOF]
//
