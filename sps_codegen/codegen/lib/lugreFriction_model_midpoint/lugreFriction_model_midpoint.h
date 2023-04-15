//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lugreFriction_model_midpoint.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:37:10
//

#ifndef LUGREFRICTION_MODEL_MIDPOINT_H
#define LUGREFRICTION_MODEL_MIDPOINT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

// Function Declarations
extern void lugreFriction_model_midpoint(
    const struct0_T *paramStruct, double h, const double omega_k[3],
    const double omega_pk[3], const double tauFVW_pk[3],
    const double tauLoad_k[3], const double T_k[3], const double z_pk[3],
    const double dz_pk[3], double tauF_k[3], double tauFVW_k[3], double z_k[3],
    double dz_k[3]);

extern void lugreFriction_model_midpoint_initialize();

extern void lugreFriction_model_midpoint_terminate();

#endif
//
// File trailer for lugreFriction_model_midpoint.h
//
// [EOF]
//
