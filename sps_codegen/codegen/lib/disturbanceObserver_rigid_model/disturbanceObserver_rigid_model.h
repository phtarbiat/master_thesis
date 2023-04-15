//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: disturbanceObserver_rigid_model.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:38:08
//

#ifndef DISTURBANCEOBSERVER_RIGID_MODEL_H
#define DISTURBANCEOBSERVER_RIGID_MODEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

// Function Declarations
extern void disturbanceObserver_rigid_model(
    const struct0_T *paramStruct, double h, const double u_k[3],
    const double theta_k[3], const double omega_k[3], const double omega_pk[3],
    const double uDisObs_pk[3], double uDisObs_k[3]);

extern void disturbanceObserver_rigid_model_initialize();

extern void disturbanceObserver_rigid_model_terminate();

#endif
//
// File trailer for disturbanceObserver_rigid_model.h
//
// [EOF]
//
