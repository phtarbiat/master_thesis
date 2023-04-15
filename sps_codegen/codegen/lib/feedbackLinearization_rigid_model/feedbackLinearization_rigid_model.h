//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feedbackLinearization_rigid_model.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:34:48
//

#ifndef FEEDBACKLINEARIZATION_RIGID_MODEL_H
#define FEEDBACKLINEARIZATION_RIGID_MODEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

// Function Declarations
extern void feedbackLinearization_rigid_model(
    const struct0_T *paramStruct, const double theta_des_k[3],
    const double omega_des_k[3], const double domega_des_k[3],
    const double theta_k[3], const double omega_k[3], double u_k[3]);

extern void feedbackLinearization_rigid_model_initialize();

extern void feedbackLinearization_rigid_model_terminate();

#endif
//
// File trailer for feedbackLinearization_rigid_model.h
//
// [EOF]
//
