//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inverseDynamics_rigid_fun.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:36:55
//

#ifndef INVERSEDYNAMICS_RIGID_FUN_H
#define INVERSEDYNAMICS_RIGID_FUN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void inverseDynamics_rigid_fun(double g, const double in2[3],
                                      const double in3[3], const double in4[3],
                                      const double in5[18], const double in6[3],
                                      const double in7[3], const double in8[3],
                                      const double in9[3], double tauMotor[3]);

extern void inverseDynamics_rigid_fun_initialize();

extern void inverseDynamics_rigid_fun_terminate();

#endif
//
// File trailer for inverseDynamics_rigid_fun.h
//
// [EOF]
//
