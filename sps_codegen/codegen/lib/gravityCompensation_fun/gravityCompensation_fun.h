//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: gravityCompensation_fun.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:36:44
//

#ifndef GRAVITYCOMPENSATION_FUN_H
#define GRAVITYCOMPENSATION_FUN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void gravityCompensation_fun(double g, const double in2[3],
                                    const double in3[3], const double in4[3],
                                    const double in5[18], const double in6[3],
                                    double u[3]);

extern void gravityCompensation_fun_initialize();

extern void gravityCompensation_fun_terminate();

#endif
//
// File trailer for gravityCompensation_fun.h
//
// [EOF]
//
