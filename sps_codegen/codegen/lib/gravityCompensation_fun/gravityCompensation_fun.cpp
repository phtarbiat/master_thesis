//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: gravityCompensation_fun.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:36:44
//

// Include Files
#include "gravityCompensation_fun.h"
#include <cmath>

// Function Definitions
//
// gravityCompensation_fun
//     U = gravityCompensation_fun(G,IN2,IN3,IN4,IN5,IN6)
//
// Arguments    : double g
//                const double in2[3]
//                const double in3[3]
//                const double in4[3]
//                const double in5[18]
//                const double in6[3]
//                double u[3]
// Return Type  : void
//
void gravityCompensation_fun(double g, const double[3], const double[3],
                             const double[3], const double in5[18],
                             const double in6[3], double u[3])
{
  double t2;
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     11-Jan-2023 09:46:35
  //  Code generated by file 'LiveEditorEvaluationHelperE1455415238'.
  //  Model: gravityCompensation_fun
  //  Inputs: g, r_P1P2, r_P2P3, r_P3P4, baseParam, thetaLoad
  //
  t2 = in6[1] + in6[2];
  u[0] = 0.0;
  t2 = -(in5[16] * g * std::cos(t2)) + -(in5[15] * g * std::sin(t2));
  u[1] = (t2 - in5[8] * g * std::cos(in6[1])) - in5[7] * g * std::sin(in6[1]);
  u[2] = t2;
}

//
// Arguments    : void
// Return Type  : void
//
void gravityCompensation_fun_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void gravityCompensation_fun_terminate()
{
}

//
// File trailer for gravityCompensation_fun.cpp
//
// [EOF]
//