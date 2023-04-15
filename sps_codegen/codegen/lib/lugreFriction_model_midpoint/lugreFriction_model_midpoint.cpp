//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lugreFriction_model_midpoint.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:37:10
//

// Include Files
#include "lugreFriction_model_midpoint.h"
#include "lugreFriction_model_midpoint_types.h"
#include <algorithm>
#include <cmath>

// Function Declarations
static void binary_expand_op(double in1_data[], int *in1_size,
                             const double in2_data[], const int *in2_size,
                             const double in3[3], const signed char in4_data[],
                             const int *in4_size, const struct0_T *in5);

// Function Definitions
//
// Arguments    : double in1_data[]
//                int *in1_size
//                const double in2_data[]
//                const int *in2_size
//                const double in3[3]
//                const signed char in4_data[]
//                const int *in4_size
//                const struct0_T *in5
// Return Type  : void
//
static void binary_expand_op(double in1_data[], int *in1_size,
                             const double in2_data[], const int *in2_size,
                             const double in3[3], const signed char in4_data[],
                             const int *in4_size, const struct0_T *in5)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (*in4_size == 1) {
    *in1_size = *in2_size;
  } else {
    *in1_size = *in4_size;
  }
  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in4_size != 1);
  if (*in4_size == 1) {
    loop_ub = *in2_size;
  } else {
    loop_ub = *in4_size;
  }
  for (int i{0}; i < loop_ub; i++) {
    int i1;
    i1 = in4_data[i * stride_1_0] - 1;
    in1_data[i] =
        in2_data[i * stride_0_0] * (in3[i1] / in5->lugreObserver.sigma_0[i1]);
  }
}

//
// LuGre Friction Model using the midpoint rule for numerical integration
//
//  Philipp Tarbiat
//  Chair of Automatic Control
//  TUM School of Engineering and Design
//  Technical University of Munich
//
// Arguments    : const struct0_T *paramStruct
//                double h
//                const double omega_k[3]
//                const double omega_pk[3]
//                const double tauFVW_pk[3]
//                const double tauLoad_k[3]
//                const double T_k[3]
//                const double z_pk[3]
//                const double dz_pk[3]
//                double tauF_k[3]
//                double tauFVW_k[3]
//                double z_k[3]
//                double dz_k[3]
// Return Type  : void
//
void lugreFriction_model_midpoint(
    const struct0_T *paramStruct, double h, const double omega_k[3],
    const double omega_pk[3], const double tauFVW_pk[3],
    const double tauLoad_k[3], const double T_k[3], const double z_pk[3],
    const double dz_pk[3], double tauF_k[3], double tauFVW_k[3], double z_k[3],
    double dz_k[3])
{
  double c_omega[9];
  double c_T[6];
  double c_c[6];
  double e_tmp_data[3];
  double g_tmp_data[3];
  double tauFVW_m[3];
  double z_m[3];
  double b_x;
  double d;
  double x;
  double y_idx_0_tmp;
  double y_idx_1_tmp;
  double y_idx_2_tmp;
  int b_trueCount;
  int i;
  int partialTrueCount;
  int trueCount;
  signed char b_tmp_data[3];
  signed char c_tmp_data[3];
  signed char d_tmp_data[3];
  signed char f_tmp_data[3];
  signed char h_tmp_data[3];
  signed char i_tmp_data[3];
  signed char tmp_data[3];
  boolean_T b;
  boolean_T b1;
  boolean_T idx_idx_0;
  boolean_T idx_idx_1;
  //  Load model parameters
  //  Check movement direction for every joint
  //  Static friction
  for (trueCount = 0; trueCount < 6; trueCount++) {
    c_c[trueCount] = paramStruct->staticFriction.c_c_neg[trueCount];
  }
  b_trueCount = 0;
  if (omega_k[0] >= 0.0) {
    b_trueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    b_trueCount++;
  }
  if (omega_k[2] >= 0.0) {
    b_trueCount++;
  }
  partialTrueCount = 0;
  if (omega_k[0] >= 0.0) {
    tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (omega_k[2] >= 0.0) {
    tmp_data[partialTrueCount] = 3;
  }
  for (trueCount = 0; trueCount < 2; trueCount++) {
    for (i = 0; i < b_trueCount; i++) {
      partialTrueCount = (tmp_data[i] + 3 * trueCount) - 1;
      c_c[partialTrueCount] =
          paramStruct->staticFriction.c_c_pos[partialTrueCount];
    }
  }
  std::copy(&paramStruct->staticFriction.c_omega_neg[0],
            &paramStruct->staticFriction.c_omega_neg[9], &c_omega[0]);
  b_trueCount = 0;
  if (omega_k[0] >= 0.0) {
    b_trueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    b_trueCount++;
  }
  if (omega_k[2] >= 0.0) {
    b_trueCount++;
  }
  partialTrueCount = 0;
  if (omega_k[0] >= 0.0) {
    b_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    b_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (omega_k[2] >= 0.0) {
    b_tmp_data[partialTrueCount] = 3;
  }
  for (trueCount = 0; trueCount < 3; trueCount++) {
    for (i = 0; i < b_trueCount; i++) {
      partialTrueCount = (b_tmp_data[i] + 3 * trueCount) - 1;
      c_omega[partialTrueCount] =
          paramStruct->staticFriction.c_omega_pos[partialTrueCount];
    }
  }
  for (trueCount = 0; trueCount < 6; trueCount++) {
    c_T[trueCount] = paramStruct->staticFriction.c_T_neg[trueCount];
  }
  b_trueCount = 0;
  if (omega_k[0] >= 0.0) {
    b_trueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    b_trueCount++;
  }
  if (omega_k[2] >= 0.0) {
    b_trueCount++;
  }
  partialTrueCount = 0;
  if (omega_k[0] >= 0.0) {
    c_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (omega_k[1] >= 0.0) {
    c_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (omega_k[2] >= 0.0) {
    c_tmp_data[partialTrueCount] = 3;
  }
  for (trueCount = 0; trueCount < 2; trueCount++) {
    for (i = 0; i < b_trueCount; i++) {
      partialTrueCount = (c_tmp_data[i] + 3 * trueCount) - 1;
      c_T[partialTrueCount] =
          paramStruct->staticFriction.c_T_pos[partialTrueCount];
    }
  }
  //  LuGre Friction
  //  Damping coefficient is set to zero
  //  Calculate Static Friction
  //  Static Friction Model: friction torque equation considering velocity,
  //  load torque and temperature dependence
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Velocity weakening friction
  //  Velocity strengthening friction
  //  Calculate internal state and its derivative
  //  LuGre Friction Model: Numeric solution of the ODE using the
  //  midpoint rule
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Calculate midpoint values
  //  Equation of midpoint numerical integration
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Equation of midpoint numerical integration
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Limit maximum value of z_m
  b_trueCount = 0;
  d = c_c[0] +
      paramStruct->staticFriction.c_load[0] * (tauLoad_k[0] * tauLoad_k[0]);
  tauFVW_k[0] = d;
  y_idx_0_tmp = std::abs(omega_k[0]);
  x = omega_k[0];
  if (!std::isnan(omega_k[0])) {
    if (omega_k[0] < 0.0) {
      x = -1.0;
    } else {
      x = (omega_k[0] > 0.0);
    }
  }
  tauF_k[0] = x;
  if (omega_k[0] == 0.0) {
    d = 1.0E-10;
    tauFVW_k[0] = 1.0E-10;
  }
  z_k[0] = (omega_k[0] + omega_pk[0]) / 2.0;
  d = (d + tauFVW_pk[0]) / 2.0;
  tauFVW_m[0] = d;
  b_x = z_pk[0] + 0.5 * h * dz_pk[0];
  z_m[0] = b_x;
  b = (std::abs(b_x) > d / paramStruct->lugreObserver.sigma_0[0]);
  idx_idx_0 = b;
  if (b) {
    b_trueCount = 1;
  }
  d = c_c[1] +
      paramStruct->staticFriction.c_load[1] * (tauLoad_k[1] * tauLoad_k[1]);
  tauFVW_k[1] = d;
  y_idx_1_tmp = std::abs(omega_k[1]);
  x = omega_k[1];
  if (!std::isnan(omega_k[1])) {
    if (omega_k[1] < 0.0) {
      x = -1.0;
    } else {
      x = (omega_k[1] > 0.0);
    }
  }
  tauF_k[1] = x;
  if (omega_k[1] == 0.0) {
    d = 1.0E-10;
    tauFVW_k[1] = 1.0E-10;
  }
  z_k[1] = (omega_k[1] + omega_pk[1]) / 2.0;
  d = (d + tauFVW_pk[1]) / 2.0;
  tauFVW_m[1] = d;
  b_x = z_pk[1] + 0.5 * h * dz_pk[1];
  z_m[1] = b_x;
  b = (std::abs(b_x) > d / paramStruct->lugreObserver.sigma_0[1]);
  idx_idx_1 = b;
  if (b) {
    b_trueCount++;
  }
  d = c_c[2] +
      paramStruct->staticFriction.c_load[2] * (tauLoad_k[2] * tauLoad_k[2]);
  tauFVW_k[2] = d;
  y_idx_2_tmp = std::abs(omega_k[2]);
  x = omega_k[2];
  if (!std::isnan(omega_k[2])) {
    if (omega_k[2] < 0.0) {
      x = -1.0;
    } else {
      x = (omega_k[2] > 0.0);
    }
  }
  if (omega_k[2] == 0.0) {
    d = 1.0E-10;
    tauFVW_k[2] = 1.0E-10;
  }
  z_k[2] = (omega_k[2] + omega_pk[2]) / 2.0;
  d = (d + tauFVW_pk[2]) / 2.0;
  tauFVW_m[2] = d;
  b_x = z_pk[2] + 0.5 * h * dz_pk[2];
  z_m[2] = b_x;
  b = (std::abs(b_x) > d / paramStruct->lugreObserver.sigma_0[2]);
  if (b) {
    b_trueCount++;
  }
  partialTrueCount = 0;
  if (idx_idx_0) {
    d_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (idx_idx_1) {
    d_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (b) {
    d_tmp_data[partialTrueCount] = 3;
  }
  for (trueCount = 0; trueCount < b_trueCount; trueCount++) {
    e_tmp_data[trueCount] = z_m[d_tmp_data[trueCount] - 1];
  }
  for (partialTrueCount = 0; partialTrueCount < b_trueCount;
       partialTrueCount++) {
    b_x = e_tmp_data[partialTrueCount];
    if (!std::isnan(b_x)) {
      if (b_x < 0.0) {
        b_x = -1.0;
      } else {
        b_x = (b_x > 0.0);
      }
    }
    e_tmp_data[partialTrueCount] = b_x;
  }
  trueCount = 0;
  if (idx_idx_0) {
    trueCount = 1;
  }
  if (idx_idx_1) {
    trueCount++;
  }
  if (b) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (idx_idx_0) {
    f_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (idx_idx_1) {
    f_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (b) {
    f_tmp_data[partialTrueCount] = 3;
  }
  if (b_trueCount == trueCount) {
    for (trueCount = 0; trueCount < b_trueCount; trueCount++) {
      i = f_tmp_data[trueCount] - 1;
      g_tmp_data[trueCount] =
          e_tmp_data[trueCount] *
          (tauFVW_m[i] / paramStruct->lugreObserver.sigma_0[i]);
    }
  } else {
    binary_expand_op(g_tmp_data, &partialTrueCount, e_tmp_data, &b_trueCount,
                     tauFVW_m, f_tmp_data, &trueCount, paramStruct);
  }
  partialTrueCount = 0;
  //  Calculate new internal state
  //  LuGre Friction Model: ODE of the bristle element
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Limit maximum value of z_k
  b_trueCount = 0;
  if (idx_idx_0) {
    z_m[0] = g_tmp_data[0];
    partialTrueCount = 1;
  }
  b_x =
      z_pk[0] +
      h * (z_k[0] - z_m[0] * std::abs(z_k[0]) *
                        (paramStruct->lugreObserver.sigma_0[0] / tauFVW_m[0]));
  z_k[0] = b_x;
  b1 = (std::abs(b_x) > tauFVW_k[0] / paramStruct->lugreObserver.sigma_0[0]);
  idx_idx_0 = b1;
  if (b1) {
    b_trueCount = 1;
  }
  if (idx_idx_1) {
    z_m[1] = g_tmp_data[partialTrueCount];
    partialTrueCount++;
  }
  b_x =
      z_pk[1] +
      h * (z_k[1] - z_m[1] * std::abs(z_k[1]) *
                        (paramStruct->lugreObserver.sigma_0[1] / tauFVW_m[1]));
  z_k[1] = b_x;
  b1 = (std::abs(b_x) > tauFVW_k[1] / paramStruct->lugreObserver.sigma_0[1]);
  idx_idx_1 = b1;
  if (b1) {
    b_trueCount++;
  }
  if (b) {
    z_m[2] = g_tmp_data[partialTrueCount];
  }
  b_x =
      z_pk[2] + h * (z_k[2] - z_m[2] * std::abs(z_k[2]) *
                                  (paramStruct->lugreObserver.sigma_0[2] / d));
  z_k[2] = b_x;
  b1 = (std::abs(b_x) > tauFVW_k[2] / paramStruct->lugreObserver.sigma_0[2]);
  if (b1) {
    b_trueCount++;
  }
  partialTrueCount = 0;
  if (idx_idx_0) {
    h_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (idx_idx_1) {
    h_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (b1) {
    h_tmp_data[partialTrueCount] = 3;
  }
  for (trueCount = 0; trueCount < b_trueCount; trueCount++) {
    e_tmp_data[trueCount] = z_k[h_tmp_data[trueCount] - 1];
  }
  for (partialTrueCount = 0; partialTrueCount < b_trueCount;
       partialTrueCount++) {
    b_x = e_tmp_data[partialTrueCount];
    if (!std::isnan(b_x)) {
      if (b_x < 0.0) {
        b_x = -1.0;
      } else {
        b_x = (b_x > 0.0);
      }
    }
    e_tmp_data[partialTrueCount] = b_x;
  }
  trueCount = 0;
  if (idx_idx_0) {
    trueCount = 1;
  }
  if (idx_idx_1) {
    trueCount++;
  }
  if (b1) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (idx_idx_0) {
    i_tmp_data[0] = 1;
    partialTrueCount = 1;
  }
  if (idx_idx_1) {
    i_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }
  if (b1) {
    i_tmp_data[partialTrueCount] = 3;
  }
  if (b_trueCount == trueCount) {
    for (trueCount = 0; trueCount < b_trueCount; trueCount++) {
      i = i_tmp_data[trueCount] - 1;
      g_tmp_data[trueCount] =
          e_tmp_data[trueCount] *
          (tauFVW_k[i] / paramStruct->lugreObserver.sigma_0[i]);
    }
  } else {
    binary_expand_op(g_tmp_data, &partialTrueCount, e_tmp_data, &b_trueCount,
                     tauFVW_k, i_tmp_data, &trueCount, paramStruct);
  }
  partialTrueCount = 0;
  //  LuGre Friction Model: ODE of the bristle element
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  //  Calculate LuGre Friction
  //  LuGre Friction Model: friction torque equation
  //
  //  Philipp Tarbiat
  //  Chair of Automatic Control
  //  TUM School of Engineering and Design
  //  Technical University of Munich
  if (idx_idx_0) {
    z_k[0] = g_tmp_data[0];
    partialTrueCount = 1;
  }
  d = omega_k[0] - z_k[0] * y_idx_0_tmp *
                       (paramStruct->lugreObserver.sigma_0[0] / tauFVW_k[0]);
  dz_k[0] = d;
  tauF_k[0] =
      (paramStruct->lugreObserver.sigma_0[0] * z_k[0] + 0.0 * d) +
      tauF_k[0] *
          ((c_omega[0] - c_T[0] * T_k[0]) *
           ((1.0 - std::exp(-y_idx_0_tmp / (c_omega[3] + T_k[0] * c_T[3]))) +
            c_omega[6] * std::sqrt(y_idx_0_tmp)));
  if (idx_idx_1) {
    z_k[1] = g_tmp_data[partialTrueCount];
    partialTrueCount++;
  }
  d = omega_k[1] - z_k[1] * y_idx_1_tmp *
                       (paramStruct->lugreObserver.sigma_0[1] / tauFVW_k[1]);
  dz_k[1] = d;
  tauF_k[1] =
      (paramStruct->lugreObserver.sigma_0[1] * z_k[1] + 0.0 * d) +
      tauF_k[1] *
          ((c_omega[1] - c_T[1] * T_k[1]) *
           ((1.0 - std::exp(-y_idx_1_tmp / (c_omega[4] + T_k[1] * c_T[4]))) +
            c_omega[7] * std::sqrt(y_idx_1_tmp)));
  if (b1) {
    z_k[2] = g_tmp_data[partialTrueCount];
  }
  d = omega_k[2] - z_k[2] * y_idx_2_tmp *
                       (paramStruct->lugreObserver.sigma_0[2] / tauFVW_k[2]);
  dz_k[2] = d;
  tauF_k[2] =
      (paramStruct->lugreObserver.sigma_0[2] * z_k[2] + 0.0 * d) +
      x * ((c_omega[2] - c_T[2] * T_k[2]) *
           ((1.0 - std::exp(-y_idx_2_tmp / (c_omega[5] + T_k[2] * c_T[5]))) +
            c_omega[8] * std::sqrt(y_idx_2_tmp)));
}

//
// Arguments    : void
// Return Type  : void
//
void lugreFriction_model_midpoint_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void lugreFriction_model_midpoint_terminate()
{
}

//
// File trailer for lugreFriction_model_midpoint.cpp
//
// [EOF]
//
