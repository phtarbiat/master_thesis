//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:38:08
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "disturbanceObserver_rigid_model.h"
#include "disturbanceObserver_rigid_model_types.h"
#include <algorithm>

// Function Declarations
static void argInit_18x1_real_T(double result[18]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_3x2_real_T(double result[6]);

static void argInit_3x3_real_T(double result[9]);

static void argInit_3x6_real_T(double result[18]);

static void argInit_3x737_real_T(double result[2211]);

static double argInit_real_T();

static void argInit_struct0_T(struct0_T *result);

static void argInit_struct1_T(struct1_T *result);

static void argInit_struct2_T(struct2_T *result);

static void argInit_struct3_T(struct3_T *result);

static struct4_T argInit_struct4_T();

static void argInit_struct5_T(struct5_T *result);

static void main_disturbanceObserver_rigid_model();

// Function Definitions
//
// Arguments    : double result[18]
// Return Type  : void
//
static void argInit_18x1_real_T(double result[18])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 18; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_3x2_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 2; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 3; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[18]
// Return Type  : void
//
static void argInit_3x6_real_T(double result[18])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 6; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[2211]
// Return Type  : void
//
static void argInit_3x737_real_T(double result[2211])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 737; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T *result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct1_T(&result->robot);
  argInit_struct2_T(&result->kinError);
  argInit_struct3_T(&result->staticFriction);
  result->lugreObserver = argInit_struct4_T();
  argInit_struct5_T(&result->controller);
}

//
// Arguments    : struct1_T *result
// Return Type  : void
//
static void argInit_struct1_T(struct1_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result->i_g);
  argInit_3x3_real_T(result->k_dataSheet);
  result->g = argInit_real_T();
  argInit_18x1_real_T(result->baseParam);
  argInit_3x2_real_T(result->c_k);
  result->r_P1P2[0] = result->i_g[0];
  result->r_P2P3[0] = result->i_g[0];
  result->r_P3P4[0] = result->i_g[0];
  result->r_P1P2[1] = result->i_g[1];
  result->r_P2P3[1] = result->i_g[1];
  result->r_P3P4[1] = result->i_g[1];
  result->r_P1P2[2] = result->i_g[2];
  result->r_P2P3[2] = result->i_g[2];
  result->r_P3P4[2] = result->i_g[2];
  std::copy(&result->k_dataSheet[0], &result->k_dataSheet[9],
            &result->k_thetaDiff_lim[0]);
}

//
// Arguments    : struct2_T *result
// Return Type  : void
//
static void argInit_struct2_T(struct2_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x737_real_T(result->xLookup);
  argInit_3x1_real_T(result->hasPError);
  argInit_3x6_real_T(result->c_pError);
  std::copy(&result->xLookup[0], &result->xLookup[2211], &result->yLookup[0]);
}

//
// Arguments    : struct3_T *result
// Return Type  : void
//
static void argInit_struct3_T(struct3_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x2_real_T(result->c_c_pos);
  argInit_3x3_real_T(result->c_omega_pos);
  argInit_3x1_real_T(result->c_load);
  for (int i{0}; i < 6; i++) {
    result->c_c_neg[i] = result->c_c_pos[i];
  }
  std::copy(&result->c_omega_pos[0], &result->c_omega_pos[9],
            &result->c_omega_neg[0]);
  for (int i{0}; i < 6; i++) {
    double d;
    d = result->c_c_pos[i];
    result->c_T_pos[i] = d;
    result->c_T_neg[i] = d;
  }
}

//
// Arguments    : void
// Return Type  : struct4_T
//
static struct4_T argInit_struct4_T()
{
  struct4_T result;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result.sigma_0);
  return result;
}

//
// Arguments    : struct5_T *result
// Return Type  : void
//
static void argInit_struct5_T(struct5_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result->k_theta);
  result->cFilter = argInit_real_T();
  result->k_omega[0] = result->k_theta[0];
  result->k_domega[0] = result->k_theta[0];
  result->k_ddomega[0] = result->k_theta[0];
  result->k_omega[1] = result->k_theta[1];
  result->k_domega[1] = result->k_theta[1];
  result->k_ddomega[1] = result->k_theta[1];
  result->k_omega[2] = result->k_theta[2];
  result->k_domega[2] = result->k_theta[2];
  result->k_ddomega[2] = result->k_theta[2];
}

//
// Arguments    : void
// Return Type  : void
//
static void main_disturbanceObserver_rigid_model()
{
  struct0_T r;
  double uDisObs_k[3];
  double u_k_tmp[3];
  // Initialize function 'disturbanceObserver_rigid_model' input arguments.
  // Initialize function input argument 'paramStruct'.
  // Initialize function input argument 'u_k'.
  argInit_3x1_real_T(u_k_tmp);
  // Initialize function input argument 'theta_k'.
  // Initialize function input argument 'omega_k'.
  // Initialize function input argument 'omega_pk'.
  // Initialize function input argument 'uDisObs_pk'.
  // Call the entry-point 'disturbanceObserver_rigid_model'.
  argInit_struct0_T(&r);
  disturbanceObserver_rigid_model(&r, argInit_real_T(), u_k_tmp, u_k_tmp,
                                  u_k_tmp, u_k_tmp, u_k_tmp, uDisObs_k);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_disturbanceObserver_rigid_model();
  // Terminate the application.
  // You do not need to do this more than one time.
  disturbanceObserver_rigid_model_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
