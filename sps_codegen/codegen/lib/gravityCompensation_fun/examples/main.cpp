//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:36:44
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
#include "gravityCompensation_fun.h"

// Function Declarations
static void argInit_18x1_real_T(double result[18]);

static void argInit_3x1_real_T(double result[3]);

static double argInit_real_T();

static void main_gravityCompensation_fun();

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
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_gravityCompensation_fun()
{
  double dv[18];
  double in2_tmp[3];
  double u[3];
  // Initialize function 'gravityCompensation_fun' input arguments.
  // Initialize function input argument 'in2'.
  argInit_3x1_real_T(in2_tmp);
  // Initialize function input argument 'in3'.
  // Initialize function input argument 'in4'.
  // Initialize function input argument 'in5'.
  // Initialize function input argument 'in6'.
  // Call the entry-point 'gravityCompensation_fun'.
  argInit_18x1_real_T(dv);
  gravityCompensation_fun(argInit_real_T(), in2_tmp, in2_tmp, in2_tmp, dv,
                          in2_tmp, u);
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
  main_gravityCompensation_fun();
  // Terminate the application.
  // You do not need to do this more than one time.
  gravityCompensation_fun_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
