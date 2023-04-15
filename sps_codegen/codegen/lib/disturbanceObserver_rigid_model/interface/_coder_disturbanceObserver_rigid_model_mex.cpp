//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_disturbanceObserver_rigid_model_mex.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:38:08
//

// Include Files
#include "_coder_disturbanceObserver_rigid_model_mex.h"
#include "_coder_disturbanceObserver_rigid_model_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&disturbanceObserver_rigid_model_atexit);
  // Module initialization.
  disturbanceObserver_rigid_model_initialize();
  // Dispatch the entry-point.
  unsafe_disturbanceObserver_rigid_model_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  disturbanceObserver_rigid_model_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, (const char_T *)"windows-1252", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[7]
// Return Type  : void
//
void unsafe_disturbanceObserver_rigid_model_mexFunction(int32_T nlhs,
                                                        mxArray *plhs[1],
                                                        int32_T nrhs,
                                                        const mxArray *prhs[7])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        31, "disturbanceObserver_rigid_model");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 31,
                        "disturbanceObserver_rigid_model");
  }
  // Call the function.
  disturbanceObserver_rigid_model_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_disturbanceObserver_rigid_model_mex.cpp
//
// [EOF]
//
