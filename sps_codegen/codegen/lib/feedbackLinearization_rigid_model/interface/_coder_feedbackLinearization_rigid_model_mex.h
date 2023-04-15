//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_feedbackLinearization_rigid_model_mex.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:34:48
//

#ifndef _CODER_FEEDBACKLINEARIZATION_RIGID_MODEL_MEX_H
#define _CODER_FEEDBACKLINEARIZATION_RIGID_MODEL_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_feedbackLinearization_rigid_model_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[6]);

#endif
//
// File trailer for _coder_feedbackLinearization_rigid_model_mex.h
//
// [EOF]
//
