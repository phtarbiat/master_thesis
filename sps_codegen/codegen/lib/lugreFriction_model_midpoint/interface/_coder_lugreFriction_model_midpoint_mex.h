//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_lugreFriction_model_midpoint_mex.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 19-Feb-2023 09:37:10
//

#ifndef _CODER_LUGREFRICTION_MODEL_MIDPOINT_MEX_H
#define _CODER_LUGREFRICTION_MODEL_MIDPOINT_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_lugreFriction_model_midpoint_mexFunction(int32_T nlhs,
                                                     mxArray *plhs[4],
                                                     int32_T nrhs,
                                                     const mxArray *prhs[9]);

#endif
//
// File trailer for _coder_lugreFriction_model_midpoint_mex.h
//
// [EOF]
//
