#include "mex.h"
#include <iostream>
#include "math.h"
#include <Eigen/Sparse>
#include "LCP_Solvers.h"
#include "NumericsMatrix.h"

using namespace Eigen;
using namespace std;


void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  int M_rows = mxGetN(prhs[0]);
  int M_cols = mxGetM(prhs[0]);
  int w_length = mxGetNumberOfElements(prhs[1]);


  NumericsMatrix* mat = createNumericsMatrix(0, M_rows, M_cols, mxGetPr(prhs[0])); 

  SolverOptions options;
  LinearComplementarityProblem lcp;
  lcp.M = mat;
  lcp.q = mxGetPr(prhs[1]);
  lcp.size = w_length;
  plhs[0] = mxCreateDoubleMatrix(w_length, 1, mxREAL);
  Map<VectorXd> z(mxGetPr(plhs[0]),w_length);
  z = VectorXd::Zero(w_length);
  VectorXd w_out = VectorXd::Zero(w_length);
  int info;
  linearComplementarity_nsqp_setDefaultSolverOptions(&options);
  lcp_nsqp(&lcp, z.data(), w_out.data(), &info, &options);
  plhs[1] = mxCreateDoubleScalar(static_cast<double>(info));
  // cout << "z'*(M*z + w): " << z.transpose()*(M*z + w) << endl;
  // deleteSolverOptions(&options);
}


