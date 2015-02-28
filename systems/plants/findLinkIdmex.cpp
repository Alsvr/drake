#include "mex.h"
#include <iostream>
#include <Eigen/Dense>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 */


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
	size_t linkstrlen = mxGetNumberOfElements(prhs[1]) + 1;
	char *linkname = new char[linkstrlen];
	RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
	mxGetString(prhs[1], &linkname[0], linkstrlen);
	int robot = static_cast<int>(mxGetScalar(prhs[2]));
	plhs[0] = mxCreateDoubleScalar(model->findLinkId(linkname, robot-1) + 1);
	delete [] linkname;
}

