/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


extern "C"
{
#include "nmpc_common.h"
}

#include "INCLUDE/QProblemB.hpp"

#if NMPC_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* NMPC_COMPUTE_COVARIANCE_MATRIX */

static int nmpc_nWSR;



#if NMPC_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis nmpc_sa;
#endif /* NMPC_COMPUTE_COVARIANCE_MATRIX */

int nmpc_solve( void )
{
	nmpc_nWSR = QPOASES_NWSRMAX;

	QProblemB qp( 129 );
	
	returnValue retVal = qp.init(nmpcWorkspace.H, nmpcWorkspace.g, nmpcWorkspace.lb, nmpcWorkspace.ub, nmpc_nWSR, nmpcWorkspace.y);

    qp.getPrimalSolution( nmpcWorkspace.x );
    qp.getDualSolution( nmpcWorkspace.y );
	
#if NMPC_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = nmpc_sa.getHessianInverse( &qp,var );

#endif /* NMPC_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int nmpc_getNWSR( void )
{
	return nmpc_nWSR;
}

const char* nmpc_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
