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


#ifndef NMPC_COMMON_H
#define NMPC_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup NMPC ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define NMPC_QPOASES  0
#define NMPC_QPOASES3 1
/** FORCES QP solver indicator.*/
#define NMPC_FORCES   2
/** qpDUNES QP solver indicator.*/
#define NMPC_QPDUNES  3
/** HPMPC QP solver indicator. */
#define NMPC_HPMPC    4

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define NMPC_QP_SOLVER NMPC_QPOASES

#include "nmpc_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define NMPC_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define NMPC_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define NMPC_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define NMPC_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define NMPC_N 30
/** Number of online data values. */
#define NMPC_NOD 6
/** Number of control variables. */
#define NMPC_NU 4
/** Number of differential variables. */
#define NMPC_NX 13
/** Number of algebraic variables. */
#define NMPC_NXA 0
/** Number of differential derivative variables. */
#define NMPC_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define NMPC_NY 16
/** Number of references/measurements on the last (N + 1)st node. */
#define NMPC_NYN 10
/** Total number of QP optimization variables. */
#define NMPC_QP_NV 133
/** Number of integration steps per shooting interval. */
#define NMPC_RK_NIS 1
/** Number of Runge-Kutta stages per integration step. */
#define NMPC_RK_NSTAGES 4
/** Providing interface for arrival cost. */
#define NMPC_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define NMPC_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define NMPC_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct NMPCvariables_
{
int dummy;
/** Matrix of size: 31 x 13 (row major format)
 * 
 *  Matrix containing 31 differential variable vectors.
 */
real_t x[ 403 ];

/** Matrix of size: 30 x 4 (row major format)
 * 
 *  Matrix containing 30 control variable vectors.
 */
real_t u[ 120 ];

/** Matrix of size: 31 x 6 (row major format)
 * 
 *  Matrix containing 31 online data vectors.
 */
real_t od[ 186 ];

/** Column vector of size: 480
 * 
 *  Matrix containing 30 reference/measurement vectors of size 16 for first 30 nodes.
 */
real_t y[ 480 ];

/** Column vector of size: 10
 * 
 *  Reference/measurement vector for the 31. node.
 */
real_t yN[ 10 ];

/** Matrix of size: 16 x 16 (row major format) */
real_t W[ 256 ];

/** Matrix of size: 10 x 10 (row major format) */
real_t WN[ 100 ];

/** Column vector of size: 13
 * 
 *  Current state feedback vector.
 */
real_t x0[ 13 ];


} NMPCvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct NMPCworkspace_
{
real_t rk_ttt;

/** Row vector of size: 244 */
real_t rk_xxx[ 244 ];

/** Matrix of size: 4 x 234 (row major format) */
real_t rk_kkk[ 936 ];

/** Row vector of size: 244 */
real_t state[ 244 ];

/** Column vector of size: 390 */
real_t d[ 390 ];

/** Column vector of size: 480 */
real_t Dy[ 480 ];

/** Column vector of size: 10 */
real_t DyN[ 10 ];

/** Matrix of size: 390 x 13 (row major format) */
real_t evGx[ 5070 ];

/** Matrix of size: 390 x 4 (row major format) */
real_t evGu[ 1560 ];

/** Column vector of size: 30 */
real_t objAuxVar[ 30 ];

/** Row vector of size: 23 */
real_t objValueIn[ 23 ];

/** Row vector of size: 288 */
real_t objValueOut[ 288 ];

/** Matrix of size: 390 x 13 (row major format) */
real_t Q1[ 5070 ];

/** Matrix of size: 390 x 16 (row major format) */
real_t Q2[ 6240 ];

/** Matrix of size: 120 x 4 (row major format) */
real_t R1[ 480 ];

/** Matrix of size: 120 x 16 (row major format) */
real_t R2[ 1920 ];

/** Matrix of size: 13 x 13 (row major format) */
real_t QN1[ 169 ];

/** Matrix of size: 13 x 10 (row major format) */
real_t QN2[ 130 ];

/** Column vector of size: 13 */
real_t Dx0[ 13 ];

/** Matrix of size: 13 x 13 (row major format) */
real_t T[ 169 ];

/** Matrix of size: 6045 x 4 (row major format) */
real_t E[ 24180 ];

/** Matrix of size: 6045 x 4 (row major format) */
real_t QE[ 24180 ];

/** Matrix of size: 390 x 13 (row major format) */
real_t QGx[ 5070 ];

/** Column vector of size: 390 */
real_t Qd[ 390 ];

/** Column vector of size: 403 */
real_t QDy[ 403 ];

/** Matrix of size: 120 x 13 (row major format) */
real_t H10[ 1560 ];

/** Matrix of size: 133 x 133 (row major format) */
real_t H[ 17689 ];

/** Column vector of size: 133 */
real_t g[ 133 ];

/** Column vector of size: 133 */
real_t lb[ 133 ];

/** Column vector of size: 133 */
real_t ub[ 133 ];

/** Column vector of size: 133 */
real_t x[ 133 ];

/** Column vector of size: 133 */
real_t y[ 133 ];


} NMPCworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int nmpc_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void nmpc_rhs_forw(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int nmpc_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int nmpc_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int nmpc_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void nmpc_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 31 with xEnd. 2. Initialize node 31 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmpc_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t nmpc_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t nmpc_getObjective(  );


/* 
 * Extern declarations. 
 */

extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* NMPC_COMMON_H */
