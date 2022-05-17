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


#ifndef NMPC_AUXILIARY_FUNCTIONS_H
#define NMPC_AUXILIARY_FUNCTIONS_H

#include "nmpc_common.h"

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** Get pointer to the matrix with differential variables. */
real_t* nmpc_getVariablesX( );

/** Get pointer to the matrix with control variables. */
real_t* nmpc_getVariablesU( );

#if NMPC_NY > 0
/** Get pointer to the matrix with references/measurements. */
real_t* nmpc_getVariablesY( );
#endif

#if NMPC_NYN > 0
/** Get pointer to the vector with references/measurement on the last node. */
real_t* nmpc_getVariablesYN( );
#endif

/** Get pointer to the current state feedback vector. Only applicable for NMPC. */
real_t* nmpc_getVariablesX0( );

/** Print differential variables. */
void nmpc_printDifferentialVariables( );

/** Print control variables. */
void nmpc_printControlVariables( );

/** Print ACADO code generation notice. */
void nmpc_printHeader( );

/*
 * A huge thanks goes to Alexander Domahidi from ETHZ, Switzerland, for 
 * providing us with the following timing routines.
 */

#if !(defined _DSPACE)
#if (defined _WIN32 || defined _WIN64) && !(defined __MINGW32__ || defined __MINGW64__)

/* Use Windows QueryPerformanceCounter for timing. */
#include <Windows.h>

/** A structure for keeping internal timer data. */
typedef struct nmpc_timer_
{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} nmpc_timer;


#elif (defined __APPLE__)

#include "unistd.h"
#include <mach/mach_time.h>

/** A structure for keeping internal timer data. */
typedef struct nmpc_timer_
{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;
} nmpc_timer;

#else

/* Use POSIX clock_gettime() for timing on non-Windows machines. */
#include <time.h>

#if __STDC_VERSION__ >= 199901L
/* C99 mode of operation. */

#include <sys/stat.h>
#include <sys/time.h>

typedef struct nmpc_timer_
{
	struct timeval tic;
	struct timeval toc;
} nmpc_timer;

#else
/* ANSI C */

/** A structure for keeping internal timer data. */
typedef struct nmpc_timer_
{
	struct timespec tic;
	struct timespec toc;
} nmpc_timer;

#endif /* __STDC_VERSION__ >= 199901L */

#endif /* (defined _WIN32 || defined _WIN64) */

/** A function for measurement of the current time. */
void nmpc_tic( nmpc_timer* t );

/** A function which returns the elapsed time. */
real_t nmpc_toc( nmpc_timer* t );

#endif

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* NMPC_AUXILIARY_FUNCTIONS_H */
