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


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 6];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 6 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 6 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 6 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 6 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 6 + 5];

nmpcWorkspace.state[66] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[67] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[68] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[69] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[70] = nmpcVariables.od[lRun1 * 6];
nmpcWorkspace.state[71] = nmpcVariables.od[lRun1 * 6 + 1];
nmpcWorkspace.state[72] = nmpcVariables.od[lRun1 * 6 + 2];
nmpcWorkspace.state[73] = nmpcVariables.od[lRun1 * 6 + 3];
nmpcWorkspace.state[74] = nmpcVariables.od[lRun1 * 6 + 4];
nmpcWorkspace.state[75] = nmpcVariables.od[lRun1 * 6 + 5];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 6] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 6 + 6];
nmpcWorkspace.d[lRun1 * 6 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 6 + 7];
nmpcWorkspace.d[lRun1 * 6 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 6 + 8];
nmpcWorkspace.d[lRun1 * 6 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 6 + 9];
nmpcWorkspace.d[lRun1 * 6 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 6 + 10];
nmpcWorkspace.d[lRun1 * 6 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 6 + 11];

nmpcWorkspace.evGx[lRun1 * 36] = nmpcWorkspace.state[6];
nmpcWorkspace.evGx[lRun1 * 36 + 1] = nmpcWorkspace.state[7];
nmpcWorkspace.evGx[lRun1 * 36 + 2] = nmpcWorkspace.state[8];
nmpcWorkspace.evGx[lRun1 * 36 + 3] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 36 + 4] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 36 + 5] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 36 + 6] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 36 + 7] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 36 + 8] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 36 + 9] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 36 + 10] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 36 + 11] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 36 + 12] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 36 + 13] = nmpcWorkspace.state[19];
nmpcWorkspace.evGx[lRun1 * 36 + 14] = nmpcWorkspace.state[20];
nmpcWorkspace.evGx[lRun1 * 36 + 15] = nmpcWorkspace.state[21];
nmpcWorkspace.evGx[lRun1 * 36 + 16] = nmpcWorkspace.state[22];
nmpcWorkspace.evGx[lRun1 * 36 + 17] = nmpcWorkspace.state[23];
nmpcWorkspace.evGx[lRun1 * 36 + 18] = nmpcWorkspace.state[24];
nmpcWorkspace.evGx[lRun1 * 36 + 19] = nmpcWorkspace.state[25];
nmpcWorkspace.evGx[lRun1 * 36 + 20] = nmpcWorkspace.state[26];
nmpcWorkspace.evGx[lRun1 * 36 + 21] = nmpcWorkspace.state[27];
nmpcWorkspace.evGx[lRun1 * 36 + 22] = nmpcWorkspace.state[28];
nmpcWorkspace.evGx[lRun1 * 36 + 23] = nmpcWorkspace.state[29];
nmpcWorkspace.evGx[lRun1 * 36 + 24] = nmpcWorkspace.state[30];
nmpcWorkspace.evGx[lRun1 * 36 + 25] = nmpcWorkspace.state[31];
nmpcWorkspace.evGx[lRun1 * 36 + 26] = nmpcWorkspace.state[32];
nmpcWorkspace.evGx[lRun1 * 36 + 27] = nmpcWorkspace.state[33];
nmpcWorkspace.evGx[lRun1 * 36 + 28] = nmpcWorkspace.state[34];
nmpcWorkspace.evGx[lRun1 * 36 + 29] = nmpcWorkspace.state[35];
nmpcWorkspace.evGx[lRun1 * 36 + 30] = nmpcWorkspace.state[36];
nmpcWorkspace.evGx[lRun1 * 36 + 31] = nmpcWorkspace.state[37];
nmpcWorkspace.evGx[lRun1 * 36 + 32] = nmpcWorkspace.state[38];
nmpcWorkspace.evGx[lRun1 * 36 + 33] = nmpcWorkspace.state[39];
nmpcWorkspace.evGx[lRun1 * 36 + 34] = nmpcWorkspace.state[40];
nmpcWorkspace.evGx[lRun1 * 36 + 35] = nmpcWorkspace.state[41];

nmpcWorkspace.evGu[lRun1 * 24] = nmpcWorkspace.state[42];
nmpcWorkspace.evGu[lRun1 * 24 + 1] = nmpcWorkspace.state[43];
nmpcWorkspace.evGu[lRun1 * 24 + 2] = nmpcWorkspace.state[44];
nmpcWorkspace.evGu[lRun1 * 24 + 3] = nmpcWorkspace.state[45];
nmpcWorkspace.evGu[lRun1 * 24 + 4] = nmpcWorkspace.state[46];
nmpcWorkspace.evGu[lRun1 * 24 + 5] = nmpcWorkspace.state[47];
nmpcWorkspace.evGu[lRun1 * 24 + 6] = nmpcWorkspace.state[48];
nmpcWorkspace.evGu[lRun1 * 24 + 7] = nmpcWorkspace.state[49];
nmpcWorkspace.evGu[lRun1 * 24 + 8] = nmpcWorkspace.state[50];
nmpcWorkspace.evGu[lRun1 * 24 + 9] = nmpcWorkspace.state[51];
nmpcWorkspace.evGu[lRun1 * 24 + 10] = nmpcWorkspace.state[52];
nmpcWorkspace.evGu[lRun1 * 24 + 11] = nmpcWorkspace.state[53];
nmpcWorkspace.evGu[lRun1 * 24 + 12] = nmpcWorkspace.state[54];
nmpcWorkspace.evGu[lRun1 * 24 + 13] = nmpcWorkspace.state[55];
nmpcWorkspace.evGu[lRun1 * 24 + 14] = nmpcWorkspace.state[56];
nmpcWorkspace.evGu[lRun1 * 24 + 15] = nmpcWorkspace.state[57];
nmpcWorkspace.evGu[lRun1 * 24 + 16] = nmpcWorkspace.state[58];
nmpcWorkspace.evGu[lRun1 * 24 + 17] = nmpcWorkspace.state[59];
nmpcWorkspace.evGu[lRun1 * 24 + 18] = nmpcWorkspace.state[60];
nmpcWorkspace.evGu[lRun1 * 24 + 19] = nmpcWorkspace.state[61];
nmpcWorkspace.evGu[lRun1 * 24 + 20] = nmpcWorkspace.state[62];
nmpcWorkspace.evGu[lRun1 * 24 + 21] = nmpcWorkspace.state[63];
nmpcWorkspace.evGu[lRun1 * 24 + 22] = nmpcWorkspace.state[64];
nmpcWorkspace.evGu[lRun1 * 24 + 23] = nmpcWorkspace.state[65];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
out[7] = u[1];
out[8] = u[2];
out[9] = u[3];
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void nmpc_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[10];
tmpQ1[7] = + tmpQ2[11];
tmpQ1[8] = + tmpQ2[12];
tmpQ1[9] = + tmpQ2[13];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[20];
tmpQ1[13] = + tmpQ2[21];
tmpQ1[14] = + tmpQ2[22];
tmpQ1[15] = + tmpQ2[23];
tmpQ1[16] = + tmpQ2[24];
tmpQ1[17] = + tmpQ2[25];
tmpQ1[18] = + tmpQ2[30];
tmpQ1[19] = + tmpQ2[31];
tmpQ1[20] = + tmpQ2[32];
tmpQ1[21] = + tmpQ2[33];
tmpQ1[22] = + tmpQ2[34];
tmpQ1[23] = + tmpQ2[35];
tmpQ1[24] = + tmpQ2[40];
tmpQ1[25] = + tmpQ2[41];
tmpQ1[26] = + tmpQ2[42];
tmpQ1[27] = + tmpQ2[43];
tmpQ1[28] = + tmpQ2[44];
tmpQ1[29] = + tmpQ2[45];
tmpQ1[30] = + tmpQ2[50];
tmpQ1[31] = + tmpQ2[51];
tmpQ1[32] = + tmpQ2[52];
tmpQ1[33] = + tmpQ2[53];
tmpQ1[34] = + tmpQ2[54];
tmpQ1[35] = + tmpQ2[55];
}

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[60];
tmpR2[1] = +tmpObjS[61];
tmpR2[2] = +tmpObjS[62];
tmpR2[3] = +tmpObjS[63];
tmpR2[4] = +tmpObjS[64];
tmpR2[5] = +tmpObjS[65];
tmpR2[6] = +tmpObjS[66];
tmpR2[7] = +tmpObjS[67];
tmpR2[8] = +tmpObjS[68];
tmpR2[9] = +tmpObjS[69];
tmpR2[10] = +tmpObjS[70];
tmpR2[11] = +tmpObjS[71];
tmpR2[12] = +tmpObjS[72];
tmpR2[13] = +tmpObjS[73];
tmpR2[14] = +tmpObjS[74];
tmpR2[15] = +tmpObjS[75];
tmpR2[16] = +tmpObjS[76];
tmpR2[17] = +tmpObjS[77];
tmpR2[18] = +tmpObjS[78];
tmpR2[19] = +tmpObjS[79];
tmpR2[20] = +tmpObjS[80];
tmpR2[21] = +tmpObjS[81];
tmpR2[22] = +tmpObjS[82];
tmpR2[23] = +tmpObjS[83];
tmpR2[24] = +tmpObjS[84];
tmpR2[25] = +tmpObjS[85];
tmpR2[26] = +tmpObjS[86];
tmpR2[27] = +tmpObjS[87];
tmpR2[28] = +tmpObjS[88];
tmpR2[29] = +tmpObjS[89];
tmpR2[30] = +tmpObjS[90];
tmpR2[31] = +tmpObjS[91];
tmpR2[32] = +tmpObjS[92];
tmpR2[33] = +tmpObjS[93];
tmpR2[34] = +tmpObjS[94];
tmpR2[35] = +tmpObjS[95];
tmpR2[36] = +tmpObjS[96];
tmpR2[37] = +tmpObjS[97];
tmpR2[38] = +tmpObjS[98];
tmpR2[39] = +tmpObjS[99];
tmpR1[0] = + tmpR2[6];
tmpR1[1] = + tmpR2[7];
tmpR1[2] = + tmpR2[8];
tmpR1[3] = + tmpR2[9];
tmpR1[4] = + tmpR2[16];
tmpR1[5] = + tmpR2[17];
tmpR1[6] = + tmpR2[18];
tmpR1[7] = + tmpR2[19];
tmpR1[8] = + tmpR2[26];
tmpR1[9] = + tmpR2[27];
tmpR1[10] = + tmpR2[28];
tmpR1[11] = + tmpR2[29];
tmpR1[12] = + tmpR2[36];
tmpR1[13] = + tmpR2[37];
tmpR1[14] = + tmpR2[38];
tmpR1[15] = + tmpR2[39];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 6];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 6 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 6 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 6 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 6 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 6 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[7] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[runObj * 6];
nmpcWorkspace.objValueIn[11] = nmpcVariables.od[runObj * 6 + 1];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[runObj * 6 + 2];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[runObj * 6 + 3];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[runObj * 6 + 4];
nmpcWorkspace.objValueIn[15] = nmpcVariables.od[runObj * 6 + 5];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 10] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 10 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 10 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 10 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 10 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 10 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 10 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 10 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 10 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 10 + 9] = nmpcWorkspace.objValueOut[9];

nmpc_setObjQ1Q2( nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 36 ]), &(nmpcWorkspace.Q2[ runObj * 60 ]) );

nmpc_setObjR1R2( nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 40 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[180];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[181];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[182];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[183];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[184];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[185];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[180];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[181];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[182];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[183];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[184];
nmpcWorkspace.objValueIn[11] = nmpcVariables.od[185];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5];
dNew[1] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2] + Gx1[9]*dOld[3] + Gx1[10]*dOld[4] + Gx1[11]*dOld[5];
dNew[2] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5];
dNew[3] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5];
dNew[4] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5];
dNew[5] += + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[24] + Gx1[5]*Gx2[30];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[25] + Gx1[5]*Gx2[31];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[20] + Gx1[4]*Gx2[26] + Gx1[5]*Gx2[32];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[27] + Gx1[5]*Gx2[33];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[34];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[35];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[30];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[31];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[8]*Gx2[14] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[32];
Gx3[9] = + Gx1[6]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[21] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[33];
Gx3[10] = + Gx1[6]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[22] + Gx1[10]*Gx2[28] + Gx1[11]*Gx2[34];
Gx3[11] = + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[23] + Gx1[10]*Gx2[29] + Gx1[11]*Gx2[35];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[15]*Gx2[18] + Gx1[16]*Gx2[24] + Gx1[17]*Gx2[30];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[15]*Gx2[19] + Gx1[16]*Gx2[25] + Gx1[17]*Gx2[31];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[15]*Gx2[20] + Gx1[16]*Gx2[26] + Gx1[17]*Gx2[32];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[15]*Gx2[21] + Gx1[16]*Gx2[27] + Gx1[17]*Gx2[33];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[15]*Gx2[22] + Gx1[16]*Gx2[28] + Gx1[17]*Gx2[34];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[15]*Gx2[23] + Gx1[16]*Gx2[29] + Gx1[17]*Gx2[35];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[6] + Gx1[20]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[30];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[7] + Gx1[20]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[31];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[8] + Gx1[20]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[32];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[33];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[34];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[35];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[6] + Gx1[26]*Gx2[12] + Gx1[27]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[29]*Gx2[30];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[7] + Gx1[26]*Gx2[13] + Gx1[27]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[29]*Gx2[31];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[14] + Gx1[27]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[29]*Gx2[32];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[15] + Gx1[27]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[29]*Gx2[33];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[29]*Gx2[34];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[29]*Gx2[35];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[6] + Gx1[32]*Gx2[12] + Gx1[33]*Gx2[18] + Gx1[34]*Gx2[24] + Gx1[35]*Gx2[30];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[7] + Gx1[32]*Gx2[13] + Gx1[33]*Gx2[19] + Gx1[34]*Gx2[25] + Gx1[35]*Gx2[31];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[8] + Gx1[32]*Gx2[14] + Gx1[33]*Gx2[20] + Gx1[34]*Gx2[26] + Gx1[35]*Gx2[32];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[9] + Gx1[32]*Gx2[15] + Gx1[33]*Gx2[21] + Gx1[34]*Gx2[27] + Gx1[35]*Gx2[33];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[16] + Gx1[33]*Gx2[22] + Gx1[34]*Gx2[28] + Gx1[35]*Gx2[34];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[17] + Gx1[33]*Gx2[23] + Gx1[34]*Gx2[29] + Gx1[35]*Gx2[35];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23];
Gu2[4] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[8] + Gx1[9]*Gu1[12] + Gx1[10]*Gu1[16] + Gx1[11]*Gu1[20];
Gu2[5] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[9] + Gx1[9]*Gu1[13] + Gx1[10]*Gu1[17] + Gx1[11]*Gu1[21];
Gu2[6] = + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[6] + Gx1[8]*Gu1[10] + Gx1[9]*Gu1[14] + Gx1[10]*Gu1[18] + Gx1[11]*Gu1[22];
Gu2[7] = + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[7] + Gx1[8]*Gu1[11] + Gx1[9]*Gu1[15] + Gx1[10]*Gu1[19] + Gx1[11]*Gu1[23];
Gu2[8] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20];
Gu2[9] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21];
Gu2[10] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22];
Gu2[11] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23];
Gu2[12] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[12] + Gx1[22]*Gu1[16] + Gx1[23]*Gu1[20];
Gu2[13] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[21]*Gu1[13] + Gx1[22]*Gu1[17] + Gx1[23]*Gu1[21];
Gu2[14] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[21]*Gu1[14] + Gx1[22]*Gu1[18] + Gx1[23]*Gu1[22];
Gu2[15] = + Gx1[18]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[21]*Gu1[15] + Gx1[22]*Gu1[19] + Gx1[23]*Gu1[23];
Gu2[16] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20];
Gu2[17] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21];
Gu2[18] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22];
Gu2[19] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23];
Gu2[20] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[34]*Gu1[16] + Gx1[35]*Gu1[20];
Gu2[21] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[34]*Gu1[17] + Gx1[35]*Gu1[21];
Gu2[22] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[34]*Gu1[18] + Gx1[35]*Gu1[22];
Gu2[23] = + Gx1[30]*Gu1[3] + Gx1[31]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[34]*Gu1[19] + Gx1[35]*Gu1[23];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 6)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 7)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 8)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 9)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 6)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 7)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 8)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 9)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 6)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 7)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 8)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 9)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 6)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 7)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 8)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 9)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 6)] = R11[0];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 7)] = R11[1];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 8)] = R11[2];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 9)] = R11[3];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 6)] = R11[4];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 7)] = R11[5];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 8)] = R11[6];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 9)] = R11[7];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 6)] = R11[8];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 7)] = R11[9];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 8)] = R11[10];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 9)] = R11[11];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 6)] = R11[12];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 7)] = R11[13];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 8)] = R11[14];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 9)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 6)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 7)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 6)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 7)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 6)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 7)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 6)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 7)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 6)] = nmpcWorkspace.H[(iCol * 504 + 756) + (iRow * 4 + 6)];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 7)] = nmpcWorkspace.H[(iCol * 504 + 882) + (iRow * 4 + 6)];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 504 + 1008) + (iRow * 4 + 6)];
nmpcWorkspace.H[(iRow * 504 + 756) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 504 + 1134) + (iRow * 4 + 6)];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 6)] = nmpcWorkspace.H[(iCol * 504 + 756) + (iRow * 4 + 7)];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 7)] = nmpcWorkspace.H[(iCol * 504 + 882) + (iRow * 4 + 7)];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 504 + 1008) + (iRow * 4 + 7)];
nmpcWorkspace.H[(iRow * 504 + 882) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 504 + 1134) + (iRow * 4 + 7)];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 6)] = nmpcWorkspace.H[(iCol * 504 + 756) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 7)] = nmpcWorkspace.H[(iCol * 504 + 882) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 504 + 1008) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 504 + 1008) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 504 + 1134) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 6)] = nmpcWorkspace.H[(iCol * 504 + 756) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 7)] = nmpcWorkspace.H[(iCol * 504 + 882) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 504 + 1008) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 504 + 1134) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 504 + 1134) + (iRow * 4 + 9)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5];
dNew[1] = + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2] + Gx1[9]*dOld[3] + Gx1[10]*dOld[4] + Gx1[11]*dOld[5];
dNew[2] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5];
dNew[3] = + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5];
dNew[4] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5];
dNew[5] = + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5];
dNew[1] = + nmpcWorkspace.QN1[6]*dOld[0] + nmpcWorkspace.QN1[7]*dOld[1] + nmpcWorkspace.QN1[8]*dOld[2] + nmpcWorkspace.QN1[9]*dOld[3] + nmpcWorkspace.QN1[10]*dOld[4] + nmpcWorkspace.QN1[11]*dOld[5];
dNew[2] = + nmpcWorkspace.QN1[12]*dOld[0] + nmpcWorkspace.QN1[13]*dOld[1] + nmpcWorkspace.QN1[14]*dOld[2] + nmpcWorkspace.QN1[15]*dOld[3] + nmpcWorkspace.QN1[16]*dOld[4] + nmpcWorkspace.QN1[17]*dOld[5];
dNew[3] = + nmpcWorkspace.QN1[18]*dOld[0] + nmpcWorkspace.QN1[19]*dOld[1] + nmpcWorkspace.QN1[20]*dOld[2] + nmpcWorkspace.QN1[21]*dOld[3] + nmpcWorkspace.QN1[22]*dOld[4] + nmpcWorkspace.QN1[23]*dOld[5];
dNew[4] = + nmpcWorkspace.QN1[24]*dOld[0] + nmpcWorkspace.QN1[25]*dOld[1] + nmpcWorkspace.QN1[26]*dOld[2] + nmpcWorkspace.QN1[27]*dOld[3] + nmpcWorkspace.QN1[28]*dOld[4] + nmpcWorkspace.QN1[29]*dOld[5];
dNew[5] = + nmpcWorkspace.QN1[30]*dOld[0] + nmpcWorkspace.QN1[31]*dOld[1] + nmpcWorkspace.QN1[32]*dOld[2] + nmpcWorkspace.QN1[33]*dOld[3] + nmpcWorkspace.QN1[34]*dOld[4] + nmpcWorkspace.QN1[35]*dOld[5];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9];
RDy1[1] = + R2[10]*Dy1[0] + R2[11]*Dy1[1] + R2[12]*Dy1[2] + R2[13]*Dy1[3] + R2[14]*Dy1[4] + R2[15]*Dy1[5] + R2[16]*Dy1[6] + R2[17]*Dy1[7] + R2[18]*Dy1[8] + R2[19]*Dy1[9];
RDy1[2] = + R2[20]*Dy1[0] + R2[21]*Dy1[1] + R2[22]*Dy1[2] + R2[23]*Dy1[3] + R2[24]*Dy1[4] + R2[25]*Dy1[5] + R2[26]*Dy1[6] + R2[27]*Dy1[7] + R2[28]*Dy1[8] + R2[29]*Dy1[9];
RDy1[3] = + R2[30]*Dy1[0] + R2[31]*Dy1[1] + R2[32]*Dy1[2] + R2[33]*Dy1[3] + R2[34]*Dy1[4] + R2[35]*Dy1[5] + R2[36]*Dy1[6] + R2[37]*Dy1[7] + R2[38]*Dy1[8] + R2[39]*Dy1[9];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9];
QDy1[1] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4] + Q2[15]*Dy1[5] + Q2[16]*Dy1[6] + Q2[17]*Dy1[7] + Q2[18]*Dy1[8] + Q2[19]*Dy1[9];
QDy1[2] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4] + Q2[25]*Dy1[5] + Q2[26]*Dy1[6] + Q2[27]*Dy1[7] + Q2[28]*Dy1[8] + Q2[29]*Dy1[9];
QDy1[3] = + Q2[30]*Dy1[0] + Q2[31]*Dy1[1] + Q2[32]*Dy1[2] + Q2[33]*Dy1[3] + Q2[34]*Dy1[4] + Q2[35]*Dy1[5] + Q2[36]*Dy1[6] + Q2[37]*Dy1[7] + Q2[38]*Dy1[8] + Q2[39]*Dy1[9];
QDy1[4] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7] + Q2[48]*Dy1[8] + Q2[49]*Dy1[9];
QDy1[5] = + Q2[50]*Dy1[0] + Q2[51]*Dy1[1] + Q2[52]*Dy1[2] + Q2[53]*Dy1[3] + Q2[54]*Dy1[4] + Q2[55]*Dy1[5] + Q2[56]*Dy1[6] + Q2[57]*Dy1[7] + Q2[58]*Dy1[8] + Q2[59]*Dy1[9];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[6] + E1[8]*Gx1[12] + E1[12]*Gx1[18] + E1[16]*Gx1[24] + E1[20]*Gx1[30];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[7] + E1[8]*Gx1[13] + E1[12]*Gx1[19] + E1[16]*Gx1[25] + E1[20]*Gx1[31];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[8] + E1[8]*Gx1[14] + E1[12]*Gx1[20] + E1[16]*Gx1[26] + E1[20]*Gx1[32];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[9] + E1[8]*Gx1[15] + E1[12]*Gx1[21] + E1[16]*Gx1[27] + E1[20]*Gx1[33];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[10] + E1[8]*Gx1[16] + E1[12]*Gx1[22] + E1[16]*Gx1[28] + E1[20]*Gx1[34];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[11] + E1[8]*Gx1[17] + E1[12]*Gx1[23] + E1[16]*Gx1[29] + E1[20]*Gx1[35];
H101[6] += + E1[1]*Gx1[0] + E1[5]*Gx1[6] + E1[9]*Gx1[12] + E1[13]*Gx1[18] + E1[17]*Gx1[24] + E1[21]*Gx1[30];
H101[7] += + E1[1]*Gx1[1] + E1[5]*Gx1[7] + E1[9]*Gx1[13] + E1[13]*Gx1[19] + E1[17]*Gx1[25] + E1[21]*Gx1[31];
H101[8] += + E1[1]*Gx1[2] + E1[5]*Gx1[8] + E1[9]*Gx1[14] + E1[13]*Gx1[20] + E1[17]*Gx1[26] + E1[21]*Gx1[32];
H101[9] += + E1[1]*Gx1[3] + E1[5]*Gx1[9] + E1[9]*Gx1[15] + E1[13]*Gx1[21] + E1[17]*Gx1[27] + E1[21]*Gx1[33];
H101[10] += + E1[1]*Gx1[4] + E1[5]*Gx1[10] + E1[9]*Gx1[16] + E1[13]*Gx1[22] + E1[17]*Gx1[28] + E1[21]*Gx1[34];
H101[11] += + E1[1]*Gx1[5] + E1[5]*Gx1[11] + E1[9]*Gx1[17] + E1[13]*Gx1[23] + E1[17]*Gx1[29] + E1[21]*Gx1[35];
H101[12] += + E1[2]*Gx1[0] + E1[6]*Gx1[6] + E1[10]*Gx1[12] + E1[14]*Gx1[18] + E1[18]*Gx1[24] + E1[22]*Gx1[30];
H101[13] += + E1[2]*Gx1[1] + E1[6]*Gx1[7] + E1[10]*Gx1[13] + E1[14]*Gx1[19] + E1[18]*Gx1[25] + E1[22]*Gx1[31];
H101[14] += + E1[2]*Gx1[2] + E1[6]*Gx1[8] + E1[10]*Gx1[14] + E1[14]*Gx1[20] + E1[18]*Gx1[26] + E1[22]*Gx1[32];
H101[15] += + E1[2]*Gx1[3] + E1[6]*Gx1[9] + E1[10]*Gx1[15] + E1[14]*Gx1[21] + E1[18]*Gx1[27] + E1[22]*Gx1[33];
H101[16] += + E1[2]*Gx1[4] + E1[6]*Gx1[10] + E1[10]*Gx1[16] + E1[14]*Gx1[22] + E1[18]*Gx1[28] + E1[22]*Gx1[34];
H101[17] += + E1[2]*Gx1[5] + E1[6]*Gx1[11] + E1[10]*Gx1[17] + E1[14]*Gx1[23] + E1[18]*Gx1[29] + E1[22]*Gx1[35];
H101[18] += + E1[3]*Gx1[0] + E1[7]*Gx1[6] + E1[11]*Gx1[12] + E1[15]*Gx1[18] + E1[19]*Gx1[24] + E1[23]*Gx1[30];
H101[19] += + E1[3]*Gx1[1] + E1[7]*Gx1[7] + E1[11]*Gx1[13] + E1[15]*Gx1[19] + E1[19]*Gx1[25] + E1[23]*Gx1[31];
H101[20] += + E1[3]*Gx1[2] + E1[7]*Gx1[8] + E1[11]*Gx1[14] + E1[15]*Gx1[20] + E1[19]*Gx1[26] + E1[23]*Gx1[32];
H101[21] += + E1[3]*Gx1[3] + E1[7]*Gx1[9] + E1[11]*Gx1[15] + E1[15]*Gx1[21] + E1[19]*Gx1[27] + E1[23]*Gx1[33];
H101[22] += + E1[3]*Gx1[4] + E1[7]*Gx1[10] + E1[11]*Gx1[16] + E1[15]*Gx1[22] + E1[19]*Gx1[28] + E1[23]*Gx1[34];
H101[23] += + E1[3]*Gx1[5] + E1[7]*Gx1[11] + E1[11]*Gx1[17] + E1[15]*Gx1[23] + E1[19]*Gx1[29] + E1[23]*Gx1[35];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 24; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[126] = 0.0000000000000000e+00;
nmpcWorkspace.H[127] = 0.0000000000000000e+00;
nmpcWorkspace.H[128] = 0.0000000000000000e+00;
nmpcWorkspace.H[129] = 0.0000000000000000e+00;
nmpcWorkspace.H[130] = 0.0000000000000000e+00;
nmpcWorkspace.H[131] = 0.0000000000000000e+00;
nmpcWorkspace.H[252] = 0.0000000000000000e+00;
nmpcWorkspace.H[253] = 0.0000000000000000e+00;
nmpcWorkspace.H[254] = 0.0000000000000000e+00;
nmpcWorkspace.H[255] = 0.0000000000000000e+00;
nmpcWorkspace.H[256] = 0.0000000000000000e+00;
nmpcWorkspace.H[257] = 0.0000000000000000e+00;
nmpcWorkspace.H[378] = 0.0000000000000000e+00;
nmpcWorkspace.H[379] = 0.0000000000000000e+00;
nmpcWorkspace.H[380] = 0.0000000000000000e+00;
nmpcWorkspace.H[381] = 0.0000000000000000e+00;
nmpcWorkspace.H[382] = 0.0000000000000000e+00;
nmpcWorkspace.H[383] = 0.0000000000000000e+00;
nmpcWorkspace.H[504] = 0.0000000000000000e+00;
nmpcWorkspace.H[505] = 0.0000000000000000e+00;
nmpcWorkspace.H[506] = 0.0000000000000000e+00;
nmpcWorkspace.H[507] = 0.0000000000000000e+00;
nmpcWorkspace.H[508] = 0.0000000000000000e+00;
nmpcWorkspace.H[509] = 0.0000000000000000e+00;
nmpcWorkspace.H[630] = 0.0000000000000000e+00;
nmpcWorkspace.H[631] = 0.0000000000000000e+00;
nmpcWorkspace.H[632] = 0.0000000000000000e+00;
nmpcWorkspace.H[633] = 0.0000000000000000e+00;
nmpcWorkspace.H[634] = 0.0000000000000000e+00;
nmpcWorkspace.H[635] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[6]*Gx2[6] + Gx1[12]*Gx2[12] + Gx1[18]*Gx2[18] + Gx1[24]*Gx2[24] + Gx1[30]*Gx2[30];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[6]*Gx2[7] + Gx1[12]*Gx2[13] + Gx1[18]*Gx2[19] + Gx1[24]*Gx2[25] + Gx1[30]*Gx2[31];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[6]*Gx2[8] + Gx1[12]*Gx2[14] + Gx1[18]*Gx2[20] + Gx1[24]*Gx2[26] + Gx1[30]*Gx2[32];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[6]*Gx2[9] + Gx1[12]*Gx2[15] + Gx1[18]*Gx2[21] + Gx1[24]*Gx2[27] + Gx1[30]*Gx2[33];
nmpcWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[6]*Gx2[10] + Gx1[12]*Gx2[16] + Gx1[18]*Gx2[22] + Gx1[24]*Gx2[28] + Gx1[30]*Gx2[34];
nmpcWorkspace.H[5] += + Gx1[0]*Gx2[5] + Gx1[6]*Gx2[11] + Gx1[12]*Gx2[17] + Gx1[18]*Gx2[23] + Gx1[24]*Gx2[29] + Gx1[30]*Gx2[35];
nmpcWorkspace.H[126] += + Gx1[1]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[13]*Gx2[12] + Gx1[19]*Gx2[18] + Gx1[25]*Gx2[24] + Gx1[31]*Gx2[30];
nmpcWorkspace.H[127] += + Gx1[1]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[13]*Gx2[13] + Gx1[19]*Gx2[19] + Gx1[25]*Gx2[25] + Gx1[31]*Gx2[31];
nmpcWorkspace.H[128] += + Gx1[1]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[13]*Gx2[14] + Gx1[19]*Gx2[20] + Gx1[25]*Gx2[26] + Gx1[31]*Gx2[32];
nmpcWorkspace.H[129] += + Gx1[1]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[13]*Gx2[15] + Gx1[19]*Gx2[21] + Gx1[25]*Gx2[27] + Gx1[31]*Gx2[33];
nmpcWorkspace.H[130] += + Gx1[1]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[13]*Gx2[16] + Gx1[19]*Gx2[22] + Gx1[25]*Gx2[28] + Gx1[31]*Gx2[34];
nmpcWorkspace.H[131] += + Gx1[1]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[13]*Gx2[17] + Gx1[19]*Gx2[23] + Gx1[25]*Gx2[29] + Gx1[31]*Gx2[35];
nmpcWorkspace.H[252] += + Gx1[2]*Gx2[0] + Gx1[8]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[20]*Gx2[18] + Gx1[26]*Gx2[24] + Gx1[32]*Gx2[30];
nmpcWorkspace.H[253] += + Gx1[2]*Gx2[1] + Gx1[8]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[20]*Gx2[19] + Gx1[26]*Gx2[25] + Gx1[32]*Gx2[31];
nmpcWorkspace.H[254] += + Gx1[2]*Gx2[2] + Gx1[8]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[20]*Gx2[20] + Gx1[26]*Gx2[26] + Gx1[32]*Gx2[32];
nmpcWorkspace.H[255] += + Gx1[2]*Gx2[3] + Gx1[8]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[20]*Gx2[21] + Gx1[26]*Gx2[27] + Gx1[32]*Gx2[33];
nmpcWorkspace.H[256] += + Gx1[2]*Gx2[4] + Gx1[8]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[20]*Gx2[22] + Gx1[26]*Gx2[28] + Gx1[32]*Gx2[34];
nmpcWorkspace.H[257] += + Gx1[2]*Gx2[5] + Gx1[8]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[20]*Gx2[23] + Gx1[26]*Gx2[29] + Gx1[32]*Gx2[35];
nmpcWorkspace.H[378] += + Gx1[3]*Gx2[0] + Gx1[9]*Gx2[6] + Gx1[15]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[27]*Gx2[24] + Gx1[33]*Gx2[30];
nmpcWorkspace.H[379] += + Gx1[3]*Gx2[1] + Gx1[9]*Gx2[7] + Gx1[15]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[27]*Gx2[25] + Gx1[33]*Gx2[31];
nmpcWorkspace.H[380] += + Gx1[3]*Gx2[2] + Gx1[9]*Gx2[8] + Gx1[15]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[27]*Gx2[26] + Gx1[33]*Gx2[32];
nmpcWorkspace.H[381] += + Gx1[3]*Gx2[3] + Gx1[9]*Gx2[9] + Gx1[15]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[27]*Gx2[27] + Gx1[33]*Gx2[33];
nmpcWorkspace.H[382] += + Gx1[3]*Gx2[4] + Gx1[9]*Gx2[10] + Gx1[15]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[27]*Gx2[28] + Gx1[33]*Gx2[34];
nmpcWorkspace.H[383] += + Gx1[3]*Gx2[5] + Gx1[9]*Gx2[11] + Gx1[15]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[27]*Gx2[29] + Gx1[33]*Gx2[35];
nmpcWorkspace.H[504] += + Gx1[4]*Gx2[0] + Gx1[10]*Gx2[6] + Gx1[16]*Gx2[12] + Gx1[22]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[34]*Gx2[30];
nmpcWorkspace.H[505] += + Gx1[4]*Gx2[1] + Gx1[10]*Gx2[7] + Gx1[16]*Gx2[13] + Gx1[22]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[34]*Gx2[31];
nmpcWorkspace.H[506] += + Gx1[4]*Gx2[2] + Gx1[10]*Gx2[8] + Gx1[16]*Gx2[14] + Gx1[22]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[34]*Gx2[32];
nmpcWorkspace.H[507] += + Gx1[4]*Gx2[3] + Gx1[10]*Gx2[9] + Gx1[16]*Gx2[15] + Gx1[22]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[34]*Gx2[33];
nmpcWorkspace.H[508] += + Gx1[4]*Gx2[4] + Gx1[10]*Gx2[10] + Gx1[16]*Gx2[16] + Gx1[22]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[34]*Gx2[34];
nmpcWorkspace.H[509] += + Gx1[4]*Gx2[5] + Gx1[10]*Gx2[11] + Gx1[16]*Gx2[17] + Gx1[22]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[34]*Gx2[35];
nmpcWorkspace.H[630] += + Gx1[5]*Gx2[0] + Gx1[11]*Gx2[6] + Gx1[17]*Gx2[12] + Gx1[23]*Gx2[18] + Gx1[29]*Gx2[24] + Gx1[35]*Gx2[30];
nmpcWorkspace.H[631] += + Gx1[5]*Gx2[1] + Gx1[11]*Gx2[7] + Gx1[17]*Gx2[13] + Gx1[23]*Gx2[19] + Gx1[29]*Gx2[25] + Gx1[35]*Gx2[31];
nmpcWorkspace.H[632] += + Gx1[5]*Gx2[2] + Gx1[11]*Gx2[8] + Gx1[17]*Gx2[14] + Gx1[23]*Gx2[20] + Gx1[29]*Gx2[26] + Gx1[35]*Gx2[32];
nmpcWorkspace.H[633] += + Gx1[5]*Gx2[3] + Gx1[11]*Gx2[9] + Gx1[17]*Gx2[15] + Gx1[23]*Gx2[21] + Gx1[29]*Gx2[27] + Gx1[35]*Gx2[33];
nmpcWorkspace.H[634] += + Gx1[5]*Gx2[4] + Gx1[11]*Gx2[10] + Gx1[17]*Gx2[16] + Gx1[23]*Gx2[22] + Gx1[29]*Gx2[28] + Gx1[35]*Gx2[34];
nmpcWorkspace.H[635] += + Gx1[5]*Gx2[5] + Gx1[11]*Gx2[11] + Gx1[17]*Gx2[17] + Gx1[23]*Gx2[23] + Gx1[29]*Gx2[29] + Gx1[35]*Gx2[35];
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 36 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 6-6 ]), &(nmpcWorkspace.evGx[ lRun1 * 36 ]), &(nmpcWorkspace.d[ lRun1 * 6 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 36-36 ]), &(nmpcWorkspace.evGx[ lRun1 * 36 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 24 ]), &(nmpcWorkspace.E[ lRun3 * 24 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 24 ]), &(nmpcWorkspace.E[ lRun3 * 24 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 36 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 72 ]), &(nmpcWorkspace.evGx[ 36 ]), &(nmpcWorkspace.QGx[ 36 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 108 ]), &(nmpcWorkspace.evGx[ 72 ]), &(nmpcWorkspace.QGx[ 72 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 144 ]), &(nmpcWorkspace.evGx[ 108 ]), &(nmpcWorkspace.QGx[ 108 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 180 ]), &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 216 ]), &(nmpcWorkspace.evGx[ 180 ]), &(nmpcWorkspace.QGx[ 180 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 252 ]), &(nmpcWorkspace.evGx[ 216 ]), &(nmpcWorkspace.QGx[ 216 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.evGx[ 252 ]), &(nmpcWorkspace.QGx[ 252 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 324 ]), &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 360 ]), &(nmpcWorkspace.evGx[ 324 ]), &(nmpcWorkspace.QGx[ 324 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 396 ]), &(nmpcWorkspace.evGx[ 360 ]), &(nmpcWorkspace.QGx[ 360 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.evGx[ 396 ]), &(nmpcWorkspace.QGx[ 396 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 468 ]), &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 504 ]), &(nmpcWorkspace.evGx[ 468 ]), &(nmpcWorkspace.QGx[ 468 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 540 ]), &(nmpcWorkspace.evGx[ 504 ]), &(nmpcWorkspace.QGx[ 504 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.evGx[ 540 ]), &(nmpcWorkspace.QGx[ 540 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 612 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 648 ]), &(nmpcWorkspace.evGx[ 612 ]), &(nmpcWorkspace.QGx[ 612 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 684 ]), &(nmpcWorkspace.evGx[ 648 ]), &(nmpcWorkspace.QGx[ 648 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 720 ]), &(nmpcWorkspace.evGx[ 684 ]), &(nmpcWorkspace.QGx[ 684 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 756 ]), &(nmpcWorkspace.evGx[ 720 ]), &(nmpcWorkspace.QGx[ 720 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 792 ]), &(nmpcWorkspace.evGx[ 756 ]), &(nmpcWorkspace.QGx[ 756 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 828 ]), &(nmpcWorkspace.evGx[ 792 ]), &(nmpcWorkspace.QGx[ 792 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 864 ]), &(nmpcWorkspace.evGx[ 828 ]), &(nmpcWorkspace.QGx[ 828 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 900 ]), &(nmpcWorkspace.evGx[ 864 ]), &(nmpcWorkspace.QGx[ 864 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 936 ]), &(nmpcWorkspace.evGx[ 900 ]), &(nmpcWorkspace.QGx[ 900 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 972 ]), &(nmpcWorkspace.evGx[ 936 ]), &(nmpcWorkspace.QGx[ 936 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1008 ]), &(nmpcWorkspace.evGx[ 972 ]), &(nmpcWorkspace.QGx[ 972 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1044 ]), &(nmpcWorkspace.evGx[ 1008 ]), &(nmpcWorkspace.QGx[ 1008 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 1044 ]), &(nmpcWorkspace.QGx[ 1044 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 36 + 36 ]), &(nmpcWorkspace.E[ lRun3 * 24 ]), &(nmpcWorkspace.QE[ lRun3 * 24 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 24 ]), &(nmpcWorkspace.QE[ lRun3 * 24 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 36 ]), &(nmpcWorkspace.QGx[ 36 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 72 ]), &(nmpcWorkspace.QGx[ 72 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 108 ]), &(nmpcWorkspace.QGx[ 108 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 180 ]), &(nmpcWorkspace.QGx[ 180 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 216 ]), &(nmpcWorkspace.QGx[ 216 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 252 ]), &(nmpcWorkspace.QGx[ 252 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 324 ]), &(nmpcWorkspace.QGx[ 324 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 360 ]), &(nmpcWorkspace.QGx[ 360 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 396 ]), &(nmpcWorkspace.QGx[ 396 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 468 ]), &(nmpcWorkspace.QGx[ 468 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 504 ]), &(nmpcWorkspace.QGx[ 504 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 540 ]), &(nmpcWorkspace.QGx[ 540 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 612 ]), &(nmpcWorkspace.QGx[ 612 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 648 ]), &(nmpcWorkspace.QGx[ 648 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 684 ]), &(nmpcWorkspace.QGx[ 684 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 720 ]), &(nmpcWorkspace.QGx[ 720 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 756 ]), &(nmpcWorkspace.QGx[ 756 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 792 ]), &(nmpcWorkspace.QGx[ 792 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 828 ]), &(nmpcWorkspace.QGx[ 828 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 864 ]), &(nmpcWorkspace.QGx[ 864 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 900 ]), &(nmpcWorkspace.QGx[ 900 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 936 ]), &(nmpcWorkspace.QGx[ 936 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 972 ]), &(nmpcWorkspace.QGx[ 972 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1008 ]), &(nmpcWorkspace.QGx[ 1008 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1044 ]), &(nmpcWorkspace.QGx[ 1044 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 24 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 24 ]), &(nmpcWorkspace.evGx[ lRun2 * 36 ]), &(nmpcWorkspace.H10[ lRun1 * 24 ]) );
}
}

for (lRun1 = 0;lRun1 < 6; ++lRun1)
for (lRun2 = 0;lRun2 < 120; ++lRun2)
nmpcWorkspace.H[(lRun1 * 126) + (lRun2 + 6)] = nmpcWorkspace.H10[(lRun2 * 6) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 24 ]), &(nmpcWorkspace.QE[ lRun5 * 24 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 24 ]), &(nmpcWorkspace.QE[ lRun5 * 24 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 120; ++lRun1)
for (lRun2 = 0;lRun2 < 6; ++lRun2)
nmpcWorkspace.H[(lRun1 * 126 + 756) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 6) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 36 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 72 ]), &(nmpcWorkspace.d[ 6 ]), &(nmpcWorkspace.Qd[ 6 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 108 ]), &(nmpcWorkspace.d[ 12 ]), &(nmpcWorkspace.Qd[ 12 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 144 ]), &(nmpcWorkspace.d[ 18 ]), &(nmpcWorkspace.Qd[ 18 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 180 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 216 ]), &(nmpcWorkspace.d[ 30 ]), &(nmpcWorkspace.Qd[ 30 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 252 ]), &(nmpcWorkspace.d[ 36 ]), &(nmpcWorkspace.Qd[ 36 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.d[ 42 ]), &(nmpcWorkspace.Qd[ 42 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 324 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 360 ]), &(nmpcWorkspace.d[ 54 ]), &(nmpcWorkspace.Qd[ 54 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 396 ]), &(nmpcWorkspace.d[ 60 ]), &(nmpcWorkspace.Qd[ 60 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.d[ 66 ]), &(nmpcWorkspace.Qd[ 66 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 468 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 504 ]), &(nmpcWorkspace.d[ 78 ]), &(nmpcWorkspace.Qd[ 78 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 540 ]), &(nmpcWorkspace.d[ 84 ]), &(nmpcWorkspace.Qd[ 84 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.d[ 90 ]), &(nmpcWorkspace.Qd[ 90 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 612 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 648 ]), &(nmpcWorkspace.d[ 102 ]), &(nmpcWorkspace.Qd[ 102 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 684 ]), &(nmpcWorkspace.d[ 108 ]), &(nmpcWorkspace.Qd[ 108 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 720 ]), &(nmpcWorkspace.d[ 114 ]), &(nmpcWorkspace.Qd[ 114 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 756 ]), &(nmpcWorkspace.d[ 120 ]), &(nmpcWorkspace.Qd[ 120 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 792 ]), &(nmpcWorkspace.d[ 126 ]), &(nmpcWorkspace.Qd[ 126 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 828 ]), &(nmpcWorkspace.d[ 132 ]), &(nmpcWorkspace.Qd[ 132 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 864 ]), &(nmpcWorkspace.d[ 138 ]), &(nmpcWorkspace.Qd[ 138 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 900 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 936 ]), &(nmpcWorkspace.d[ 150 ]), &(nmpcWorkspace.Qd[ 150 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 972 ]), &(nmpcWorkspace.d[ 156 ]), &(nmpcWorkspace.Qd[ 156 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1008 ]), &(nmpcWorkspace.d[ 162 ]), &(nmpcWorkspace.Qd[ 162 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1044 ]), &(nmpcWorkspace.d[ 168 ]), &(nmpcWorkspace.Qd[ 168 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 174 ]), &(nmpcWorkspace.Qd[ 174 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 36 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 72 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 108 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 180 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 216 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 252 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 288 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 324 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 360 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 396 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 432 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 468 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 504 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 540 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 612 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 648 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 684 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 720 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 756 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 792 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 828 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 864 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 900 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 936 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 972 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1008 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1044 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 24 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 6 ]) );
}
}
nmpcWorkspace.lb[6] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[0];
nmpcWorkspace.lb[7] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[1];
nmpcWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.lb[9] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[3];
nmpcWorkspace.lb[10] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[4];
nmpcWorkspace.lb[11] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[5];
nmpcWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.lb[13] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[7];
nmpcWorkspace.lb[14] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[8];
nmpcWorkspace.lb[15] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[9];
nmpcWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.lb[17] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[11];
nmpcWorkspace.lb[18] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[12];
nmpcWorkspace.lb[19] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[13];
nmpcWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.lb[21] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[15];
nmpcWorkspace.lb[22] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[16];
nmpcWorkspace.lb[23] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[17];
nmpcWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.lb[25] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[19];
nmpcWorkspace.lb[26] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[20];
nmpcWorkspace.lb[27] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[21];
nmpcWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.lb[29] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[23];
nmpcWorkspace.lb[30] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[24];
nmpcWorkspace.lb[31] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[25];
nmpcWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.lb[33] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[27];
nmpcWorkspace.lb[34] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[28];
nmpcWorkspace.lb[35] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[29];
nmpcWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.lb[37] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[31];
nmpcWorkspace.lb[38] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[32];
nmpcWorkspace.lb[39] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[33];
nmpcWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.lb[41] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[35];
nmpcWorkspace.lb[42] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[36];
nmpcWorkspace.lb[43] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[37];
nmpcWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.lb[45] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[39];
nmpcWorkspace.lb[46] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[40];
nmpcWorkspace.lb[47] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[41];
nmpcWorkspace.lb[48] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.lb[49] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[43];
nmpcWorkspace.lb[50] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[44];
nmpcWorkspace.lb[51] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[45];
nmpcWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.lb[53] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[47];
nmpcWorkspace.lb[54] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[48];
nmpcWorkspace.lb[55] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[49];
nmpcWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.lb[57] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[51];
nmpcWorkspace.lb[58] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[52];
nmpcWorkspace.lb[59] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[53];
nmpcWorkspace.lb[60] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.lb[61] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[55];
nmpcWorkspace.lb[62] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[56];
nmpcWorkspace.lb[63] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[57];
nmpcWorkspace.lb[64] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.lb[65] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[59];
nmpcWorkspace.lb[66] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[60];
nmpcWorkspace.lb[67] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[61];
nmpcWorkspace.lb[68] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[62];
nmpcWorkspace.lb[69] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[63];
nmpcWorkspace.lb[70] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[64];
nmpcWorkspace.lb[71] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[65];
nmpcWorkspace.lb[72] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[66];
nmpcWorkspace.lb[73] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[67];
nmpcWorkspace.lb[74] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[68];
nmpcWorkspace.lb[75] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[69];
nmpcWorkspace.lb[76] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[70];
nmpcWorkspace.lb[77] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[71];
nmpcWorkspace.lb[78] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[72];
nmpcWorkspace.lb[79] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[73];
nmpcWorkspace.lb[80] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[74];
nmpcWorkspace.lb[81] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[75];
nmpcWorkspace.lb[82] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[76];
nmpcWorkspace.lb[83] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[77];
nmpcWorkspace.lb[84] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[78];
nmpcWorkspace.lb[85] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[79];
nmpcWorkspace.lb[86] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[80];
nmpcWorkspace.lb[87] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[81];
nmpcWorkspace.lb[88] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[82];
nmpcWorkspace.lb[89] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[83];
nmpcWorkspace.lb[90] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[84];
nmpcWorkspace.lb[91] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[85];
nmpcWorkspace.lb[92] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[86];
nmpcWorkspace.lb[93] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[87];
nmpcWorkspace.lb[94] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[88];
nmpcWorkspace.lb[95] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[89];
nmpcWorkspace.lb[96] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[90];
nmpcWorkspace.lb[97] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[91];
nmpcWorkspace.lb[98] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[92];
nmpcWorkspace.lb[99] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[93];
nmpcWorkspace.lb[100] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[94];
nmpcWorkspace.lb[101] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[95];
nmpcWorkspace.lb[102] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[96];
nmpcWorkspace.lb[103] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[97];
nmpcWorkspace.lb[104] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[98];
nmpcWorkspace.lb[105] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[99];
nmpcWorkspace.lb[106] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[100];
nmpcWorkspace.lb[107] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[101];
nmpcWorkspace.lb[108] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[102];
nmpcWorkspace.lb[109] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[103];
nmpcWorkspace.lb[110] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[104];
nmpcWorkspace.lb[111] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[105];
nmpcWorkspace.lb[112] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[106];
nmpcWorkspace.lb[113] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[107];
nmpcWorkspace.lb[114] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[108];
nmpcWorkspace.lb[115] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[109];
nmpcWorkspace.lb[116] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[110];
nmpcWorkspace.lb[117] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[111];
nmpcWorkspace.lb[118] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[112];
nmpcWorkspace.lb[119] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[113];
nmpcWorkspace.lb[120] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[114];
nmpcWorkspace.lb[121] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[115];
nmpcWorkspace.lb[122] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[116];
nmpcWorkspace.lb[123] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[117];
nmpcWorkspace.lb[124] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[118];
nmpcWorkspace.lb[125] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[119];
nmpcWorkspace.ub[6] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[0];
nmpcWorkspace.ub[7] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[1];
nmpcWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.ub[9] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[3];
nmpcWorkspace.ub[10] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[4];
nmpcWorkspace.ub[11] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[5];
nmpcWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.ub[13] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[7];
nmpcWorkspace.ub[14] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[8];
nmpcWorkspace.ub[15] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[9];
nmpcWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.ub[17] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[11];
nmpcWorkspace.ub[18] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[12];
nmpcWorkspace.ub[19] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[13];
nmpcWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.ub[21] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[15];
nmpcWorkspace.ub[22] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[16];
nmpcWorkspace.ub[23] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[17];
nmpcWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.ub[25] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[19];
nmpcWorkspace.ub[26] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[20];
nmpcWorkspace.ub[27] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[21];
nmpcWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.ub[29] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[23];
nmpcWorkspace.ub[30] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[24];
nmpcWorkspace.ub[31] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[25];
nmpcWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.ub[33] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[27];
nmpcWorkspace.ub[34] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[28];
nmpcWorkspace.ub[35] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[29];
nmpcWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.ub[37] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[31];
nmpcWorkspace.ub[38] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[32];
nmpcWorkspace.ub[39] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[33];
nmpcWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.ub[41] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[35];
nmpcWorkspace.ub[42] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[36];
nmpcWorkspace.ub[43] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[37];
nmpcWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.ub[45] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[39];
nmpcWorkspace.ub[46] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[40];
nmpcWorkspace.ub[47] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[41];
nmpcWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.ub[49] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[43];
nmpcWorkspace.ub[50] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[44];
nmpcWorkspace.ub[51] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[45];
nmpcWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.ub[53] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[47];
nmpcWorkspace.ub[54] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[48];
nmpcWorkspace.ub[55] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[49];
nmpcWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.ub[57] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[51];
nmpcWorkspace.ub[58] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[52];
nmpcWorkspace.ub[59] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[53];
nmpcWorkspace.ub[60] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.ub[61] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[55];
nmpcWorkspace.ub[62] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[56];
nmpcWorkspace.ub[63] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[57];
nmpcWorkspace.ub[64] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.ub[65] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[59];
nmpcWorkspace.ub[66] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[60];
nmpcWorkspace.ub[67] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[61];
nmpcWorkspace.ub[68] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[62];
nmpcWorkspace.ub[69] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[63];
nmpcWorkspace.ub[70] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[64];
nmpcWorkspace.ub[71] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[65];
nmpcWorkspace.ub[72] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[66];
nmpcWorkspace.ub[73] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[67];
nmpcWorkspace.ub[74] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[68];
nmpcWorkspace.ub[75] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[69];
nmpcWorkspace.ub[76] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[70];
nmpcWorkspace.ub[77] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[71];
nmpcWorkspace.ub[78] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[72];
nmpcWorkspace.ub[79] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[73];
nmpcWorkspace.ub[80] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[74];
nmpcWorkspace.ub[81] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[75];
nmpcWorkspace.ub[82] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[76];
nmpcWorkspace.ub[83] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[77];
nmpcWorkspace.ub[84] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[78];
nmpcWorkspace.ub[85] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[79];
nmpcWorkspace.ub[86] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[80];
nmpcWorkspace.ub[87] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[81];
nmpcWorkspace.ub[88] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[82];
nmpcWorkspace.ub[89] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[83];
nmpcWorkspace.ub[90] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[84];
nmpcWorkspace.ub[91] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[85];
nmpcWorkspace.ub[92] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[86];
nmpcWorkspace.ub[93] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[87];
nmpcWorkspace.ub[94] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[88];
nmpcWorkspace.ub[95] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[89];
nmpcWorkspace.ub[96] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[90];
nmpcWorkspace.ub[97] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[91];
nmpcWorkspace.ub[98] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[92];
nmpcWorkspace.ub[99] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[93];
nmpcWorkspace.ub[100] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[94];
nmpcWorkspace.ub[101] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[95];
nmpcWorkspace.ub[102] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[96];
nmpcWorkspace.ub[103] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[97];
nmpcWorkspace.ub[104] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[98];
nmpcWorkspace.ub[105] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[99];
nmpcWorkspace.ub[106] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[100];
nmpcWorkspace.ub[107] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[101];
nmpcWorkspace.ub[108] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[102];
nmpcWorkspace.ub[109] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[103];
nmpcWorkspace.ub[110] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[104];
nmpcWorkspace.ub[111] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[105];
nmpcWorkspace.ub[112] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[106];
nmpcWorkspace.ub[113] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[107];
nmpcWorkspace.ub[114] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[108];
nmpcWorkspace.ub[115] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[109];
nmpcWorkspace.ub[116] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[110];
nmpcWorkspace.ub[117] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[111];
nmpcWorkspace.ub[118] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[112];
nmpcWorkspace.ub[119] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[113];
nmpcWorkspace.ub[120] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[114];
nmpcWorkspace.ub[121] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[115];
nmpcWorkspace.ub[122] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[116];
nmpcWorkspace.ub[123] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[117];
nmpcWorkspace.ub[124] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[118];
nmpcWorkspace.ub[125] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];

for (lRun2 = 0; lRun2 < 300; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 6 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 40 ]), &(nmpcWorkspace.Dy[ 10 ]), &(nmpcWorkspace.g[ 10 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 80 ]), &(nmpcWorkspace.Dy[ 20 ]), &(nmpcWorkspace.g[ 14 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 120 ]), &(nmpcWorkspace.Dy[ 30 ]), &(nmpcWorkspace.g[ 18 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 160 ]), &(nmpcWorkspace.Dy[ 40 ]), &(nmpcWorkspace.g[ 22 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 200 ]), &(nmpcWorkspace.Dy[ 50 ]), &(nmpcWorkspace.g[ 26 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 240 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.g[ 30 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 280 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.g[ 34 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 320 ]), &(nmpcWorkspace.Dy[ 80 ]), &(nmpcWorkspace.g[ 38 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 360 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.g[ 42 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 400 ]), &(nmpcWorkspace.Dy[ 100 ]), &(nmpcWorkspace.g[ 46 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 440 ]), &(nmpcWorkspace.Dy[ 110 ]), &(nmpcWorkspace.g[ 50 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 480 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.g[ 54 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 520 ]), &(nmpcWorkspace.Dy[ 130 ]), &(nmpcWorkspace.g[ 58 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 560 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.g[ 62 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 600 ]), &(nmpcWorkspace.Dy[ 150 ]), &(nmpcWorkspace.g[ 66 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 640 ]), &(nmpcWorkspace.Dy[ 160 ]), &(nmpcWorkspace.g[ 70 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 680 ]), &(nmpcWorkspace.Dy[ 170 ]), &(nmpcWorkspace.g[ 74 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 720 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.g[ 78 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 760 ]), &(nmpcWorkspace.Dy[ 190 ]), &(nmpcWorkspace.g[ 82 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 800 ]), &(nmpcWorkspace.Dy[ 200 ]), &(nmpcWorkspace.g[ 86 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 840 ]), &(nmpcWorkspace.Dy[ 210 ]), &(nmpcWorkspace.g[ 90 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 880 ]), &(nmpcWorkspace.Dy[ 220 ]), &(nmpcWorkspace.g[ 94 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 920 ]), &(nmpcWorkspace.Dy[ 230 ]), &(nmpcWorkspace.g[ 98 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 960 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.g[ 102 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1000 ]), &(nmpcWorkspace.Dy[ 250 ]), &(nmpcWorkspace.g[ 106 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1040 ]), &(nmpcWorkspace.Dy[ 260 ]), &(nmpcWorkspace.g[ 110 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1080 ]), &(nmpcWorkspace.Dy[ 270 ]), &(nmpcWorkspace.g[ 114 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1120 ]), &(nmpcWorkspace.Dy[ 280 ]), &(nmpcWorkspace.g[ 118 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1160 ]), &(nmpcWorkspace.Dy[ 290 ]), &(nmpcWorkspace.g[ 122 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 60 ]), &(nmpcWorkspace.Dy[ 10 ]), &(nmpcWorkspace.QDy[ 6 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 120 ]), &(nmpcWorkspace.Dy[ 20 ]), &(nmpcWorkspace.QDy[ 12 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 180 ]), &(nmpcWorkspace.Dy[ 30 ]), &(nmpcWorkspace.QDy[ 18 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 240 ]), &(nmpcWorkspace.Dy[ 40 ]), &(nmpcWorkspace.QDy[ 24 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 300 ]), &(nmpcWorkspace.Dy[ 50 ]), &(nmpcWorkspace.QDy[ 30 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 360 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.QDy[ 36 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 420 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.QDy[ 42 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 480 ]), &(nmpcWorkspace.Dy[ 80 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 540 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.QDy[ 54 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 600 ]), &(nmpcWorkspace.Dy[ 100 ]), &(nmpcWorkspace.QDy[ 60 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 660 ]), &(nmpcWorkspace.Dy[ 110 ]), &(nmpcWorkspace.QDy[ 66 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 720 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 780 ]), &(nmpcWorkspace.Dy[ 130 ]), &(nmpcWorkspace.QDy[ 78 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 840 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.QDy[ 84 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 900 ]), &(nmpcWorkspace.Dy[ 150 ]), &(nmpcWorkspace.QDy[ 90 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 960 ]), &(nmpcWorkspace.Dy[ 160 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1020 ]), &(nmpcWorkspace.Dy[ 170 ]), &(nmpcWorkspace.QDy[ 102 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1080 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.QDy[ 108 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1140 ]), &(nmpcWorkspace.Dy[ 190 ]), &(nmpcWorkspace.QDy[ 114 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1200 ]), &(nmpcWorkspace.Dy[ 200 ]), &(nmpcWorkspace.QDy[ 120 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1260 ]), &(nmpcWorkspace.Dy[ 210 ]), &(nmpcWorkspace.QDy[ 126 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1320 ]), &(nmpcWorkspace.Dy[ 220 ]), &(nmpcWorkspace.QDy[ 132 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1380 ]), &(nmpcWorkspace.Dy[ 230 ]), &(nmpcWorkspace.QDy[ 138 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1440 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1500 ]), &(nmpcWorkspace.Dy[ 250 ]), &(nmpcWorkspace.QDy[ 150 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1560 ]), &(nmpcWorkspace.Dy[ 260 ]), &(nmpcWorkspace.QDy[ 156 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1620 ]), &(nmpcWorkspace.Dy[ 270 ]), &(nmpcWorkspace.QDy[ 162 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1680 ]), &(nmpcWorkspace.Dy[ 280 ]), &(nmpcWorkspace.QDy[ 168 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1740 ]), &(nmpcWorkspace.Dy[ 290 ]), &(nmpcWorkspace.QDy[ 174 ]) );

nmpcWorkspace.QDy[180] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[181] = + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[182] = + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[183] = + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[184] = + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[185] = + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[5];

for (lRun2 = 0; lRun2 < 180; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 6] += nmpcWorkspace.Qd[lRun2];


nmpcWorkspace.g[0] = + nmpcWorkspace.evGx[0]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[6]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[12]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[18]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[24]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[30]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[36]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[42]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[48]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[54]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[60]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[66]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[72]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[78]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[84]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[90]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[96]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[102]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[108]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[114]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[120]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[126]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[132]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[138]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[144]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[150]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[156]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[162]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[168]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[174]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[180]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[186]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[192]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[198]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[204]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[210]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[216]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[222]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[228]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[234]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[240]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[246]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[252]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[258]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[264]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[270]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[276]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[282]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[288]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[294]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[300]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[306]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[312]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[318]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[324]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[330]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[336]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[342]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[348]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[354]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[360]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[366]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[372]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[378]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[384]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[390]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[396]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[402]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[408]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[414]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[420]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[426]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[432]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[438]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[444]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[450]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[456]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[462]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[468]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[474]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[480]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[486]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[492]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[498]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[504]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[510]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[516]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[522]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[528]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[534]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[540]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[546]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[552]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[558]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[564]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[570]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[576]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[582]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[588]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[594]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[600]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[606]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[612]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[618]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[624]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[630]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[636]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[642]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[648]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[654]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[660]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[666]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[672]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[678]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[684]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[690]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[696]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[702]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[708]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[714]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[720]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[726]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[732]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[738]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[744]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[750]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[756]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[762]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[768]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[774]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[780]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[786]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[792]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[798]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[804]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[810]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[816]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[822]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[828]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[834]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[840]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[846]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[852]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[858]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[864]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[870]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[876]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[882]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[888]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[894]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[900]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[906]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[912]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[918]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[924]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[930]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[936]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[942]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[948]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[954]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[960]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[966]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[972]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[978]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[984]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[990]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[996]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1002]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1008]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1014]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1020]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1026]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1032]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1038]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1044]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1050]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1056]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1062]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1068]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1074]*nmpcWorkspace.QDy[185];
nmpcWorkspace.g[1] = + nmpcWorkspace.evGx[1]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[7]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[13]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[19]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[25]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[31]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[37]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[43]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[49]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[55]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[61]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[67]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[73]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[79]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[85]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[91]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[97]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[103]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[109]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[115]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[121]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[127]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[133]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[139]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[145]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[151]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[157]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[163]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[169]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[175]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[181]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[187]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[193]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[199]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[205]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[211]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[217]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[223]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[229]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[235]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[241]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[247]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[253]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[259]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[265]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[271]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[277]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[283]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[289]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[295]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[301]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[307]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[313]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[319]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[325]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[331]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[337]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[343]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[349]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[355]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[361]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[367]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[373]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[379]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[385]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[391]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[397]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[403]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[409]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[415]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[421]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[427]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[433]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[439]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[445]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[451]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[457]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[463]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[469]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[475]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[481]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[487]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[493]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[499]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[505]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[511]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[517]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[523]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[529]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[535]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[541]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[547]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[553]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[559]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[565]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[571]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[577]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[583]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[589]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[595]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[601]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[607]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[613]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[619]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[625]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[631]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[637]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[643]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[649]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[655]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[661]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[667]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[673]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[679]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[685]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[691]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[697]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[703]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[709]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[715]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[721]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[727]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[733]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[739]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[745]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[751]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[757]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[763]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[769]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[775]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[781]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[787]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[793]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[799]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[805]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[811]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[817]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[823]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[829]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[835]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[841]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[847]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[853]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[859]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[865]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[871]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[877]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[883]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[889]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[895]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[901]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[907]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[913]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[919]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[925]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[931]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[937]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[943]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[949]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[955]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[961]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[967]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[973]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[979]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[985]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[991]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[997]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1003]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1009]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1015]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1021]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1027]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1033]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1039]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1045]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1051]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1057]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1063]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1069]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1075]*nmpcWorkspace.QDy[185];
nmpcWorkspace.g[2] = + nmpcWorkspace.evGx[2]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[8]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[14]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[20]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[26]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[32]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[38]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[44]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[50]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[56]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[62]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[68]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[74]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[80]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[86]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[92]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[98]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[104]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[110]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[116]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[122]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[128]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[134]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[140]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[146]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[152]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[158]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[164]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[170]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[176]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[182]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[188]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[194]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[200]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[206]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[212]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[218]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[224]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[230]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[236]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[242]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[248]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[254]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[260]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[266]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[272]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[278]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[284]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[290]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[296]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[302]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[308]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[314]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[320]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[326]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[332]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[338]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[344]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[350]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[356]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[362]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[368]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[374]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[380]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[386]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[392]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[398]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[404]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[410]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[416]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[422]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[428]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[434]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[440]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[446]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[452]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[458]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[464]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[470]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[476]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[482]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[488]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[494]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[500]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[506]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[512]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[518]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[524]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[530]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[536]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[542]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[548]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[554]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[560]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[566]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[572]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[578]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[584]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[590]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[596]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[602]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[608]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[614]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[620]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[626]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[632]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[638]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[644]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[650]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[656]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[662]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[668]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[674]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[680]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[686]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[692]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[698]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[704]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[710]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[716]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[722]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[728]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[734]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[740]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[746]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[752]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[758]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[764]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[770]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[776]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[782]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[788]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[794]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[800]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[806]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[812]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[818]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[824]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[830]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[836]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[842]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[848]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[854]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[860]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[866]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[872]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[878]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[884]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[890]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[896]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[902]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[908]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[914]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[920]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[926]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[932]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[938]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[944]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[950]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[956]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[962]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[968]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[974]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[980]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[986]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[992]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[998]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1004]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1010]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1016]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1022]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1028]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1034]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1040]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1046]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1052]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1058]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1064]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1070]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1076]*nmpcWorkspace.QDy[185];
nmpcWorkspace.g[3] = + nmpcWorkspace.evGx[3]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[9]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[15]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[21]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[27]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[33]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[39]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[45]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[51]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[57]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[63]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[69]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[75]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[81]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[87]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[93]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[99]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[105]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[111]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[117]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[123]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[129]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[135]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[141]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[147]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[153]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[159]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[165]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[171]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[177]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[183]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[189]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[195]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[201]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[207]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[213]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[219]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[225]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[231]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[237]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[243]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[249]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[255]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[261]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[267]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[273]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[279]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[285]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[291]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[297]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[303]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[309]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[315]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[321]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[327]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[333]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[339]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[345]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[351]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[357]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[363]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[369]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[375]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[381]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[387]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[393]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[399]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[405]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[411]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[417]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[423]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[429]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[435]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[441]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[447]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[453]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[459]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[465]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[471]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[477]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[483]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[489]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[495]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[501]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[507]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[513]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[519]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[525]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[531]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[537]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[543]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[549]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[555]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[561]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[567]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[573]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[579]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[585]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[591]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[597]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[603]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[609]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[615]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[621]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[627]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[633]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[639]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[645]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[651]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[657]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[663]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[669]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[675]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[681]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[687]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[693]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[699]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[705]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[711]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[717]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[723]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[729]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[735]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[741]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[747]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[753]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[759]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[765]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[771]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[777]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[783]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[789]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[795]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[801]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[807]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[813]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[819]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[825]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[831]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[837]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[843]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[849]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[855]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[861]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[867]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[873]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[879]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[885]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[891]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[897]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[903]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[909]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[915]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[921]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[927]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[933]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[939]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[945]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[951]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[957]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[963]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[969]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[975]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[981]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[987]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[993]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[999]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1005]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1011]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1017]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1023]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1029]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1035]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1041]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1047]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1053]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1059]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1065]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1071]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1077]*nmpcWorkspace.QDy[185];
nmpcWorkspace.g[4] = + nmpcWorkspace.evGx[4]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[10]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[16]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[22]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[28]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[34]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[40]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[46]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[52]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[58]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[64]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[70]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[76]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[82]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[88]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[94]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[100]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[106]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[112]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[118]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[124]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[130]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[136]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[142]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[148]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[154]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[160]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[166]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[172]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[178]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[184]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[190]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[196]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[202]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[208]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[214]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[220]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[226]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[232]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[238]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[244]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[250]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[256]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[262]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[268]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[274]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[280]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[286]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[292]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[298]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[304]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[310]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[316]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[322]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[328]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[334]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[340]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[346]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[352]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[358]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[364]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[370]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[376]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[382]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[388]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[394]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[400]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[406]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[412]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[418]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[424]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[430]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[436]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[442]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[448]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[454]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[460]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[466]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[472]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[478]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[484]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[490]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[496]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[502]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[508]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[514]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[520]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[526]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[532]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[538]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[544]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[550]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[556]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[562]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[568]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[574]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[580]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[586]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[592]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[598]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[604]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[610]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[616]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[622]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[628]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[634]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[640]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[646]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[652]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[658]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[664]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[670]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[676]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[682]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[688]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[694]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[700]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[706]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[712]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[718]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[724]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[730]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[736]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[742]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[748]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[754]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[760]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[766]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[772]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[778]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[784]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[790]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[796]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[802]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[808]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[814]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[820]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[826]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[832]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[838]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[844]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[850]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[856]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[862]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[868]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[874]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[880]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[886]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[892]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[898]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[904]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[910]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[916]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[922]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[928]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[934]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[940]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[946]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[952]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[958]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[964]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[970]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[976]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[982]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[988]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[994]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1000]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1006]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1012]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1018]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1024]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1030]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1036]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1042]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1048]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1054]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1060]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1066]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1072]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1078]*nmpcWorkspace.QDy[185];
nmpcWorkspace.g[5] = + nmpcWorkspace.evGx[5]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[11]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[17]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[23]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[29]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[35]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[41]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[47]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[53]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[59]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[65]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[71]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[77]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[83]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[89]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[95]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[101]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[107]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[113]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[119]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[125]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[131]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[137]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[143]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[149]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[155]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[161]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[167]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[173]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[179]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[185]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[191]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[197]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[203]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[209]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[215]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[221]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[227]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[233]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[239]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[245]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[251]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[257]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[263]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[269]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[275]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[281]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[287]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[293]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[299]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[305]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[311]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[317]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[323]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[329]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[335]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[341]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[347]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[353]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[359]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[365]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[371]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[377]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[383]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[389]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[395]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[401]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[407]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[413]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[419]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[425]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[431]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[437]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[443]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[449]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[455]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[461]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[467]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[473]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[479]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[485]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[491]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[497]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[503]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[509]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[515]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[521]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[527]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[533]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[539]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[545]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[551]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[557]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[563]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[569]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[575]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[581]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[587]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[593]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[599]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[605]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[611]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[617]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[623]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[629]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[635]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[641]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[647]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[653]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[659]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[665]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[671]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[677]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[683]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[689]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[695]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[701]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[707]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[713]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[719]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[725]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[731]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[737]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[743]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[749]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[755]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[761]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[767]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[773]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[779]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[785]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[791]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[797]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[803]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[809]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[815]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[821]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[827]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[833]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[839]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[845]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[851]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[857]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[863]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[869]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[875]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[881]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[887]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[893]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[899]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[905]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[911]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[917]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[923]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[929]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[935]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[941]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[947]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[953]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[959]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[965]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[971]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[977]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[983]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[989]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[995]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1001]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1007]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1013]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1019]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1025]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1031]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1037]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1043]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1049]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1055]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1061]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1067]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1073]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1079]*nmpcWorkspace.QDy[185];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 24 ]), &(nmpcWorkspace.QDy[ lRun2 * 6 + 6 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 6 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];

nmpcVariables.u[0] += nmpcWorkspace.x[6];
nmpcVariables.u[1] += nmpcWorkspace.x[7];
nmpcVariables.u[2] += nmpcWorkspace.x[8];
nmpcVariables.u[3] += nmpcWorkspace.x[9];
nmpcVariables.u[4] += nmpcWorkspace.x[10];
nmpcVariables.u[5] += nmpcWorkspace.x[11];
nmpcVariables.u[6] += nmpcWorkspace.x[12];
nmpcVariables.u[7] += nmpcWorkspace.x[13];
nmpcVariables.u[8] += nmpcWorkspace.x[14];
nmpcVariables.u[9] += nmpcWorkspace.x[15];
nmpcVariables.u[10] += nmpcWorkspace.x[16];
nmpcVariables.u[11] += nmpcWorkspace.x[17];
nmpcVariables.u[12] += nmpcWorkspace.x[18];
nmpcVariables.u[13] += nmpcWorkspace.x[19];
nmpcVariables.u[14] += nmpcWorkspace.x[20];
nmpcVariables.u[15] += nmpcWorkspace.x[21];
nmpcVariables.u[16] += nmpcWorkspace.x[22];
nmpcVariables.u[17] += nmpcWorkspace.x[23];
nmpcVariables.u[18] += nmpcWorkspace.x[24];
nmpcVariables.u[19] += nmpcWorkspace.x[25];
nmpcVariables.u[20] += nmpcWorkspace.x[26];
nmpcVariables.u[21] += nmpcWorkspace.x[27];
nmpcVariables.u[22] += nmpcWorkspace.x[28];
nmpcVariables.u[23] += nmpcWorkspace.x[29];
nmpcVariables.u[24] += nmpcWorkspace.x[30];
nmpcVariables.u[25] += nmpcWorkspace.x[31];
nmpcVariables.u[26] += nmpcWorkspace.x[32];
nmpcVariables.u[27] += nmpcWorkspace.x[33];
nmpcVariables.u[28] += nmpcWorkspace.x[34];
nmpcVariables.u[29] += nmpcWorkspace.x[35];
nmpcVariables.u[30] += nmpcWorkspace.x[36];
nmpcVariables.u[31] += nmpcWorkspace.x[37];
nmpcVariables.u[32] += nmpcWorkspace.x[38];
nmpcVariables.u[33] += nmpcWorkspace.x[39];
nmpcVariables.u[34] += nmpcWorkspace.x[40];
nmpcVariables.u[35] += nmpcWorkspace.x[41];
nmpcVariables.u[36] += nmpcWorkspace.x[42];
nmpcVariables.u[37] += nmpcWorkspace.x[43];
nmpcVariables.u[38] += nmpcWorkspace.x[44];
nmpcVariables.u[39] += nmpcWorkspace.x[45];
nmpcVariables.u[40] += nmpcWorkspace.x[46];
nmpcVariables.u[41] += nmpcWorkspace.x[47];
nmpcVariables.u[42] += nmpcWorkspace.x[48];
nmpcVariables.u[43] += nmpcWorkspace.x[49];
nmpcVariables.u[44] += nmpcWorkspace.x[50];
nmpcVariables.u[45] += nmpcWorkspace.x[51];
nmpcVariables.u[46] += nmpcWorkspace.x[52];
nmpcVariables.u[47] += nmpcWorkspace.x[53];
nmpcVariables.u[48] += nmpcWorkspace.x[54];
nmpcVariables.u[49] += nmpcWorkspace.x[55];
nmpcVariables.u[50] += nmpcWorkspace.x[56];
nmpcVariables.u[51] += nmpcWorkspace.x[57];
nmpcVariables.u[52] += nmpcWorkspace.x[58];
nmpcVariables.u[53] += nmpcWorkspace.x[59];
nmpcVariables.u[54] += nmpcWorkspace.x[60];
nmpcVariables.u[55] += nmpcWorkspace.x[61];
nmpcVariables.u[56] += nmpcWorkspace.x[62];
nmpcVariables.u[57] += nmpcWorkspace.x[63];
nmpcVariables.u[58] += nmpcWorkspace.x[64];
nmpcVariables.u[59] += nmpcWorkspace.x[65];
nmpcVariables.u[60] += nmpcWorkspace.x[66];
nmpcVariables.u[61] += nmpcWorkspace.x[67];
nmpcVariables.u[62] += nmpcWorkspace.x[68];
nmpcVariables.u[63] += nmpcWorkspace.x[69];
nmpcVariables.u[64] += nmpcWorkspace.x[70];
nmpcVariables.u[65] += nmpcWorkspace.x[71];
nmpcVariables.u[66] += nmpcWorkspace.x[72];
nmpcVariables.u[67] += nmpcWorkspace.x[73];
nmpcVariables.u[68] += nmpcWorkspace.x[74];
nmpcVariables.u[69] += nmpcWorkspace.x[75];
nmpcVariables.u[70] += nmpcWorkspace.x[76];
nmpcVariables.u[71] += nmpcWorkspace.x[77];
nmpcVariables.u[72] += nmpcWorkspace.x[78];
nmpcVariables.u[73] += nmpcWorkspace.x[79];
nmpcVariables.u[74] += nmpcWorkspace.x[80];
nmpcVariables.u[75] += nmpcWorkspace.x[81];
nmpcVariables.u[76] += nmpcWorkspace.x[82];
nmpcVariables.u[77] += nmpcWorkspace.x[83];
nmpcVariables.u[78] += nmpcWorkspace.x[84];
nmpcVariables.u[79] += nmpcWorkspace.x[85];
nmpcVariables.u[80] += nmpcWorkspace.x[86];
nmpcVariables.u[81] += nmpcWorkspace.x[87];
nmpcVariables.u[82] += nmpcWorkspace.x[88];
nmpcVariables.u[83] += nmpcWorkspace.x[89];
nmpcVariables.u[84] += nmpcWorkspace.x[90];
nmpcVariables.u[85] += nmpcWorkspace.x[91];
nmpcVariables.u[86] += nmpcWorkspace.x[92];
nmpcVariables.u[87] += nmpcWorkspace.x[93];
nmpcVariables.u[88] += nmpcWorkspace.x[94];
nmpcVariables.u[89] += nmpcWorkspace.x[95];
nmpcVariables.u[90] += nmpcWorkspace.x[96];
nmpcVariables.u[91] += nmpcWorkspace.x[97];
nmpcVariables.u[92] += nmpcWorkspace.x[98];
nmpcVariables.u[93] += nmpcWorkspace.x[99];
nmpcVariables.u[94] += nmpcWorkspace.x[100];
nmpcVariables.u[95] += nmpcWorkspace.x[101];
nmpcVariables.u[96] += nmpcWorkspace.x[102];
nmpcVariables.u[97] += nmpcWorkspace.x[103];
nmpcVariables.u[98] += nmpcWorkspace.x[104];
nmpcVariables.u[99] += nmpcWorkspace.x[105];
nmpcVariables.u[100] += nmpcWorkspace.x[106];
nmpcVariables.u[101] += nmpcWorkspace.x[107];
nmpcVariables.u[102] += nmpcWorkspace.x[108];
nmpcVariables.u[103] += nmpcWorkspace.x[109];
nmpcVariables.u[104] += nmpcWorkspace.x[110];
nmpcVariables.u[105] += nmpcWorkspace.x[111];
nmpcVariables.u[106] += nmpcWorkspace.x[112];
nmpcVariables.u[107] += nmpcWorkspace.x[113];
nmpcVariables.u[108] += nmpcWorkspace.x[114];
nmpcVariables.u[109] += nmpcWorkspace.x[115];
nmpcVariables.u[110] += nmpcWorkspace.x[116];
nmpcVariables.u[111] += nmpcWorkspace.x[117];
nmpcVariables.u[112] += nmpcWorkspace.x[118];
nmpcVariables.u[113] += nmpcWorkspace.x[119];
nmpcVariables.u[114] += nmpcWorkspace.x[120];
nmpcVariables.u[115] += nmpcWorkspace.x[121];
nmpcVariables.u[116] += nmpcWorkspace.x[122];
nmpcVariables.u[117] += nmpcWorkspace.x[123];
nmpcVariables.u[118] += nmpcWorkspace.x[124];
nmpcVariables.u[119] += nmpcWorkspace.x[125];

nmpcVariables.x[6] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[4]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[5]*nmpcWorkspace.x[5] + nmpcWorkspace.d[0];
nmpcVariables.x[7] += + nmpcWorkspace.evGx[6]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[7]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[8]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[9]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[10]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[11]*nmpcWorkspace.x[5] + nmpcWorkspace.d[1];
nmpcVariables.x[8] += + nmpcWorkspace.evGx[12]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[13]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[14]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[15]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[16]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[17]*nmpcWorkspace.x[5] + nmpcWorkspace.d[2];
nmpcVariables.x[9] += + nmpcWorkspace.evGx[18]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[19]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[20]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[21]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[22]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[23]*nmpcWorkspace.x[5] + nmpcWorkspace.d[3];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[24]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[25]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[26]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[27]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[28]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[29]*nmpcWorkspace.x[5] + nmpcWorkspace.d[4];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[30]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[31]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[32]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[33]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[34]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[35]*nmpcWorkspace.x[5] + nmpcWorkspace.d[5];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[36]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[37]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[38]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[39]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[40]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[41]*nmpcWorkspace.x[5] + nmpcWorkspace.d[6];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[42]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[43]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[44]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[45]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[46]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[47]*nmpcWorkspace.x[5] + nmpcWorkspace.d[7];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[48]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[49]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[50]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[51]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[52]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[53]*nmpcWorkspace.x[5] + nmpcWorkspace.d[8];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[54]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[55]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[56]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[57]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[58]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[59]*nmpcWorkspace.x[5] + nmpcWorkspace.d[9];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[60]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[61]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[62]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[63]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[64]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[65]*nmpcWorkspace.x[5] + nmpcWorkspace.d[10];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[66]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[67]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[68]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[69]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[70]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[71]*nmpcWorkspace.x[5] + nmpcWorkspace.d[11];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[72]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[73]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[74]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[75]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[76]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[77]*nmpcWorkspace.x[5] + nmpcWorkspace.d[12];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[78]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[79]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[80]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[81]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[82]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[83]*nmpcWorkspace.x[5] + nmpcWorkspace.d[13];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[84]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[85]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[86]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[87]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[88]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[89]*nmpcWorkspace.x[5] + nmpcWorkspace.d[14];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[90]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[91]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[92]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[93]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[94]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[95]*nmpcWorkspace.x[5] + nmpcWorkspace.d[15];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[96]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[97]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[98]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[99]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[100]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[101]*nmpcWorkspace.x[5] + nmpcWorkspace.d[16];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[102]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[103]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[104]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[105]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[106]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[107]*nmpcWorkspace.x[5] + nmpcWorkspace.d[17];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[108]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[109]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[110]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[111]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[112]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[113]*nmpcWorkspace.x[5] + nmpcWorkspace.d[18];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[114]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[115]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[116]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[117]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[118]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[119]*nmpcWorkspace.x[5] + nmpcWorkspace.d[19];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[123]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[124]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[125]*nmpcWorkspace.x[5] + nmpcWorkspace.d[20];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[126]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[127]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[128]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[129]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[130]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[131]*nmpcWorkspace.x[5] + nmpcWorkspace.d[21];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[132]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[133]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[134]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[135]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[136]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[137]*nmpcWorkspace.x[5] + nmpcWorkspace.d[22];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[138]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[139]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[140]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[141]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[142]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[143]*nmpcWorkspace.x[5] + nmpcWorkspace.d[23];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[144]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[145]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[146]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[147]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[148]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[149]*nmpcWorkspace.x[5] + nmpcWorkspace.d[24];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[150]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[151]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[152]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[153]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[154]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[155]*nmpcWorkspace.x[5] + nmpcWorkspace.d[25];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[156]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[157]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[158]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[159]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[160]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[161]*nmpcWorkspace.x[5] + nmpcWorkspace.d[26];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[162]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[163]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[164]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[165]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[166]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[167]*nmpcWorkspace.x[5] + nmpcWorkspace.d[27];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[168]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[169]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[170]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[171]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[172]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[173]*nmpcWorkspace.x[5] + nmpcWorkspace.d[28];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[174]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[175]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[176]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[177]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[178]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[179]*nmpcWorkspace.x[5] + nmpcWorkspace.d[29];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[180]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[181]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[182]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[183]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[184]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[185]*nmpcWorkspace.x[5] + nmpcWorkspace.d[30];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[186]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[187]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[188]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[189]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[190]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[191]*nmpcWorkspace.x[5] + nmpcWorkspace.d[31];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[192]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[193]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[194]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[195]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[196]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[197]*nmpcWorkspace.x[5] + nmpcWorkspace.d[32];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[198]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[199]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[200]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[201]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[202]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[203]*nmpcWorkspace.x[5] + nmpcWorkspace.d[33];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[204]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[205]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[206]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[207]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[208]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[209]*nmpcWorkspace.x[5] + nmpcWorkspace.d[34];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[210]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[211]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[212]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[213]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[214]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[215]*nmpcWorkspace.x[5] + nmpcWorkspace.d[35];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[216]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[217]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[218]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[219]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[220]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[221]*nmpcWorkspace.x[5] + nmpcWorkspace.d[36];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[222]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[223]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[224]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[225]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[226]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[227]*nmpcWorkspace.x[5] + nmpcWorkspace.d[37];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[228]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[229]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[230]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[231]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[232]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[233]*nmpcWorkspace.x[5] + nmpcWorkspace.d[38];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[234]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[235]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[236]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[237]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[238]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[239]*nmpcWorkspace.x[5] + nmpcWorkspace.d[39];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[243]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[244]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[245]*nmpcWorkspace.x[5] + nmpcWorkspace.d[40];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[246]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[247]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[248]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[249]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[250]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[251]*nmpcWorkspace.x[5] + nmpcWorkspace.d[41];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[252]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[253]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[254]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[255]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[256]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[257]*nmpcWorkspace.x[5] + nmpcWorkspace.d[42];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[258]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[259]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[260]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[261]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[262]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[263]*nmpcWorkspace.x[5] + nmpcWorkspace.d[43];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[264]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[265]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[266]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[267]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[268]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[269]*nmpcWorkspace.x[5] + nmpcWorkspace.d[44];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[270]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[271]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[272]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[273]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[274]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[275]*nmpcWorkspace.x[5] + nmpcWorkspace.d[45];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[276]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[277]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[278]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[279]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[280]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[281]*nmpcWorkspace.x[5] + nmpcWorkspace.d[46];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[282]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[283]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[284]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[285]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[286]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[287]*nmpcWorkspace.x[5] + nmpcWorkspace.d[47];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[288]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[289]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[290]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[291]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[292]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[293]*nmpcWorkspace.x[5] + nmpcWorkspace.d[48];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[294]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[295]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[296]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[297]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[298]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[299]*nmpcWorkspace.x[5] + nmpcWorkspace.d[49];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[300]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[301]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[302]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[303]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[304]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[305]*nmpcWorkspace.x[5] + nmpcWorkspace.d[50];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[306]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[307]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[308]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[309]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[310]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[311]*nmpcWorkspace.x[5] + nmpcWorkspace.d[51];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[312]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[313]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[314]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[315]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[316]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[317]*nmpcWorkspace.x[5] + nmpcWorkspace.d[52];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[318]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[319]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[320]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[321]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[322]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[323]*nmpcWorkspace.x[5] + nmpcWorkspace.d[53];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[324]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[325]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[326]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[327]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[328]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[329]*nmpcWorkspace.x[5] + nmpcWorkspace.d[54];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[330]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[331]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[332]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[333]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[334]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[335]*nmpcWorkspace.x[5] + nmpcWorkspace.d[55];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[336]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[337]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[338]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[339]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[340]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[341]*nmpcWorkspace.x[5] + nmpcWorkspace.d[56];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[342]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[343]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[344]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[345]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[346]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[347]*nmpcWorkspace.x[5] + nmpcWorkspace.d[57];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[348]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[349]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[350]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[351]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[352]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[353]*nmpcWorkspace.x[5] + nmpcWorkspace.d[58];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[354]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[355]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[356]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[357]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[358]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[359]*nmpcWorkspace.x[5] + nmpcWorkspace.d[59];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[364]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[365]*nmpcWorkspace.x[5] + nmpcWorkspace.d[60];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[366]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[367]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[368]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[369]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[370]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[371]*nmpcWorkspace.x[5] + nmpcWorkspace.d[61];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[372]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[373]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[374]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[375]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[376]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[377]*nmpcWorkspace.x[5] + nmpcWorkspace.d[62];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[378]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[379]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[380]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[381]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[382]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[383]*nmpcWorkspace.x[5] + nmpcWorkspace.d[63];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[384]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[385]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[386]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[387]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[388]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[389]*nmpcWorkspace.x[5] + nmpcWorkspace.d[64];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[390]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[391]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[392]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[393]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[394]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[395]*nmpcWorkspace.x[5] + nmpcWorkspace.d[65];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[396]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[397]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[398]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[399]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[400]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[401]*nmpcWorkspace.x[5] + nmpcWorkspace.d[66];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[402]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[403]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[404]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[405]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[406]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[407]*nmpcWorkspace.x[5] + nmpcWorkspace.d[67];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[408]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[409]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[410]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[411]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[412]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[413]*nmpcWorkspace.x[5] + nmpcWorkspace.d[68];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[414]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[415]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[416]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[417]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[418]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[419]*nmpcWorkspace.x[5] + nmpcWorkspace.d[69];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[420]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[421]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[422]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[423]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[424]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[425]*nmpcWorkspace.x[5] + nmpcWorkspace.d[70];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[426]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[427]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[428]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[429]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[430]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[431]*nmpcWorkspace.x[5] + nmpcWorkspace.d[71];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[432]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[433]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[434]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[435]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[436]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[437]*nmpcWorkspace.x[5] + nmpcWorkspace.d[72];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[438]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[439]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[440]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[441]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[442]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[443]*nmpcWorkspace.x[5] + nmpcWorkspace.d[73];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[444]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[445]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[446]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[447]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[448]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[449]*nmpcWorkspace.x[5] + nmpcWorkspace.d[74];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[450]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[451]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[452]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[453]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[454]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[455]*nmpcWorkspace.x[5] + nmpcWorkspace.d[75];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[456]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[457]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[458]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[459]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[460]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[461]*nmpcWorkspace.x[5] + nmpcWorkspace.d[76];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[462]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[463]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[464]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[465]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[466]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[467]*nmpcWorkspace.x[5] + nmpcWorkspace.d[77];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[468]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[469]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[470]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[471]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[472]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[473]*nmpcWorkspace.x[5] + nmpcWorkspace.d[78];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[474]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[475]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[476]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[477]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[478]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[479]*nmpcWorkspace.x[5] + nmpcWorkspace.d[79];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[480]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[481]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[482]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[483]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[484]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[485]*nmpcWorkspace.x[5] + nmpcWorkspace.d[80];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[486]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[487]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[488]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[489]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[490]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[491]*nmpcWorkspace.x[5] + nmpcWorkspace.d[81];
nmpcVariables.x[88] += + nmpcWorkspace.evGx[492]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[493]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[494]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[495]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[496]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[497]*nmpcWorkspace.x[5] + nmpcWorkspace.d[82];
nmpcVariables.x[89] += + nmpcWorkspace.evGx[498]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[499]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[500]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[501]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[502]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[503]*nmpcWorkspace.x[5] + nmpcWorkspace.d[83];
nmpcVariables.x[90] += + nmpcWorkspace.evGx[504]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[505]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[506]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[507]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[508]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[509]*nmpcWorkspace.x[5] + nmpcWorkspace.d[84];
nmpcVariables.x[91] += + nmpcWorkspace.evGx[510]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[511]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[512]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[513]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[514]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[515]*nmpcWorkspace.x[5] + nmpcWorkspace.d[85];
nmpcVariables.x[92] += + nmpcWorkspace.evGx[516]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[517]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[518]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[519]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[520]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[521]*nmpcWorkspace.x[5] + nmpcWorkspace.d[86];
nmpcVariables.x[93] += + nmpcWorkspace.evGx[522]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[523]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[524]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[525]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[526]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[527]*nmpcWorkspace.x[5] + nmpcWorkspace.d[87];
nmpcVariables.x[94] += + nmpcWorkspace.evGx[528]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[529]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[530]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[531]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[532]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[533]*nmpcWorkspace.x[5] + nmpcWorkspace.d[88];
nmpcVariables.x[95] += + nmpcWorkspace.evGx[534]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[535]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[536]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[537]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[538]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[539]*nmpcWorkspace.x[5] + nmpcWorkspace.d[89];
nmpcVariables.x[96] += + nmpcWorkspace.evGx[540]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[541]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[542]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[543]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[544]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[545]*nmpcWorkspace.x[5] + nmpcWorkspace.d[90];
nmpcVariables.x[97] += + nmpcWorkspace.evGx[546]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[547]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[548]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[549]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[550]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[551]*nmpcWorkspace.x[5] + nmpcWorkspace.d[91];
nmpcVariables.x[98] += + nmpcWorkspace.evGx[552]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[553]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[554]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[555]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[556]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[557]*nmpcWorkspace.x[5] + nmpcWorkspace.d[92];
nmpcVariables.x[99] += + nmpcWorkspace.evGx[558]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[559]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[560]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[561]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[562]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[563]*nmpcWorkspace.x[5] + nmpcWorkspace.d[93];
nmpcVariables.x[100] += + nmpcWorkspace.evGx[564]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[565]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[566]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[567]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[568]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[569]*nmpcWorkspace.x[5] + nmpcWorkspace.d[94];
nmpcVariables.x[101] += + nmpcWorkspace.evGx[570]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[571]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[572]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[573]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[574]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[575]*nmpcWorkspace.x[5] + nmpcWorkspace.d[95];
nmpcVariables.x[102] += + nmpcWorkspace.evGx[576]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[577]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[578]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[579]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[580]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[581]*nmpcWorkspace.x[5] + nmpcWorkspace.d[96];
nmpcVariables.x[103] += + nmpcWorkspace.evGx[582]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[583]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[584]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[585]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[586]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[587]*nmpcWorkspace.x[5] + nmpcWorkspace.d[97];
nmpcVariables.x[104] += + nmpcWorkspace.evGx[588]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[589]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[590]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[591]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[592]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[593]*nmpcWorkspace.x[5] + nmpcWorkspace.d[98];
nmpcVariables.x[105] += + nmpcWorkspace.evGx[594]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[595]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[596]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[597]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[598]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[599]*nmpcWorkspace.x[5] + nmpcWorkspace.d[99];
nmpcVariables.x[106] += + nmpcWorkspace.evGx[600]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[601]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[602]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[603]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[604]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[605]*nmpcWorkspace.x[5] + nmpcWorkspace.d[100];
nmpcVariables.x[107] += + nmpcWorkspace.evGx[606]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[607]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[608]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[609]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[610]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[611]*nmpcWorkspace.x[5] + nmpcWorkspace.d[101];
nmpcVariables.x[108] += + nmpcWorkspace.evGx[612]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[613]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[614]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[615]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[616]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[617]*nmpcWorkspace.x[5] + nmpcWorkspace.d[102];
nmpcVariables.x[109] += + nmpcWorkspace.evGx[618]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[619]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[620]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[621]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[622]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[623]*nmpcWorkspace.x[5] + nmpcWorkspace.d[103];
nmpcVariables.x[110] += + nmpcWorkspace.evGx[624]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[625]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[626]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[627]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[628]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[629]*nmpcWorkspace.x[5] + nmpcWorkspace.d[104];
nmpcVariables.x[111] += + nmpcWorkspace.evGx[630]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[631]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[632]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[633]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[634]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[635]*nmpcWorkspace.x[5] + nmpcWorkspace.d[105];
nmpcVariables.x[112] += + nmpcWorkspace.evGx[636]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[637]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[638]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[639]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[640]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[641]*nmpcWorkspace.x[5] + nmpcWorkspace.d[106];
nmpcVariables.x[113] += + nmpcWorkspace.evGx[642]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[643]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[644]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[645]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[646]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[647]*nmpcWorkspace.x[5] + nmpcWorkspace.d[107];
nmpcVariables.x[114] += + nmpcWorkspace.evGx[648]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[649]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[650]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[651]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[652]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[653]*nmpcWorkspace.x[5] + nmpcWorkspace.d[108];
nmpcVariables.x[115] += + nmpcWorkspace.evGx[654]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[655]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[656]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[657]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[658]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[659]*nmpcWorkspace.x[5] + nmpcWorkspace.d[109];
nmpcVariables.x[116] += + nmpcWorkspace.evGx[660]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[661]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[662]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[663]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[664]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[665]*nmpcWorkspace.x[5] + nmpcWorkspace.d[110];
nmpcVariables.x[117] += + nmpcWorkspace.evGx[666]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[667]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[668]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[669]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[670]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[671]*nmpcWorkspace.x[5] + nmpcWorkspace.d[111];
nmpcVariables.x[118] += + nmpcWorkspace.evGx[672]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[673]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[674]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[675]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[676]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[677]*nmpcWorkspace.x[5] + nmpcWorkspace.d[112];
nmpcVariables.x[119] += + nmpcWorkspace.evGx[678]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[679]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[680]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[681]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[682]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[683]*nmpcWorkspace.x[5] + nmpcWorkspace.d[113];
nmpcVariables.x[120] += + nmpcWorkspace.evGx[684]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[685]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[686]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[687]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[688]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[689]*nmpcWorkspace.x[5] + nmpcWorkspace.d[114];
nmpcVariables.x[121] += + nmpcWorkspace.evGx[690]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[691]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[692]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[693]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[694]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[695]*nmpcWorkspace.x[5] + nmpcWorkspace.d[115];
nmpcVariables.x[122] += + nmpcWorkspace.evGx[696]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[697]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[698]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[699]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[700]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[701]*nmpcWorkspace.x[5] + nmpcWorkspace.d[116];
nmpcVariables.x[123] += + nmpcWorkspace.evGx[702]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[703]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[704]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[705]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[706]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[707]*nmpcWorkspace.x[5] + nmpcWorkspace.d[117];
nmpcVariables.x[124] += + nmpcWorkspace.evGx[708]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[709]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[710]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[711]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[712]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[713]*nmpcWorkspace.x[5] + nmpcWorkspace.d[118];
nmpcVariables.x[125] += + nmpcWorkspace.evGx[714]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[715]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[716]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[717]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[718]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[719]*nmpcWorkspace.x[5] + nmpcWorkspace.d[119];
nmpcVariables.x[126] += + nmpcWorkspace.evGx[720]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[721]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[722]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[723]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[724]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[725]*nmpcWorkspace.x[5] + nmpcWorkspace.d[120];
nmpcVariables.x[127] += + nmpcWorkspace.evGx[726]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[727]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[728]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[729]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[730]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[731]*nmpcWorkspace.x[5] + nmpcWorkspace.d[121];
nmpcVariables.x[128] += + nmpcWorkspace.evGx[732]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[733]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[734]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[735]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[736]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[737]*nmpcWorkspace.x[5] + nmpcWorkspace.d[122];
nmpcVariables.x[129] += + nmpcWorkspace.evGx[738]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[739]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[740]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[741]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[742]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[743]*nmpcWorkspace.x[5] + nmpcWorkspace.d[123];
nmpcVariables.x[130] += + nmpcWorkspace.evGx[744]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[745]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[746]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[747]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[748]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[749]*nmpcWorkspace.x[5] + nmpcWorkspace.d[124];
nmpcVariables.x[131] += + nmpcWorkspace.evGx[750]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[751]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[752]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[753]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[754]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[755]*nmpcWorkspace.x[5] + nmpcWorkspace.d[125];
nmpcVariables.x[132] += + nmpcWorkspace.evGx[756]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[757]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[758]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[759]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[760]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[761]*nmpcWorkspace.x[5] + nmpcWorkspace.d[126];
nmpcVariables.x[133] += + nmpcWorkspace.evGx[762]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[763]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[764]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[765]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[766]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[767]*nmpcWorkspace.x[5] + nmpcWorkspace.d[127];
nmpcVariables.x[134] += + nmpcWorkspace.evGx[768]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[769]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[770]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[771]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[772]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[773]*nmpcWorkspace.x[5] + nmpcWorkspace.d[128];
nmpcVariables.x[135] += + nmpcWorkspace.evGx[774]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[775]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[776]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[777]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[778]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[779]*nmpcWorkspace.x[5] + nmpcWorkspace.d[129];
nmpcVariables.x[136] += + nmpcWorkspace.evGx[780]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[781]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[782]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[783]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[784]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[785]*nmpcWorkspace.x[5] + nmpcWorkspace.d[130];
nmpcVariables.x[137] += + nmpcWorkspace.evGx[786]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[787]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[788]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[789]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[790]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[791]*nmpcWorkspace.x[5] + nmpcWorkspace.d[131];
nmpcVariables.x[138] += + nmpcWorkspace.evGx[792]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[793]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[794]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[795]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[796]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[797]*nmpcWorkspace.x[5] + nmpcWorkspace.d[132];
nmpcVariables.x[139] += + nmpcWorkspace.evGx[798]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[799]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[800]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[801]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[802]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[803]*nmpcWorkspace.x[5] + nmpcWorkspace.d[133];
nmpcVariables.x[140] += + nmpcWorkspace.evGx[804]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[805]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[806]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[807]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[808]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[809]*nmpcWorkspace.x[5] + nmpcWorkspace.d[134];
nmpcVariables.x[141] += + nmpcWorkspace.evGx[810]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[811]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[812]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[813]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[814]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[815]*nmpcWorkspace.x[5] + nmpcWorkspace.d[135];
nmpcVariables.x[142] += + nmpcWorkspace.evGx[816]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[817]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[818]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[819]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[820]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[821]*nmpcWorkspace.x[5] + nmpcWorkspace.d[136];
nmpcVariables.x[143] += + nmpcWorkspace.evGx[822]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[823]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[824]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[825]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[826]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[827]*nmpcWorkspace.x[5] + nmpcWorkspace.d[137];
nmpcVariables.x[144] += + nmpcWorkspace.evGx[828]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[829]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[830]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[831]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[832]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[833]*nmpcWorkspace.x[5] + nmpcWorkspace.d[138];
nmpcVariables.x[145] += + nmpcWorkspace.evGx[834]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[835]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[836]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[837]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[838]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[839]*nmpcWorkspace.x[5] + nmpcWorkspace.d[139];
nmpcVariables.x[146] += + nmpcWorkspace.evGx[840]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[841]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[842]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[843]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[844]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[845]*nmpcWorkspace.x[5] + nmpcWorkspace.d[140];
nmpcVariables.x[147] += + nmpcWorkspace.evGx[846]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[847]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[848]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[849]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[850]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[851]*nmpcWorkspace.x[5] + nmpcWorkspace.d[141];
nmpcVariables.x[148] += + nmpcWorkspace.evGx[852]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[853]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[854]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[855]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[856]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[857]*nmpcWorkspace.x[5] + nmpcWorkspace.d[142];
nmpcVariables.x[149] += + nmpcWorkspace.evGx[858]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[859]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[860]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[861]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[862]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[863]*nmpcWorkspace.x[5] + nmpcWorkspace.d[143];
nmpcVariables.x[150] += + nmpcWorkspace.evGx[864]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[865]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[866]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[867]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[868]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[869]*nmpcWorkspace.x[5] + nmpcWorkspace.d[144];
nmpcVariables.x[151] += + nmpcWorkspace.evGx[870]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[871]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[872]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[873]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[874]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[875]*nmpcWorkspace.x[5] + nmpcWorkspace.d[145];
nmpcVariables.x[152] += + nmpcWorkspace.evGx[876]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[877]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[878]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[879]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[880]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[881]*nmpcWorkspace.x[5] + nmpcWorkspace.d[146];
nmpcVariables.x[153] += + nmpcWorkspace.evGx[882]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[883]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[884]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[885]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[886]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[887]*nmpcWorkspace.x[5] + nmpcWorkspace.d[147];
nmpcVariables.x[154] += + nmpcWorkspace.evGx[888]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[889]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[890]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[891]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[892]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[893]*nmpcWorkspace.x[5] + nmpcWorkspace.d[148];
nmpcVariables.x[155] += + nmpcWorkspace.evGx[894]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[895]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[896]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[897]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[898]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[899]*nmpcWorkspace.x[5] + nmpcWorkspace.d[149];
nmpcVariables.x[156] += + nmpcWorkspace.evGx[900]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[901]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[902]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[903]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[904]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[905]*nmpcWorkspace.x[5] + nmpcWorkspace.d[150];
nmpcVariables.x[157] += + nmpcWorkspace.evGx[906]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[907]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[908]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[909]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[910]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[911]*nmpcWorkspace.x[5] + nmpcWorkspace.d[151];
nmpcVariables.x[158] += + nmpcWorkspace.evGx[912]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[913]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[914]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[915]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[916]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[917]*nmpcWorkspace.x[5] + nmpcWorkspace.d[152];
nmpcVariables.x[159] += + nmpcWorkspace.evGx[918]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[919]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[920]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[921]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[922]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[923]*nmpcWorkspace.x[5] + nmpcWorkspace.d[153];
nmpcVariables.x[160] += + nmpcWorkspace.evGx[924]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[925]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[926]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[927]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[928]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[929]*nmpcWorkspace.x[5] + nmpcWorkspace.d[154];
nmpcVariables.x[161] += + nmpcWorkspace.evGx[930]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[931]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[932]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[933]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[934]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[935]*nmpcWorkspace.x[5] + nmpcWorkspace.d[155];
nmpcVariables.x[162] += + nmpcWorkspace.evGx[936]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[937]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[938]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[939]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[940]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[941]*nmpcWorkspace.x[5] + nmpcWorkspace.d[156];
nmpcVariables.x[163] += + nmpcWorkspace.evGx[942]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[943]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[944]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[945]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[946]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[947]*nmpcWorkspace.x[5] + nmpcWorkspace.d[157];
nmpcVariables.x[164] += + nmpcWorkspace.evGx[948]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[949]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[950]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[951]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[952]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[953]*nmpcWorkspace.x[5] + nmpcWorkspace.d[158];
nmpcVariables.x[165] += + nmpcWorkspace.evGx[954]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[955]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[956]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[957]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[958]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[959]*nmpcWorkspace.x[5] + nmpcWorkspace.d[159];
nmpcVariables.x[166] += + nmpcWorkspace.evGx[960]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[961]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[962]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[963]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[964]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[965]*nmpcWorkspace.x[5] + nmpcWorkspace.d[160];
nmpcVariables.x[167] += + nmpcWorkspace.evGx[966]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[967]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[968]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[969]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[970]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[971]*nmpcWorkspace.x[5] + nmpcWorkspace.d[161];
nmpcVariables.x[168] += + nmpcWorkspace.evGx[972]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[973]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[974]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[975]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[976]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[977]*nmpcWorkspace.x[5] + nmpcWorkspace.d[162];
nmpcVariables.x[169] += + nmpcWorkspace.evGx[978]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[979]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[980]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[981]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[982]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[983]*nmpcWorkspace.x[5] + nmpcWorkspace.d[163];
nmpcVariables.x[170] += + nmpcWorkspace.evGx[984]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[985]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[986]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[987]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[988]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[989]*nmpcWorkspace.x[5] + nmpcWorkspace.d[164];
nmpcVariables.x[171] += + nmpcWorkspace.evGx[990]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[991]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[992]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[993]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[994]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[995]*nmpcWorkspace.x[5] + nmpcWorkspace.d[165];
nmpcVariables.x[172] += + nmpcWorkspace.evGx[996]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[997]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[998]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[999]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1000]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1001]*nmpcWorkspace.x[5] + nmpcWorkspace.d[166];
nmpcVariables.x[173] += + nmpcWorkspace.evGx[1002]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1003]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1004]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1005]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1006]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1007]*nmpcWorkspace.x[5] + nmpcWorkspace.d[167];
nmpcVariables.x[174] += + nmpcWorkspace.evGx[1008]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1009]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1010]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1011]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1012]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1013]*nmpcWorkspace.x[5] + nmpcWorkspace.d[168];
nmpcVariables.x[175] += + nmpcWorkspace.evGx[1014]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1015]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1016]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1017]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1018]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1019]*nmpcWorkspace.x[5] + nmpcWorkspace.d[169];
nmpcVariables.x[176] += + nmpcWorkspace.evGx[1020]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1021]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1022]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1023]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1024]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1025]*nmpcWorkspace.x[5] + nmpcWorkspace.d[170];
nmpcVariables.x[177] += + nmpcWorkspace.evGx[1026]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1027]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1028]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1029]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1030]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1031]*nmpcWorkspace.x[5] + nmpcWorkspace.d[171];
nmpcVariables.x[178] += + nmpcWorkspace.evGx[1032]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1033]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1034]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1035]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1036]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1037]*nmpcWorkspace.x[5] + nmpcWorkspace.d[172];
nmpcVariables.x[179] += + nmpcWorkspace.evGx[1038]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1039]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1040]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1041]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1042]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1043]*nmpcWorkspace.x[5] + nmpcWorkspace.d[173];
nmpcVariables.x[180] += + nmpcWorkspace.evGx[1044]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1045]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1046]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1047]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1048]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1049]*nmpcWorkspace.x[5] + nmpcWorkspace.d[174];
nmpcVariables.x[181] += + nmpcWorkspace.evGx[1050]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1051]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1052]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1053]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1054]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1055]*nmpcWorkspace.x[5] + nmpcWorkspace.d[175];
nmpcVariables.x[182] += + nmpcWorkspace.evGx[1056]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1057]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1058]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1059]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1060]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1061]*nmpcWorkspace.x[5] + nmpcWorkspace.d[176];
nmpcVariables.x[183] += + nmpcWorkspace.evGx[1062]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1063]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1064]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1065]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1066]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1067]*nmpcWorkspace.x[5] + nmpcWorkspace.d[177];
nmpcVariables.x[184] += + nmpcWorkspace.evGx[1068]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1069]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1070]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1071]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1072]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1073]*nmpcWorkspace.x[5] + nmpcWorkspace.d[178];
nmpcVariables.x[185] += + nmpcWorkspace.evGx[1074]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1075]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1076]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1077]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1078]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1079]*nmpcWorkspace.x[5] + nmpcWorkspace.d[179];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 24 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 6 ]), &(nmpcVariables.x[ lRun1 * 6 + 6 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 6];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 6 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 6 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 6 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 6 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 6 + 5];
nmpcWorkspace.state[66] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[67] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[68] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[69] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[70] = nmpcVariables.od[index * 6];
nmpcWorkspace.state[71] = nmpcVariables.od[index * 6 + 1];
nmpcWorkspace.state[72] = nmpcVariables.od[index * 6 + 2];
nmpcWorkspace.state[73] = nmpcVariables.od[index * 6 + 3];
nmpcWorkspace.state[74] = nmpcVariables.od[index * 6 + 4];
nmpcWorkspace.state[75] = nmpcVariables.od[index * 6 + 5];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 6 + 6] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 6 + 7] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 6 + 8] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 6 + 9] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 6 + 10] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 6 + 11] = nmpcWorkspace.state[5];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 6] = nmpcVariables.x[index * 6 + 6];
nmpcVariables.x[index * 6 + 1] = nmpcVariables.x[index * 6 + 7];
nmpcVariables.x[index * 6 + 2] = nmpcVariables.x[index * 6 + 8];
nmpcVariables.x[index * 6 + 3] = nmpcVariables.x[index * 6 + 9];
nmpcVariables.x[index * 6 + 4] = nmpcVariables.x[index * 6 + 10];
nmpcVariables.x[index * 6 + 5] = nmpcVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[180] = xEnd[0];
nmpcVariables.x[181] = xEnd[1];
nmpcVariables.x[182] = xEnd[2];
nmpcVariables.x[183] = xEnd[3];
nmpcVariables.x[184] = xEnd[4];
nmpcVariables.x[185] = xEnd[5];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[180];
nmpcWorkspace.state[1] = nmpcVariables.x[181];
nmpcWorkspace.state[2] = nmpcVariables.x[182];
nmpcWorkspace.state[3] = nmpcVariables.x[183];
nmpcWorkspace.state[4] = nmpcVariables.x[184];
nmpcWorkspace.state[5] = nmpcVariables.x[185];
if (uEnd != 0)
{
nmpcWorkspace.state[66] = uEnd[0];
nmpcWorkspace.state[67] = uEnd[1];
nmpcWorkspace.state[68] = uEnd[2];
nmpcWorkspace.state[69] = uEnd[3];
}
else
{
nmpcWorkspace.state[66] = nmpcVariables.u[116];
nmpcWorkspace.state[67] = nmpcVariables.u[117];
nmpcWorkspace.state[68] = nmpcVariables.u[118];
nmpcWorkspace.state[69] = nmpcVariables.u[119];
}
nmpcWorkspace.state[70] = nmpcVariables.od[180];
nmpcWorkspace.state[71] = nmpcVariables.od[181];
nmpcWorkspace.state[72] = nmpcVariables.od[182];
nmpcWorkspace.state[73] = nmpcVariables.od[183];
nmpcWorkspace.state[74] = nmpcVariables.od[184];
nmpcWorkspace.state[75] = nmpcVariables.od[185];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[180] = nmpcWorkspace.state[0];
nmpcVariables.x[181] = nmpcWorkspace.state[1];
nmpcVariables.x[182] = nmpcWorkspace.state[2];
nmpcVariables.x[183] = nmpcWorkspace.state[3];
nmpcVariables.x[184] = nmpcWorkspace.state[4];
nmpcVariables.x[185] = nmpcWorkspace.state[5];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[116] = uEnd[0];
nmpcVariables.u[117] = uEnd[1];
nmpcVariables.u[118] = uEnd[2];
nmpcVariables.u[119] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125];
kkt = fabs( kkt );
for (index = 0; index < 126; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 10 */
real_t tmpDy[ 10 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 6];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 6 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 6 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 6 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 6 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 6 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[7] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[lRun1 * 6];
nmpcWorkspace.objValueIn[11] = nmpcVariables.od[lRun1 * 6 + 1];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[lRun1 * 6 + 2];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[lRun1 * 6 + 3];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[lRun1 * 6 + 4];
nmpcWorkspace.objValueIn[15] = nmpcVariables.od[lRun1 * 6 + 5];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 10] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 10];
nmpcWorkspace.Dy[lRun1 * 10 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 10 + 1];
nmpcWorkspace.Dy[lRun1 * 10 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 10 + 2];
nmpcWorkspace.Dy[lRun1 * 10 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 10 + 3];
nmpcWorkspace.Dy[lRun1 * 10 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 10 + 4];
nmpcWorkspace.Dy[lRun1 * 10 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 10 + 5];
nmpcWorkspace.Dy[lRun1 * 10 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 10 + 6];
nmpcWorkspace.Dy[lRun1 * 10 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 10 + 7];
nmpcWorkspace.Dy[lRun1 * 10 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 10 + 8];
nmpcWorkspace.Dy[lRun1 * 10 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 10 + 9];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[180];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[181];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[182];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[183];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[184];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[185];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[180];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[181];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[182];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[183];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[184];
nmpcWorkspace.objValueIn[11] = nmpcVariables.od[185];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 10]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 10 + 1]*nmpcVariables.W[11];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 10 + 2]*nmpcVariables.W[22];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 10 + 3]*nmpcVariables.W[33];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 10 + 4]*nmpcVariables.W[44];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 10 + 5]*nmpcVariables.W[55];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 10 + 6]*nmpcVariables.W[66];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 10 + 7]*nmpcVariables.W[77];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 10 + 8]*nmpcVariables.W[88];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 10 + 9]*nmpcVariables.W[99];
objVal += + nmpcWorkspace.Dy[lRun1 * 10]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 10 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 10 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 10 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 10 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 10 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 10 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 10 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 10 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 10 + 9]*tmpDy[9];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[7];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[14];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[21];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[28];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[35];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

