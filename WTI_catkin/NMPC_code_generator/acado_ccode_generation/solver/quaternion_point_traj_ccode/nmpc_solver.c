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
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 16];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 16 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 16 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 16 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 16 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 16 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 16 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 16 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[lRun1 * 16 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[lRun1 * 16 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[lRun1 * 16 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[lRun1 * 16 + 11];
nmpcWorkspace.state[12] = nmpcVariables.x[lRun1 * 16 + 12];
nmpcWorkspace.state[13] = nmpcVariables.x[lRun1 * 16 + 13];
nmpcWorkspace.state[14] = nmpcVariables.x[lRun1 * 16 + 14];
nmpcWorkspace.state[15] = nmpcVariables.x[lRun1 * 16 + 15];

nmpcWorkspace.state[336] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[337] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[338] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[339] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[340] = nmpcVariables.od[lRun1 * 9];
nmpcWorkspace.state[341] = nmpcVariables.od[lRun1 * 9 + 1];
nmpcWorkspace.state[342] = nmpcVariables.od[lRun1 * 9 + 2];
nmpcWorkspace.state[343] = nmpcVariables.od[lRun1 * 9 + 3];
nmpcWorkspace.state[344] = nmpcVariables.od[lRun1 * 9 + 4];
nmpcWorkspace.state[345] = nmpcVariables.od[lRun1 * 9 + 5];
nmpcWorkspace.state[346] = nmpcVariables.od[lRun1 * 9 + 6];
nmpcWorkspace.state[347] = nmpcVariables.od[lRun1 * 9 + 7];
nmpcWorkspace.state[348] = nmpcVariables.od[lRun1 * 9 + 8];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 16] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 16 + 16];
nmpcWorkspace.d[lRun1 * 16 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 16 + 17];
nmpcWorkspace.d[lRun1 * 16 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 16 + 18];
nmpcWorkspace.d[lRun1 * 16 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 16 + 19];
nmpcWorkspace.d[lRun1 * 16 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 16 + 20];
nmpcWorkspace.d[lRun1 * 16 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 16 + 21];
nmpcWorkspace.d[lRun1 * 16 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 16 + 22];
nmpcWorkspace.d[lRun1 * 16 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 16 + 23];
nmpcWorkspace.d[lRun1 * 16 + 8] = nmpcWorkspace.state[8] - nmpcVariables.x[lRun1 * 16 + 24];
nmpcWorkspace.d[lRun1 * 16 + 9] = nmpcWorkspace.state[9] - nmpcVariables.x[lRun1 * 16 + 25];
nmpcWorkspace.d[lRun1 * 16 + 10] = nmpcWorkspace.state[10] - nmpcVariables.x[lRun1 * 16 + 26];
nmpcWorkspace.d[lRun1 * 16 + 11] = nmpcWorkspace.state[11] - nmpcVariables.x[lRun1 * 16 + 27];
nmpcWorkspace.d[lRun1 * 16 + 12] = nmpcWorkspace.state[12] - nmpcVariables.x[lRun1 * 16 + 28];
nmpcWorkspace.d[lRun1 * 16 + 13] = nmpcWorkspace.state[13] - nmpcVariables.x[lRun1 * 16 + 29];
nmpcWorkspace.d[lRun1 * 16 + 14] = nmpcWorkspace.state[14] - nmpcVariables.x[lRun1 * 16 + 30];
nmpcWorkspace.d[lRun1 * 16 + 15] = nmpcWorkspace.state[15] - nmpcVariables.x[lRun1 * 16 + 31];

for (lRun2 = 0; lRun2 < 256; ++lRun2)
nmpcWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 256))] = nmpcWorkspace.state[lRun2 + 16];


nmpcWorkspace.evGu[lRun1 * 64] = nmpcWorkspace.state[272];
nmpcWorkspace.evGu[lRun1 * 64 + 1] = nmpcWorkspace.state[273];
nmpcWorkspace.evGu[lRun1 * 64 + 2] = nmpcWorkspace.state[274];
nmpcWorkspace.evGu[lRun1 * 64 + 3] = nmpcWorkspace.state[275];
nmpcWorkspace.evGu[lRun1 * 64 + 4] = nmpcWorkspace.state[276];
nmpcWorkspace.evGu[lRun1 * 64 + 5] = nmpcWorkspace.state[277];
nmpcWorkspace.evGu[lRun1 * 64 + 6] = nmpcWorkspace.state[278];
nmpcWorkspace.evGu[lRun1 * 64 + 7] = nmpcWorkspace.state[279];
nmpcWorkspace.evGu[lRun1 * 64 + 8] = nmpcWorkspace.state[280];
nmpcWorkspace.evGu[lRun1 * 64 + 9] = nmpcWorkspace.state[281];
nmpcWorkspace.evGu[lRun1 * 64 + 10] = nmpcWorkspace.state[282];
nmpcWorkspace.evGu[lRun1 * 64 + 11] = nmpcWorkspace.state[283];
nmpcWorkspace.evGu[lRun1 * 64 + 12] = nmpcWorkspace.state[284];
nmpcWorkspace.evGu[lRun1 * 64 + 13] = nmpcWorkspace.state[285];
nmpcWorkspace.evGu[lRun1 * 64 + 14] = nmpcWorkspace.state[286];
nmpcWorkspace.evGu[lRun1 * 64 + 15] = nmpcWorkspace.state[287];
nmpcWorkspace.evGu[lRun1 * 64 + 16] = nmpcWorkspace.state[288];
nmpcWorkspace.evGu[lRun1 * 64 + 17] = nmpcWorkspace.state[289];
nmpcWorkspace.evGu[lRun1 * 64 + 18] = nmpcWorkspace.state[290];
nmpcWorkspace.evGu[lRun1 * 64 + 19] = nmpcWorkspace.state[291];
nmpcWorkspace.evGu[lRun1 * 64 + 20] = nmpcWorkspace.state[292];
nmpcWorkspace.evGu[lRun1 * 64 + 21] = nmpcWorkspace.state[293];
nmpcWorkspace.evGu[lRun1 * 64 + 22] = nmpcWorkspace.state[294];
nmpcWorkspace.evGu[lRun1 * 64 + 23] = nmpcWorkspace.state[295];
nmpcWorkspace.evGu[lRun1 * 64 + 24] = nmpcWorkspace.state[296];
nmpcWorkspace.evGu[lRun1 * 64 + 25] = nmpcWorkspace.state[297];
nmpcWorkspace.evGu[lRun1 * 64 + 26] = nmpcWorkspace.state[298];
nmpcWorkspace.evGu[lRun1 * 64 + 27] = nmpcWorkspace.state[299];
nmpcWorkspace.evGu[lRun1 * 64 + 28] = nmpcWorkspace.state[300];
nmpcWorkspace.evGu[lRun1 * 64 + 29] = nmpcWorkspace.state[301];
nmpcWorkspace.evGu[lRun1 * 64 + 30] = nmpcWorkspace.state[302];
nmpcWorkspace.evGu[lRun1 * 64 + 31] = nmpcWorkspace.state[303];
nmpcWorkspace.evGu[lRun1 * 64 + 32] = nmpcWorkspace.state[304];
nmpcWorkspace.evGu[lRun1 * 64 + 33] = nmpcWorkspace.state[305];
nmpcWorkspace.evGu[lRun1 * 64 + 34] = nmpcWorkspace.state[306];
nmpcWorkspace.evGu[lRun1 * 64 + 35] = nmpcWorkspace.state[307];
nmpcWorkspace.evGu[lRun1 * 64 + 36] = nmpcWorkspace.state[308];
nmpcWorkspace.evGu[lRun1 * 64 + 37] = nmpcWorkspace.state[309];
nmpcWorkspace.evGu[lRun1 * 64 + 38] = nmpcWorkspace.state[310];
nmpcWorkspace.evGu[lRun1 * 64 + 39] = nmpcWorkspace.state[311];
nmpcWorkspace.evGu[lRun1 * 64 + 40] = nmpcWorkspace.state[312];
nmpcWorkspace.evGu[lRun1 * 64 + 41] = nmpcWorkspace.state[313];
nmpcWorkspace.evGu[lRun1 * 64 + 42] = nmpcWorkspace.state[314];
nmpcWorkspace.evGu[lRun1 * 64 + 43] = nmpcWorkspace.state[315];
nmpcWorkspace.evGu[lRun1 * 64 + 44] = nmpcWorkspace.state[316];
nmpcWorkspace.evGu[lRun1 * 64 + 45] = nmpcWorkspace.state[317];
nmpcWorkspace.evGu[lRun1 * 64 + 46] = nmpcWorkspace.state[318];
nmpcWorkspace.evGu[lRun1 * 64 + 47] = nmpcWorkspace.state[319];
nmpcWorkspace.evGu[lRun1 * 64 + 48] = nmpcWorkspace.state[320];
nmpcWorkspace.evGu[lRun1 * 64 + 49] = nmpcWorkspace.state[321];
nmpcWorkspace.evGu[lRun1 * 64 + 50] = nmpcWorkspace.state[322];
nmpcWorkspace.evGu[lRun1 * 64 + 51] = nmpcWorkspace.state[323];
nmpcWorkspace.evGu[lRun1 * 64 + 52] = nmpcWorkspace.state[324];
nmpcWorkspace.evGu[lRun1 * 64 + 53] = nmpcWorkspace.state[325];
nmpcWorkspace.evGu[lRun1 * 64 + 54] = nmpcWorkspace.state[326];
nmpcWorkspace.evGu[lRun1 * 64 + 55] = nmpcWorkspace.state[327];
nmpcWorkspace.evGu[lRun1 * 64 + 56] = nmpcWorkspace.state[328];
nmpcWorkspace.evGu[lRun1 * 64 + 57] = nmpcWorkspace.state[329];
nmpcWorkspace.evGu[lRun1 * 64 + 58] = nmpcWorkspace.state[330];
nmpcWorkspace.evGu[lRun1 * 64 + 59] = nmpcWorkspace.state[331];
nmpcWorkspace.evGu[lRun1 * 64 + 60] = nmpcWorkspace.state[332];
nmpcWorkspace.evGu[lRun1 * 64 + 61] = nmpcWorkspace.state[333];
nmpcWorkspace.evGu[lRun1 * 64 + 62] = nmpcWorkspace.state[334];
nmpcWorkspace.evGu[lRun1 * 64 + 63] = nmpcWorkspace.state[335];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 16;
const real_t* od = in + 20;
/* Vector of auxiliary variables; number of elements: 111. */
real_t* a = nmpcWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (od[3]-xd[0]);
a[1] = (od[4]-xd[1]);
a[2] = (sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[3] = (a[2]+(real_t)(1.0000000000000001e-05));
a[4] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])));
a[5] = ((od[6]*a[0])+(od[7]*a[1]));
a[6] = ((a[0]-((a[0]*od[6])*od[6]))-((a[1]*od[7])*od[6]));
a[7] = ((a[1]-((a[0]*od[6])*od[7]))-((a[1]*od[7])*od[7]));
a[8] = (od[5]-xd[2]);
a[9] = (sqrt((((a[6]*a[6])+(a[7]*a[7]))+(a[8]*a[8]))));
a[10] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[11] = (1.0/sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[12] = (a[11]*(real_t)(5.0000000000000000e-01));
a[13] = (((a[10]*a[0])+(a[0]*a[10]))*a[12]);
a[14] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[15] = (a[14]*a[14]);
a[16] = ((((real_t)(0.0000000000000000e+00)-(a[13]*a[15]))*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[10])));
a[17] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[18] = (((a[17]*a[1])+(a[1]*a[17]))*a[12]);
a[19] = ((((real_t)(0.0000000000000000e+00)-(a[18]*a[15]))*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[17])));
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])+((real_t)(2.0000000000000000e+00)*xd[8])))*a[0])+(((real_t)(2.0000000000000000e+00)*xd[9])*a[1])));
a[27] = (((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*xd[8])*a[1]));
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[35] = (((a[34]*a[0])+(a[0]*a[34]))*a[12]);
a[36] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[37] = (((a[36]*a[1])+(a[1]*a[36]))*a[12]);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (real_t)(0.0000000000000000e+00);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[53] = (od[6]*a[52]);
a[54] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[55] = (od[7]*a[54]);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[71] = (a[70]-((a[70]*od[6])*od[6]));
a[72] = ((real_t)(0.0000000000000000e+00)-((a[70]*od[6])*od[7]));
a[73] = (1.0/sqrt((((a[6]*a[6])+(a[7]*a[7]))+(a[8]*a[8]))));
a[74] = (a[73]*(real_t)(5.0000000000000000e-01));
a[75] = ((((a[71]*a[6])+(a[6]*a[71]))+((a[72]*a[7])+(a[7]*a[72])))*a[74]);
a[76] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[77] = ((real_t)(0.0000000000000000e+00)-((a[76]*od[7])*od[6]));
a[78] = (a[76]-((a[76]*od[7])*od[7]));
a[79] = ((((a[77]*a[6])+(a[6]*a[77]))+((a[78]*a[7])+(a[7]*a[78])))*a[74]);
a[80] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[81] = (((a[80]*a[8])+(a[8]*a[80]))*a[74]);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (real_t)(0.0000000000000000e+00);
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (real_t)(0.0000000000000000e+00);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);
a[88] = (real_t)(0.0000000000000000e+00);
a[89] = (real_t)(0.0000000000000000e+00);
a[90] = (real_t)(0.0000000000000000e+00);
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (real_t)(0.0000000000000000e+00);
a[95] = (real_t)(0.0000000000000000e+00);
a[96] = (real_t)(0.0000000000000000e+00);
a[97] = (real_t)(0.0000000000000000e+00);
a[98] = (real_t)(0.0000000000000000e+00);
a[99] = (real_t)(0.0000000000000000e+00);
a[100] = (real_t)(0.0000000000000000e+00);
a[101] = (real_t)(0.0000000000000000e+00);
a[102] = (real_t)(0.0000000000000000e+00);
a[103] = (real_t)(0.0000000000000000e+00);
a[104] = (real_t)(0.0000000000000000e+00);
a[105] = (real_t)(0.0000000000000000e+00);
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);
a[108] = (real_t)(0.0000000000000000e+00);
a[109] = (real_t)(0.0000000000000000e+00);
a[110] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = a[4];
out[11] = a[3];
out[12] = a[5];
out[13] = a[9];
out[14] = u[0];
out[15] = u[1];
out[16] = u[2];
out[17] = u[3];
out[18] = (real_t)(1.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(1.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(1.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(1.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(1.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(1.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(1.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(1.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(1.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(1.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = a[16];
out[179] = a[19];
out[180] = a[20];
out[181] = a[21];
out[182] = a[22];
out[183] = a[23];
out[184] = a[24];
out[185] = a[25];
out[186] = a[26];
out[187] = a[27];
out[188] = a[28];
out[189] = a[29];
out[190] = a[30];
out[191] = a[31];
out[192] = a[32];
out[193] = a[33];
out[194] = a[35];
out[195] = a[37];
out[196] = a[38];
out[197] = a[39];
out[198] = a[40];
out[199] = a[41];
out[200] = a[42];
out[201] = a[43];
out[202] = a[44];
out[203] = a[45];
out[204] = a[46];
out[205] = a[47];
out[206] = a[48];
out[207] = a[49];
out[208] = a[50];
out[209] = a[51];
out[210] = a[53];
out[211] = a[55];
out[212] = a[56];
out[213] = a[57];
out[214] = a[58];
out[215] = a[59];
out[216] = a[60];
out[217] = a[61];
out[218] = a[62];
out[219] = a[63];
out[220] = a[64];
out[221] = a[65];
out[222] = a[66];
out[223] = a[67];
out[224] = a[68];
out[225] = a[69];
out[226] = a[75];
out[227] = a[79];
out[228] = a[81];
out[229] = a[82];
out[230] = a[83];
out[231] = a[84];
out[232] = a[85];
out[233] = a[86];
out[234] = a[87];
out[235] = a[88];
out[236] = a[89];
out[237] = a[90];
out[238] = a[91];
out[239] = a[92];
out[240] = a[93];
out[241] = a[94];
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = (real_t)(0.0000000000000000e+00);
out[245] = (real_t)(0.0000000000000000e+00);
out[246] = (real_t)(0.0000000000000000e+00);
out[247] = (real_t)(0.0000000000000000e+00);
out[248] = (real_t)(0.0000000000000000e+00);
out[249] = (real_t)(0.0000000000000000e+00);
out[250] = (real_t)(0.0000000000000000e+00);
out[251] = (real_t)(0.0000000000000000e+00);
out[252] = (real_t)(0.0000000000000000e+00);
out[253] = (real_t)(0.0000000000000000e+00);
out[254] = (real_t)(0.0000000000000000e+00);
out[255] = (real_t)(0.0000000000000000e+00);
out[256] = (real_t)(0.0000000000000000e+00);
out[257] = (real_t)(0.0000000000000000e+00);
out[258] = (real_t)(0.0000000000000000e+00);
out[259] = (real_t)(0.0000000000000000e+00);
out[260] = (real_t)(0.0000000000000000e+00);
out[261] = (real_t)(0.0000000000000000e+00);
out[262] = (real_t)(0.0000000000000000e+00);
out[263] = (real_t)(0.0000000000000000e+00);
out[264] = (real_t)(0.0000000000000000e+00);
out[265] = (real_t)(0.0000000000000000e+00);
out[266] = (real_t)(0.0000000000000000e+00);
out[267] = (real_t)(0.0000000000000000e+00);
out[268] = (real_t)(0.0000000000000000e+00);
out[269] = (real_t)(0.0000000000000000e+00);
out[270] = (real_t)(0.0000000000000000e+00);
out[271] = (real_t)(0.0000000000000000e+00);
out[272] = (real_t)(0.0000000000000000e+00);
out[273] = (real_t)(0.0000000000000000e+00);
out[274] = (real_t)(0.0000000000000000e+00);
out[275] = (real_t)(0.0000000000000000e+00);
out[276] = (real_t)(0.0000000000000000e+00);
out[277] = (real_t)(0.0000000000000000e+00);
out[278] = (real_t)(0.0000000000000000e+00);
out[279] = (real_t)(0.0000000000000000e+00);
out[280] = (real_t)(0.0000000000000000e+00);
out[281] = (real_t)(0.0000000000000000e+00);
out[282] = (real_t)(0.0000000000000000e+00);
out[283] = (real_t)(0.0000000000000000e+00);
out[284] = (real_t)(0.0000000000000000e+00);
out[285] = (real_t)(0.0000000000000000e+00);
out[286] = (real_t)(0.0000000000000000e+00);
out[287] = (real_t)(0.0000000000000000e+00);
out[288] = (real_t)(0.0000000000000000e+00);
out[289] = (real_t)(0.0000000000000000e+00);
out[290] = (real_t)(0.0000000000000000e+00);
out[291] = (real_t)(0.0000000000000000e+00);
out[292] = (real_t)(0.0000000000000000e+00);
out[293] = (real_t)(0.0000000000000000e+00);
out[294] = (real_t)(0.0000000000000000e+00);
out[295] = (real_t)(0.0000000000000000e+00);
out[296] = (real_t)(0.0000000000000000e+00);
out[297] = (real_t)(0.0000000000000000e+00);
out[298] = (real_t)(0.0000000000000000e+00);
out[299] = (real_t)(0.0000000000000000e+00);
out[300] = (real_t)(0.0000000000000000e+00);
out[301] = (real_t)(0.0000000000000000e+00);
out[302] = (real_t)(0.0000000000000000e+00);
out[303] = (real_t)(0.0000000000000000e+00);
out[304] = (real_t)(0.0000000000000000e+00);
out[305] = (real_t)(0.0000000000000000e+00);
out[306] = (real_t)(0.0000000000000000e+00);
out[307] = (real_t)(0.0000000000000000e+00);
out[308] = (real_t)(0.0000000000000000e+00);
out[309] = (real_t)(0.0000000000000000e+00);
out[310] = (real_t)(0.0000000000000000e+00);
out[311] = (real_t)(0.0000000000000000e+00);
out[312] = (real_t)(0.0000000000000000e+00);
out[313] = (real_t)(0.0000000000000000e+00);
out[314] = (real_t)(0.0000000000000000e+00);
out[315] = (real_t)(0.0000000000000000e+00);
out[316] = (real_t)(0.0000000000000000e+00);
out[317] = (real_t)(0.0000000000000000e+00);
out[318] = (real_t)(0.0000000000000000e+00);
out[319] = (real_t)(0.0000000000000000e+00);
out[320] = (real_t)(0.0000000000000000e+00);
out[321] = (real_t)(0.0000000000000000e+00);
out[322] = (real_t)(0.0000000000000000e+00);
out[323] = (real_t)(0.0000000000000000e+00);
out[324] = (real_t)(0.0000000000000000e+00);
out[325] = (real_t)(0.0000000000000000e+00);
out[326] = (real_t)(0.0000000000000000e+00);
out[327] = (real_t)(0.0000000000000000e+00);
out[328] = (real_t)(0.0000000000000000e+00);
out[329] = (real_t)(0.0000000000000000e+00);
out[330] = (real_t)(0.0000000000000000e+00);
out[331] = (real_t)(0.0000000000000000e+00);
out[332] = (real_t)(0.0000000000000000e+00);
out[333] = (real_t)(0.0000000000000000e+00);
out[334] = (real_t)(0.0000000000000000e+00);
out[335] = (real_t)(0.0000000000000000e+00);
out[336] = (real_t)(0.0000000000000000e+00);
out[337] = (real_t)(0.0000000000000000e+00);
out[338] = (real_t)(0.0000000000000000e+00);
out[339] = (real_t)(0.0000000000000000e+00);
out[340] = (real_t)(0.0000000000000000e+00);
out[341] = (real_t)(0.0000000000000000e+00);
out[342] = (real_t)(0.0000000000000000e+00);
out[343] = (real_t)(0.0000000000000000e+00);
out[344] = (real_t)(0.0000000000000000e+00);
out[345] = (real_t)(0.0000000000000000e+00);
out[346] = a[95];
out[347] = a[96];
out[348] = a[97];
out[349] = a[98];
out[350] = a[99];
out[351] = a[100];
out[352] = a[101];
out[353] = a[102];
out[354] = a[103];
out[355] = a[104];
out[356] = a[105];
out[357] = a[106];
out[358] = a[107];
out[359] = a[108];
out[360] = a[109];
out[361] = a[110];
out[362] = (real_t)(1.0000000000000000e+00);
out[363] = (real_t)(0.0000000000000000e+00);
out[364] = (real_t)(0.0000000000000000e+00);
out[365] = (real_t)(0.0000000000000000e+00);
out[366] = (real_t)(0.0000000000000000e+00);
out[367] = (real_t)(1.0000000000000000e+00);
out[368] = (real_t)(0.0000000000000000e+00);
out[369] = (real_t)(0.0000000000000000e+00);
out[370] = (real_t)(0.0000000000000000e+00);
out[371] = (real_t)(0.0000000000000000e+00);
out[372] = (real_t)(1.0000000000000000e+00);
out[373] = (real_t)(0.0000000000000000e+00);
out[374] = (real_t)(0.0000000000000000e+00);
out[375] = (real_t)(0.0000000000000000e+00);
out[376] = (real_t)(0.0000000000000000e+00);
out[377] = (real_t)(1.0000000000000000e+00);
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 16;
/* Vector of auxiliary variables; number of elements: 95. */
real_t* a = nmpcWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (od[3]-xd[0]);
a[1] = (od[4]-xd[1]);
a[2] = (sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[3] = (a[2]+(real_t)(1.0000000000000001e-05));
a[4] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])));
a[5] = ((od[6]*a[0])+(od[7]*a[1]));
a[6] = ((a[0]-((a[0]*od[6])*od[6]))-((a[1]*od[7])*od[6]));
a[7] = ((a[1]-((a[0]*od[6])*od[7]))-((a[1]*od[7])*od[7]));
a[8] = (od[5]-xd[2]);
a[9] = (sqrt((((a[6]*a[6])+(a[7]*a[7]))+(a[8]*a[8]))));
a[10] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[11] = (1.0/sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[12] = (a[11]*(real_t)(5.0000000000000000e-01));
a[13] = (((a[10]*a[0])+(a[0]*a[10]))*a[12]);
a[14] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[15] = (a[14]*a[14]);
a[16] = ((((real_t)(0.0000000000000000e+00)-(a[13]*a[15]))*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[10])));
a[17] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[18] = (((a[17]*a[1])+(a[1]*a[17]))*a[12]);
a[19] = ((((real_t)(0.0000000000000000e+00)-(a[18]*a[15]))*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*(xd[9]*xd[8]))*a[17])));
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])+((real_t)(2.0000000000000000e+00)*xd[8])))*a[0])+(((real_t)(2.0000000000000000e+00)*xd[9])*a[1])));
a[27] = (((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*xd[8])*a[1]));
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[35] = (((a[34]*a[0])+(a[0]*a[34]))*a[12]);
a[36] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[37] = (((a[36]*a[1])+(a[1]*a[36]))*a[12]);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (real_t)(0.0000000000000000e+00);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[53] = (od[6]*a[52]);
a[54] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[55] = (od[7]*a[54]);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[71] = (a[70]-((a[70]*od[6])*od[6]));
a[72] = ((real_t)(0.0000000000000000e+00)-((a[70]*od[6])*od[7]));
a[73] = (1.0/sqrt((((a[6]*a[6])+(a[7]*a[7]))+(a[8]*a[8]))));
a[74] = (a[73]*(real_t)(5.0000000000000000e-01));
a[75] = ((((a[71]*a[6])+(a[6]*a[71]))+((a[72]*a[7])+(a[7]*a[72])))*a[74]);
a[76] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[77] = ((real_t)(0.0000000000000000e+00)-((a[76]*od[7])*od[6]));
a[78] = (a[76]-((a[76]*od[7])*od[7]));
a[79] = ((((a[77]*a[6])+(a[6]*a[77]))+((a[78]*a[7])+(a[7]*a[78])))*a[74]);
a[80] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[81] = (((a[80]*a[8])+(a[8]*a[80]))*a[74]);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (real_t)(0.0000000000000000e+00);
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (real_t)(0.0000000000000000e+00);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);
a[88] = (real_t)(0.0000000000000000e+00);
a[89] = (real_t)(0.0000000000000000e+00);
a[90] = (real_t)(0.0000000000000000e+00);
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = a[4];
out[11] = a[3];
out[12] = a[5];
out[13] = a[9];
out[14] = (real_t)(1.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(1.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(1.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(1.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(1.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(1.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(1.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(1.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(1.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(1.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = a[16];
out[175] = a[19];
out[176] = a[20];
out[177] = a[21];
out[178] = a[22];
out[179] = a[23];
out[180] = a[24];
out[181] = a[25];
out[182] = a[26];
out[183] = a[27];
out[184] = a[28];
out[185] = a[29];
out[186] = a[30];
out[187] = a[31];
out[188] = a[32];
out[189] = a[33];
out[190] = a[35];
out[191] = a[37];
out[192] = a[38];
out[193] = a[39];
out[194] = a[40];
out[195] = a[41];
out[196] = a[42];
out[197] = a[43];
out[198] = a[44];
out[199] = a[45];
out[200] = a[46];
out[201] = a[47];
out[202] = a[48];
out[203] = a[49];
out[204] = a[50];
out[205] = a[51];
out[206] = a[53];
out[207] = a[55];
out[208] = a[56];
out[209] = a[57];
out[210] = a[58];
out[211] = a[59];
out[212] = a[60];
out[213] = a[61];
out[214] = a[62];
out[215] = a[63];
out[216] = a[64];
out[217] = a[65];
out[218] = a[66];
out[219] = a[67];
out[220] = a[68];
out[221] = a[69];
out[222] = a[75];
out[223] = a[79];
out[224] = a[81];
out[225] = a[82];
out[226] = a[83];
out[227] = a[84];
out[228] = a[85];
out[229] = a[86];
out[230] = a[87];
out[231] = a[88];
out[232] = a[89];
out[233] = a[90];
out[234] = a[91];
out[235] = a[92];
out[236] = a[93];
out[237] = a[94];
}

void nmpc_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 18; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 18; ++lRun3)
{
t += + tmpFx[(lRun3 * 16) + (lRun1)]*tmpObjS[(lRun3 * 18) + (lRun2)];
}
tmpQ2[(lRun1 * 18) + (lRun2)] = t;
}
}
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 18; ++lRun3)
{
t += + tmpQ2[(lRun1 * 18) + (lRun3)]*tmpFx[(lRun3 * 16) + (lRun2)];
}
tmpQ1[(lRun1 * 16) + (lRun2)] = t;
}
}
}

void nmpc_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[4]*tmpObjS[18] + tmpFu[8]*tmpObjS[36] + tmpFu[12]*tmpObjS[54] + tmpFu[16]*tmpObjS[72] + tmpFu[20]*tmpObjS[90] + tmpFu[24]*tmpObjS[108] + tmpFu[28]*tmpObjS[126] + tmpFu[32]*tmpObjS[144] + tmpFu[36]*tmpObjS[162] + tmpFu[40]*tmpObjS[180] + tmpFu[44]*tmpObjS[198] + tmpFu[48]*tmpObjS[216] + tmpFu[52]*tmpObjS[234] + tmpFu[56]*tmpObjS[252] + tmpFu[60]*tmpObjS[270] + tmpFu[64]*tmpObjS[288] + tmpFu[68]*tmpObjS[306];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[4]*tmpObjS[19] + tmpFu[8]*tmpObjS[37] + tmpFu[12]*tmpObjS[55] + tmpFu[16]*tmpObjS[73] + tmpFu[20]*tmpObjS[91] + tmpFu[24]*tmpObjS[109] + tmpFu[28]*tmpObjS[127] + tmpFu[32]*tmpObjS[145] + tmpFu[36]*tmpObjS[163] + tmpFu[40]*tmpObjS[181] + tmpFu[44]*tmpObjS[199] + tmpFu[48]*tmpObjS[217] + tmpFu[52]*tmpObjS[235] + tmpFu[56]*tmpObjS[253] + tmpFu[60]*tmpObjS[271] + tmpFu[64]*tmpObjS[289] + tmpFu[68]*tmpObjS[307];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[4]*tmpObjS[20] + tmpFu[8]*tmpObjS[38] + tmpFu[12]*tmpObjS[56] + tmpFu[16]*tmpObjS[74] + tmpFu[20]*tmpObjS[92] + tmpFu[24]*tmpObjS[110] + tmpFu[28]*tmpObjS[128] + tmpFu[32]*tmpObjS[146] + tmpFu[36]*tmpObjS[164] + tmpFu[40]*tmpObjS[182] + tmpFu[44]*tmpObjS[200] + tmpFu[48]*tmpObjS[218] + tmpFu[52]*tmpObjS[236] + tmpFu[56]*tmpObjS[254] + tmpFu[60]*tmpObjS[272] + tmpFu[64]*tmpObjS[290] + tmpFu[68]*tmpObjS[308];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[4]*tmpObjS[21] + tmpFu[8]*tmpObjS[39] + tmpFu[12]*tmpObjS[57] + tmpFu[16]*tmpObjS[75] + tmpFu[20]*tmpObjS[93] + tmpFu[24]*tmpObjS[111] + tmpFu[28]*tmpObjS[129] + tmpFu[32]*tmpObjS[147] + tmpFu[36]*tmpObjS[165] + tmpFu[40]*tmpObjS[183] + tmpFu[44]*tmpObjS[201] + tmpFu[48]*tmpObjS[219] + tmpFu[52]*tmpObjS[237] + tmpFu[56]*tmpObjS[255] + tmpFu[60]*tmpObjS[273] + tmpFu[64]*tmpObjS[291] + tmpFu[68]*tmpObjS[309];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[4]*tmpObjS[22] + tmpFu[8]*tmpObjS[40] + tmpFu[12]*tmpObjS[58] + tmpFu[16]*tmpObjS[76] + tmpFu[20]*tmpObjS[94] + tmpFu[24]*tmpObjS[112] + tmpFu[28]*tmpObjS[130] + tmpFu[32]*tmpObjS[148] + tmpFu[36]*tmpObjS[166] + tmpFu[40]*tmpObjS[184] + tmpFu[44]*tmpObjS[202] + tmpFu[48]*tmpObjS[220] + tmpFu[52]*tmpObjS[238] + tmpFu[56]*tmpObjS[256] + tmpFu[60]*tmpObjS[274] + tmpFu[64]*tmpObjS[292] + tmpFu[68]*tmpObjS[310];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[4]*tmpObjS[23] + tmpFu[8]*tmpObjS[41] + tmpFu[12]*tmpObjS[59] + tmpFu[16]*tmpObjS[77] + tmpFu[20]*tmpObjS[95] + tmpFu[24]*tmpObjS[113] + tmpFu[28]*tmpObjS[131] + tmpFu[32]*tmpObjS[149] + tmpFu[36]*tmpObjS[167] + tmpFu[40]*tmpObjS[185] + tmpFu[44]*tmpObjS[203] + tmpFu[48]*tmpObjS[221] + tmpFu[52]*tmpObjS[239] + tmpFu[56]*tmpObjS[257] + tmpFu[60]*tmpObjS[275] + tmpFu[64]*tmpObjS[293] + tmpFu[68]*tmpObjS[311];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[4]*tmpObjS[24] + tmpFu[8]*tmpObjS[42] + tmpFu[12]*tmpObjS[60] + tmpFu[16]*tmpObjS[78] + tmpFu[20]*tmpObjS[96] + tmpFu[24]*tmpObjS[114] + tmpFu[28]*tmpObjS[132] + tmpFu[32]*tmpObjS[150] + tmpFu[36]*tmpObjS[168] + tmpFu[40]*tmpObjS[186] + tmpFu[44]*tmpObjS[204] + tmpFu[48]*tmpObjS[222] + tmpFu[52]*tmpObjS[240] + tmpFu[56]*tmpObjS[258] + tmpFu[60]*tmpObjS[276] + tmpFu[64]*tmpObjS[294] + tmpFu[68]*tmpObjS[312];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[4]*tmpObjS[25] + tmpFu[8]*tmpObjS[43] + tmpFu[12]*tmpObjS[61] + tmpFu[16]*tmpObjS[79] + tmpFu[20]*tmpObjS[97] + tmpFu[24]*tmpObjS[115] + tmpFu[28]*tmpObjS[133] + tmpFu[32]*tmpObjS[151] + tmpFu[36]*tmpObjS[169] + tmpFu[40]*tmpObjS[187] + tmpFu[44]*tmpObjS[205] + tmpFu[48]*tmpObjS[223] + tmpFu[52]*tmpObjS[241] + tmpFu[56]*tmpObjS[259] + tmpFu[60]*tmpObjS[277] + tmpFu[64]*tmpObjS[295] + tmpFu[68]*tmpObjS[313];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[4]*tmpObjS[26] + tmpFu[8]*tmpObjS[44] + tmpFu[12]*tmpObjS[62] + tmpFu[16]*tmpObjS[80] + tmpFu[20]*tmpObjS[98] + tmpFu[24]*tmpObjS[116] + tmpFu[28]*tmpObjS[134] + tmpFu[32]*tmpObjS[152] + tmpFu[36]*tmpObjS[170] + tmpFu[40]*tmpObjS[188] + tmpFu[44]*tmpObjS[206] + tmpFu[48]*tmpObjS[224] + tmpFu[52]*tmpObjS[242] + tmpFu[56]*tmpObjS[260] + tmpFu[60]*tmpObjS[278] + tmpFu[64]*tmpObjS[296] + tmpFu[68]*tmpObjS[314];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[4]*tmpObjS[27] + tmpFu[8]*tmpObjS[45] + tmpFu[12]*tmpObjS[63] + tmpFu[16]*tmpObjS[81] + tmpFu[20]*tmpObjS[99] + tmpFu[24]*tmpObjS[117] + tmpFu[28]*tmpObjS[135] + tmpFu[32]*tmpObjS[153] + tmpFu[36]*tmpObjS[171] + tmpFu[40]*tmpObjS[189] + tmpFu[44]*tmpObjS[207] + tmpFu[48]*tmpObjS[225] + tmpFu[52]*tmpObjS[243] + tmpFu[56]*tmpObjS[261] + tmpFu[60]*tmpObjS[279] + tmpFu[64]*tmpObjS[297] + tmpFu[68]*tmpObjS[315];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[4]*tmpObjS[28] + tmpFu[8]*tmpObjS[46] + tmpFu[12]*tmpObjS[64] + tmpFu[16]*tmpObjS[82] + tmpFu[20]*tmpObjS[100] + tmpFu[24]*tmpObjS[118] + tmpFu[28]*tmpObjS[136] + tmpFu[32]*tmpObjS[154] + tmpFu[36]*tmpObjS[172] + tmpFu[40]*tmpObjS[190] + tmpFu[44]*tmpObjS[208] + tmpFu[48]*tmpObjS[226] + tmpFu[52]*tmpObjS[244] + tmpFu[56]*tmpObjS[262] + tmpFu[60]*tmpObjS[280] + tmpFu[64]*tmpObjS[298] + tmpFu[68]*tmpObjS[316];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[4]*tmpObjS[29] + tmpFu[8]*tmpObjS[47] + tmpFu[12]*tmpObjS[65] + tmpFu[16]*tmpObjS[83] + tmpFu[20]*tmpObjS[101] + tmpFu[24]*tmpObjS[119] + tmpFu[28]*tmpObjS[137] + tmpFu[32]*tmpObjS[155] + tmpFu[36]*tmpObjS[173] + tmpFu[40]*tmpObjS[191] + tmpFu[44]*tmpObjS[209] + tmpFu[48]*tmpObjS[227] + tmpFu[52]*tmpObjS[245] + tmpFu[56]*tmpObjS[263] + tmpFu[60]*tmpObjS[281] + tmpFu[64]*tmpObjS[299] + tmpFu[68]*tmpObjS[317];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[4]*tmpObjS[30] + tmpFu[8]*tmpObjS[48] + tmpFu[12]*tmpObjS[66] + tmpFu[16]*tmpObjS[84] + tmpFu[20]*tmpObjS[102] + tmpFu[24]*tmpObjS[120] + tmpFu[28]*tmpObjS[138] + tmpFu[32]*tmpObjS[156] + tmpFu[36]*tmpObjS[174] + tmpFu[40]*tmpObjS[192] + tmpFu[44]*tmpObjS[210] + tmpFu[48]*tmpObjS[228] + tmpFu[52]*tmpObjS[246] + tmpFu[56]*tmpObjS[264] + tmpFu[60]*tmpObjS[282] + tmpFu[64]*tmpObjS[300] + tmpFu[68]*tmpObjS[318];
tmpR2[13] = + tmpFu[0]*tmpObjS[13] + tmpFu[4]*tmpObjS[31] + tmpFu[8]*tmpObjS[49] + tmpFu[12]*tmpObjS[67] + tmpFu[16]*tmpObjS[85] + tmpFu[20]*tmpObjS[103] + tmpFu[24]*tmpObjS[121] + tmpFu[28]*tmpObjS[139] + tmpFu[32]*tmpObjS[157] + tmpFu[36]*tmpObjS[175] + tmpFu[40]*tmpObjS[193] + tmpFu[44]*tmpObjS[211] + tmpFu[48]*tmpObjS[229] + tmpFu[52]*tmpObjS[247] + tmpFu[56]*tmpObjS[265] + tmpFu[60]*tmpObjS[283] + tmpFu[64]*tmpObjS[301] + tmpFu[68]*tmpObjS[319];
tmpR2[14] = + tmpFu[0]*tmpObjS[14] + tmpFu[4]*tmpObjS[32] + tmpFu[8]*tmpObjS[50] + tmpFu[12]*tmpObjS[68] + tmpFu[16]*tmpObjS[86] + tmpFu[20]*tmpObjS[104] + tmpFu[24]*tmpObjS[122] + tmpFu[28]*tmpObjS[140] + tmpFu[32]*tmpObjS[158] + tmpFu[36]*tmpObjS[176] + tmpFu[40]*tmpObjS[194] + tmpFu[44]*tmpObjS[212] + tmpFu[48]*tmpObjS[230] + tmpFu[52]*tmpObjS[248] + tmpFu[56]*tmpObjS[266] + tmpFu[60]*tmpObjS[284] + tmpFu[64]*tmpObjS[302] + tmpFu[68]*tmpObjS[320];
tmpR2[15] = + tmpFu[0]*tmpObjS[15] + tmpFu[4]*tmpObjS[33] + tmpFu[8]*tmpObjS[51] + tmpFu[12]*tmpObjS[69] + tmpFu[16]*tmpObjS[87] + tmpFu[20]*tmpObjS[105] + tmpFu[24]*tmpObjS[123] + tmpFu[28]*tmpObjS[141] + tmpFu[32]*tmpObjS[159] + tmpFu[36]*tmpObjS[177] + tmpFu[40]*tmpObjS[195] + tmpFu[44]*tmpObjS[213] + tmpFu[48]*tmpObjS[231] + tmpFu[52]*tmpObjS[249] + tmpFu[56]*tmpObjS[267] + tmpFu[60]*tmpObjS[285] + tmpFu[64]*tmpObjS[303] + tmpFu[68]*tmpObjS[321];
tmpR2[16] = + tmpFu[0]*tmpObjS[16] + tmpFu[4]*tmpObjS[34] + tmpFu[8]*tmpObjS[52] + tmpFu[12]*tmpObjS[70] + tmpFu[16]*tmpObjS[88] + tmpFu[20]*tmpObjS[106] + tmpFu[24]*tmpObjS[124] + tmpFu[28]*tmpObjS[142] + tmpFu[32]*tmpObjS[160] + tmpFu[36]*tmpObjS[178] + tmpFu[40]*tmpObjS[196] + tmpFu[44]*tmpObjS[214] + tmpFu[48]*tmpObjS[232] + tmpFu[52]*tmpObjS[250] + tmpFu[56]*tmpObjS[268] + tmpFu[60]*tmpObjS[286] + tmpFu[64]*tmpObjS[304] + tmpFu[68]*tmpObjS[322];
tmpR2[17] = + tmpFu[0]*tmpObjS[17] + tmpFu[4]*tmpObjS[35] + tmpFu[8]*tmpObjS[53] + tmpFu[12]*tmpObjS[71] + tmpFu[16]*tmpObjS[89] + tmpFu[20]*tmpObjS[107] + tmpFu[24]*tmpObjS[125] + tmpFu[28]*tmpObjS[143] + tmpFu[32]*tmpObjS[161] + tmpFu[36]*tmpObjS[179] + tmpFu[40]*tmpObjS[197] + tmpFu[44]*tmpObjS[215] + tmpFu[48]*tmpObjS[233] + tmpFu[52]*tmpObjS[251] + tmpFu[56]*tmpObjS[269] + tmpFu[60]*tmpObjS[287] + tmpFu[64]*tmpObjS[305] + tmpFu[68]*tmpObjS[323];
tmpR2[18] = + tmpFu[1]*tmpObjS[0] + tmpFu[5]*tmpObjS[18] + tmpFu[9]*tmpObjS[36] + tmpFu[13]*tmpObjS[54] + tmpFu[17]*tmpObjS[72] + tmpFu[21]*tmpObjS[90] + tmpFu[25]*tmpObjS[108] + tmpFu[29]*tmpObjS[126] + tmpFu[33]*tmpObjS[144] + tmpFu[37]*tmpObjS[162] + tmpFu[41]*tmpObjS[180] + tmpFu[45]*tmpObjS[198] + tmpFu[49]*tmpObjS[216] + tmpFu[53]*tmpObjS[234] + tmpFu[57]*tmpObjS[252] + tmpFu[61]*tmpObjS[270] + tmpFu[65]*tmpObjS[288] + tmpFu[69]*tmpObjS[306];
tmpR2[19] = + tmpFu[1]*tmpObjS[1] + tmpFu[5]*tmpObjS[19] + tmpFu[9]*tmpObjS[37] + tmpFu[13]*tmpObjS[55] + tmpFu[17]*tmpObjS[73] + tmpFu[21]*tmpObjS[91] + tmpFu[25]*tmpObjS[109] + tmpFu[29]*tmpObjS[127] + tmpFu[33]*tmpObjS[145] + tmpFu[37]*tmpObjS[163] + tmpFu[41]*tmpObjS[181] + tmpFu[45]*tmpObjS[199] + tmpFu[49]*tmpObjS[217] + tmpFu[53]*tmpObjS[235] + tmpFu[57]*tmpObjS[253] + tmpFu[61]*tmpObjS[271] + tmpFu[65]*tmpObjS[289] + tmpFu[69]*tmpObjS[307];
tmpR2[20] = + tmpFu[1]*tmpObjS[2] + tmpFu[5]*tmpObjS[20] + tmpFu[9]*tmpObjS[38] + tmpFu[13]*tmpObjS[56] + tmpFu[17]*tmpObjS[74] + tmpFu[21]*tmpObjS[92] + tmpFu[25]*tmpObjS[110] + tmpFu[29]*tmpObjS[128] + tmpFu[33]*tmpObjS[146] + tmpFu[37]*tmpObjS[164] + tmpFu[41]*tmpObjS[182] + tmpFu[45]*tmpObjS[200] + tmpFu[49]*tmpObjS[218] + tmpFu[53]*tmpObjS[236] + tmpFu[57]*tmpObjS[254] + tmpFu[61]*tmpObjS[272] + tmpFu[65]*tmpObjS[290] + tmpFu[69]*tmpObjS[308];
tmpR2[21] = + tmpFu[1]*tmpObjS[3] + tmpFu[5]*tmpObjS[21] + tmpFu[9]*tmpObjS[39] + tmpFu[13]*tmpObjS[57] + tmpFu[17]*tmpObjS[75] + tmpFu[21]*tmpObjS[93] + tmpFu[25]*tmpObjS[111] + tmpFu[29]*tmpObjS[129] + tmpFu[33]*tmpObjS[147] + tmpFu[37]*tmpObjS[165] + tmpFu[41]*tmpObjS[183] + tmpFu[45]*tmpObjS[201] + tmpFu[49]*tmpObjS[219] + tmpFu[53]*tmpObjS[237] + tmpFu[57]*tmpObjS[255] + tmpFu[61]*tmpObjS[273] + tmpFu[65]*tmpObjS[291] + tmpFu[69]*tmpObjS[309];
tmpR2[22] = + tmpFu[1]*tmpObjS[4] + tmpFu[5]*tmpObjS[22] + tmpFu[9]*tmpObjS[40] + tmpFu[13]*tmpObjS[58] + tmpFu[17]*tmpObjS[76] + tmpFu[21]*tmpObjS[94] + tmpFu[25]*tmpObjS[112] + tmpFu[29]*tmpObjS[130] + tmpFu[33]*tmpObjS[148] + tmpFu[37]*tmpObjS[166] + tmpFu[41]*tmpObjS[184] + tmpFu[45]*tmpObjS[202] + tmpFu[49]*tmpObjS[220] + tmpFu[53]*tmpObjS[238] + tmpFu[57]*tmpObjS[256] + tmpFu[61]*tmpObjS[274] + tmpFu[65]*tmpObjS[292] + tmpFu[69]*tmpObjS[310];
tmpR2[23] = + tmpFu[1]*tmpObjS[5] + tmpFu[5]*tmpObjS[23] + tmpFu[9]*tmpObjS[41] + tmpFu[13]*tmpObjS[59] + tmpFu[17]*tmpObjS[77] + tmpFu[21]*tmpObjS[95] + tmpFu[25]*tmpObjS[113] + tmpFu[29]*tmpObjS[131] + tmpFu[33]*tmpObjS[149] + tmpFu[37]*tmpObjS[167] + tmpFu[41]*tmpObjS[185] + tmpFu[45]*tmpObjS[203] + tmpFu[49]*tmpObjS[221] + tmpFu[53]*tmpObjS[239] + tmpFu[57]*tmpObjS[257] + tmpFu[61]*tmpObjS[275] + tmpFu[65]*tmpObjS[293] + tmpFu[69]*tmpObjS[311];
tmpR2[24] = + tmpFu[1]*tmpObjS[6] + tmpFu[5]*tmpObjS[24] + tmpFu[9]*tmpObjS[42] + tmpFu[13]*tmpObjS[60] + tmpFu[17]*tmpObjS[78] + tmpFu[21]*tmpObjS[96] + tmpFu[25]*tmpObjS[114] + tmpFu[29]*tmpObjS[132] + tmpFu[33]*tmpObjS[150] + tmpFu[37]*tmpObjS[168] + tmpFu[41]*tmpObjS[186] + tmpFu[45]*tmpObjS[204] + tmpFu[49]*tmpObjS[222] + tmpFu[53]*tmpObjS[240] + tmpFu[57]*tmpObjS[258] + tmpFu[61]*tmpObjS[276] + tmpFu[65]*tmpObjS[294] + tmpFu[69]*tmpObjS[312];
tmpR2[25] = + tmpFu[1]*tmpObjS[7] + tmpFu[5]*tmpObjS[25] + tmpFu[9]*tmpObjS[43] + tmpFu[13]*tmpObjS[61] + tmpFu[17]*tmpObjS[79] + tmpFu[21]*tmpObjS[97] + tmpFu[25]*tmpObjS[115] + tmpFu[29]*tmpObjS[133] + tmpFu[33]*tmpObjS[151] + tmpFu[37]*tmpObjS[169] + tmpFu[41]*tmpObjS[187] + tmpFu[45]*tmpObjS[205] + tmpFu[49]*tmpObjS[223] + tmpFu[53]*tmpObjS[241] + tmpFu[57]*tmpObjS[259] + tmpFu[61]*tmpObjS[277] + tmpFu[65]*tmpObjS[295] + tmpFu[69]*tmpObjS[313];
tmpR2[26] = + tmpFu[1]*tmpObjS[8] + tmpFu[5]*tmpObjS[26] + tmpFu[9]*tmpObjS[44] + tmpFu[13]*tmpObjS[62] + tmpFu[17]*tmpObjS[80] + tmpFu[21]*tmpObjS[98] + tmpFu[25]*tmpObjS[116] + tmpFu[29]*tmpObjS[134] + tmpFu[33]*tmpObjS[152] + tmpFu[37]*tmpObjS[170] + tmpFu[41]*tmpObjS[188] + tmpFu[45]*tmpObjS[206] + tmpFu[49]*tmpObjS[224] + tmpFu[53]*tmpObjS[242] + tmpFu[57]*tmpObjS[260] + tmpFu[61]*tmpObjS[278] + tmpFu[65]*tmpObjS[296] + tmpFu[69]*tmpObjS[314];
tmpR2[27] = + tmpFu[1]*tmpObjS[9] + tmpFu[5]*tmpObjS[27] + tmpFu[9]*tmpObjS[45] + tmpFu[13]*tmpObjS[63] + tmpFu[17]*tmpObjS[81] + tmpFu[21]*tmpObjS[99] + tmpFu[25]*tmpObjS[117] + tmpFu[29]*tmpObjS[135] + tmpFu[33]*tmpObjS[153] + tmpFu[37]*tmpObjS[171] + tmpFu[41]*tmpObjS[189] + tmpFu[45]*tmpObjS[207] + tmpFu[49]*tmpObjS[225] + tmpFu[53]*tmpObjS[243] + tmpFu[57]*tmpObjS[261] + tmpFu[61]*tmpObjS[279] + tmpFu[65]*tmpObjS[297] + tmpFu[69]*tmpObjS[315];
tmpR2[28] = + tmpFu[1]*tmpObjS[10] + tmpFu[5]*tmpObjS[28] + tmpFu[9]*tmpObjS[46] + tmpFu[13]*tmpObjS[64] + tmpFu[17]*tmpObjS[82] + tmpFu[21]*tmpObjS[100] + tmpFu[25]*tmpObjS[118] + tmpFu[29]*tmpObjS[136] + tmpFu[33]*tmpObjS[154] + tmpFu[37]*tmpObjS[172] + tmpFu[41]*tmpObjS[190] + tmpFu[45]*tmpObjS[208] + tmpFu[49]*tmpObjS[226] + tmpFu[53]*tmpObjS[244] + tmpFu[57]*tmpObjS[262] + tmpFu[61]*tmpObjS[280] + tmpFu[65]*tmpObjS[298] + tmpFu[69]*tmpObjS[316];
tmpR2[29] = + tmpFu[1]*tmpObjS[11] + tmpFu[5]*tmpObjS[29] + tmpFu[9]*tmpObjS[47] + tmpFu[13]*tmpObjS[65] + tmpFu[17]*tmpObjS[83] + tmpFu[21]*tmpObjS[101] + tmpFu[25]*tmpObjS[119] + tmpFu[29]*tmpObjS[137] + tmpFu[33]*tmpObjS[155] + tmpFu[37]*tmpObjS[173] + tmpFu[41]*tmpObjS[191] + tmpFu[45]*tmpObjS[209] + tmpFu[49]*tmpObjS[227] + tmpFu[53]*tmpObjS[245] + tmpFu[57]*tmpObjS[263] + tmpFu[61]*tmpObjS[281] + tmpFu[65]*tmpObjS[299] + tmpFu[69]*tmpObjS[317];
tmpR2[30] = + tmpFu[1]*tmpObjS[12] + tmpFu[5]*tmpObjS[30] + tmpFu[9]*tmpObjS[48] + tmpFu[13]*tmpObjS[66] + tmpFu[17]*tmpObjS[84] + tmpFu[21]*tmpObjS[102] + tmpFu[25]*tmpObjS[120] + tmpFu[29]*tmpObjS[138] + tmpFu[33]*tmpObjS[156] + tmpFu[37]*tmpObjS[174] + tmpFu[41]*tmpObjS[192] + tmpFu[45]*tmpObjS[210] + tmpFu[49]*tmpObjS[228] + tmpFu[53]*tmpObjS[246] + tmpFu[57]*tmpObjS[264] + tmpFu[61]*tmpObjS[282] + tmpFu[65]*tmpObjS[300] + tmpFu[69]*tmpObjS[318];
tmpR2[31] = + tmpFu[1]*tmpObjS[13] + tmpFu[5]*tmpObjS[31] + tmpFu[9]*tmpObjS[49] + tmpFu[13]*tmpObjS[67] + tmpFu[17]*tmpObjS[85] + tmpFu[21]*tmpObjS[103] + tmpFu[25]*tmpObjS[121] + tmpFu[29]*tmpObjS[139] + tmpFu[33]*tmpObjS[157] + tmpFu[37]*tmpObjS[175] + tmpFu[41]*tmpObjS[193] + tmpFu[45]*tmpObjS[211] + tmpFu[49]*tmpObjS[229] + tmpFu[53]*tmpObjS[247] + tmpFu[57]*tmpObjS[265] + tmpFu[61]*tmpObjS[283] + tmpFu[65]*tmpObjS[301] + tmpFu[69]*tmpObjS[319];
tmpR2[32] = + tmpFu[1]*tmpObjS[14] + tmpFu[5]*tmpObjS[32] + tmpFu[9]*tmpObjS[50] + tmpFu[13]*tmpObjS[68] + tmpFu[17]*tmpObjS[86] + tmpFu[21]*tmpObjS[104] + tmpFu[25]*tmpObjS[122] + tmpFu[29]*tmpObjS[140] + tmpFu[33]*tmpObjS[158] + tmpFu[37]*tmpObjS[176] + tmpFu[41]*tmpObjS[194] + tmpFu[45]*tmpObjS[212] + tmpFu[49]*tmpObjS[230] + tmpFu[53]*tmpObjS[248] + tmpFu[57]*tmpObjS[266] + tmpFu[61]*tmpObjS[284] + tmpFu[65]*tmpObjS[302] + tmpFu[69]*tmpObjS[320];
tmpR2[33] = + tmpFu[1]*tmpObjS[15] + tmpFu[5]*tmpObjS[33] + tmpFu[9]*tmpObjS[51] + tmpFu[13]*tmpObjS[69] + tmpFu[17]*tmpObjS[87] + tmpFu[21]*tmpObjS[105] + tmpFu[25]*tmpObjS[123] + tmpFu[29]*tmpObjS[141] + tmpFu[33]*tmpObjS[159] + tmpFu[37]*tmpObjS[177] + tmpFu[41]*tmpObjS[195] + tmpFu[45]*tmpObjS[213] + tmpFu[49]*tmpObjS[231] + tmpFu[53]*tmpObjS[249] + tmpFu[57]*tmpObjS[267] + tmpFu[61]*tmpObjS[285] + tmpFu[65]*tmpObjS[303] + tmpFu[69]*tmpObjS[321];
tmpR2[34] = + tmpFu[1]*tmpObjS[16] + tmpFu[5]*tmpObjS[34] + tmpFu[9]*tmpObjS[52] + tmpFu[13]*tmpObjS[70] + tmpFu[17]*tmpObjS[88] + tmpFu[21]*tmpObjS[106] + tmpFu[25]*tmpObjS[124] + tmpFu[29]*tmpObjS[142] + tmpFu[33]*tmpObjS[160] + tmpFu[37]*tmpObjS[178] + tmpFu[41]*tmpObjS[196] + tmpFu[45]*tmpObjS[214] + tmpFu[49]*tmpObjS[232] + tmpFu[53]*tmpObjS[250] + tmpFu[57]*tmpObjS[268] + tmpFu[61]*tmpObjS[286] + tmpFu[65]*tmpObjS[304] + tmpFu[69]*tmpObjS[322];
tmpR2[35] = + tmpFu[1]*tmpObjS[17] + tmpFu[5]*tmpObjS[35] + tmpFu[9]*tmpObjS[53] + tmpFu[13]*tmpObjS[71] + tmpFu[17]*tmpObjS[89] + tmpFu[21]*tmpObjS[107] + tmpFu[25]*tmpObjS[125] + tmpFu[29]*tmpObjS[143] + tmpFu[33]*tmpObjS[161] + tmpFu[37]*tmpObjS[179] + tmpFu[41]*tmpObjS[197] + tmpFu[45]*tmpObjS[215] + tmpFu[49]*tmpObjS[233] + tmpFu[53]*tmpObjS[251] + tmpFu[57]*tmpObjS[269] + tmpFu[61]*tmpObjS[287] + tmpFu[65]*tmpObjS[305] + tmpFu[69]*tmpObjS[323];
tmpR2[36] = + tmpFu[2]*tmpObjS[0] + tmpFu[6]*tmpObjS[18] + tmpFu[10]*tmpObjS[36] + tmpFu[14]*tmpObjS[54] + tmpFu[18]*tmpObjS[72] + tmpFu[22]*tmpObjS[90] + tmpFu[26]*tmpObjS[108] + tmpFu[30]*tmpObjS[126] + tmpFu[34]*tmpObjS[144] + tmpFu[38]*tmpObjS[162] + tmpFu[42]*tmpObjS[180] + tmpFu[46]*tmpObjS[198] + tmpFu[50]*tmpObjS[216] + tmpFu[54]*tmpObjS[234] + tmpFu[58]*tmpObjS[252] + tmpFu[62]*tmpObjS[270] + tmpFu[66]*tmpObjS[288] + tmpFu[70]*tmpObjS[306];
tmpR2[37] = + tmpFu[2]*tmpObjS[1] + tmpFu[6]*tmpObjS[19] + tmpFu[10]*tmpObjS[37] + tmpFu[14]*tmpObjS[55] + tmpFu[18]*tmpObjS[73] + tmpFu[22]*tmpObjS[91] + tmpFu[26]*tmpObjS[109] + tmpFu[30]*tmpObjS[127] + tmpFu[34]*tmpObjS[145] + tmpFu[38]*tmpObjS[163] + tmpFu[42]*tmpObjS[181] + tmpFu[46]*tmpObjS[199] + tmpFu[50]*tmpObjS[217] + tmpFu[54]*tmpObjS[235] + tmpFu[58]*tmpObjS[253] + tmpFu[62]*tmpObjS[271] + tmpFu[66]*tmpObjS[289] + tmpFu[70]*tmpObjS[307];
tmpR2[38] = + tmpFu[2]*tmpObjS[2] + tmpFu[6]*tmpObjS[20] + tmpFu[10]*tmpObjS[38] + tmpFu[14]*tmpObjS[56] + tmpFu[18]*tmpObjS[74] + tmpFu[22]*tmpObjS[92] + tmpFu[26]*tmpObjS[110] + tmpFu[30]*tmpObjS[128] + tmpFu[34]*tmpObjS[146] + tmpFu[38]*tmpObjS[164] + tmpFu[42]*tmpObjS[182] + tmpFu[46]*tmpObjS[200] + tmpFu[50]*tmpObjS[218] + tmpFu[54]*tmpObjS[236] + tmpFu[58]*tmpObjS[254] + tmpFu[62]*tmpObjS[272] + tmpFu[66]*tmpObjS[290] + tmpFu[70]*tmpObjS[308];
tmpR2[39] = + tmpFu[2]*tmpObjS[3] + tmpFu[6]*tmpObjS[21] + tmpFu[10]*tmpObjS[39] + tmpFu[14]*tmpObjS[57] + tmpFu[18]*tmpObjS[75] + tmpFu[22]*tmpObjS[93] + tmpFu[26]*tmpObjS[111] + tmpFu[30]*tmpObjS[129] + tmpFu[34]*tmpObjS[147] + tmpFu[38]*tmpObjS[165] + tmpFu[42]*tmpObjS[183] + tmpFu[46]*tmpObjS[201] + tmpFu[50]*tmpObjS[219] + tmpFu[54]*tmpObjS[237] + tmpFu[58]*tmpObjS[255] + tmpFu[62]*tmpObjS[273] + tmpFu[66]*tmpObjS[291] + tmpFu[70]*tmpObjS[309];
tmpR2[40] = + tmpFu[2]*tmpObjS[4] + tmpFu[6]*tmpObjS[22] + tmpFu[10]*tmpObjS[40] + tmpFu[14]*tmpObjS[58] + tmpFu[18]*tmpObjS[76] + tmpFu[22]*tmpObjS[94] + tmpFu[26]*tmpObjS[112] + tmpFu[30]*tmpObjS[130] + tmpFu[34]*tmpObjS[148] + tmpFu[38]*tmpObjS[166] + tmpFu[42]*tmpObjS[184] + tmpFu[46]*tmpObjS[202] + tmpFu[50]*tmpObjS[220] + tmpFu[54]*tmpObjS[238] + tmpFu[58]*tmpObjS[256] + tmpFu[62]*tmpObjS[274] + tmpFu[66]*tmpObjS[292] + tmpFu[70]*tmpObjS[310];
tmpR2[41] = + tmpFu[2]*tmpObjS[5] + tmpFu[6]*tmpObjS[23] + tmpFu[10]*tmpObjS[41] + tmpFu[14]*tmpObjS[59] + tmpFu[18]*tmpObjS[77] + tmpFu[22]*tmpObjS[95] + tmpFu[26]*tmpObjS[113] + tmpFu[30]*tmpObjS[131] + tmpFu[34]*tmpObjS[149] + tmpFu[38]*tmpObjS[167] + tmpFu[42]*tmpObjS[185] + tmpFu[46]*tmpObjS[203] + tmpFu[50]*tmpObjS[221] + tmpFu[54]*tmpObjS[239] + tmpFu[58]*tmpObjS[257] + tmpFu[62]*tmpObjS[275] + tmpFu[66]*tmpObjS[293] + tmpFu[70]*tmpObjS[311];
tmpR2[42] = + tmpFu[2]*tmpObjS[6] + tmpFu[6]*tmpObjS[24] + tmpFu[10]*tmpObjS[42] + tmpFu[14]*tmpObjS[60] + tmpFu[18]*tmpObjS[78] + tmpFu[22]*tmpObjS[96] + tmpFu[26]*tmpObjS[114] + tmpFu[30]*tmpObjS[132] + tmpFu[34]*tmpObjS[150] + tmpFu[38]*tmpObjS[168] + tmpFu[42]*tmpObjS[186] + tmpFu[46]*tmpObjS[204] + tmpFu[50]*tmpObjS[222] + tmpFu[54]*tmpObjS[240] + tmpFu[58]*tmpObjS[258] + tmpFu[62]*tmpObjS[276] + tmpFu[66]*tmpObjS[294] + tmpFu[70]*tmpObjS[312];
tmpR2[43] = + tmpFu[2]*tmpObjS[7] + tmpFu[6]*tmpObjS[25] + tmpFu[10]*tmpObjS[43] + tmpFu[14]*tmpObjS[61] + tmpFu[18]*tmpObjS[79] + tmpFu[22]*tmpObjS[97] + tmpFu[26]*tmpObjS[115] + tmpFu[30]*tmpObjS[133] + tmpFu[34]*tmpObjS[151] + tmpFu[38]*tmpObjS[169] + tmpFu[42]*tmpObjS[187] + tmpFu[46]*tmpObjS[205] + tmpFu[50]*tmpObjS[223] + tmpFu[54]*tmpObjS[241] + tmpFu[58]*tmpObjS[259] + tmpFu[62]*tmpObjS[277] + tmpFu[66]*tmpObjS[295] + tmpFu[70]*tmpObjS[313];
tmpR2[44] = + tmpFu[2]*tmpObjS[8] + tmpFu[6]*tmpObjS[26] + tmpFu[10]*tmpObjS[44] + tmpFu[14]*tmpObjS[62] + tmpFu[18]*tmpObjS[80] + tmpFu[22]*tmpObjS[98] + tmpFu[26]*tmpObjS[116] + tmpFu[30]*tmpObjS[134] + tmpFu[34]*tmpObjS[152] + tmpFu[38]*tmpObjS[170] + tmpFu[42]*tmpObjS[188] + tmpFu[46]*tmpObjS[206] + tmpFu[50]*tmpObjS[224] + tmpFu[54]*tmpObjS[242] + tmpFu[58]*tmpObjS[260] + tmpFu[62]*tmpObjS[278] + tmpFu[66]*tmpObjS[296] + tmpFu[70]*tmpObjS[314];
tmpR2[45] = + tmpFu[2]*tmpObjS[9] + tmpFu[6]*tmpObjS[27] + tmpFu[10]*tmpObjS[45] + tmpFu[14]*tmpObjS[63] + tmpFu[18]*tmpObjS[81] + tmpFu[22]*tmpObjS[99] + tmpFu[26]*tmpObjS[117] + tmpFu[30]*tmpObjS[135] + tmpFu[34]*tmpObjS[153] + tmpFu[38]*tmpObjS[171] + tmpFu[42]*tmpObjS[189] + tmpFu[46]*tmpObjS[207] + tmpFu[50]*tmpObjS[225] + tmpFu[54]*tmpObjS[243] + tmpFu[58]*tmpObjS[261] + tmpFu[62]*tmpObjS[279] + tmpFu[66]*tmpObjS[297] + tmpFu[70]*tmpObjS[315];
tmpR2[46] = + tmpFu[2]*tmpObjS[10] + tmpFu[6]*tmpObjS[28] + tmpFu[10]*tmpObjS[46] + tmpFu[14]*tmpObjS[64] + tmpFu[18]*tmpObjS[82] + tmpFu[22]*tmpObjS[100] + tmpFu[26]*tmpObjS[118] + tmpFu[30]*tmpObjS[136] + tmpFu[34]*tmpObjS[154] + tmpFu[38]*tmpObjS[172] + tmpFu[42]*tmpObjS[190] + tmpFu[46]*tmpObjS[208] + tmpFu[50]*tmpObjS[226] + tmpFu[54]*tmpObjS[244] + tmpFu[58]*tmpObjS[262] + tmpFu[62]*tmpObjS[280] + tmpFu[66]*tmpObjS[298] + tmpFu[70]*tmpObjS[316];
tmpR2[47] = + tmpFu[2]*tmpObjS[11] + tmpFu[6]*tmpObjS[29] + tmpFu[10]*tmpObjS[47] + tmpFu[14]*tmpObjS[65] + tmpFu[18]*tmpObjS[83] + tmpFu[22]*tmpObjS[101] + tmpFu[26]*tmpObjS[119] + tmpFu[30]*tmpObjS[137] + tmpFu[34]*tmpObjS[155] + tmpFu[38]*tmpObjS[173] + tmpFu[42]*tmpObjS[191] + tmpFu[46]*tmpObjS[209] + tmpFu[50]*tmpObjS[227] + tmpFu[54]*tmpObjS[245] + tmpFu[58]*tmpObjS[263] + tmpFu[62]*tmpObjS[281] + tmpFu[66]*tmpObjS[299] + tmpFu[70]*tmpObjS[317];
tmpR2[48] = + tmpFu[2]*tmpObjS[12] + tmpFu[6]*tmpObjS[30] + tmpFu[10]*tmpObjS[48] + tmpFu[14]*tmpObjS[66] + tmpFu[18]*tmpObjS[84] + tmpFu[22]*tmpObjS[102] + tmpFu[26]*tmpObjS[120] + tmpFu[30]*tmpObjS[138] + tmpFu[34]*tmpObjS[156] + tmpFu[38]*tmpObjS[174] + tmpFu[42]*tmpObjS[192] + tmpFu[46]*tmpObjS[210] + tmpFu[50]*tmpObjS[228] + tmpFu[54]*tmpObjS[246] + tmpFu[58]*tmpObjS[264] + tmpFu[62]*tmpObjS[282] + tmpFu[66]*tmpObjS[300] + tmpFu[70]*tmpObjS[318];
tmpR2[49] = + tmpFu[2]*tmpObjS[13] + tmpFu[6]*tmpObjS[31] + tmpFu[10]*tmpObjS[49] + tmpFu[14]*tmpObjS[67] + tmpFu[18]*tmpObjS[85] + tmpFu[22]*tmpObjS[103] + tmpFu[26]*tmpObjS[121] + tmpFu[30]*tmpObjS[139] + tmpFu[34]*tmpObjS[157] + tmpFu[38]*tmpObjS[175] + tmpFu[42]*tmpObjS[193] + tmpFu[46]*tmpObjS[211] + tmpFu[50]*tmpObjS[229] + tmpFu[54]*tmpObjS[247] + tmpFu[58]*tmpObjS[265] + tmpFu[62]*tmpObjS[283] + tmpFu[66]*tmpObjS[301] + tmpFu[70]*tmpObjS[319];
tmpR2[50] = + tmpFu[2]*tmpObjS[14] + tmpFu[6]*tmpObjS[32] + tmpFu[10]*tmpObjS[50] + tmpFu[14]*tmpObjS[68] + tmpFu[18]*tmpObjS[86] + tmpFu[22]*tmpObjS[104] + tmpFu[26]*tmpObjS[122] + tmpFu[30]*tmpObjS[140] + tmpFu[34]*tmpObjS[158] + tmpFu[38]*tmpObjS[176] + tmpFu[42]*tmpObjS[194] + tmpFu[46]*tmpObjS[212] + tmpFu[50]*tmpObjS[230] + tmpFu[54]*tmpObjS[248] + tmpFu[58]*tmpObjS[266] + tmpFu[62]*tmpObjS[284] + tmpFu[66]*tmpObjS[302] + tmpFu[70]*tmpObjS[320];
tmpR2[51] = + tmpFu[2]*tmpObjS[15] + tmpFu[6]*tmpObjS[33] + tmpFu[10]*tmpObjS[51] + tmpFu[14]*tmpObjS[69] + tmpFu[18]*tmpObjS[87] + tmpFu[22]*tmpObjS[105] + tmpFu[26]*tmpObjS[123] + tmpFu[30]*tmpObjS[141] + tmpFu[34]*tmpObjS[159] + tmpFu[38]*tmpObjS[177] + tmpFu[42]*tmpObjS[195] + tmpFu[46]*tmpObjS[213] + tmpFu[50]*tmpObjS[231] + tmpFu[54]*tmpObjS[249] + tmpFu[58]*tmpObjS[267] + tmpFu[62]*tmpObjS[285] + tmpFu[66]*tmpObjS[303] + tmpFu[70]*tmpObjS[321];
tmpR2[52] = + tmpFu[2]*tmpObjS[16] + tmpFu[6]*tmpObjS[34] + tmpFu[10]*tmpObjS[52] + tmpFu[14]*tmpObjS[70] + tmpFu[18]*tmpObjS[88] + tmpFu[22]*tmpObjS[106] + tmpFu[26]*tmpObjS[124] + tmpFu[30]*tmpObjS[142] + tmpFu[34]*tmpObjS[160] + tmpFu[38]*tmpObjS[178] + tmpFu[42]*tmpObjS[196] + tmpFu[46]*tmpObjS[214] + tmpFu[50]*tmpObjS[232] + tmpFu[54]*tmpObjS[250] + tmpFu[58]*tmpObjS[268] + tmpFu[62]*tmpObjS[286] + tmpFu[66]*tmpObjS[304] + tmpFu[70]*tmpObjS[322];
tmpR2[53] = + tmpFu[2]*tmpObjS[17] + tmpFu[6]*tmpObjS[35] + tmpFu[10]*tmpObjS[53] + tmpFu[14]*tmpObjS[71] + tmpFu[18]*tmpObjS[89] + tmpFu[22]*tmpObjS[107] + tmpFu[26]*tmpObjS[125] + tmpFu[30]*tmpObjS[143] + tmpFu[34]*tmpObjS[161] + tmpFu[38]*tmpObjS[179] + tmpFu[42]*tmpObjS[197] + tmpFu[46]*tmpObjS[215] + tmpFu[50]*tmpObjS[233] + tmpFu[54]*tmpObjS[251] + tmpFu[58]*tmpObjS[269] + tmpFu[62]*tmpObjS[287] + tmpFu[66]*tmpObjS[305] + tmpFu[70]*tmpObjS[323];
tmpR2[54] = + tmpFu[3]*tmpObjS[0] + tmpFu[7]*tmpObjS[18] + tmpFu[11]*tmpObjS[36] + tmpFu[15]*tmpObjS[54] + tmpFu[19]*tmpObjS[72] + tmpFu[23]*tmpObjS[90] + tmpFu[27]*tmpObjS[108] + tmpFu[31]*tmpObjS[126] + tmpFu[35]*tmpObjS[144] + tmpFu[39]*tmpObjS[162] + tmpFu[43]*tmpObjS[180] + tmpFu[47]*tmpObjS[198] + tmpFu[51]*tmpObjS[216] + tmpFu[55]*tmpObjS[234] + tmpFu[59]*tmpObjS[252] + tmpFu[63]*tmpObjS[270] + tmpFu[67]*tmpObjS[288] + tmpFu[71]*tmpObjS[306];
tmpR2[55] = + tmpFu[3]*tmpObjS[1] + tmpFu[7]*tmpObjS[19] + tmpFu[11]*tmpObjS[37] + tmpFu[15]*tmpObjS[55] + tmpFu[19]*tmpObjS[73] + tmpFu[23]*tmpObjS[91] + tmpFu[27]*tmpObjS[109] + tmpFu[31]*tmpObjS[127] + tmpFu[35]*tmpObjS[145] + tmpFu[39]*tmpObjS[163] + tmpFu[43]*tmpObjS[181] + tmpFu[47]*tmpObjS[199] + tmpFu[51]*tmpObjS[217] + tmpFu[55]*tmpObjS[235] + tmpFu[59]*tmpObjS[253] + tmpFu[63]*tmpObjS[271] + tmpFu[67]*tmpObjS[289] + tmpFu[71]*tmpObjS[307];
tmpR2[56] = + tmpFu[3]*tmpObjS[2] + tmpFu[7]*tmpObjS[20] + tmpFu[11]*tmpObjS[38] + tmpFu[15]*tmpObjS[56] + tmpFu[19]*tmpObjS[74] + tmpFu[23]*tmpObjS[92] + tmpFu[27]*tmpObjS[110] + tmpFu[31]*tmpObjS[128] + tmpFu[35]*tmpObjS[146] + tmpFu[39]*tmpObjS[164] + tmpFu[43]*tmpObjS[182] + tmpFu[47]*tmpObjS[200] + tmpFu[51]*tmpObjS[218] + tmpFu[55]*tmpObjS[236] + tmpFu[59]*tmpObjS[254] + tmpFu[63]*tmpObjS[272] + tmpFu[67]*tmpObjS[290] + tmpFu[71]*tmpObjS[308];
tmpR2[57] = + tmpFu[3]*tmpObjS[3] + tmpFu[7]*tmpObjS[21] + tmpFu[11]*tmpObjS[39] + tmpFu[15]*tmpObjS[57] + tmpFu[19]*tmpObjS[75] + tmpFu[23]*tmpObjS[93] + tmpFu[27]*tmpObjS[111] + tmpFu[31]*tmpObjS[129] + tmpFu[35]*tmpObjS[147] + tmpFu[39]*tmpObjS[165] + tmpFu[43]*tmpObjS[183] + tmpFu[47]*tmpObjS[201] + tmpFu[51]*tmpObjS[219] + tmpFu[55]*tmpObjS[237] + tmpFu[59]*tmpObjS[255] + tmpFu[63]*tmpObjS[273] + tmpFu[67]*tmpObjS[291] + tmpFu[71]*tmpObjS[309];
tmpR2[58] = + tmpFu[3]*tmpObjS[4] + tmpFu[7]*tmpObjS[22] + tmpFu[11]*tmpObjS[40] + tmpFu[15]*tmpObjS[58] + tmpFu[19]*tmpObjS[76] + tmpFu[23]*tmpObjS[94] + tmpFu[27]*tmpObjS[112] + tmpFu[31]*tmpObjS[130] + tmpFu[35]*tmpObjS[148] + tmpFu[39]*tmpObjS[166] + tmpFu[43]*tmpObjS[184] + tmpFu[47]*tmpObjS[202] + tmpFu[51]*tmpObjS[220] + tmpFu[55]*tmpObjS[238] + tmpFu[59]*tmpObjS[256] + tmpFu[63]*tmpObjS[274] + tmpFu[67]*tmpObjS[292] + tmpFu[71]*tmpObjS[310];
tmpR2[59] = + tmpFu[3]*tmpObjS[5] + tmpFu[7]*tmpObjS[23] + tmpFu[11]*tmpObjS[41] + tmpFu[15]*tmpObjS[59] + tmpFu[19]*tmpObjS[77] + tmpFu[23]*tmpObjS[95] + tmpFu[27]*tmpObjS[113] + tmpFu[31]*tmpObjS[131] + tmpFu[35]*tmpObjS[149] + tmpFu[39]*tmpObjS[167] + tmpFu[43]*tmpObjS[185] + tmpFu[47]*tmpObjS[203] + tmpFu[51]*tmpObjS[221] + tmpFu[55]*tmpObjS[239] + tmpFu[59]*tmpObjS[257] + tmpFu[63]*tmpObjS[275] + tmpFu[67]*tmpObjS[293] + tmpFu[71]*tmpObjS[311];
tmpR2[60] = + tmpFu[3]*tmpObjS[6] + tmpFu[7]*tmpObjS[24] + tmpFu[11]*tmpObjS[42] + tmpFu[15]*tmpObjS[60] + tmpFu[19]*tmpObjS[78] + tmpFu[23]*tmpObjS[96] + tmpFu[27]*tmpObjS[114] + tmpFu[31]*tmpObjS[132] + tmpFu[35]*tmpObjS[150] + tmpFu[39]*tmpObjS[168] + tmpFu[43]*tmpObjS[186] + tmpFu[47]*tmpObjS[204] + tmpFu[51]*tmpObjS[222] + tmpFu[55]*tmpObjS[240] + tmpFu[59]*tmpObjS[258] + tmpFu[63]*tmpObjS[276] + tmpFu[67]*tmpObjS[294] + tmpFu[71]*tmpObjS[312];
tmpR2[61] = + tmpFu[3]*tmpObjS[7] + tmpFu[7]*tmpObjS[25] + tmpFu[11]*tmpObjS[43] + tmpFu[15]*tmpObjS[61] + tmpFu[19]*tmpObjS[79] + tmpFu[23]*tmpObjS[97] + tmpFu[27]*tmpObjS[115] + tmpFu[31]*tmpObjS[133] + tmpFu[35]*tmpObjS[151] + tmpFu[39]*tmpObjS[169] + tmpFu[43]*tmpObjS[187] + tmpFu[47]*tmpObjS[205] + tmpFu[51]*tmpObjS[223] + tmpFu[55]*tmpObjS[241] + tmpFu[59]*tmpObjS[259] + tmpFu[63]*tmpObjS[277] + tmpFu[67]*tmpObjS[295] + tmpFu[71]*tmpObjS[313];
tmpR2[62] = + tmpFu[3]*tmpObjS[8] + tmpFu[7]*tmpObjS[26] + tmpFu[11]*tmpObjS[44] + tmpFu[15]*tmpObjS[62] + tmpFu[19]*tmpObjS[80] + tmpFu[23]*tmpObjS[98] + tmpFu[27]*tmpObjS[116] + tmpFu[31]*tmpObjS[134] + tmpFu[35]*tmpObjS[152] + tmpFu[39]*tmpObjS[170] + tmpFu[43]*tmpObjS[188] + tmpFu[47]*tmpObjS[206] + tmpFu[51]*tmpObjS[224] + tmpFu[55]*tmpObjS[242] + tmpFu[59]*tmpObjS[260] + tmpFu[63]*tmpObjS[278] + tmpFu[67]*tmpObjS[296] + tmpFu[71]*tmpObjS[314];
tmpR2[63] = + tmpFu[3]*tmpObjS[9] + tmpFu[7]*tmpObjS[27] + tmpFu[11]*tmpObjS[45] + tmpFu[15]*tmpObjS[63] + tmpFu[19]*tmpObjS[81] + tmpFu[23]*tmpObjS[99] + tmpFu[27]*tmpObjS[117] + tmpFu[31]*tmpObjS[135] + tmpFu[35]*tmpObjS[153] + tmpFu[39]*tmpObjS[171] + tmpFu[43]*tmpObjS[189] + tmpFu[47]*tmpObjS[207] + tmpFu[51]*tmpObjS[225] + tmpFu[55]*tmpObjS[243] + tmpFu[59]*tmpObjS[261] + tmpFu[63]*tmpObjS[279] + tmpFu[67]*tmpObjS[297] + tmpFu[71]*tmpObjS[315];
tmpR2[64] = + tmpFu[3]*tmpObjS[10] + tmpFu[7]*tmpObjS[28] + tmpFu[11]*tmpObjS[46] + tmpFu[15]*tmpObjS[64] + tmpFu[19]*tmpObjS[82] + tmpFu[23]*tmpObjS[100] + tmpFu[27]*tmpObjS[118] + tmpFu[31]*tmpObjS[136] + tmpFu[35]*tmpObjS[154] + tmpFu[39]*tmpObjS[172] + tmpFu[43]*tmpObjS[190] + tmpFu[47]*tmpObjS[208] + tmpFu[51]*tmpObjS[226] + tmpFu[55]*tmpObjS[244] + tmpFu[59]*tmpObjS[262] + tmpFu[63]*tmpObjS[280] + tmpFu[67]*tmpObjS[298] + tmpFu[71]*tmpObjS[316];
tmpR2[65] = + tmpFu[3]*tmpObjS[11] + tmpFu[7]*tmpObjS[29] + tmpFu[11]*tmpObjS[47] + tmpFu[15]*tmpObjS[65] + tmpFu[19]*tmpObjS[83] + tmpFu[23]*tmpObjS[101] + tmpFu[27]*tmpObjS[119] + tmpFu[31]*tmpObjS[137] + tmpFu[35]*tmpObjS[155] + tmpFu[39]*tmpObjS[173] + tmpFu[43]*tmpObjS[191] + tmpFu[47]*tmpObjS[209] + tmpFu[51]*tmpObjS[227] + tmpFu[55]*tmpObjS[245] + tmpFu[59]*tmpObjS[263] + tmpFu[63]*tmpObjS[281] + tmpFu[67]*tmpObjS[299] + tmpFu[71]*tmpObjS[317];
tmpR2[66] = + tmpFu[3]*tmpObjS[12] + tmpFu[7]*tmpObjS[30] + tmpFu[11]*tmpObjS[48] + tmpFu[15]*tmpObjS[66] + tmpFu[19]*tmpObjS[84] + tmpFu[23]*tmpObjS[102] + tmpFu[27]*tmpObjS[120] + tmpFu[31]*tmpObjS[138] + tmpFu[35]*tmpObjS[156] + tmpFu[39]*tmpObjS[174] + tmpFu[43]*tmpObjS[192] + tmpFu[47]*tmpObjS[210] + tmpFu[51]*tmpObjS[228] + tmpFu[55]*tmpObjS[246] + tmpFu[59]*tmpObjS[264] + tmpFu[63]*tmpObjS[282] + tmpFu[67]*tmpObjS[300] + tmpFu[71]*tmpObjS[318];
tmpR2[67] = + tmpFu[3]*tmpObjS[13] + tmpFu[7]*tmpObjS[31] + tmpFu[11]*tmpObjS[49] + tmpFu[15]*tmpObjS[67] + tmpFu[19]*tmpObjS[85] + tmpFu[23]*tmpObjS[103] + tmpFu[27]*tmpObjS[121] + tmpFu[31]*tmpObjS[139] + tmpFu[35]*tmpObjS[157] + tmpFu[39]*tmpObjS[175] + tmpFu[43]*tmpObjS[193] + tmpFu[47]*tmpObjS[211] + tmpFu[51]*tmpObjS[229] + tmpFu[55]*tmpObjS[247] + tmpFu[59]*tmpObjS[265] + tmpFu[63]*tmpObjS[283] + tmpFu[67]*tmpObjS[301] + tmpFu[71]*tmpObjS[319];
tmpR2[68] = + tmpFu[3]*tmpObjS[14] + tmpFu[7]*tmpObjS[32] + tmpFu[11]*tmpObjS[50] + tmpFu[15]*tmpObjS[68] + tmpFu[19]*tmpObjS[86] + tmpFu[23]*tmpObjS[104] + tmpFu[27]*tmpObjS[122] + tmpFu[31]*tmpObjS[140] + tmpFu[35]*tmpObjS[158] + tmpFu[39]*tmpObjS[176] + tmpFu[43]*tmpObjS[194] + tmpFu[47]*tmpObjS[212] + tmpFu[51]*tmpObjS[230] + tmpFu[55]*tmpObjS[248] + tmpFu[59]*tmpObjS[266] + tmpFu[63]*tmpObjS[284] + tmpFu[67]*tmpObjS[302] + tmpFu[71]*tmpObjS[320];
tmpR2[69] = + tmpFu[3]*tmpObjS[15] + tmpFu[7]*tmpObjS[33] + tmpFu[11]*tmpObjS[51] + tmpFu[15]*tmpObjS[69] + tmpFu[19]*tmpObjS[87] + tmpFu[23]*tmpObjS[105] + tmpFu[27]*tmpObjS[123] + tmpFu[31]*tmpObjS[141] + tmpFu[35]*tmpObjS[159] + tmpFu[39]*tmpObjS[177] + tmpFu[43]*tmpObjS[195] + tmpFu[47]*tmpObjS[213] + tmpFu[51]*tmpObjS[231] + tmpFu[55]*tmpObjS[249] + tmpFu[59]*tmpObjS[267] + tmpFu[63]*tmpObjS[285] + tmpFu[67]*tmpObjS[303] + tmpFu[71]*tmpObjS[321];
tmpR2[70] = + tmpFu[3]*tmpObjS[16] + tmpFu[7]*tmpObjS[34] + tmpFu[11]*tmpObjS[52] + tmpFu[15]*tmpObjS[70] + tmpFu[19]*tmpObjS[88] + tmpFu[23]*tmpObjS[106] + tmpFu[27]*tmpObjS[124] + tmpFu[31]*tmpObjS[142] + tmpFu[35]*tmpObjS[160] + tmpFu[39]*tmpObjS[178] + tmpFu[43]*tmpObjS[196] + tmpFu[47]*tmpObjS[214] + tmpFu[51]*tmpObjS[232] + tmpFu[55]*tmpObjS[250] + tmpFu[59]*tmpObjS[268] + tmpFu[63]*tmpObjS[286] + tmpFu[67]*tmpObjS[304] + tmpFu[71]*tmpObjS[322];
tmpR2[71] = + tmpFu[3]*tmpObjS[17] + tmpFu[7]*tmpObjS[35] + tmpFu[11]*tmpObjS[53] + tmpFu[15]*tmpObjS[71] + tmpFu[19]*tmpObjS[89] + tmpFu[23]*tmpObjS[107] + tmpFu[27]*tmpObjS[125] + tmpFu[31]*tmpObjS[143] + tmpFu[35]*tmpObjS[161] + tmpFu[39]*tmpObjS[179] + tmpFu[43]*tmpObjS[197] + tmpFu[47]*tmpObjS[215] + tmpFu[51]*tmpObjS[233] + tmpFu[55]*tmpObjS[251] + tmpFu[59]*tmpObjS[269] + tmpFu[63]*tmpObjS[287] + tmpFu[67]*tmpObjS[305] + tmpFu[71]*tmpObjS[323];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[12] + tmpR2[4]*tmpFu[16] + tmpR2[5]*tmpFu[20] + tmpR2[6]*tmpFu[24] + tmpR2[7]*tmpFu[28] + tmpR2[8]*tmpFu[32] + tmpR2[9]*tmpFu[36] + tmpR2[10]*tmpFu[40] + tmpR2[11]*tmpFu[44] + tmpR2[12]*tmpFu[48] + tmpR2[13]*tmpFu[52] + tmpR2[14]*tmpFu[56] + tmpR2[15]*tmpFu[60] + tmpR2[16]*tmpFu[64] + tmpR2[17]*tmpFu[68];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[9] + tmpR2[3]*tmpFu[13] + tmpR2[4]*tmpFu[17] + tmpR2[5]*tmpFu[21] + tmpR2[6]*tmpFu[25] + tmpR2[7]*tmpFu[29] + tmpR2[8]*tmpFu[33] + tmpR2[9]*tmpFu[37] + tmpR2[10]*tmpFu[41] + tmpR2[11]*tmpFu[45] + tmpR2[12]*tmpFu[49] + tmpR2[13]*tmpFu[53] + tmpR2[14]*tmpFu[57] + tmpR2[15]*tmpFu[61] + tmpR2[16]*tmpFu[65] + tmpR2[17]*tmpFu[69];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[6] + tmpR2[2]*tmpFu[10] + tmpR2[3]*tmpFu[14] + tmpR2[4]*tmpFu[18] + tmpR2[5]*tmpFu[22] + tmpR2[6]*tmpFu[26] + tmpR2[7]*tmpFu[30] + tmpR2[8]*tmpFu[34] + tmpR2[9]*tmpFu[38] + tmpR2[10]*tmpFu[42] + tmpR2[11]*tmpFu[46] + tmpR2[12]*tmpFu[50] + tmpR2[13]*tmpFu[54] + tmpR2[14]*tmpFu[58] + tmpR2[15]*tmpFu[62] + tmpR2[16]*tmpFu[66] + tmpR2[17]*tmpFu[70];
tmpR1[3] = + tmpR2[0]*tmpFu[3] + tmpR2[1]*tmpFu[7] + tmpR2[2]*tmpFu[11] + tmpR2[3]*tmpFu[15] + tmpR2[4]*tmpFu[19] + tmpR2[5]*tmpFu[23] + tmpR2[6]*tmpFu[27] + tmpR2[7]*tmpFu[31] + tmpR2[8]*tmpFu[35] + tmpR2[9]*tmpFu[39] + tmpR2[10]*tmpFu[43] + tmpR2[11]*tmpFu[47] + tmpR2[12]*tmpFu[51] + tmpR2[13]*tmpFu[55] + tmpR2[14]*tmpFu[59] + tmpR2[15]*tmpFu[63] + tmpR2[16]*tmpFu[67] + tmpR2[17]*tmpFu[71];
tmpR1[4] = + tmpR2[18]*tmpFu[0] + tmpR2[19]*tmpFu[4] + tmpR2[20]*tmpFu[8] + tmpR2[21]*tmpFu[12] + tmpR2[22]*tmpFu[16] + tmpR2[23]*tmpFu[20] + tmpR2[24]*tmpFu[24] + tmpR2[25]*tmpFu[28] + tmpR2[26]*tmpFu[32] + tmpR2[27]*tmpFu[36] + tmpR2[28]*tmpFu[40] + tmpR2[29]*tmpFu[44] + tmpR2[30]*tmpFu[48] + tmpR2[31]*tmpFu[52] + tmpR2[32]*tmpFu[56] + tmpR2[33]*tmpFu[60] + tmpR2[34]*tmpFu[64] + tmpR2[35]*tmpFu[68];
tmpR1[5] = + tmpR2[18]*tmpFu[1] + tmpR2[19]*tmpFu[5] + tmpR2[20]*tmpFu[9] + tmpR2[21]*tmpFu[13] + tmpR2[22]*tmpFu[17] + tmpR2[23]*tmpFu[21] + tmpR2[24]*tmpFu[25] + tmpR2[25]*tmpFu[29] + tmpR2[26]*tmpFu[33] + tmpR2[27]*tmpFu[37] + tmpR2[28]*tmpFu[41] + tmpR2[29]*tmpFu[45] + tmpR2[30]*tmpFu[49] + tmpR2[31]*tmpFu[53] + tmpR2[32]*tmpFu[57] + tmpR2[33]*tmpFu[61] + tmpR2[34]*tmpFu[65] + tmpR2[35]*tmpFu[69];
tmpR1[6] = + tmpR2[18]*tmpFu[2] + tmpR2[19]*tmpFu[6] + tmpR2[20]*tmpFu[10] + tmpR2[21]*tmpFu[14] + tmpR2[22]*tmpFu[18] + tmpR2[23]*tmpFu[22] + tmpR2[24]*tmpFu[26] + tmpR2[25]*tmpFu[30] + tmpR2[26]*tmpFu[34] + tmpR2[27]*tmpFu[38] + tmpR2[28]*tmpFu[42] + tmpR2[29]*tmpFu[46] + tmpR2[30]*tmpFu[50] + tmpR2[31]*tmpFu[54] + tmpR2[32]*tmpFu[58] + tmpR2[33]*tmpFu[62] + tmpR2[34]*tmpFu[66] + tmpR2[35]*tmpFu[70];
tmpR1[7] = + tmpR2[18]*tmpFu[3] + tmpR2[19]*tmpFu[7] + tmpR2[20]*tmpFu[11] + tmpR2[21]*tmpFu[15] + tmpR2[22]*tmpFu[19] + tmpR2[23]*tmpFu[23] + tmpR2[24]*tmpFu[27] + tmpR2[25]*tmpFu[31] + tmpR2[26]*tmpFu[35] + tmpR2[27]*tmpFu[39] + tmpR2[28]*tmpFu[43] + tmpR2[29]*tmpFu[47] + tmpR2[30]*tmpFu[51] + tmpR2[31]*tmpFu[55] + tmpR2[32]*tmpFu[59] + tmpR2[33]*tmpFu[63] + tmpR2[34]*tmpFu[67] + tmpR2[35]*tmpFu[71];
tmpR1[8] = + tmpR2[36]*tmpFu[0] + tmpR2[37]*tmpFu[4] + tmpR2[38]*tmpFu[8] + tmpR2[39]*tmpFu[12] + tmpR2[40]*tmpFu[16] + tmpR2[41]*tmpFu[20] + tmpR2[42]*tmpFu[24] + tmpR2[43]*tmpFu[28] + tmpR2[44]*tmpFu[32] + tmpR2[45]*tmpFu[36] + tmpR2[46]*tmpFu[40] + tmpR2[47]*tmpFu[44] + tmpR2[48]*tmpFu[48] + tmpR2[49]*tmpFu[52] + tmpR2[50]*tmpFu[56] + tmpR2[51]*tmpFu[60] + tmpR2[52]*tmpFu[64] + tmpR2[53]*tmpFu[68];
tmpR1[9] = + tmpR2[36]*tmpFu[1] + tmpR2[37]*tmpFu[5] + tmpR2[38]*tmpFu[9] + tmpR2[39]*tmpFu[13] + tmpR2[40]*tmpFu[17] + tmpR2[41]*tmpFu[21] + tmpR2[42]*tmpFu[25] + tmpR2[43]*tmpFu[29] + tmpR2[44]*tmpFu[33] + tmpR2[45]*tmpFu[37] + tmpR2[46]*tmpFu[41] + tmpR2[47]*tmpFu[45] + tmpR2[48]*tmpFu[49] + tmpR2[49]*tmpFu[53] + tmpR2[50]*tmpFu[57] + tmpR2[51]*tmpFu[61] + tmpR2[52]*tmpFu[65] + tmpR2[53]*tmpFu[69];
tmpR1[10] = + tmpR2[36]*tmpFu[2] + tmpR2[37]*tmpFu[6] + tmpR2[38]*tmpFu[10] + tmpR2[39]*tmpFu[14] + tmpR2[40]*tmpFu[18] + tmpR2[41]*tmpFu[22] + tmpR2[42]*tmpFu[26] + tmpR2[43]*tmpFu[30] + tmpR2[44]*tmpFu[34] + tmpR2[45]*tmpFu[38] + tmpR2[46]*tmpFu[42] + tmpR2[47]*tmpFu[46] + tmpR2[48]*tmpFu[50] + tmpR2[49]*tmpFu[54] + tmpR2[50]*tmpFu[58] + tmpR2[51]*tmpFu[62] + tmpR2[52]*tmpFu[66] + tmpR2[53]*tmpFu[70];
tmpR1[11] = + tmpR2[36]*tmpFu[3] + tmpR2[37]*tmpFu[7] + tmpR2[38]*tmpFu[11] + tmpR2[39]*tmpFu[15] + tmpR2[40]*tmpFu[19] + tmpR2[41]*tmpFu[23] + tmpR2[42]*tmpFu[27] + tmpR2[43]*tmpFu[31] + tmpR2[44]*tmpFu[35] + tmpR2[45]*tmpFu[39] + tmpR2[46]*tmpFu[43] + tmpR2[47]*tmpFu[47] + tmpR2[48]*tmpFu[51] + tmpR2[49]*tmpFu[55] + tmpR2[50]*tmpFu[59] + tmpR2[51]*tmpFu[63] + tmpR2[52]*tmpFu[67] + tmpR2[53]*tmpFu[71];
tmpR1[12] = + tmpR2[54]*tmpFu[0] + tmpR2[55]*tmpFu[4] + tmpR2[56]*tmpFu[8] + tmpR2[57]*tmpFu[12] + tmpR2[58]*tmpFu[16] + tmpR2[59]*tmpFu[20] + tmpR2[60]*tmpFu[24] + tmpR2[61]*tmpFu[28] + tmpR2[62]*tmpFu[32] + tmpR2[63]*tmpFu[36] + tmpR2[64]*tmpFu[40] + tmpR2[65]*tmpFu[44] + tmpR2[66]*tmpFu[48] + tmpR2[67]*tmpFu[52] + tmpR2[68]*tmpFu[56] + tmpR2[69]*tmpFu[60] + tmpR2[70]*tmpFu[64] + tmpR2[71]*tmpFu[68];
tmpR1[13] = + tmpR2[54]*tmpFu[1] + tmpR2[55]*tmpFu[5] + tmpR2[56]*tmpFu[9] + tmpR2[57]*tmpFu[13] + tmpR2[58]*tmpFu[17] + tmpR2[59]*tmpFu[21] + tmpR2[60]*tmpFu[25] + tmpR2[61]*tmpFu[29] + tmpR2[62]*tmpFu[33] + tmpR2[63]*tmpFu[37] + tmpR2[64]*tmpFu[41] + tmpR2[65]*tmpFu[45] + tmpR2[66]*tmpFu[49] + tmpR2[67]*tmpFu[53] + tmpR2[68]*tmpFu[57] + tmpR2[69]*tmpFu[61] + tmpR2[70]*tmpFu[65] + tmpR2[71]*tmpFu[69];
tmpR1[14] = + tmpR2[54]*tmpFu[2] + tmpR2[55]*tmpFu[6] + tmpR2[56]*tmpFu[10] + tmpR2[57]*tmpFu[14] + tmpR2[58]*tmpFu[18] + tmpR2[59]*tmpFu[22] + tmpR2[60]*tmpFu[26] + tmpR2[61]*tmpFu[30] + tmpR2[62]*tmpFu[34] + tmpR2[63]*tmpFu[38] + tmpR2[64]*tmpFu[42] + tmpR2[65]*tmpFu[46] + tmpR2[66]*tmpFu[50] + tmpR2[67]*tmpFu[54] + tmpR2[68]*tmpFu[58] + tmpR2[69]*tmpFu[62] + tmpR2[70]*tmpFu[66] + tmpR2[71]*tmpFu[70];
tmpR1[15] = + tmpR2[54]*tmpFu[3] + tmpR2[55]*tmpFu[7] + tmpR2[56]*tmpFu[11] + tmpR2[57]*tmpFu[15] + tmpR2[58]*tmpFu[19] + tmpR2[59]*tmpFu[23] + tmpR2[60]*tmpFu[27] + tmpR2[61]*tmpFu[31] + tmpR2[62]*tmpFu[35] + tmpR2[63]*tmpFu[39] + tmpR2[64]*tmpFu[43] + tmpR2[65]*tmpFu[47] + tmpR2[66]*tmpFu[51] + tmpR2[67]*tmpFu[55] + tmpR2[68]*tmpFu[59] + tmpR2[69]*tmpFu[63] + tmpR2[70]*tmpFu[67] + tmpR2[71]*tmpFu[71];
}

void nmpc_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[16]*tmpObjSEndTerm[14] + tmpFx[32]*tmpObjSEndTerm[28] + tmpFx[48]*tmpObjSEndTerm[42] + tmpFx[64]*tmpObjSEndTerm[56] + tmpFx[80]*tmpObjSEndTerm[70] + tmpFx[96]*tmpObjSEndTerm[84] + tmpFx[112]*tmpObjSEndTerm[98] + tmpFx[128]*tmpObjSEndTerm[112] + tmpFx[144]*tmpObjSEndTerm[126] + tmpFx[160]*tmpObjSEndTerm[140] + tmpFx[176]*tmpObjSEndTerm[154] + tmpFx[192]*tmpObjSEndTerm[168] + tmpFx[208]*tmpObjSEndTerm[182];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[16]*tmpObjSEndTerm[15] + tmpFx[32]*tmpObjSEndTerm[29] + tmpFx[48]*tmpObjSEndTerm[43] + tmpFx[64]*tmpObjSEndTerm[57] + tmpFx[80]*tmpObjSEndTerm[71] + tmpFx[96]*tmpObjSEndTerm[85] + tmpFx[112]*tmpObjSEndTerm[99] + tmpFx[128]*tmpObjSEndTerm[113] + tmpFx[144]*tmpObjSEndTerm[127] + tmpFx[160]*tmpObjSEndTerm[141] + tmpFx[176]*tmpObjSEndTerm[155] + tmpFx[192]*tmpObjSEndTerm[169] + tmpFx[208]*tmpObjSEndTerm[183];
tmpQN2[2] = + tmpFx[0]*tmpObjSEndTerm[2] + tmpFx[16]*tmpObjSEndTerm[16] + tmpFx[32]*tmpObjSEndTerm[30] + tmpFx[48]*tmpObjSEndTerm[44] + tmpFx[64]*tmpObjSEndTerm[58] + tmpFx[80]*tmpObjSEndTerm[72] + tmpFx[96]*tmpObjSEndTerm[86] + tmpFx[112]*tmpObjSEndTerm[100] + tmpFx[128]*tmpObjSEndTerm[114] + tmpFx[144]*tmpObjSEndTerm[128] + tmpFx[160]*tmpObjSEndTerm[142] + tmpFx[176]*tmpObjSEndTerm[156] + tmpFx[192]*tmpObjSEndTerm[170] + tmpFx[208]*tmpObjSEndTerm[184];
tmpQN2[3] = + tmpFx[0]*tmpObjSEndTerm[3] + tmpFx[16]*tmpObjSEndTerm[17] + tmpFx[32]*tmpObjSEndTerm[31] + tmpFx[48]*tmpObjSEndTerm[45] + tmpFx[64]*tmpObjSEndTerm[59] + tmpFx[80]*tmpObjSEndTerm[73] + tmpFx[96]*tmpObjSEndTerm[87] + tmpFx[112]*tmpObjSEndTerm[101] + tmpFx[128]*tmpObjSEndTerm[115] + tmpFx[144]*tmpObjSEndTerm[129] + tmpFx[160]*tmpObjSEndTerm[143] + tmpFx[176]*tmpObjSEndTerm[157] + tmpFx[192]*tmpObjSEndTerm[171] + tmpFx[208]*tmpObjSEndTerm[185];
tmpQN2[4] = + tmpFx[0]*tmpObjSEndTerm[4] + tmpFx[16]*tmpObjSEndTerm[18] + tmpFx[32]*tmpObjSEndTerm[32] + tmpFx[48]*tmpObjSEndTerm[46] + tmpFx[64]*tmpObjSEndTerm[60] + tmpFx[80]*tmpObjSEndTerm[74] + tmpFx[96]*tmpObjSEndTerm[88] + tmpFx[112]*tmpObjSEndTerm[102] + tmpFx[128]*tmpObjSEndTerm[116] + tmpFx[144]*tmpObjSEndTerm[130] + tmpFx[160]*tmpObjSEndTerm[144] + tmpFx[176]*tmpObjSEndTerm[158] + tmpFx[192]*tmpObjSEndTerm[172] + tmpFx[208]*tmpObjSEndTerm[186];
tmpQN2[5] = + tmpFx[0]*tmpObjSEndTerm[5] + tmpFx[16]*tmpObjSEndTerm[19] + tmpFx[32]*tmpObjSEndTerm[33] + tmpFx[48]*tmpObjSEndTerm[47] + tmpFx[64]*tmpObjSEndTerm[61] + tmpFx[80]*tmpObjSEndTerm[75] + tmpFx[96]*tmpObjSEndTerm[89] + tmpFx[112]*tmpObjSEndTerm[103] + tmpFx[128]*tmpObjSEndTerm[117] + tmpFx[144]*tmpObjSEndTerm[131] + tmpFx[160]*tmpObjSEndTerm[145] + tmpFx[176]*tmpObjSEndTerm[159] + tmpFx[192]*tmpObjSEndTerm[173] + tmpFx[208]*tmpObjSEndTerm[187];
tmpQN2[6] = + tmpFx[0]*tmpObjSEndTerm[6] + tmpFx[16]*tmpObjSEndTerm[20] + tmpFx[32]*tmpObjSEndTerm[34] + tmpFx[48]*tmpObjSEndTerm[48] + tmpFx[64]*tmpObjSEndTerm[62] + tmpFx[80]*tmpObjSEndTerm[76] + tmpFx[96]*tmpObjSEndTerm[90] + tmpFx[112]*tmpObjSEndTerm[104] + tmpFx[128]*tmpObjSEndTerm[118] + tmpFx[144]*tmpObjSEndTerm[132] + tmpFx[160]*tmpObjSEndTerm[146] + tmpFx[176]*tmpObjSEndTerm[160] + tmpFx[192]*tmpObjSEndTerm[174] + tmpFx[208]*tmpObjSEndTerm[188];
tmpQN2[7] = + tmpFx[0]*tmpObjSEndTerm[7] + tmpFx[16]*tmpObjSEndTerm[21] + tmpFx[32]*tmpObjSEndTerm[35] + tmpFx[48]*tmpObjSEndTerm[49] + tmpFx[64]*tmpObjSEndTerm[63] + tmpFx[80]*tmpObjSEndTerm[77] + tmpFx[96]*tmpObjSEndTerm[91] + tmpFx[112]*tmpObjSEndTerm[105] + tmpFx[128]*tmpObjSEndTerm[119] + tmpFx[144]*tmpObjSEndTerm[133] + tmpFx[160]*tmpObjSEndTerm[147] + tmpFx[176]*tmpObjSEndTerm[161] + tmpFx[192]*tmpObjSEndTerm[175] + tmpFx[208]*tmpObjSEndTerm[189];
tmpQN2[8] = + tmpFx[0]*tmpObjSEndTerm[8] + tmpFx[16]*tmpObjSEndTerm[22] + tmpFx[32]*tmpObjSEndTerm[36] + tmpFx[48]*tmpObjSEndTerm[50] + tmpFx[64]*tmpObjSEndTerm[64] + tmpFx[80]*tmpObjSEndTerm[78] + tmpFx[96]*tmpObjSEndTerm[92] + tmpFx[112]*tmpObjSEndTerm[106] + tmpFx[128]*tmpObjSEndTerm[120] + tmpFx[144]*tmpObjSEndTerm[134] + tmpFx[160]*tmpObjSEndTerm[148] + tmpFx[176]*tmpObjSEndTerm[162] + tmpFx[192]*tmpObjSEndTerm[176] + tmpFx[208]*tmpObjSEndTerm[190];
tmpQN2[9] = + tmpFx[0]*tmpObjSEndTerm[9] + tmpFx[16]*tmpObjSEndTerm[23] + tmpFx[32]*tmpObjSEndTerm[37] + tmpFx[48]*tmpObjSEndTerm[51] + tmpFx[64]*tmpObjSEndTerm[65] + tmpFx[80]*tmpObjSEndTerm[79] + tmpFx[96]*tmpObjSEndTerm[93] + tmpFx[112]*tmpObjSEndTerm[107] + tmpFx[128]*tmpObjSEndTerm[121] + tmpFx[144]*tmpObjSEndTerm[135] + tmpFx[160]*tmpObjSEndTerm[149] + tmpFx[176]*tmpObjSEndTerm[163] + tmpFx[192]*tmpObjSEndTerm[177] + tmpFx[208]*tmpObjSEndTerm[191];
tmpQN2[10] = + tmpFx[0]*tmpObjSEndTerm[10] + tmpFx[16]*tmpObjSEndTerm[24] + tmpFx[32]*tmpObjSEndTerm[38] + tmpFx[48]*tmpObjSEndTerm[52] + tmpFx[64]*tmpObjSEndTerm[66] + tmpFx[80]*tmpObjSEndTerm[80] + tmpFx[96]*tmpObjSEndTerm[94] + tmpFx[112]*tmpObjSEndTerm[108] + tmpFx[128]*tmpObjSEndTerm[122] + tmpFx[144]*tmpObjSEndTerm[136] + tmpFx[160]*tmpObjSEndTerm[150] + tmpFx[176]*tmpObjSEndTerm[164] + tmpFx[192]*tmpObjSEndTerm[178] + tmpFx[208]*tmpObjSEndTerm[192];
tmpQN2[11] = + tmpFx[0]*tmpObjSEndTerm[11] + tmpFx[16]*tmpObjSEndTerm[25] + tmpFx[32]*tmpObjSEndTerm[39] + tmpFx[48]*tmpObjSEndTerm[53] + tmpFx[64]*tmpObjSEndTerm[67] + tmpFx[80]*tmpObjSEndTerm[81] + tmpFx[96]*tmpObjSEndTerm[95] + tmpFx[112]*tmpObjSEndTerm[109] + tmpFx[128]*tmpObjSEndTerm[123] + tmpFx[144]*tmpObjSEndTerm[137] + tmpFx[160]*tmpObjSEndTerm[151] + tmpFx[176]*tmpObjSEndTerm[165] + tmpFx[192]*tmpObjSEndTerm[179] + tmpFx[208]*tmpObjSEndTerm[193];
tmpQN2[12] = + tmpFx[0]*tmpObjSEndTerm[12] + tmpFx[16]*tmpObjSEndTerm[26] + tmpFx[32]*tmpObjSEndTerm[40] + tmpFx[48]*tmpObjSEndTerm[54] + tmpFx[64]*tmpObjSEndTerm[68] + tmpFx[80]*tmpObjSEndTerm[82] + tmpFx[96]*tmpObjSEndTerm[96] + tmpFx[112]*tmpObjSEndTerm[110] + tmpFx[128]*tmpObjSEndTerm[124] + tmpFx[144]*tmpObjSEndTerm[138] + tmpFx[160]*tmpObjSEndTerm[152] + tmpFx[176]*tmpObjSEndTerm[166] + tmpFx[192]*tmpObjSEndTerm[180] + tmpFx[208]*tmpObjSEndTerm[194];
tmpQN2[13] = + tmpFx[0]*tmpObjSEndTerm[13] + tmpFx[16]*tmpObjSEndTerm[27] + tmpFx[32]*tmpObjSEndTerm[41] + tmpFx[48]*tmpObjSEndTerm[55] + tmpFx[64]*tmpObjSEndTerm[69] + tmpFx[80]*tmpObjSEndTerm[83] + tmpFx[96]*tmpObjSEndTerm[97] + tmpFx[112]*tmpObjSEndTerm[111] + tmpFx[128]*tmpObjSEndTerm[125] + tmpFx[144]*tmpObjSEndTerm[139] + tmpFx[160]*tmpObjSEndTerm[153] + tmpFx[176]*tmpObjSEndTerm[167] + tmpFx[192]*tmpObjSEndTerm[181] + tmpFx[208]*tmpObjSEndTerm[195];
tmpQN2[14] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[17]*tmpObjSEndTerm[14] + tmpFx[33]*tmpObjSEndTerm[28] + tmpFx[49]*tmpObjSEndTerm[42] + tmpFx[65]*tmpObjSEndTerm[56] + tmpFx[81]*tmpObjSEndTerm[70] + tmpFx[97]*tmpObjSEndTerm[84] + tmpFx[113]*tmpObjSEndTerm[98] + tmpFx[129]*tmpObjSEndTerm[112] + tmpFx[145]*tmpObjSEndTerm[126] + tmpFx[161]*tmpObjSEndTerm[140] + tmpFx[177]*tmpObjSEndTerm[154] + tmpFx[193]*tmpObjSEndTerm[168] + tmpFx[209]*tmpObjSEndTerm[182];
tmpQN2[15] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[17]*tmpObjSEndTerm[15] + tmpFx[33]*tmpObjSEndTerm[29] + tmpFx[49]*tmpObjSEndTerm[43] + tmpFx[65]*tmpObjSEndTerm[57] + tmpFx[81]*tmpObjSEndTerm[71] + tmpFx[97]*tmpObjSEndTerm[85] + tmpFx[113]*tmpObjSEndTerm[99] + tmpFx[129]*tmpObjSEndTerm[113] + tmpFx[145]*tmpObjSEndTerm[127] + tmpFx[161]*tmpObjSEndTerm[141] + tmpFx[177]*tmpObjSEndTerm[155] + tmpFx[193]*tmpObjSEndTerm[169] + tmpFx[209]*tmpObjSEndTerm[183];
tmpQN2[16] = + tmpFx[1]*tmpObjSEndTerm[2] + tmpFx[17]*tmpObjSEndTerm[16] + tmpFx[33]*tmpObjSEndTerm[30] + tmpFx[49]*tmpObjSEndTerm[44] + tmpFx[65]*tmpObjSEndTerm[58] + tmpFx[81]*tmpObjSEndTerm[72] + tmpFx[97]*tmpObjSEndTerm[86] + tmpFx[113]*tmpObjSEndTerm[100] + tmpFx[129]*tmpObjSEndTerm[114] + tmpFx[145]*tmpObjSEndTerm[128] + tmpFx[161]*tmpObjSEndTerm[142] + tmpFx[177]*tmpObjSEndTerm[156] + tmpFx[193]*tmpObjSEndTerm[170] + tmpFx[209]*tmpObjSEndTerm[184];
tmpQN2[17] = + tmpFx[1]*tmpObjSEndTerm[3] + tmpFx[17]*tmpObjSEndTerm[17] + tmpFx[33]*tmpObjSEndTerm[31] + tmpFx[49]*tmpObjSEndTerm[45] + tmpFx[65]*tmpObjSEndTerm[59] + tmpFx[81]*tmpObjSEndTerm[73] + tmpFx[97]*tmpObjSEndTerm[87] + tmpFx[113]*tmpObjSEndTerm[101] + tmpFx[129]*tmpObjSEndTerm[115] + tmpFx[145]*tmpObjSEndTerm[129] + tmpFx[161]*tmpObjSEndTerm[143] + tmpFx[177]*tmpObjSEndTerm[157] + tmpFx[193]*tmpObjSEndTerm[171] + tmpFx[209]*tmpObjSEndTerm[185];
tmpQN2[18] = + tmpFx[1]*tmpObjSEndTerm[4] + tmpFx[17]*tmpObjSEndTerm[18] + tmpFx[33]*tmpObjSEndTerm[32] + tmpFx[49]*tmpObjSEndTerm[46] + tmpFx[65]*tmpObjSEndTerm[60] + tmpFx[81]*tmpObjSEndTerm[74] + tmpFx[97]*tmpObjSEndTerm[88] + tmpFx[113]*tmpObjSEndTerm[102] + tmpFx[129]*tmpObjSEndTerm[116] + tmpFx[145]*tmpObjSEndTerm[130] + tmpFx[161]*tmpObjSEndTerm[144] + tmpFx[177]*tmpObjSEndTerm[158] + tmpFx[193]*tmpObjSEndTerm[172] + tmpFx[209]*tmpObjSEndTerm[186];
tmpQN2[19] = + tmpFx[1]*tmpObjSEndTerm[5] + tmpFx[17]*tmpObjSEndTerm[19] + tmpFx[33]*tmpObjSEndTerm[33] + tmpFx[49]*tmpObjSEndTerm[47] + tmpFx[65]*tmpObjSEndTerm[61] + tmpFx[81]*tmpObjSEndTerm[75] + tmpFx[97]*tmpObjSEndTerm[89] + tmpFx[113]*tmpObjSEndTerm[103] + tmpFx[129]*tmpObjSEndTerm[117] + tmpFx[145]*tmpObjSEndTerm[131] + tmpFx[161]*tmpObjSEndTerm[145] + tmpFx[177]*tmpObjSEndTerm[159] + tmpFx[193]*tmpObjSEndTerm[173] + tmpFx[209]*tmpObjSEndTerm[187];
tmpQN2[20] = + tmpFx[1]*tmpObjSEndTerm[6] + tmpFx[17]*tmpObjSEndTerm[20] + tmpFx[33]*tmpObjSEndTerm[34] + tmpFx[49]*tmpObjSEndTerm[48] + tmpFx[65]*tmpObjSEndTerm[62] + tmpFx[81]*tmpObjSEndTerm[76] + tmpFx[97]*tmpObjSEndTerm[90] + tmpFx[113]*tmpObjSEndTerm[104] + tmpFx[129]*tmpObjSEndTerm[118] + tmpFx[145]*tmpObjSEndTerm[132] + tmpFx[161]*tmpObjSEndTerm[146] + tmpFx[177]*tmpObjSEndTerm[160] + tmpFx[193]*tmpObjSEndTerm[174] + tmpFx[209]*tmpObjSEndTerm[188];
tmpQN2[21] = + tmpFx[1]*tmpObjSEndTerm[7] + tmpFx[17]*tmpObjSEndTerm[21] + tmpFx[33]*tmpObjSEndTerm[35] + tmpFx[49]*tmpObjSEndTerm[49] + tmpFx[65]*tmpObjSEndTerm[63] + tmpFx[81]*tmpObjSEndTerm[77] + tmpFx[97]*tmpObjSEndTerm[91] + tmpFx[113]*tmpObjSEndTerm[105] + tmpFx[129]*tmpObjSEndTerm[119] + tmpFx[145]*tmpObjSEndTerm[133] + tmpFx[161]*tmpObjSEndTerm[147] + tmpFx[177]*tmpObjSEndTerm[161] + tmpFx[193]*tmpObjSEndTerm[175] + tmpFx[209]*tmpObjSEndTerm[189];
tmpQN2[22] = + tmpFx[1]*tmpObjSEndTerm[8] + tmpFx[17]*tmpObjSEndTerm[22] + tmpFx[33]*tmpObjSEndTerm[36] + tmpFx[49]*tmpObjSEndTerm[50] + tmpFx[65]*tmpObjSEndTerm[64] + tmpFx[81]*tmpObjSEndTerm[78] + tmpFx[97]*tmpObjSEndTerm[92] + tmpFx[113]*tmpObjSEndTerm[106] + tmpFx[129]*tmpObjSEndTerm[120] + tmpFx[145]*tmpObjSEndTerm[134] + tmpFx[161]*tmpObjSEndTerm[148] + tmpFx[177]*tmpObjSEndTerm[162] + tmpFx[193]*tmpObjSEndTerm[176] + tmpFx[209]*tmpObjSEndTerm[190];
tmpQN2[23] = + tmpFx[1]*tmpObjSEndTerm[9] + tmpFx[17]*tmpObjSEndTerm[23] + tmpFx[33]*tmpObjSEndTerm[37] + tmpFx[49]*tmpObjSEndTerm[51] + tmpFx[65]*tmpObjSEndTerm[65] + tmpFx[81]*tmpObjSEndTerm[79] + tmpFx[97]*tmpObjSEndTerm[93] + tmpFx[113]*tmpObjSEndTerm[107] + tmpFx[129]*tmpObjSEndTerm[121] + tmpFx[145]*tmpObjSEndTerm[135] + tmpFx[161]*tmpObjSEndTerm[149] + tmpFx[177]*tmpObjSEndTerm[163] + tmpFx[193]*tmpObjSEndTerm[177] + tmpFx[209]*tmpObjSEndTerm[191];
tmpQN2[24] = + tmpFx[1]*tmpObjSEndTerm[10] + tmpFx[17]*tmpObjSEndTerm[24] + tmpFx[33]*tmpObjSEndTerm[38] + tmpFx[49]*tmpObjSEndTerm[52] + tmpFx[65]*tmpObjSEndTerm[66] + tmpFx[81]*tmpObjSEndTerm[80] + tmpFx[97]*tmpObjSEndTerm[94] + tmpFx[113]*tmpObjSEndTerm[108] + tmpFx[129]*tmpObjSEndTerm[122] + tmpFx[145]*tmpObjSEndTerm[136] + tmpFx[161]*tmpObjSEndTerm[150] + tmpFx[177]*tmpObjSEndTerm[164] + tmpFx[193]*tmpObjSEndTerm[178] + tmpFx[209]*tmpObjSEndTerm[192];
tmpQN2[25] = + tmpFx[1]*tmpObjSEndTerm[11] + tmpFx[17]*tmpObjSEndTerm[25] + tmpFx[33]*tmpObjSEndTerm[39] + tmpFx[49]*tmpObjSEndTerm[53] + tmpFx[65]*tmpObjSEndTerm[67] + tmpFx[81]*tmpObjSEndTerm[81] + tmpFx[97]*tmpObjSEndTerm[95] + tmpFx[113]*tmpObjSEndTerm[109] + tmpFx[129]*tmpObjSEndTerm[123] + tmpFx[145]*tmpObjSEndTerm[137] + tmpFx[161]*tmpObjSEndTerm[151] + tmpFx[177]*tmpObjSEndTerm[165] + tmpFx[193]*tmpObjSEndTerm[179] + tmpFx[209]*tmpObjSEndTerm[193];
tmpQN2[26] = + tmpFx[1]*tmpObjSEndTerm[12] + tmpFx[17]*tmpObjSEndTerm[26] + tmpFx[33]*tmpObjSEndTerm[40] + tmpFx[49]*tmpObjSEndTerm[54] + tmpFx[65]*tmpObjSEndTerm[68] + tmpFx[81]*tmpObjSEndTerm[82] + tmpFx[97]*tmpObjSEndTerm[96] + tmpFx[113]*tmpObjSEndTerm[110] + tmpFx[129]*tmpObjSEndTerm[124] + tmpFx[145]*tmpObjSEndTerm[138] + tmpFx[161]*tmpObjSEndTerm[152] + tmpFx[177]*tmpObjSEndTerm[166] + tmpFx[193]*tmpObjSEndTerm[180] + tmpFx[209]*tmpObjSEndTerm[194];
tmpQN2[27] = + tmpFx[1]*tmpObjSEndTerm[13] + tmpFx[17]*tmpObjSEndTerm[27] + tmpFx[33]*tmpObjSEndTerm[41] + tmpFx[49]*tmpObjSEndTerm[55] + tmpFx[65]*tmpObjSEndTerm[69] + tmpFx[81]*tmpObjSEndTerm[83] + tmpFx[97]*tmpObjSEndTerm[97] + tmpFx[113]*tmpObjSEndTerm[111] + tmpFx[129]*tmpObjSEndTerm[125] + tmpFx[145]*tmpObjSEndTerm[139] + tmpFx[161]*tmpObjSEndTerm[153] + tmpFx[177]*tmpObjSEndTerm[167] + tmpFx[193]*tmpObjSEndTerm[181] + tmpFx[209]*tmpObjSEndTerm[195];
tmpQN2[28] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[18]*tmpObjSEndTerm[14] + tmpFx[34]*tmpObjSEndTerm[28] + tmpFx[50]*tmpObjSEndTerm[42] + tmpFx[66]*tmpObjSEndTerm[56] + tmpFx[82]*tmpObjSEndTerm[70] + tmpFx[98]*tmpObjSEndTerm[84] + tmpFx[114]*tmpObjSEndTerm[98] + tmpFx[130]*tmpObjSEndTerm[112] + tmpFx[146]*tmpObjSEndTerm[126] + tmpFx[162]*tmpObjSEndTerm[140] + tmpFx[178]*tmpObjSEndTerm[154] + tmpFx[194]*tmpObjSEndTerm[168] + tmpFx[210]*tmpObjSEndTerm[182];
tmpQN2[29] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[18]*tmpObjSEndTerm[15] + tmpFx[34]*tmpObjSEndTerm[29] + tmpFx[50]*tmpObjSEndTerm[43] + tmpFx[66]*tmpObjSEndTerm[57] + tmpFx[82]*tmpObjSEndTerm[71] + tmpFx[98]*tmpObjSEndTerm[85] + tmpFx[114]*tmpObjSEndTerm[99] + tmpFx[130]*tmpObjSEndTerm[113] + tmpFx[146]*tmpObjSEndTerm[127] + tmpFx[162]*tmpObjSEndTerm[141] + tmpFx[178]*tmpObjSEndTerm[155] + tmpFx[194]*tmpObjSEndTerm[169] + tmpFx[210]*tmpObjSEndTerm[183];
tmpQN2[30] = + tmpFx[2]*tmpObjSEndTerm[2] + tmpFx[18]*tmpObjSEndTerm[16] + tmpFx[34]*tmpObjSEndTerm[30] + tmpFx[50]*tmpObjSEndTerm[44] + tmpFx[66]*tmpObjSEndTerm[58] + tmpFx[82]*tmpObjSEndTerm[72] + tmpFx[98]*tmpObjSEndTerm[86] + tmpFx[114]*tmpObjSEndTerm[100] + tmpFx[130]*tmpObjSEndTerm[114] + tmpFx[146]*tmpObjSEndTerm[128] + tmpFx[162]*tmpObjSEndTerm[142] + tmpFx[178]*tmpObjSEndTerm[156] + tmpFx[194]*tmpObjSEndTerm[170] + tmpFx[210]*tmpObjSEndTerm[184];
tmpQN2[31] = + tmpFx[2]*tmpObjSEndTerm[3] + tmpFx[18]*tmpObjSEndTerm[17] + tmpFx[34]*tmpObjSEndTerm[31] + tmpFx[50]*tmpObjSEndTerm[45] + tmpFx[66]*tmpObjSEndTerm[59] + tmpFx[82]*tmpObjSEndTerm[73] + tmpFx[98]*tmpObjSEndTerm[87] + tmpFx[114]*tmpObjSEndTerm[101] + tmpFx[130]*tmpObjSEndTerm[115] + tmpFx[146]*tmpObjSEndTerm[129] + tmpFx[162]*tmpObjSEndTerm[143] + tmpFx[178]*tmpObjSEndTerm[157] + tmpFx[194]*tmpObjSEndTerm[171] + tmpFx[210]*tmpObjSEndTerm[185];
tmpQN2[32] = + tmpFx[2]*tmpObjSEndTerm[4] + tmpFx[18]*tmpObjSEndTerm[18] + tmpFx[34]*tmpObjSEndTerm[32] + tmpFx[50]*tmpObjSEndTerm[46] + tmpFx[66]*tmpObjSEndTerm[60] + tmpFx[82]*tmpObjSEndTerm[74] + tmpFx[98]*tmpObjSEndTerm[88] + tmpFx[114]*tmpObjSEndTerm[102] + tmpFx[130]*tmpObjSEndTerm[116] + tmpFx[146]*tmpObjSEndTerm[130] + tmpFx[162]*tmpObjSEndTerm[144] + tmpFx[178]*tmpObjSEndTerm[158] + tmpFx[194]*tmpObjSEndTerm[172] + tmpFx[210]*tmpObjSEndTerm[186];
tmpQN2[33] = + tmpFx[2]*tmpObjSEndTerm[5] + tmpFx[18]*tmpObjSEndTerm[19] + tmpFx[34]*tmpObjSEndTerm[33] + tmpFx[50]*tmpObjSEndTerm[47] + tmpFx[66]*tmpObjSEndTerm[61] + tmpFx[82]*tmpObjSEndTerm[75] + tmpFx[98]*tmpObjSEndTerm[89] + tmpFx[114]*tmpObjSEndTerm[103] + tmpFx[130]*tmpObjSEndTerm[117] + tmpFx[146]*tmpObjSEndTerm[131] + tmpFx[162]*tmpObjSEndTerm[145] + tmpFx[178]*tmpObjSEndTerm[159] + tmpFx[194]*tmpObjSEndTerm[173] + tmpFx[210]*tmpObjSEndTerm[187];
tmpQN2[34] = + tmpFx[2]*tmpObjSEndTerm[6] + tmpFx[18]*tmpObjSEndTerm[20] + tmpFx[34]*tmpObjSEndTerm[34] + tmpFx[50]*tmpObjSEndTerm[48] + tmpFx[66]*tmpObjSEndTerm[62] + tmpFx[82]*tmpObjSEndTerm[76] + tmpFx[98]*tmpObjSEndTerm[90] + tmpFx[114]*tmpObjSEndTerm[104] + tmpFx[130]*tmpObjSEndTerm[118] + tmpFx[146]*tmpObjSEndTerm[132] + tmpFx[162]*tmpObjSEndTerm[146] + tmpFx[178]*tmpObjSEndTerm[160] + tmpFx[194]*tmpObjSEndTerm[174] + tmpFx[210]*tmpObjSEndTerm[188];
tmpQN2[35] = + tmpFx[2]*tmpObjSEndTerm[7] + tmpFx[18]*tmpObjSEndTerm[21] + tmpFx[34]*tmpObjSEndTerm[35] + tmpFx[50]*tmpObjSEndTerm[49] + tmpFx[66]*tmpObjSEndTerm[63] + tmpFx[82]*tmpObjSEndTerm[77] + tmpFx[98]*tmpObjSEndTerm[91] + tmpFx[114]*tmpObjSEndTerm[105] + tmpFx[130]*tmpObjSEndTerm[119] + tmpFx[146]*tmpObjSEndTerm[133] + tmpFx[162]*tmpObjSEndTerm[147] + tmpFx[178]*tmpObjSEndTerm[161] + tmpFx[194]*tmpObjSEndTerm[175] + tmpFx[210]*tmpObjSEndTerm[189];
tmpQN2[36] = + tmpFx[2]*tmpObjSEndTerm[8] + tmpFx[18]*tmpObjSEndTerm[22] + tmpFx[34]*tmpObjSEndTerm[36] + tmpFx[50]*tmpObjSEndTerm[50] + tmpFx[66]*tmpObjSEndTerm[64] + tmpFx[82]*tmpObjSEndTerm[78] + tmpFx[98]*tmpObjSEndTerm[92] + tmpFx[114]*tmpObjSEndTerm[106] + tmpFx[130]*tmpObjSEndTerm[120] + tmpFx[146]*tmpObjSEndTerm[134] + tmpFx[162]*tmpObjSEndTerm[148] + tmpFx[178]*tmpObjSEndTerm[162] + tmpFx[194]*tmpObjSEndTerm[176] + tmpFx[210]*tmpObjSEndTerm[190];
tmpQN2[37] = + tmpFx[2]*tmpObjSEndTerm[9] + tmpFx[18]*tmpObjSEndTerm[23] + tmpFx[34]*tmpObjSEndTerm[37] + tmpFx[50]*tmpObjSEndTerm[51] + tmpFx[66]*tmpObjSEndTerm[65] + tmpFx[82]*tmpObjSEndTerm[79] + tmpFx[98]*tmpObjSEndTerm[93] + tmpFx[114]*tmpObjSEndTerm[107] + tmpFx[130]*tmpObjSEndTerm[121] + tmpFx[146]*tmpObjSEndTerm[135] + tmpFx[162]*tmpObjSEndTerm[149] + tmpFx[178]*tmpObjSEndTerm[163] + tmpFx[194]*tmpObjSEndTerm[177] + tmpFx[210]*tmpObjSEndTerm[191];
tmpQN2[38] = + tmpFx[2]*tmpObjSEndTerm[10] + tmpFx[18]*tmpObjSEndTerm[24] + tmpFx[34]*tmpObjSEndTerm[38] + tmpFx[50]*tmpObjSEndTerm[52] + tmpFx[66]*tmpObjSEndTerm[66] + tmpFx[82]*tmpObjSEndTerm[80] + tmpFx[98]*tmpObjSEndTerm[94] + tmpFx[114]*tmpObjSEndTerm[108] + tmpFx[130]*tmpObjSEndTerm[122] + tmpFx[146]*tmpObjSEndTerm[136] + tmpFx[162]*tmpObjSEndTerm[150] + tmpFx[178]*tmpObjSEndTerm[164] + tmpFx[194]*tmpObjSEndTerm[178] + tmpFx[210]*tmpObjSEndTerm[192];
tmpQN2[39] = + tmpFx[2]*tmpObjSEndTerm[11] + tmpFx[18]*tmpObjSEndTerm[25] + tmpFx[34]*tmpObjSEndTerm[39] + tmpFx[50]*tmpObjSEndTerm[53] + tmpFx[66]*tmpObjSEndTerm[67] + tmpFx[82]*tmpObjSEndTerm[81] + tmpFx[98]*tmpObjSEndTerm[95] + tmpFx[114]*tmpObjSEndTerm[109] + tmpFx[130]*tmpObjSEndTerm[123] + tmpFx[146]*tmpObjSEndTerm[137] + tmpFx[162]*tmpObjSEndTerm[151] + tmpFx[178]*tmpObjSEndTerm[165] + tmpFx[194]*tmpObjSEndTerm[179] + tmpFx[210]*tmpObjSEndTerm[193];
tmpQN2[40] = + tmpFx[2]*tmpObjSEndTerm[12] + tmpFx[18]*tmpObjSEndTerm[26] + tmpFx[34]*tmpObjSEndTerm[40] + tmpFx[50]*tmpObjSEndTerm[54] + tmpFx[66]*tmpObjSEndTerm[68] + tmpFx[82]*tmpObjSEndTerm[82] + tmpFx[98]*tmpObjSEndTerm[96] + tmpFx[114]*tmpObjSEndTerm[110] + tmpFx[130]*tmpObjSEndTerm[124] + tmpFx[146]*tmpObjSEndTerm[138] + tmpFx[162]*tmpObjSEndTerm[152] + tmpFx[178]*tmpObjSEndTerm[166] + tmpFx[194]*tmpObjSEndTerm[180] + tmpFx[210]*tmpObjSEndTerm[194];
tmpQN2[41] = + tmpFx[2]*tmpObjSEndTerm[13] + tmpFx[18]*tmpObjSEndTerm[27] + tmpFx[34]*tmpObjSEndTerm[41] + tmpFx[50]*tmpObjSEndTerm[55] + tmpFx[66]*tmpObjSEndTerm[69] + tmpFx[82]*tmpObjSEndTerm[83] + tmpFx[98]*tmpObjSEndTerm[97] + tmpFx[114]*tmpObjSEndTerm[111] + tmpFx[130]*tmpObjSEndTerm[125] + tmpFx[146]*tmpObjSEndTerm[139] + tmpFx[162]*tmpObjSEndTerm[153] + tmpFx[178]*tmpObjSEndTerm[167] + tmpFx[194]*tmpObjSEndTerm[181] + tmpFx[210]*tmpObjSEndTerm[195];
tmpQN2[42] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[19]*tmpObjSEndTerm[14] + tmpFx[35]*tmpObjSEndTerm[28] + tmpFx[51]*tmpObjSEndTerm[42] + tmpFx[67]*tmpObjSEndTerm[56] + tmpFx[83]*tmpObjSEndTerm[70] + tmpFx[99]*tmpObjSEndTerm[84] + tmpFx[115]*tmpObjSEndTerm[98] + tmpFx[131]*tmpObjSEndTerm[112] + tmpFx[147]*tmpObjSEndTerm[126] + tmpFx[163]*tmpObjSEndTerm[140] + tmpFx[179]*tmpObjSEndTerm[154] + tmpFx[195]*tmpObjSEndTerm[168] + tmpFx[211]*tmpObjSEndTerm[182];
tmpQN2[43] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[19]*tmpObjSEndTerm[15] + tmpFx[35]*tmpObjSEndTerm[29] + tmpFx[51]*tmpObjSEndTerm[43] + tmpFx[67]*tmpObjSEndTerm[57] + tmpFx[83]*tmpObjSEndTerm[71] + tmpFx[99]*tmpObjSEndTerm[85] + tmpFx[115]*tmpObjSEndTerm[99] + tmpFx[131]*tmpObjSEndTerm[113] + tmpFx[147]*tmpObjSEndTerm[127] + tmpFx[163]*tmpObjSEndTerm[141] + tmpFx[179]*tmpObjSEndTerm[155] + tmpFx[195]*tmpObjSEndTerm[169] + tmpFx[211]*tmpObjSEndTerm[183];
tmpQN2[44] = + tmpFx[3]*tmpObjSEndTerm[2] + tmpFx[19]*tmpObjSEndTerm[16] + tmpFx[35]*tmpObjSEndTerm[30] + tmpFx[51]*tmpObjSEndTerm[44] + tmpFx[67]*tmpObjSEndTerm[58] + tmpFx[83]*tmpObjSEndTerm[72] + tmpFx[99]*tmpObjSEndTerm[86] + tmpFx[115]*tmpObjSEndTerm[100] + tmpFx[131]*tmpObjSEndTerm[114] + tmpFx[147]*tmpObjSEndTerm[128] + tmpFx[163]*tmpObjSEndTerm[142] + tmpFx[179]*tmpObjSEndTerm[156] + tmpFx[195]*tmpObjSEndTerm[170] + tmpFx[211]*tmpObjSEndTerm[184];
tmpQN2[45] = + tmpFx[3]*tmpObjSEndTerm[3] + tmpFx[19]*tmpObjSEndTerm[17] + tmpFx[35]*tmpObjSEndTerm[31] + tmpFx[51]*tmpObjSEndTerm[45] + tmpFx[67]*tmpObjSEndTerm[59] + tmpFx[83]*tmpObjSEndTerm[73] + tmpFx[99]*tmpObjSEndTerm[87] + tmpFx[115]*tmpObjSEndTerm[101] + tmpFx[131]*tmpObjSEndTerm[115] + tmpFx[147]*tmpObjSEndTerm[129] + tmpFx[163]*tmpObjSEndTerm[143] + tmpFx[179]*tmpObjSEndTerm[157] + tmpFx[195]*tmpObjSEndTerm[171] + tmpFx[211]*tmpObjSEndTerm[185];
tmpQN2[46] = + tmpFx[3]*tmpObjSEndTerm[4] + tmpFx[19]*tmpObjSEndTerm[18] + tmpFx[35]*tmpObjSEndTerm[32] + tmpFx[51]*tmpObjSEndTerm[46] + tmpFx[67]*tmpObjSEndTerm[60] + tmpFx[83]*tmpObjSEndTerm[74] + tmpFx[99]*tmpObjSEndTerm[88] + tmpFx[115]*tmpObjSEndTerm[102] + tmpFx[131]*tmpObjSEndTerm[116] + tmpFx[147]*tmpObjSEndTerm[130] + tmpFx[163]*tmpObjSEndTerm[144] + tmpFx[179]*tmpObjSEndTerm[158] + tmpFx[195]*tmpObjSEndTerm[172] + tmpFx[211]*tmpObjSEndTerm[186];
tmpQN2[47] = + tmpFx[3]*tmpObjSEndTerm[5] + tmpFx[19]*tmpObjSEndTerm[19] + tmpFx[35]*tmpObjSEndTerm[33] + tmpFx[51]*tmpObjSEndTerm[47] + tmpFx[67]*tmpObjSEndTerm[61] + tmpFx[83]*tmpObjSEndTerm[75] + tmpFx[99]*tmpObjSEndTerm[89] + tmpFx[115]*tmpObjSEndTerm[103] + tmpFx[131]*tmpObjSEndTerm[117] + tmpFx[147]*tmpObjSEndTerm[131] + tmpFx[163]*tmpObjSEndTerm[145] + tmpFx[179]*tmpObjSEndTerm[159] + tmpFx[195]*tmpObjSEndTerm[173] + tmpFx[211]*tmpObjSEndTerm[187];
tmpQN2[48] = + tmpFx[3]*tmpObjSEndTerm[6] + tmpFx[19]*tmpObjSEndTerm[20] + tmpFx[35]*tmpObjSEndTerm[34] + tmpFx[51]*tmpObjSEndTerm[48] + tmpFx[67]*tmpObjSEndTerm[62] + tmpFx[83]*tmpObjSEndTerm[76] + tmpFx[99]*tmpObjSEndTerm[90] + tmpFx[115]*tmpObjSEndTerm[104] + tmpFx[131]*tmpObjSEndTerm[118] + tmpFx[147]*tmpObjSEndTerm[132] + tmpFx[163]*tmpObjSEndTerm[146] + tmpFx[179]*tmpObjSEndTerm[160] + tmpFx[195]*tmpObjSEndTerm[174] + tmpFx[211]*tmpObjSEndTerm[188];
tmpQN2[49] = + tmpFx[3]*tmpObjSEndTerm[7] + tmpFx[19]*tmpObjSEndTerm[21] + tmpFx[35]*tmpObjSEndTerm[35] + tmpFx[51]*tmpObjSEndTerm[49] + tmpFx[67]*tmpObjSEndTerm[63] + tmpFx[83]*tmpObjSEndTerm[77] + tmpFx[99]*tmpObjSEndTerm[91] + tmpFx[115]*tmpObjSEndTerm[105] + tmpFx[131]*tmpObjSEndTerm[119] + tmpFx[147]*tmpObjSEndTerm[133] + tmpFx[163]*tmpObjSEndTerm[147] + tmpFx[179]*tmpObjSEndTerm[161] + tmpFx[195]*tmpObjSEndTerm[175] + tmpFx[211]*tmpObjSEndTerm[189];
tmpQN2[50] = + tmpFx[3]*tmpObjSEndTerm[8] + tmpFx[19]*tmpObjSEndTerm[22] + tmpFx[35]*tmpObjSEndTerm[36] + tmpFx[51]*tmpObjSEndTerm[50] + tmpFx[67]*tmpObjSEndTerm[64] + tmpFx[83]*tmpObjSEndTerm[78] + tmpFx[99]*tmpObjSEndTerm[92] + tmpFx[115]*tmpObjSEndTerm[106] + tmpFx[131]*tmpObjSEndTerm[120] + tmpFx[147]*tmpObjSEndTerm[134] + tmpFx[163]*tmpObjSEndTerm[148] + tmpFx[179]*tmpObjSEndTerm[162] + tmpFx[195]*tmpObjSEndTerm[176] + tmpFx[211]*tmpObjSEndTerm[190];
tmpQN2[51] = + tmpFx[3]*tmpObjSEndTerm[9] + tmpFx[19]*tmpObjSEndTerm[23] + tmpFx[35]*tmpObjSEndTerm[37] + tmpFx[51]*tmpObjSEndTerm[51] + tmpFx[67]*tmpObjSEndTerm[65] + tmpFx[83]*tmpObjSEndTerm[79] + tmpFx[99]*tmpObjSEndTerm[93] + tmpFx[115]*tmpObjSEndTerm[107] + tmpFx[131]*tmpObjSEndTerm[121] + tmpFx[147]*tmpObjSEndTerm[135] + tmpFx[163]*tmpObjSEndTerm[149] + tmpFx[179]*tmpObjSEndTerm[163] + tmpFx[195]*tmpObjSEndTerm[177] + tmpFx[211]*tmpObjSEndTerm[191];
tmpQN2[52] = + tmpFx[3]*tmpObjSEndTerm[10] + tmpFx[19]*tmpObjSEndTerm[24] + tmpFx[35]*tmpObjSEndTerm[38] + tmpFx[51]*tmpObjSEndTerm[52] + tmpFx[67]*tmpObjSEndTerm[66] + tmpFx[83]*tmpObjSEndTerm[80] + tmpFx[99]*tmpObjSEndTerm[94] + tmpFx[115]*tmpObjSEndTerm[108] + tmpFx[131]*tmpObjSEndTerm[122] + tmpFx[147]*tmpObjSEndTerm[136] + tmpFx[163]*tmpObjSEndTerm[150] + tmpFx[179]*tmpObjSEndTerm[164] + tmpFx[195]*tmpObjSEndTerm[178] + tmpFx[211]*tmpObjSEndTerm[192];
tmpQN2[53] = + tmpFx[3]*tmpObjSEndTerm[11] + tmpFx[19]*tmpObjSEndTerm[25] + tmpFx[35]*tmpObjSEndTerm[39] + tmpFx[51]*tmpObjSEndTerm[53] + tmpFx[67]*tmpObjSEndTerm[67] + tmpFx[83]*tmpObjSEndTerm[81] + tmpFx[99]*tmpObjSEndTerm[95] + tmpFx[115]*tmpObjSEndTerm[109] + tmpFx[131]*tmpObjSEndTerm[123] + tmpFx[147]*tmpObjSEndTerm[137] + tmpFx[163]*tmpObjSEndTerm[151] + tmpFx[179]*tmpObjSEndTerm[165] + tmpFx[195]*tmpObjSEndTerm[179] + tmpFx[211]*tmpObjSEndTerm[193];
tmpQN2[54] = + tmpFx[3]*tmpObjSEndTerm[12] + tmpFx[19]*tmpObjSEndTerm[26] + tmpFx[35]*tmpObjSEndTerm[40] + tmpFx[51]*tmpObjSEndTerm[54] + tmpFx[67]*tmpObjSEndTerm[68] + tmpFx[83]*tmpObjSEndTerm[82] + tmpFx[99]*tmpObjSEndTerm[96] + tmpFx[115]*tmpObjSEndTerm[110] + tmpFx[131]*tmpObjSEndTerm[124] + tmpFx[147]*tmpObjSEndTerm[138] + tmpFx[163]*tmpObjSEndTerm[152] + tmpFx[179]*tmpObjSEndTerm[166] + tmpFx[195]*tmpObjSEndTerm[180] + tmpFx[211]*tmpObjSEndTerm[194];
tmpQN2[55] = + tmpFx[3]*tmpObjSEndTerm[13] + tmpFx[19]*tmpObjSEndTerm[27] + tmpFx[35]*tmpObjSEndTerm[41] + tmpFx[51]*tmpObjSEndTerm[55] + tmpFx[67]*tmpObjSEndTerm[69] + tmpFx[83]*tmpObjSEndTerm[83] + tmpFx[99]*tmpObjSEndTerm[97] + tmpFx[115]*tmpObjSEndTerm[111] + tmpFx[131]*tmpObjSEndTerm[125] + tmpFx[147]*tmpObjSEndTerm[139] + tmpFx[163]*tmpObjSEndTerm[153] + tmpFx[179]*tmpObjSEndTerm[167] + tmpFx[195]*tmpObjSEndTerm[181] + tmpFx[211]*tmpObjSEndTerm[195];
tmpQN2[56] = + tmpFx[4]*tmpObjSEndTerm[0] + tmpFx[20]*tmpObjSEndTerm[14] + tmpFx[36]*tmpObjSEndTerm[28] + tmpFx[52]*tmpObjSEndTerm[42] + tmpFx[68]*tmpObjSEndTerm[56] + tmpFx[84]*tmpObjSEndTerm[70] + tmpFx[100]*tmpObjSEndTerm[84] + tmpFx[116]*tmpObjSEndTerm[98] + tmpFx[132]*tmpObjSEndTerm[112] + tmpFx[148]*tmpObjSEndTerm[126] + tmpFx[164]*tmpObjSEndTerm[140] + tmpFx[180]*tmpObjSEndTerm[154] + tmpFx[196]*tmpObjSEndTerm[168] + tmpFx[212]*tmpObjSEndTerm[182];
tmpQN2[57] = + tmpFx[4]*tmpObjSEndTerm[1] + tmpFx[20]*tmpObjSEndTerm[15] + tmpFx[36]*tmpObjSEndTerm[29] + tmpFx[52]*tmpObjSEndTerm[43] + tmpFx[68]*tmpObjSEndTerm[57] + tmpFx[84]*tmpObjSEndTerm[71] + tmpFx[100]*tmpObjSEndTerm[85] + tmpFx[116]*tmpObjSEndTerm[99] + tmpFx[132]*tmpObjSEndTerm[113] + tmpFx[148]*tmpObjSEndTerm[127] + tmpFx[164]*tmpObjSEndTerm[141] + tmpFx[180]*tmpObjSEndTerm[155] + tmpFx[196]*tmpObjSEndTerm[169] + tmpFx[212]*tmpObjSEndTerm[183];
tmpQN2[58] = + tmpFx[4]*tmpObjSEndTerm[2] + tmpFx[20]*tmpObjSEndTerm[16] + tmpFx[36]*tmpObjSEndTerm[30] + tmpFx[52]*tmpObjSEndTerm[44] + tmpFx[68]*tmpObjSEndTerm[58] + tmpFx[84]*tmpObjSEndTerm[72] + tmpFx[100]*tmpObjSEndTerm[86] + tmpFx[116]*tmpObjSEndTerm[100] + tmpFx[132]*tmpObjSEndTerm[114] + tmpFx[148]*tmpObjSEndTerm[128] + tmpFx[164]*tmpObjSEndTerm[142] + tmpFx[180]*tmpObjSEndTerm[156] + tmpFx[196]*tmpObjSEndTerm[170] + tmpFx[212]*tmpObjSEndTerm[184];
tmpQN2[59] = + tmpFx[4]*tmpObjSEndTerm[3] + tmpFx[20]*tmpObjSEndTerm[17] + tmpFx[36]*tmpObjSEndTerm[31] + tmpFx[52]*tmpObjSEndTerm[45] + tmpFx[68]*tmpObjSEndTerm[59] + tmpFx[84]*tmpObjSEndTerm[73] + tmpFx[100]*tmpObjSEndTerm[87] + tmpFx[116]*tmpObjSEndTerm[101] + tmpFx[132]*tmpObjSEndTerm[115] + tmpFx[148]*tmpObjSEndTerm[129] + tmpFx[164]*tmpObjSEndTerm[143] + tmpFx[180]*tmpObjSEndTerm[157] + tmpFx[196]*tmpObjSEndTerm[171] + tmpFx[212]*tmpObjSEndTerm[185];
tmpQN2[60] = + tmpFx[4]*tmpObjSEndTerm[4] + tmpFx[20]*tmpObjSEndTerm[18] + tmpFx[36]*tmpObjSEndTerm[32] + tmpFx[52]*tmpObjSEndTerm[46] + tmpFx[68]*tmpObjSEndTerm[60] + tmpFx[84]*tmpObjSEndTerm[74] + tmpFx[100]*tmpObjSEndTerm[88] + tmpFx[116]*tmpObjSEndTerm[102] + tmpFx[132]*tmpObjSEndTerm[116] + tmpFx[148]*tmpObjSEndTerm[130] + tmpFx[164]*tmpObjSEndTerm[144] + tmpFx[180]*tmpObjSEndTerm[158] + tmpFx[196]*tmpObjSEndTerm[172] + tmpFx[212]*tmpObjSEndTerm[186];
tmpQN2[61] = + tmpFx[4]*tmpObjSEndTerm[5] + tmpFx[20]*tmpObjSEndTerm[19] + tmpFx[36]*tmpObjSEndTerm[33] + tmpFx[52]*tmpObjSEndTerm[47] + tmpFx[68]*tmpObjSEndTerm[61] + tmpFx[84]*tmpObjSEndTerm[75] + tmpFx[100]*tmpObjSEndTerm[89] + tmpFx[116]*tmpObjSEndTerm[103] + tmpFx[132]*tmpObjSEndTerm[117] + tmpFx[148]*tmpObjSEndTerm[131] + tmpFx[164]*tmpObjSEndTerm[145] + tmpFx[180]*tmpObjSEndTerm[159] + tmpFx[196]*tmpObjSEndTerm[173] + tmpFx[212]*tmpObjSEndTerm[187];
tmpQN2[62] = + tmpFx[4]*tmpObjSEndTerm[6] + tmpFx[20]*tmpObjSEndTerm[20] + tmpFx[36]*tmpObjSEndTerm[34] + tmpFx[52]*tmpObjSEndTerm[48] + tmpFx[68]*tmpObjSEndTerm[62] + tmpFx[84]*tmpObjSEndTerm[76] + tmpFx[100]*tmpObjSEndTerm[90] + tmpFx[116]*tmpObjSEndTerm[104] + tmpFx[132]*tmpObjSEndTerm[118] + tmpFx[148]*tmpObjSEndTerm[132] + tmpFx[164]*tmpObjSEndTerm[146] + tmpFx[180]*tmpObjSEndTerm[160] + tmpFx[196]*tmpObjSEndTerm[174] + tmpFx[212]*tmpObjSEndTerm[188];
tmpQN2[63] = + tmpFx[4]*tmpObjSEndTerm[7] + tmpFx[20]*tmpObjSEndTerm[21] + tmpFx[36]*tmpObjSEndTerm[35] + tmpFx[52]*tmpObjSEndTerm[49] + tmpFx[68]*tmpObjSEndTerm[63] + tmpFx[84]*tmpObjSEndTerm[77] + tmpFx[100]*tmpObjSEndTerm[91] + tmpFx[116]*tmpObjSEndTerm[105] + tmpFx[132]*tmpObjSEndTerm[119] + tmpFx[148]*tmpObjSEndTerm[133] + tmpFx[164]*tmpObjSEndTerm[147] + tmpFx[180]*tmpObjSEndTerm[161] + tmpFx[196]*tmpObjSEndTerm[175] + tmpFx[212]*tmpObjSEndTerm[189];
tmpQN2[64] = + tmpFx[4]*tmpObjSEndTerm[8] + tmpFx[20]*tmpObjSEndTerm[22] + tmpFx[36]*tmpObjSEndTerm[36] + tmpFx[52]*tmpObjSEndTerm[50] + tmpFx[68]*tmpObjSEndTerm[64] + tmpFx[84]*tmpObjSEndTerm[78] + tmpFx[100]*tmpObjSEndTerm[92] + tmpFx[116]*tmpObjSEndTerm[106] + tmpFx[132]*tmpObjSEndTerm[120] + tmpFx[148]*tmpObjSEndTerm[134] + tmpFx[164]*tmpObjSEndTerm[148] + tmpFx[180]*tmpObjSEndTerm[162] + tmpFx[196]*tmpObjSEndTerm[176] + tmpFx[212]*tmpObjSEndTerm[190];
tmpQN2[65] = + tmpFx[4]*tmpObjSEndTerm[9] + tmpFx[20]*tmpObjSEndTerm[23] + tmpFx[36]*tmpObjSEndTerm[37] + tmpFx[52]*tmpObjSEndTerm[51] + tmpFx[68]*tmpObjSEndTerm[65] + tmpFx[84]*tmpObjSEndTerm[79] + tmpFx[100]*tmpObjSEndTerm[93] + tmpFx[116]*tmpObjSEndTerm[107] + tmpFx[132]*tmpObjSEndTerm[121] + tmpFx[148]*tmpObjSEndTerm[135] + tmpFx[164]*tmpObjSEndTerm[149] + tmpFx[180]*tmpObjSEndTerm[163] + tmpFx[196]*tmpObjSEndTerm[177] + tmpFx[212]*tmpObjSEndTerm[191];
tmpQN2[66] = + tmpFx[4]*tmpObjSEndTerm[10] + tmpFx[20]*tmpObjSEndTerm[24] + tmpFx[36]*tmpObjSEndTerm[38] + tmpFx[52]*tmpObjSEndTerm[52] + tmpFx[68]*tmpObjSEndTerm[66] + tmpFx[84]*tmpObjSEndTerm[80] + tmpFx[100]*tmpObjSEndTerm[94] + tmpFx[116]*tmpObjSEndTerm[108] + tmpFx[132]*tmpObjSEndTerm[122] + tmpFx[148]*tmpObjSEndTerm[136] + tmpFx[164]*tmpObjSEndTerm[150] + tmpFx[180]*tmpObjSEndTerm[164] + tmpFx[196]*tmpObjSEndTerm[178] + tmpFx[212]*tmpObjSEndTerm[192];
tmpQN2[67] = + tmpFx[4]*tmpObjSEndTerm[11] + tmpFx[20]*tmpObjSEndTerm[25] + tmpFx[36]*tmpObjSEndTerm[39] + tmpFx[52]*tmpObjSEndTerm[53] + tmpFx[68]*tmpObjSEndTerm[67] + tmpFx[84]*tmpObjSEndTerm[81] + tmpFx[100]*tmpObjSEndTerm[95] + tmpFx[116]*tmpObjSEndTerm[109] + tmpFx[132]*tmpObjSEndTerm[123] + tmpFx[148]*tmpObjSEndTerm[137] + tmpFx[164]*tmpObjSEndTerm[151] + tmpFx[180]*tmpObjSEndTerm[165] + tmpFx[196]*tmpObjSEndTerm[179] + tmpFx[212]*tmpObjSEndTerm[193];
tmpQN2[68] = + tmpFx[4]*tmpObjSEndTerm[12] + tmpFx[20]*tmpObjSEndTerm[26] + tmpFx[36]*tmpObjSEndTerm[40] + tmpFx[52]*tmpObjSEndTerm[54] + tmpFx[68]*tmpObjSEndTerm[68] + tmpFx[84]*tmpObjSEndTerm[82] + tmpFx[100]*tmpObjSEndTerm[96] + tmpFx[116]*tmpObjSEndTerm[110] + tmpFx[132]*tmpObjSEndTerm[124] + tmpFx[148]*tmpObjSEndTerm[138] + tmpFx[164]*tmpObjSEndTerm[152] + tmpFx[180]*tmpObjSEndTerm[166] + tmpFx[196]*tmpObjSEndTerm[180] + tmpFx[212]*tmpObjSEndTerm[194];
tmpQN2[69] = + tmpFx[4]*tmpObjSEndTerm[13] + tmpFx[20]*tmpObjSEndTerm[27] + tmpFx[36]*tmpObjSEndTerm[41] + tmpFx[52]*tmpObjSEndTerm[55] + tmpFx[68]*tmpObjSEndTerm[69] + tmpFx[84]*tmpObjSEndTerm[83] + tmpFx[100]*tmpObjSEndTerm[97] + tmpFx[116]*tmpObjSEndTerm[111] + tmpFx[132]*tmpObjSEndTerm[125] + tmpFx[148]*tmpObjSEndTerm[139] + tmpFx[164]*tmpObjSEndTerm[153] + tmpFx[180]*tmpObjSEndTerm[167] + tmpFx[196]*tmpObjSEndTerm[181] + tmpFx[212]*tmpObjSEndTerm[195];
tmpQN2[70] = + tmpFx[5]*tmpObjSEndTerm[0] + tmpFx[21]*tmpObjSEndTerm[14] + tmpFx[37]*tmpObjSEndTerm[28] + tmpFx[53]*tmpObjSEndTerm[42] + tmpFx[69]*tmpObjSEndTerm[56] + tmpFx[85]*tmpObjSEndTerm[70] + tmpFx[101]*tmpObjSEndTerm[84] + tmpFx[117]*tmpObjSEndTerm[98] + tmpFx[133]*tmpObjSEndTerm[112] + tmpFx[149]*tmpObjSEndTerm[126] + tmpFx[165]*tmpObjSEndTerm[140] + tmpFx[181]*tmpObjSEndTerm[154] + tmpFx[197]*tmpObjSEndTerm[168] + tmpFx[213]*tmpObjSEndTerm[182];
tmpQN2[71] = + tmpFx[5]*tmpObjSEndTerm[1] + tmpFx[21]*tmpObjSEndTerm[15] + tmpFx[37]*tmpObjSEndTerm[29] + tmpFx[53]*tmpObjSEndTerm[43] + tmpFx[69]*tmpObjSEndTerm[57] + tmpFx[85]*tmpObjSEndTerm[71] + tmpFx[101]*tmpObjSEndTerm[85] + tmpFx[117]*tmpObjSEndTerm[99] + tmpFx[133]*tmpObjSEndTerm[113] + tmpFx[149]*tmpObjSEndTerm[127] + tmpFx[165]*tmpObjSEndTerm[141] + tmpFx[181]*tmpObjSEndTerm[155] + tmpFx[197]*tmpObjSEndTerm[169] + tmpFx[213]*tmpObjSEndTerm[183];
tmpQN2[72] = + tmpFx[5]*tmpObjSEndTerm[2] + tmpFx[21]*tmpObjSEndTerm[16] + tmpFx[37]*tmpObjSEndTerm[30] + tmpFx[53]*tmpObjSEndTerm[44] + tmpFx[69]*tmpObjSEndTerm[58] + tmpFx[85]*tmpObjSEndTerm[72] + tmpFx[101]*tmpObjSEndTerm[86] + tmpFx[117]*tmpObjSEndTerm[100] + tmpFx[133]*tmpObjSEndTerm[114] + tmpFx[149]*tmpObjSEndTerm[128] + tmpFx[165]*tmpObjSEndTerm[142] + tmpFx[181]*tmpObjSEndTerm[156] + tmpFx[197]*tmpObjSEndTerm[170] + tmpFx[213]*tmpObjSEndTerm[184];
tmpQN2[73] = + tmpFx[5]*tmpObjSEndTerm[3] + tmpFx[21]*tmpObjSEndTerm[17] + tmpFx[37]*tmpObjSEndTerm[31] + tmpFx[53]*tmpObjSEndTerm[45] + tmpFx[69]*tmpObjSEndTerm[59] + tmpFx[85]*tmpObjSEndTerm[73] + tmpFx[101]*tmpObjSEndTerm[87] + tmpFx[117]*tmpObjSEndTerm[101] + tmpFx[133]*tmpObjSEndTerm[115] + tmpFx[149]*tmpObjSEndTerm[129] + tmpFx[165]*tmpObjSEndTerm[143] + tmpFx[181]*tmpObjSEndTerm[157] + tmpFx[197]*tmpObjSEndTerm[171] + tmpFx[213]*tmpObjSEndTerm[185];
tmpQN2[74] = + tmpFx[5]*tmpObjSEndTerm[4] + tmpFx[21]*tmpObjSEndTerm[18] + tmpFx[37]*tmpObjSEndTerm[32] + tmpFx[53]*tmpObjSEndTerm[46] + tmpFx[69]*tmpObjSEndTerm[60] + tmpFx[85]*tmpObjSEndTerm[74] + tmpFx[101]*tmpObjSEndTerm[88] + tmpFx[117]*tmpObjSEndTerm[102] + tmpFx[133]*tmpObjSEndTerm[116] + tmpFx[149]*tmpObjSEndTerm[130] + tmpFx[165]*tmpObjSEndTerm[144] + tmpFx[181]*tmpObjSEndTerm[158] + tmpFx[197]*tmpObjSEndTerm[172] + tmpFx[213]*tmpObjSEndTerm[186];
tmpQN2[75] = + tmpFx[5]*tmpObjSEndTerm[5] + tmpFx[21]*tmpObjSEndTerm[19] + tmpFx[37]*tmpObjSEndTerm[33] + tmpFx[53]*tmpObjSEndTerm[47] + tmpFx[69]*tmpObjSEndTerm[61] + tmpFx[85]*tmpObjSEndTerm[75] + tmpFx[101]*tmpObjSEndTerm[89] + tmpFx[117]*tmpObjSEndTerm[103] + tmpFx[133]*tmpObjSEndTerm[117] + tmpFx[149]*tmpObjSEndTerm[131] + tmpFx[165]*tmpObjSEndTerm[145] + tmpFx[181]*tmpObjSEndTerm[159] + tmpFx[197]*tmpObjSEndTerm[173] + tmpFx[213]*tmpObjSEndTerm[187];
tmpQN2[76] = + tmpFx[5]*tmpObjSEndTerm[6] + tmpFx[21]*tmpObjSEndTerm[20] + tmpFx[37]*tmpObjSEndTerm[34] + tmpFx[53]*tmpObjSEndTerm[48] + tmpFx[69]*tmpObjSEndTerm[62] + tmpFx[85]*tmpObjSEndTerm[76] + tmpFx[101]*tmpObjSEndTerm[90] + tmpFx[117]*tmpObjSEndTerm[104] + tmpFx[133]*tmpObjSEndTerm[118] + tmpFx[149]*tmpObjSEndTerm[132] + tmpFx[165]*tmpObjSEndTerm[146] + tmpFx[181]*tmpObjSEndTerm[160] + tmpFx[197]*tmpObjSEndTerm[174] + tmpFx[213]*tmpObjSEndTerm[188];
tmpQN2[77] = + tmpFx[5]*tmpObjSEndTerm[7] + tmpFx[21]*tmpObjSEndTerm[21] + tmpFx[37]*tmpObjSEndTerm[35] + tmpFx[53]*tmpObjSEndTerm[49] + tmpFx[69]*tmpObjSEndTerm[63] + tmpFx[85]*tmpObjSEndTerm[77] + tmpFx[101]*tmpObjSEndTerm[91] + tmpFx[117]*tmpObjSEndTerm[105] + tmpFx[133]*tmpObjSEndTerm[119] + tmpFx[149]*tmpObjSEndTerm[133] + tmpFx[165]*tmpObjSEndTerm[147] + tmpFx[181]*tmpObjSEndTerm[161] + tmpFx[197]*tmpObjSEndTerm[175] + tmpFx[213]*tmpObjSEndTerm[189];
tmpQN2[78] = + tmpFx[5]*tmpObjSEndTerm[8] + tmpFx[21]*tmpObjSEndTerm[22] + tmpFx[37]*tmpObjSEndTerm[36] + tmpFx[53]*tmpObjSEndTerm[50] + tmpFx[69]*tmpObjSEndTerm[64] + tmpFx[85]*tmpObjSEndTerm[78] + tmpFx[101]*tmpObjSEndTerm[92] + tmpFx[117]*tmpObjSEndTerm[106] + tmpFx[133]*tmpObjSEndTerm[120] + tmpFx[149]*tmpObjSEndTerm[134] + tmpFx[165]*tmpObjSEndTerm[148] + tmpFx[181]*tmpObjSEndTerm[162] + tmpFx[197]*tmpObjSEndTerm[176] + tmpFx[213]*tmpObjSEndTerm[190];
tmpQN2[79] = + tmpFx[5]*tmpObjSEndTerm[9] + tmpFx[21]*tmpObjSEndTerm[23] + tmpFx[37]*tmpObjSEndTerm[37] + tmpFx[53]*tmpObjSEndTerm[51] + tmpFx[69]*tmpObjSEndTerm[65] + tmpFx[85]*tmpObjSEndTerm[79] + tmpFx[101]*tmpObjSEndTerm[93] + tmpFx[117]*tmpObjSEndTerm[107] + tmpFx[133]*tmpObjSEndTerm[121] + tmpFx[149]*tmpObjSEndTerm[135] + tmpFx[165]*tmpObjSEndTerm[149] + tmpFx[181]*tmpObjSEndTerm[163] + tmpFx[197]*tmpObjSEndTerm[177] + tmpFx[213]*tmpObjSEndTerm[191];
tmpQN2[80] = + tmpFx[5]*tmpObjSEndTerm[10] + tmpFx[21]*tmpObjSEndTerm[24] + tmpFx[37]*tmpObjSEndTerm[38] + tmpFx[53]*tmpObjSEndTerm[52] + tmpFx[69]*tmpObjSEndTerm[66] + tmpFx[85]*tmpObjSEndTerm[80] + tmpFx[101]*tmpObjSEndTerm[94] + tmpFx[117]*tmpObjSEndTerm[108] + tmpFx[133]*tmpObjSEndTerm[122] + tmpFx[149]*tmpObjSEndTerm[136] + tmpFx[165]*tmpObjSEndTerm[150] + tmpFx[181]*tmpObjSEndTerm[164] + tmpFx[197]*tmpObjSEndTerm[178] + tmpFx[213]*tmpObjSEndTerm[192];
tmpQN2[81] = + tmpFx[5]*tmpObjSEndTerm[11] + tmpFx[21]*tmpObjSEndTerm[25] + tmpFx[37]*tmpObjSEndTerm[39] + tmpFx[53]*tmpObjSEndTerm[53] + tmpFx[69]*tmpObjSEndTerm[67] + tmpFx[85]*tmpObjSEndTerm[81] + tmpFx[101]*tmpObjSEndTerm[95] + tmpFx[117]*tmpObjSEndTerm[109] + tmpFx[133]*tmpObjSEndTerm[123] + tmpFx[149]*tmpObjSEndTerm[137] + tmpFx[165]*tmpObjSEndTerm[151] + tmpFx[181]*tmpObjSEndTerm[165] + tmpFx[197]*tmpObjSEndTerm[179] + tmpFx[213]*tmpObjSEndTerm[193];
tmpQN2[82] = + tmpFx[5]*tmpObjSEndTerm[12] + tmpFx[21]*tmpObjSEndTerm[26] + tmpFx[37]*tmpObjSEndTerm[40] + tmpFx[53]*tmpObjSEndTerm[54] + tmpFx[69]*tmpObjSEndTerm[68] + tmpFx[85]*tmpObjSEndTerm[82] + tmpFx[101]*tmpObjSEndTerm[96] + tmpFx[117]*tmpObjSEndTerm[110] + tmpFx[133]*tmpObjSEndTerm[124] + tmpFx[149]*tmpObjSEndTerm[138] + tmpFx[165]*tmpObjSEndTerm[152] + tmpFx[181]*tmpObjSEndTerm[166] + tmpFx[197]*tmpObjSEndTerm[180] + tmpFx[213]*tmpObjSEndTerm[194];
tmpQN2[83] = + tmpFx[5]*tmpObjSEndTerm[13] + tmpFx[21]*tmpObjSEndTerm[27] + tmpFx[37]*tmpObjSEndTerm[41] + tmpFx[53]*tmpObjSEndTerm[55] + tmpFx[69]*tmpObjSEndTerm[69] + tmpFx[85]*tmpObjSEndTerm[83] + tmpFx[101]*tmpObjSEndTerm[97] + tmpFx[117]*tmpObjSEndTerm[111] + tmpFx[133]*tmpObjSEndTerm[125] + tmpFx[149]*tmpObjSEndTerm[139] + tmpFx[165]*tmpObjSEndTerm[153] + tmpFx[181]*tmpObjSEndTerm[167] + tmpFx[197]*tmpObjSEndTerm[181] + tmpFx[213]*tmpObjSEndTerm[195];
tmpQN2[84] = + tmpFx[6]*tmpObjSEndTerm[0] + tmpFx[22]*tmpObjSEndTerm[14] + tmpFx[38]*tmpObjSEndTerm[28] + tmpFx[54]*tmpObjSEndTerm[42] + tmpFx[70]*tmpObjSEndTerm[56] + tmpFx[86]*tmpObjSEndTerm[70] + tmpFx[102]*tmpObjSEndTerm[84] + tmpFx[118]*tmpObjSEndTerm[98] + tmpFx[134]*tmpObjSEndTerm[112] + tmpFx[150]*tmpObjSEndTerm[126] + tmpFx[166]*tmpObjSEndTerm[140] + tmpFx[182]*tmpObjSEndTerm[154] + tmpFx[198]*tmpObjSEndTerm[168] + tmpFx[214]*tmpObjSEndTerm[182];
tmpQN2[85] = + tmpFx[6]*tmpObjSEndTerm[1] + tmpFx[22]*tmpObjSEndTerm[15] + tmpFx[38]*tmpObjSEndTerm[29] + tmpFx[54]*tmpObjSEndTerm[43] + tmpFx[70]*tmpObjSEndTerm[57] + tmpFx[86]*tmpObjSEndTerm[71] + tmpFx[102]*tmpObjSEndTerm[85] + tmpFx[118]*tmpObjSEndTerm[99] + tmpFx[134]*tmpObjSEndTerm[113] + tmpFx[150]*tmpObjSEndTerm[127] + tmpFx[166]*tmpObjSEndTerm[141] + tmpFx[182]*tmpObjSEndTerm[155] + tmpFx[198]*tmpObjSEndTerm[169] + tmpFx[214]*tmpObjSEndTerm[183];
tmpQN2[86] = + tmpFx[6]*tmpObjSEndTerm[2] + tmpFx[22]*tmpObjSEndTerm[16] + tmpFx[38]*tmpObjSEndTerm[30] + tmpFx[54]*tmpObjSEndTerm[44] + tmpFx[70]*tmpObjSEndTerm[58] + tmpFx[86]*tmpObjSEndTerm[72] + tmpFx[102]*tmpObjSEndTerm[86] + tmpFx[118]*tmpObjSEndTerm[100] + tmpFx[134]*tmpObjSEndTerm[114] + tmpFx[150]*tmpObjSEndTerm[128] + tmpFx[166]*tmpObjSEndTerm[142] + tmpFx[182]*tmpObjSEndTerm[156] + tmpFx[198]*tmpObjSEndTerm[170] + tmpFx[214]*tmpObjSEndTerm[184];
tmpQN2[87] = + tmpFx[6]*tmpObjSEndTerm[3] + tmpFx[22]*tmpObjSEndTerm[17] + tmpFx[38]*tmpObjSEndTerm[31] + tmpFx[54]*tmpObjSEndTerm[45] + tmpFx[70]*tmpObjSEndTerm[59] + tmpFx[86]*tmpObjSEndTerm[73] + tmpFx[102]*tmpObjSEndTerm[87] + tmpFx[118]*tmpObjSEndTerm[101] + tmpFx[134]*tmpObjSEndTerm[115] + tmpFx[150]*tmpObjSEndTerm[129] + tmpFx[166]*tmpObjSEndTerm[143] + tmpFx[182]*tmpObjSEndTerm[157] + tmpFx[198]*tmpObjSEndTerm[171] + tmpFx[214]*tmpObjSEndTerm[185];
tmpQN2[88] = + tmpFx[6]*tmpObjSEndTerm[4] + tmpFx[22]*tmpObjSEndTerm[18] + tmpFx[38]*tmpObjSEndTerm[32] + tmpFx[54]*tmpObjSEndTerm[46] + tmpFx[70]*tmpObjSEndTerm[60] + tmpFx[86]*tmpObjSEndTerm[74] + tmpFx[102]*tmpObjSEndTerm[88] + tmpFx[118]*tmpObjSEndTerm[102] + tmpFx[134]*tmpObjSEndTerm[116] + tmpFx[150]*tmpObjSEndTerm[130] + tmpFx[166]*tmpObjSEndTerm[144] + tmpFx[182]*tmpObjSEndTerm[158] + tmpFx[198]*tmpObjSEndTerm[172] + tmpFx[214]*tmpObjSEndTerm[186];
tmpQN2[89] = + tmpFx[6]*tmpObjSEndTerm[5] + tmpFx[22]*tmpObjSEndTerm[19] + tmpFx[38]*tmpObjSEndTerm[33] + tmpFx[54]*tmpObjSEndTerm[47] + tmpFx[70]*tmpObjSEndTerm[61] + tmpFx[86]*tmpObjSEndTerm[75] + tmpFx[102]*tmpObjSEndTerm[89] + tmpFx[118]*tmpObjSEndTerm[103] + tmpFx[134]*tmpObjSEndTerm[117] + tmpFx[150]*tmpObjSEndTerm[131] + tmpFx[166]*tmpObjSEndTerm[145] + tmpFx[182]*tmpObjSEndTerm[159] + tmpFx[198]*tmpObjSEndTerm[173] + tmpFx[214]*tmpObjSEndTerm[187];
tmpQN2[90] = + tmpFx[6]*tmpObjSEndTerm[6] + tmpFx[22]*tmpObjSEndTerm[20] + tmpFx[38]*tmpObjSEndTerm[34] + tmpFx[54]*tmpObjSEndTerm[48] + tmpFx[70]*tmpObjSEndTerm[62] + tmpFx[86]*tmpObjSEndTerm[76] + tmpFx[102]*tmpObjSEndTerm[90] + tmpFx[118]*tmpObjSEndTerm[104] + tmpFx[134]*tmpObjSEndTerm[118] + tmpFx[150]*tmpObjSEndTerm[132] + tmpFx[166]*tmpObjSEndTerm[146] + tmpFx[182]*tmpObjSEndTerm[160] + tmpFx[198]*tmpObjSEndTerm[174] + tmpFx[214]*tmpObjSEndTerm[188];
tmpQN2[91] = + tmpFx[6]*tmpObjSEndTerm[7] + tmpFx[22]*tmpObjSEndTerm[21] + tmpFx[38]*tmpObjSEndTerm[35] + tmpFx[54]*tmpObjSEndTerm[49] + tmpFx[70]*tmpObjSEndTerm[63] + tmpFx[86]*tmpObjSEndTerm[77] + tmpFx[102]*tmpObjSEndTerm[91] + tmpFx[118]*tmpObjSEndTerm[105] + tmpFx[134]*tmpObjSEndTerm[119] + tmpFx[150]*tmpObjSEndTerm[133] + tmpFx[166]*tmpObjSEndTerm[147] + tmpFx[182]*tmpObjSEndTerm[161] + tmpFx[198]*tmpObjSEndTerm[175] + tmpFx[214]*tmpObjSEndTerm[189];
tmpQN2[92] = + tmpFx[6]*tmpObjSEndTerm[8] + tmpFx[22]*tmpObjSEndTerm[22] + tmpFx[38]*tmpObjSEndTerm[36] + tmpFx[54]*tmpObjSEndTerm[50] + tmpFx[70]*tmpObjSEndTerm[64] + tmpFx[86]*tmpObjSEndTerm[78] + tmpFx[102]*tmpObjSEndTerm[92] + tmpFx[118]*tmpObjSEndTerm[106] + tmpFx[134]*tmpObjSEndTerm[120] + tmpFx[150]*tmpObjSEndTerm[134] + tmpFx[166]*tmpObjSEndTerm[148] + tmpFx[182]*tmpObjSEndTerm[162] + tmpFx[198]*tmpObjSEndTerm[176] + tmpFx[214]*tmpObjSEndTerm[190];
tmpQN2[93] = + tmpFx[6]*tmpObjSEndTerm[9] + tmpFx[22]*tmpObjSEndTerm[23] + tmpFx[38]*tmpObjSEndTerm[37] + tmpFx[54]*tmpObjSEndTerm[51] + tmpFx[70]*tmpObjSEndTerm[65] + tmpFx[86]*tmpObjSEndTerm[79] + tmpFx[102]*tmpObjSEndTerm[93] + tmpFx[118]*tmpObjSEndTerm[107] + tmpFx[134]*tmpObjSEndTerm[121] + tmpFx[150]*tmpObjSEndTerm[135] + tmpFx[166]*tmpObjSEndTerm[149] + tmpFx[182]*tmpObjSEndTerm[163] + tmpFx[198]*tmpObjSEndTerm[177] + tmpFx[214]*tmpObjSEndTerm[191];
tmpQN2[94] = + tmpFx[6]*tmpObjSEndTerm[10] + tmpFx[22]*tmpObjSEndTerm[24] + tmpFx[38]*tmpObjSEndTerm[38] + tmpFx[54]*tmpObjSEndTerm[52] + tmpFx[70]*tmpObjSEndTerm[66] + tmpFx[86]*tmpObjSEndTerm[80] + tmpFx[102]*tmpObjSEndTerm[94] + tmpFx[118]*tmpObjSEndTerm[108] + tmpFx[134]*tmpObjSEndTerm[122] + tmpFx[150]*tmpObjSEndTerm[136] + tmpFx[166]*tmpObjSEndTerm[150] + tmpFx[182]*tmpObjSEndTerm[164] + tmpFx[198]*tmpObjSEndTerm[178] + tmpFx[214]*tmpObjSEndTerm[192];
tmpQN2[95] = + tmpFx[6]*tmpObjSEndTerm[11] + tmpFx[22]*tmpObjSEndTerm[25] + tmpFx[38]*tmpObjSEndTerm[39] + tmpFx[54]*tmpObjSEndTerm[53] + tmpFx[70]*tmpObjSEndTerm[67] + tmpFx[86]*tmpObjSEndTerm[81] + tmpFx[102]*tmpObjSEndTerm[95] + tmpFx[118]*tmpObjSEndTerm[109] + tmpFx[134]*tmpObjSEndTerm[123] + tmpFx[150]*tmpObjSEndTerm[137] + tmpFx[166]*tmpObjSEndTerm[151] + tmpFx[182]*tmpObjSEndTerm[165] + tmpFx[198]*tmpObjSEndTerm[179] + tmpFx[214]*tmpObjSEndTerm[193];
tmpQN2[96] = + tmpFx[6]*tmpObjSEndTerm[12] + tmpFx[22]*tmpObjSEndTerm[26] + tmpFx[38]*tmpObjSEndTerm[40] + tmpFx[54]*tmpObjSEndTerm[54] + tmpFx[70]*tmpObjSEndTerm[68] + tmpFx[86]*tmpObjSEndTerm[82] + tmpFx[102]*tmpObjSEndTerm[96] + tmpFx[118]*tmpObjSEndTerm[110] + tmpFx[134]*tmpObjSEndTerm[124] + tmpFx[150]*tmpObjSEndTerm[138] + tmpFx[166]*tmpObjSEndTerm[152] + tmpFx[182]*tmpObjSEndTerm[166] + tmpFx[198]*tmpObjSEndTerm[180] + tmpFx[214]*tmpObjSEndTerm[194];
tmpQN2[97] = + tmpFx[6]*tmpObjSEndTerm[13] + tmpFx[22]*tmpObjSEndTerm[27] + tmpFx[38]*tmpObjSEndTerm[41] + tmpFx[54]*tmpObjSEndTerm[55] + tmpFx[70]*tmpObjSEndTerm[69] + tmpFx[86]*tmpObjSEndTerm[83] + tmpFx[102]*tmpObjSEndTerm[97] + tmpFx[118]*tmpObjSEndTerm[111] + tmpFx[134]*tmpObjSEndTerm[125] + tmpFx[150]*tmpObjSEndTerm[139] + tmpFx[166]*tmpObjSEndTerm[153] + tmpFx[182]*tmpObjSEndTerm[167] + tmpFx[198]*tmpObjSEndTerm[181] + tmpFx[214]*tmpObjSEndTerm[195];
tmpQN2[98] = + tmpFx[7]*tmpObjSEndTerm[0] + tmpFx[23]*tmpObjSEndTerm[14] + tmpFx[39]*tmpObjSEndTerm[28] + tmpFx[55]*tmpObjSEndTerm[42] + tmpFx[71]*tmpObjSEndTerm[56] + tmpFx[87]*tmpObjSEndTerm[70] + tmpFx[103]*tmpObjSEndTerm[84] + tmpFx[119]*tmpObjSEndTerm[98] + tmpFx[135]*tmpObjSEndTerm[112] + tmpFx[151]*tmpObjSEndTerm[126] + tmpFx[167]*tmpObjSEndTerm[140] + tmpFx[183]*tmpObjSEndTerm[154] + tmpFx[199]*tmpObjSEndTerm[168] + tmpFx[215]*tmpObjSEndTerm[182];
tmpQN2[99] = + tmpFx[7]*tmpObjSEndTerm[1] + tmpFx[23]*tmpObjSEndTerm[15] + tmpFx[39]*tmpObjSEndTerm[29] + tmpFx[55]*tmpObjSEndTerm[43] + tmpFx[71]*tmpObjSEndTerm[57] + tmpFx[87]*tmpObjSEndTerm[71] + tmpFx[103]*tmpObjSEndTerm[85] + tmpFx[119]*tmpObjSEndTerm[99] + tmpFx[135]*tmpObjSEndTerm[113] + tmpFx[151]*tmpObjSEndTerm[127] + tmpFx[167]*tmpObjSEndTerm[141] + tmpFx[183]*tmpObjSEndTerm[155] + tmpFx[199]*tmpObjSEndTerm[169] + tmpFx[215]*tmpObjSEndTerm[183];
tmpQN2[100] = + tmpFx[7]*tmpObjSEndTerm[2] + tmpFx[23]*tmpObjSEndTerm[16] + tmpFx[39]*tmpObjSEndTerm[30] + tmpFx[55]*tmpObjSEndTerm[44] + tmpFx[71]*tmpObjSEndTerm[58] + tmpFx[87]*tmpObjSEndTerm[72] + tmpFx[103]*tmpObjSEndTerm[86] + tmpFx[119]*tmpObjSEndTerm[100] + tmpFx[135]*tmpObjSEndTerm[114] + tmpFx[151]*tmpObjSEndTerm[128] + tmpFx[167]*tmpObjSEndTerm[142] + tmpFx[183]*tmpObjSEndTerm[156] + tmpFx[199]*tmpObjSEndTerm[170] + tmpFx[215]*tmpObjSEndTerm[184];
tmpQN2[101] = + tmpFx[7]*tmpObjSEndTerm[3] + tmpFx[23]*tmpObjSEndTerm[17] + tmpFx[39]*tmpObjSEndTerm[31] + tmpFx[55]*tmpObjSEndTerm[45] + tmpFx[71]*tmpObjSEndTerm[59] + tmpFx[87]*tmpObjSEndTerm[73] + tmpFx[103]*tmpObjSEndTerm[87] + tmpFx[119]*tmpObjSEndTerm[101] + tmpFx[135]*tmpObjSEndTerm[115] + tmpFx[151]*tmpObjSEndTerm[129] + tmpFx[167]*tmpObjSEndTerm[143] + tmpFx[183]*tmpObjSEndTerm[157] + tmpFx[199]*tmpObjSEndTerm[171] + tmpFx[215]*tmpObjSEndTerm[185];
tmpQN2[102] = + tmpFx[7]*tmpObjSEndTerm[4] + tmpFx[23]*tmpObjSEndTerm[18] + tmpFx[39]*tmpObjSEndTerm[32] + tmpFx[55]*tmpObjSEndTerm[46] + tmpFx[71]*tmpObjSEndTerm[60] + tmpFx[87]*tmpObjSEndTerm[74] + tmpFx[103]*tmpObjSEndTerm[88] + tmpFx[119]*tmpObjSEndTerm[102] + tmpFx[135]*tmpObjSEndTerm[116] + tmpFx[151]*tmpObjSEndTerm[130] + tmpFx[167]*tmpObjSEndTerm[144] + tmpFx[183]*tmpObjSEndTerm[158] + tmpFx[199]*tmpObjSEndTerm[172] + tmpFx[215]*tmpObjSEndTerm[186];
tmpQN2[103] = + tmpFx[7]*tmpObjSEndTerm[5] + tmpFx[23]*tmpObjSEndTerm[19] + tmpFx[39]*tmpObjSEndTerm[33] + tmpFx[55]*tmpObjSEndTerm[47] + tmpFx[71]*tmpObjSEndTerm[61] + tmpFx[87]*tmpObjSEndTerm[75] + tmpFx[103]*tmpObjSEndTerm[89] + tmpFx[119]*tmpObjSEndTerm[103] + tmpFx[135]*tmpObjSEndTerm[117] + tmpFx[151]*tmpObjSEndTerm[131] + tmpFx[167]*tmpObjSEndTerm[145] + tmpFx[183]*tmpObjSEndTerm[159] + tmpFx[199]*tmpObjSEndTerm[173] + tmpFx[215]*tmpObjSEndTerm[187];
tmpQN2[104] = + tmpFx[7]*tmpObjSEndTerm[6] + tmpFx[23]*tmpObjSEndTerm[20] + tmpFx[39]*tmpObjSEndTerm[34] + tmpFx[55]*tmpObjSEndTerm[48] + tmpFx[71]*tmpObjSEndTerm[62] + tmpFx[87]*tmpObjSEndTerm[76] + tmpFx[103]*tmpObjSEndTerm[90] + tmpFx[119]*tmpObjSEndTerm[104] + tmpFx[135]*tmpObjSEndTerm[118] + tmpFx[151]*tmpObjSEndTerm[132] + tmpFx[167]*tmpObjSEndTerm[146] + tmpFx[183]*tmpObjSEndTerm[160] + tmpFx[199]*tmpObjSEndTerm[174] + tmpFx[215]*tmpObjSEndTerm[188];
tmpQN2[105] = + tmpFx[7]*tmpObjSEndTerm[7] + tmpFx[23]*tmpObjSEndTerm[21] + tmpFx[39]*tmpObjSEndTerm[35] + tmpFx[55]*tmpObjSEndTerm[49] + tmpFx[71]*tmpObjSEndTerm[63] + tmpFx[87]*tmpObjSEndTerm[77] + tmpFx[103]*tmpObjSEndTerm[91] + tmpFx[119]*tmpObjSEndTerm[105] + tmpFx[135]*tmpObjSEndTerm[119] + tmpFx[151]*tmpObjSEndTerm[133] + tmpFx[167]*tmpObjSEndTerm[147] + tmpFx[183]*tmpObjSEndTerm[161] + tmpFx[199]*tmpObjSEndTerm[175] + tmpFx[215]*tmpObjSEndTerm[189];
tmpQN2[106] = + tmpFx[7]*tmpObjSEndTerm[8] + tmpFx[23]*tmpObjSEndTerm[22] + tmpFx[39]*tmpObjSEndTerm[36] + tmpFx[55]*tmpObjSEndTerm[50] + tmpFx[71]*tmpObjSEndTerm[64] + tmpFx[87]*tmpObjSEndTerm[78] + tmpFx[103]*tmpObjSEndTerm[92] + tmpFx[119]*tmpObjSEndTerm[106] + tmpFx[135]*tmpObjSEndTerm[120] + tmpFx[151]*tmpObjSEndTerm[134] + tmpFx[167]*tmpObjSEndTerm[148] + tmpFx[183]*tmpObjSEndTerm[162] + tmpFx[199]*tmpObjSEndTerm[176] + tmpFx[215]*tmpObjSEndTerm[190];
tmpQN2[107] = + tmpFx[7]*tmpObjSEndTerm[9] + tmpFx[23]*tmpObjSEndTerm[23] + tmpFx[39]*tmpObjSEndTerm[37] + tmpFx[55]*tmpObjSEndTerm[51] + tmpFx[71]*tmpObjSEndTerm[65] + tmpFx[87]*tmpObjSEndTerm[79] + tmpFx[103]*tmpObjSEndTerm[93] + tmpFx[119]*tmpObjSEndTerm[107] + tmpFx[135]*tmpObjSEndTerm[121] + tmpFx[151]*tmpObjSEndTerm[135] + tmpFx[167]*tmpObjSEndTerm[149] + tmpFx[183]*tmpObjSEndTerm[163] + tmpFx[199]*tmpObjSEndTerm[177] + tmpFx[215]*tmpObjSEndTerm[191];
tmpQN2[108] = + tmpFx[7]*tmpObjSEndTerm[10] + tmpFx[23]*tmpObjSEndTerm[24] + tmpFx[39]*tmpObjSEndTerm[38] + tmpFx[55]*tmpObjSEndTerm[52] + tmpFx[71]*tmpObjSEndTerm[66] + tmpFx[87]*tmpObjSEndTerm[80] + tmpFx[103]*tmpObjSEndTerm[94] + tmpFx[119]*tmpObjSEndTerm[108] + tmpFx[135]*tmpObjSEndTerm[122] + tmpFx[151]*tmpObjSEndTerm[136] + tmpFx[167]*tmpObjSEndTerm[150] + tmpFx[183]*tmpObjSEndTerm[164] + tmpFx[199]*tmpObjSEndTerm[178] + tmpFx[215]*tmpObjSEndTerm[192];
tmpQN2[109] = + tmpFx[7]*tmpObjSEndTerm[11] + tmpFx[23]*tmpObjSEndTerm[25] + tmpFx[39]*tmpObjSEndTerm[39] + tmpFx[55]*tmpObjSEndTerm[53] + tmpFx[71]*tmpObjSEndTerm[67] + tmpFx[87]*tmpObjSEndTerm[81] + tmpFx[103]*tmpObjSEndTerm[95] + tmpFx[119]*tmpObjSEndTerm[109] + tmpFx[135]*tmpObjSEndTerm[123] + tmpFx[151]*tmpObjSEndTerm[137] + tmpFx[167]*tmpObjSEndTerm[151] + tmpFx[183]*tmpObjSEndTerm[165] + tmpFx[199]*tmpObjSEndTerm[179] + tmpFx[215]*tmpObjSEndTerm[193];
tmpQN2[110] = + tmpFx[7]*tmpObjSEndTerm[12] + tmpFx[23]*tmpObjSEndTerm[26] + tmpFx[39]*tmpObjSEndTerm[40] + tmpFx[55]*tmpObjSEndTerm[54] + tmpFx[71]*tmpObjSEndTerm[68] + tmpFx[87]*tmpObjSEndTerm[82] + tmpFx[103]*tmpObjSEndTerm[96] + tmpFx[119]*tmpObjSEndTerm[110] + tmpFx[135]*tmpObjSEndTerm[124] + tmpFx[151]*tmpObjSEndTerm[138] + tmpFx[167]*tmpObjSEndTerm[152] + tmpFx[183]*tmpObjSEndTerm[166] + tmpFx[199]*tmpObjSEndTerm[180] + tmpFx[215]*tmpObjSEndTerm[194];
tmpQN2[111] = + tmpFx[7]*tmpObjSEndTerm[13] + tmpFx[23]*tmpObjSEndTerm[27] + tmpFx[39]*tmpObjSEndTerm[41] + tmpFx[55]*tmpObjSEndTerm[55] + tmpFx[71]*tmpObjSEndTerm[69] + tmpFx[87]*tmpObjSEndTerm[83] + tmpFx[103]*tmpObjSEndTerm[97] + tmpFx[119]*tmpObjSEndTerm[111] + tmpFx[135]*tmpObjSEndTerm[125] + tmpFx[151]*tmpObjSEndTerm[139] + tmpFx[167]*tmpObjSEndTerm[153] + tmpFx[183]*tmpObjSEndTerm[167] + tmpFx[199]*tmpObjSEndTerm[181] + tmpFx[215]*tmpObjSEndTerm[195];
tmpQN2[112] = + tmpFx[8]*tmpObjSEndTerm[0] + tmpFx[24]*tmpObjSEndTerm[14] + tmpFx[40]*tmpObjSEndTerm[28] + tmpFx[56]*tmpObjSEndTerm[42] + tmpFx[72]*tmpObjSEndTerm[56] + tmpFx[88]*tmpObjSEndTerm[70] + tmpFx[104]*tmpObjSEndTerm[84] + tmpFx[120]*tmpObjSEndTerm[98] + tmpFx[136]*tmpObjSEndTerm[112] + tmpFx[152]*tmpObjSEndTerm[126] + tmpFx[168]*tmpObjSEndTerm[140] + tmpFx[184]*tmpObjSEndTerm[154] + tmpFx[200]*tmpObjSEndTerm[168] + tmpFx[216]*tmpObjSEndTerm[182];
tmpQN2[113] = + tmpFx[8]*tmpObjSEndTerm[1] + tmpFx[24]*tmpObjSEndTerm[15] + tmpFx[40]*tmpObjSEndTerm[29] + tmpFx[56]*tmpObjSEndTerm[43] + tmpFx[72]*tmpObjSEndTerm[57] + tmpFx[88]*tmpObjSEndTerm[71] + tmpFx[104]*tmpObjSEndTerm[85] + tmpFx[120]*tmpObjSEndTerm[99] + tmpFx[136]*tmpObjSEndTerm[113] + tmpFx[152]*tmpObjSEndTerm[127] + tmpFx[168]*tmpObjSEndTerm[141] + tmpFx[184]*tmpObjSEndTerm[155] + tmpFx[200]*tmpObjSEndTerm[169] + tmpFx[216]*tmpObjSEndTerm[183];
tmpQN2[114] = + tmpFx[8]*tmpObjSEndTerm[2] + tmpFx[24]*tmpObjSEndTerm[16] + tmpFx[40]*tmpObjSEndTerm[30] + tmpFx[56]*tmpObjSEndTerm[44] + tmpFx[72]*tmpObjSEndTerm[58] + tmpFx[88]*tmpObjSEndTerm[72] + tmpFx[104]*tmpObjSEndTerm[86] + tmpFx[120]*tmpObjSEndTerm[100] + tmpFx[136]*tmpObjSEndTerm[114] + tmpFx[152]*tmpObjSEndTerm[128] + tmpFx[168]*tmpObjSEndTerm[142] + tmpFx[184]*tmpObjSEndTerm[156] + tmpFx[200]*tmpObjSEndTerm[170] + tmpFx[216]*tmpObjSEndTerm[184];
tmpQN2[115] = + tmpFx[8]*tmpObjSEndTerm[3] + tmpFx[24]*tmpObjSEndTerm[17] + tmpFx[40]*tmpObjSEndTerm[31] + tmpFx[56]*tmpObjSEndTerm[45] + tmpFx[72]*tmpObjSEndTerm[59] + tmpFx[88]*tmpObjSEndTerm[73] + tmpFx[104]*tmpObjSEndTerm[87] + tmpFx[120]*tmpObjSEndTerm[101] + tmpFx[136]*tmpObjSEndTerm[115] + tmpFx[152]*tmpObjSEndTerm[129] + tmpFx[168]*tmpObjSEndTerm[143] + tmpFx[184]*tmpObjSEndTerm[157] + tmpFx[200]*tmpObjSEndTerm[171] + tmpFx[216]*tmpObjSEndTerm[185];
tmpQN2[116] = + tmpFx[8]*tmpObjSEndTerm[4] + tmpFx[24]*tmpObjSEndTerm[18] + tmpFx[40]*tmpObjSEndTerm[32] + tmpFx[56]*tmpObjSEndTerm[46] + tmpFx[72]*tmpObjSEndTerm[60] + tmpFx[88]*tmpObjSEndTerm[74] + tmpFx[104]*tmpObjSEndTerm[88] + tmpFx[120]*tmpObjSEndTerm[102] + tmpFx[136]*tmpObjSEndTerm[116] + tmpFx[152]*tmpObjSEndTerm[130] + tmpFx[168]*tmpObjSEndTerm[144] + tmpFx[184]*tmpObjSEndTerm[158] + tmpFx[200]*tmpObjSEndTerm[172] + tmpFx[216]*tmpObjSEndTerm[186];
tmpQN2[117] = + tmpFx[8]*tmpObjSEndTerm[5] + tmpFx[24]*tmpObjSEndTerm[19] + tmpFx[40]*tmpObjSEndTerm[33] + tmpFx[56]*tmpObjSEndTerm[47] + tmpFx[72]*tmpObjSEndTerm[61] + tmpFx[88]*tmpObjSEndTerm[75] + tmpFx[104]*tmpObjSEndTerm[89] + tmpFx[120]*tmpObjSEndTerm[103] + tmpFx[136]*tmpObjSEndTerm[117] + tmpFx[152]*tmpObjSEndTerm[131] + tmpFx[168]*tmpObjSEndTerm[145] + tmpFx[184]*tmpObjSEndTerm[159] + tmpFx[200]*tmpObjSEndTerm[173] + tmpFx[216]*tmpObjSEndTerm[187];
tmpQN2[118] = + tmpFx[8]*tmpObjSEndTerm[6] + tmpFx[24]*tmpObjSEndTerm[20] + tmpFx[40]*tmpObjSEndTerm[34] + tmpFx[56]*tmpObjSEndTerm[48] + tmpFx[72]*tmpObjSEndTerm[62] + tmpFx[88]*tmpObjSEndTerm[76] + tmpFx[104]*tmpObjSEndTerm[90] + tmpFx[120]*tmpObjSEndTerm[104] + tmpFx[136]*tmpObjSEndTerm[118] + tmpFx[152]*tmpObjSEndTerm[132] + tmpFx[168]*tmpObjSEndTerm[146] + tmpFx[184]*tmpObjSEndTerm[160] + tmpFx[200]*tmpObjSEndTerm[174] + tmpFx[216]*tmpObjSEndTerm[188];
tmpQN2[119] = + tmpFx[8]*tmpObjSEndTerm[7] + tmpFx[24]*tmpObjSEndTerm[21] + tmpFx[40]*tmpObjSEndTerm[35] + tmpFx[56]*tmpObjSEndTerm[49] + tmpFx[72]*tmpObjSEndTerm[63] + tmpFx[88]*tmpObjSEndTerm[77] + tmpFx[104]*tmpObjSEndTerm[91] + tmpFx[120]*tmpObjSEndTerm[105] + tmpFx[136]*tmpObjSEndTerm[119] + tmpFx[152]*tmpObjSEndTerm[133] + tmpFx[168]*tmpObjSEndTerm[147] + tmpFx[184]*tmpObjSEndTerm[161] + tmpFx[200]*tmpObjSEndTerm[175] + tmpFx[216]*tmpObjSEndTerm[189];
tmpQN2[120] = + tmpFx[8]*tmpObjSEndTerm[8] + tmpFx[24]*tmpObjSEndTerm[22] + tmpFx[40]*tmpObjSEndTerm[36] + tmpFx[56]*tmpObjSEndTerm[50] + tmpFx[72]*tmpObjSEndTerm[64] + tmpFx[88]*tmpObjSEndTerm[78] + tmpFx[104]*tmpObjSEndTerm[92] + tmpFx[120]*tmpObjSEndTerm[106] + tmpFx[136]*tmpObjSEndTerm[120] + tmpFx[152]*tmpObjSEndTerm[134] + tmpFx[168]*tmpObjSEndTerm[148] + tmpFx[184]*tmpObjSEndTerm[162] + tmpFx[200]*tmpObjSEndTerm[176] + tmpFx[216]*tmpObjSEndTerm[190];
tmpQN2[121] = + tmpFx[8]*tmpObjSEndTerm[9] + tmpFx[24]*tmpObjSEndTerm[23] + tmpFx[40]*tmpObjSEndTerm[37] + tmpFx[56]*tmpObjSEndTerm[51] + tmpFx[72]*tmpObjSEndTerm[65] + tmpFx[88]*tmpObjSEndTerm[79] + tmpFx[104]*tmpObjSEndTerm[93] + tmpFx[120]*tmpObjSEndTerm[107] + tmpFx[136]*tmpObjSEndTerm[121] + tmpFx[152]*tmpObjSEndTerm[135] + tmpFx[168]*tmpObjSEndTerm[149] + tmpFx[184]*tmpObjSEndTerm[163] + tmpFx[200]*tmpObjSEndTerm[177] + tmpFx[216]*tmpObjSEndTerm[191];
tmpQN2[122] = + tmpFx[8]*tmpObjSEndTerm[10] + tmpFx[24]*tmpObjSEndTerm[24] + tmpFx[40]*tmpObjSEndTerm[38] + tmpFx[56]*tmpObjSEndTerm[52] + tmpFx[72]*tmpObjSEndTerm[66] + tmpFx[88]*tmpObjSEndTerm[80] + tmpFx[104]*tmpObjSEndTerm[94] + tmpFx[120]*tmpObjSEndTerm[108] + tmpFx[136]*tmpObjSEndTerm[122] + tmpFx[152]*tmpObjSEndTerm[136] + tmpFx[168]*tmpObjSEndTerm[150] + tmpFx[184]*tmpObjSEndTerm[164] + tmpFx[200]*tmpObjSEndTerm[178] + tmpFx[216]*tmpObjSEndTerm[192];
tmpQN2[123] = + tmpFx[8]*tmpObjSEndTerm[11] + tmpFx[24]*tmpObjSEndTerm[25] + tmpFx[40]*tmpObjSEndTerm[39] + tmpFx[56]*tmpObjSEndTerm[53] + tmpFx[72]*tmpObjSEndTerm[67] + tmpFx[88]*tmpObjSEndTerm[81] + tmpFx[104]*tmpObjSEndTerm[95] + tmpFx[120]*tmpObjSEndTerm[109] + tmpFx[136]*tmpObjSEndTerm[123] + tmpFx[152]*tmpObjSEndTerm[137] + tmpFx[168]*tmpObjSEndTerm[151] + tmpFx[184]*tmpObjSEndTerm[165] + tmpFx[200]*tmpObjSEndTerm[179] + tmpFx[216]*tmpObjSEndTerm[193];
tmpQN2[124] = + tmpFx[8]*tmpObjSEndTerm[12] + tmpFx[24]*tmpObjSEndTerm[26] + tmpFx[40]*tmpObjSEndTerm[40] + tmpFx[56]*tmpObjSEndTerm[54] + tmpFx[72]*tmpObjSEndTerm[68] + tmpFx[88]*tmpObjSEndTerm[82] + tmpFx[104]*tmpObjSEndTerm[96] + tmpFx[120]*tmpObjSEndTerm[110] + tmpFx[136]*tmpObjSEndTerm[124] + tmpFx[152]*tmpObjSEndTerm[138] + tmpFx[168]*tmpObjSEndTerm[152] + tmpFx[184]*tmpObjSEndTerm[166] + tmpFx[200]*tmpObjSEndTerm[180] + tmpFx[216]*tmpObjSEndTerm[194];
tmpQN2[125] = + tmpFx[8]*tmpObjSEndTerm[13] + tmpFx[24]*tmpObjSEndTerm[27] + tmpFx[40]*tmpObjSEndTerm[41] + tmpFx[56]*tmpObjSEndTerm[55] + tmpFx[72]*tmpObjSEndTerm[69] + tmpFx[88]*tmpObjSEndTerm[83] + tmpFx[104]*tmpObjSEndTerm[97] + tmpFx[120]*tmpObjSEndTerm[111] + tmpFx[136]*tmpObjSEndTerm[125] + tmpFx[152]*tmpObjSEndTerm[139] + tmpFx[168]*tmpObjSEndTerm[153] + tmpFx[184]*tmpObjSEndTerm[167] + tmpFx[200]*tmpObjSEndTerm[181] + tmpFx[216]*tmpObjSEndTerm[195];
tmpQN2[126] = + tmpFx[9]*tmpObjSEndTerm[0] + tmpFx[25]*tmpObjSEndTerm[14] + tmpFx[41]*tmpObjSEndTerm[28] + tmpFx[57]*tmpObjSEndTerm[42] + tmpFx[73]*tmpObjSEndTerm[56] + tmpFx[89]*tmpObjSEndTerm[70] + tmpFx[105]*tmpObjSEndTerm[84] + tmpFx[121]*tmpObjSEndTerm[98] + tmpFx[137]*tmpObjSEndTerm[112] + tmpFx[153]*tmpObjSEndTerm[126] + tmpFx[169]*tmpObjSEndTerm[140] + tmpFx[185]*tmpObjSEndTerm[154] + tmpFx[201]*tmpObjSEndTerm[168] + tmpFx[217]*tmpObjSEndTerm[182];
tmpQN2[127] = + tmpFx[9]*tmpObjSEndTerm[1] + tmpFx[25]*tmpObjSEndTerm[15] + tmpFx[41]*tmpObjSEndTerm[29] + tmpFx[57]*tmpObjSEndTerm[43] + tmpFx[73]*tmpObjSEndTerm[57] + tmpFx[89]*tmpObjSEndTerm[71] + tmpFx[105]*tmpObjSEndTerm[85] + tmpFx[121]*tmpObjSEndTerm[99] + tmpFx[137]*tmpObjSEndTerm[113] + tmpFx[153]*tmpObjSEndTerm[127] + tmpFx[169]*tmpObjSEndTerm[141] + tmpFx[185]*tmpObjSEndTerm[155] + tmpFx[201]*tmpObjSEndTerm[169] + tmpFx[217]*tmpObjSEndTerm[183];
tmpQN2[128] = + tmpFx[9]*tmpObjSEndTerm[2] + tmpFx[25]*tmpObjSEndTerm[16] + tmpFx[41]*tmpObjSEndTerm[30] + tmpFx[57]*tmpObjSEndTerm[44] + tmpFx[73]*tmpObjSEndTerm[58] + tmpFx[89]*tmpObjSEndTerm[72] + tmpFx[105]*tmpObjSEndTerm[86] + tmpFx[121]*tmpObjSEndTerm[100] + tmpFx[137]*tmpObjSEndTerm[114] + tmpFx[153]*tmpObjSEndTerm[128] + tmpFx[169]*tmpObjSEndTerm[142] + tmpFx[185]*tmpObjSEndTerm[156] + tmpFx[201]*tmpObjSEndTerm[170] + tmpFx[217]*tmpObjSEndTerm[184];
tmpQN2[129] = + tmpFx[9]*tmpObjSEndTerm[3] + tmpFx[25]*tmpObjSEndTerm[17] + tmpFx[41]*tmpObjSEndTerm[31] + tmpFx[57]*tmpObjSEndTerm[45] + tmpFx[73]*tmpObjSEndTerm[59] + tmpFx[89]*tmpObjSEndTerm[73] + tmpFx[105]*tmpObjSEndTerm[87] + tmpFx[121]*tmpObjSEndTerm[101] + tmpFx[137]*tmpObjSEndTerm[115] + tmpFx[153]*tmpObjSEndTerm[129] + tmpFx[169]*tmpObjSEndTerm[143] + tmpFx[185]*tmpObjSEndTerm[157] + tmpFx[201]*tmpObjSEndTerm[171] + tmpFx[217]*tmpObjSEndTerm[185];
tmpQN2[130] = + tmpFx[9]*tmpObjSEndTerm[4] + tmpFx[25]*tmpObjSEndTerm[18] + tmpFx[41]*tmpObjSEndTerm[32] + tmpFx[57]*tmpObjSEndTerm[46] + tmpFx[73]*tmpObjSEndTerm[60] + tmpFx[89]*tmpObjSEndTerm[74] + tmpFx[105]*tmpObjSEndTerm[88] + tmpFx[121]*tmpObjSEndTerm[102] + tmpFx[137]*tmpObjSEndTerm[116] + tmpFx[153]*tmpObjSEndTerm[130] + tmpFx[169]*tmpObjSEndTerm[144] + tmpFx[185]*tmpObjSEndTerm[158] + tmpFx[201]*tmpObjSEndTerm[172] + tmpFx[217]*tmpObjSEndTerm[186];
tmpQN2[131] = + tmpFx[9]*tmpObjSEndTerm[5] + tmpFx[25]*tmpObjSEndTerm[19] + tmpFx[41]*tmpObjSEndTerm[33] + tmpFx[57]*tmpObjSEndTerm[47] + tmpFx[73]*tmpObjSEndTerm[61] + tmpFx[89]*tmpObjSEndTerm[75] + tmpFx[105]*tmpObjSEndTerm[89] + tmpFx[121]*tmpObjSEndTerm[103] + tmpFx[137]*tmpObjSEndTerm[117] + tmpFx[153]*tmpObjSEndTerm[131] + tmpFx[169]*tmpObjSEndTerm[145] + tmpFx[185]*tmpObjSEndTerm[159] + tmpFx[201]*tmpObjSEndTerm[173] + tmpFx[217]*tmpObjSEndTerm[187];
tmpQN2[132] = + tmpFx[9]*tmpObjSEndTerm[6] + tmpFx[25]*tmpObjSEndTerm[20] + tmpFx[41]*tmpObjSEndTerm[34] + tmpFx[57]*tmpObjSEndTerm[48] + tmpFx[73]*tmpObjSEndTerm[62] + tmpFx[89]*tmpObjSEndTerm[76] + tmpFx[105]*tmpObjSEndTerm[90] + tmpFx[121]*tmpObjSEndTerm[104] + tmpFx[137]*tmpObjSEndTerm[118] + tmpFx[153]*tmpObjSEndTerm[132] + tmpFx[169]*tmpObjSEndTerm[146] + tmpFx[185]*tmpObjSEndTerm[160] + tmpFx[201]*tmpObjSEndTerm[174] + tmpFx[217]*tmpObjSEndTerm[188];
tmpQN2[133] = + tmpFx[9]*tmpObjSEndTerm[7] + tmpFx[25]*tmpObjSEndTerm[21] + tmpFx[41]*tmpObjSEndTerm[35] + tmpFx[57]*tmpObjSEndTerm[49] + tmpFx[73]*tmpObjSEndTerm[63] + tmpFx[89]*tmpObjSEndTerm[77] + tmpFx[105]*tmpObjSEndTerm[91] + tmpFx[121]*tmpObjSEndTerm[105] + tmpFx[137]*tmpObjSEndTerm[119] + tmpFx[153]*tmpObjSEndTerm[133] + tmpFx[169]*tmpObjSEndTerm[147] + tmpFx[185]*tmpObjSEndTerm[161] + tmpFx[201]*tmpObjSEndTerm[175] + tmpFx[217]*tmpObjSEndTerm[189];
tmpQN2[134] = + tmpFx[9]*tmpObjSEndTerm[8] + tmpFx[25]*tmpObjSEndTerm[22] + tmpFx[41]*tmpObjSEndTerm[36] + tmpFx[57]*tmpObjSEndTerm[50] + tmpFx[73]*tmpObjSEndTerm[64] + tmpFx[89]*tmpObjSEndTerm[78] + tmpFx[105]*tmpObjSEndTerm[92] + tmpFx[121]*tmpObjSEndTerm[106] + tmpFx[137]*tmpObjSEndTerm[120] + tmpFx[153]*tmpObjSEndTerm[134] + tmpFx[169]*tmpObjSEndTerm[148] + tmpFx[185]*tmpObjSEndTerm[162] + tmpFx[201]*tmpObjSEndTerm[176] + tmpFx[217]*tmpObjSEndTerm[190];
tmpQN2[135] = + tmpFx[9]*tmpObjSEndTerm[9] + tmpFx[25]*tmpObjSEndTerm[23] + tmpFx[41]*tmpObjSEndTerm[37] + tmpFx[57]*tmpObjSEndTerm[51] + tmpFx[73]*tmpObjSEndTerm[65] + tmpFx[89]*tmpObjSEndTerm[79] + tmpFx[105]*tmpObjSEndTerm[93] + tmpFx[121]*tmpObjSEndTerm[107] + tmpFx[137]*tmpObjSEndTerm[121] + tmpFx[153]*tmpObjSEndTerm[135] + tmpFx[169]*tmpObjSEndTerm[149] + tmpFx[185]*tmpObjSEndTerm[163] + tmpFx[201]*tmpObjSEndTerm[177] + tmpFx[217]*tmpObjSEndTerm[191];
tmpQN2[136] = + tmpFx[9]*tmpObjSEndTerm[10] + tmpFx[25]*tmpObjSEndTerm[24] + tmpFx[41]*tmpObjSEndTerm[38] + tmpFx[57]*tmpObjSEndTerm[52] + tmpFx[73]*tmpObjSEndTerm[66] + tmpFx[89]*tmpObjSEndTerm[80] + tmpFx[105]*tmpObjSEndTerm[94] + tmpFx[121]*tmpObjSEndTerm[108] + tmpFx[137]*tmpObjSEndTerm[122] + tmpFx[153]*tmpObjSEndTerm[136] + tmpFx[169]*tmpObjSEndTerm[150] + tmpFx[185]*tmpObjSEndTerm[164] + tmpFx[201]*tmpObjSEndTerm[178] + tmpFx[217]*tmpObjSEndTerm[192];
tmpQN2[137] = + tmpFx[9]*tmpObjSEndTerm[11] + tmpFx[25]*tmpObjSEndTerm[25] + tmpFx[41]*tmpObjSEndTerm[39] + tmpFx[57]*tmpObjSEndTerm[53] + tmpFx[73]*tmpObjSEndTerm[67] + tmpFx[89]*tmpObjSEndTerm[81] + tmpFx[105]*tmpObjSEndTerm[95] + tmpFx[121]*tmpObjSEndTerm[109] + tmpFx[137]*tmpObjSEndTerm[123] + tmpFx[153]*tmpObjSEndTerm[137] + tmpFx[169]*tmpObjSEndTerm[151] + tmpFx[185]*tmpObjSEndTerm[165] + tmpFx[201]*tmpObjSEndTerm[179] + tmpFx[217]*tmpObjSEndTerm[193];
tmpQN2[138] = + tmpFx[9]*tmpObjSEndTerm[12] + tmpFx[25]*tmpObjSEndTerm[26] + tmpFx[41]*tmpObjSEndTerm[40] + tmpFx[57]*tmpObjSEndTerm[54] + tmpFx[73]*tmpObjSEndTerm[68] + tmpFx[89]*tmpObjSEndTerm[82] + tmpFx[105]*tmpObjSEndTerm[96] + tmpFx[121]*tmpObjSEndTerm[110] + tmpFx[137]*tmpObjSEndTerm[124] + tmpFx[153]*tmpObjSEndTerm[138] + tmpFx[169]*tmpObjSEndTerm[152] + tmpFx[185]*tmpObjSEndTerm[166] + tmpFx[201]*tmpObjSEndTerm[180] + tmpFx[217]*tmpObjSEndTerm[194];
tmpQN2[139] = + tmpFx[9]*tmpObjSEndTerm[13] + tmpFx[25]*tmpObjSEndTerm[27] + tmpFx[41]*tmpObjSEndTerm[41] + tmpFx[57]*tmpObjSEndTerm[55] + tmpFx[73]*tmpObjSEndTerm[69] + tmpFx[89]*tmpObjSEndTerm[83] + tmpFx[105]*tmpObjSEndTerm[97] + tmpFx[121]*tmpObjSEndTerm[111] + tmpFx[137]*tmpObjSEndTerm[125] + tmpFx[153]*tmpObjSEndTerm[139] + tmpFx[169]*tmpObjSEndTerm[153] + tmpFx[185]*tmpObjSEndTerm[167] + tmpFx[201]*tmpObjSEndTerm[181] + tmpFx[217]*tmpObjSEndTerm[195];
tmpQN2[140] = + tmpFx[10]*tmpObjSEndTerm[0] + tmpFx[26]*tmpObjSEndTerm[14] + tmpFx[42]*tmpObjSEndTerm[28] + tmpFx[58]*tmpObjSEndTerm[42] + tmpFx[74]*tmpObjSEndTerm[56] + tmpFx[90]*tmpObjSEndTerm[70] + tmpFx[106]*tmpObjSEndTerm[84] + tmpFx[122]*tmpObjSEndTerm[98] + tmpFx[138]*tmpObjSEndTerm[112] + tmpFx[154]*tmpObjSEndTerm[126] + tmpFx[170]*tmpObjSEndTerm[140] + tmpFx[186]*tmpObjSEndTerm[154] + tmpFx[202]*tmpObjSEndTerm[168] + tmpFx[218]*tmpObjSEndTerm[182];
tmpQN2[141] = + tmpFx[10]*tmpObjSEndTerm[1] + tmpFx[26]*tmpObjSEndTerm[15] + tmpFx[42]*tmpObjSEndTerm[29] + tmpFx[58]*tmpObjSEndTerm[43] + tmpFx[74]*tmpObjSEndTerm[57] + tmpFx[90]*tmpObjSEndTerm[71] + tmpFx[106]*tmpObjSEndTerm[85] + tmpFx[122]*tmpObjSEndTerm[99] + tmpFx[138]*tmpObjSEndTerm[113] + tmpFx[154]*tmpObjSEndTerm[127] + tmpFx[170]*tmpObjSEndTerm[141] + tmpFx[186]*tmpObjSEndTerm[155] + tmpFx[202]*tmpObjSEndTerm[169] + tmpFx[218]*tmpObjSEndTerm[183];
tmpQN2[142] = + tmpFx[10]*tmpObjSEndTerm[2] + tmpFx[26]*tmpObjSEndTerm[16] + tmpFx[42]*tmpObjSEndTerm[30] + tmpFx[58]*tmpObjSEndTerm[44] + tmpFx[74]*tmpObjSEndTerm[58] + tmpFx[90]*tmpObjSEndTerm[72] + tmpFx[106]*tmpObjSEndTerm[86] + tmpFx[122]*tmpObjSEndTerm[100] + tmpFx[138]*tmpObjSEndTerm[114] + tmpFx[154]*tmpObjSEndTerm[128] + tmpFx[170]*tmpObjSEndTerm[142] + tmpFx[186]*tmpObjSEndTerm[156] + tmpFx[202]*tmpObjSEndTerm[170] + tmpFx[218]*tmpObjSEndTerm[184];
tmpQN2[143] = + tmpFx[10]*tmpObjSEndTerm[3] + tmpFx[26]*tmpObjSEndTerm[17] + tmpFx[42]*tmpObjSEndTerm[31] + tmpFx[58]*tmpObjSEndTerm[45] + tmpFx[74]*tmpObjSEndTerm[59] + tmpFx[90]*tmpObjSEndTerm[73] + tmpFx[106]*tmpObjSEndTerm[87] + tmpFx[122]*tmpObjSEndTerm[101] + tmpFx[138]*tmpObjSEndTerm[115] + tmpFx[154]*tmpObjSEndTerm[129] + tmpFx[170]*tmpObjSEndTerm[143] + tmpFx[186]*tmpObjSEndTerm[157] + tmpFx[202]*tmpObjSEndTerm[171] + tmpFx[218]*tmpObjSEndTerm[185];
tmpQN2[144] = + tmpFx[10]*tmpObjSEndTerm[4] + tmpFx[26]*tmpObjSEndTerm[18] + tmpFx[42]*tmpObjSEndTerm[32] + tmpFx[58]*tmpObjSEndTerm[46] + tmpFx[74]*tmpObjSEndTerm[60] + tmpFx[90]*tmpObjSEndTerm[74] + tmpFx[106]*tmpObjSEndTerm[88] + tmpFx[122]*tmpObjSEndTerm[102] + tmpFx[138]*tmpObjSEndTerm[116] + tmpFx[154]*tmpObjSEndTerm[130] + tmpFx[170]*tmpObjSEndTerm[144] + tmpFx[186]*tmpObjSEndTerm[158] + tmpFx[202]*tmpObjSEndTerm[172] + tmpFx[218]*tmpObjSEndTerm[186];
tmpQN2[145] = + tmpFx[10]*tmpObjSEndTerm[5] + tmpFx[26]*tmpObjSEndTerm[19] + tmpFx[42]*tmpObjSEndTerm[33] + tmpFx[58]*tmpObjSEndTerm[47] + tmpFx[74]*tmpObjSEndTerm[61] + tmpFx[90]*tmpObjSEndTerm[75] + tmpFx[106]*tmpObjSEndTerm[89] + tmpFx[122]*tmpObjSEndTerm[103] + tmpFx[138]*tmpObjSEndTerm[117] + tmpFx[154]*tmpObjSEndTerm[131] + tmpFx[170]*tmpObjSEndTerm[145] + tmpFx[186]*tmpObjSEndTerm[159] + tmpFx[202]*tmpObjSEndTerm[173] + tmpFx[218]*tmpObjSEndTerm[187];
tmpQN2[146] = + tmpFx[10]*tmpObjSEndTerm[6] + tmpFx[26]*tmpObjSEndTerm[20] + tmpFx[42]*tmpObjSEndTerm[34] + tmpFx[58]*tmpObjSEndTerm[48] + tmpFx[74]*tmpObjSEndTerm[62] + tmpFx[90]*tmpObjSEndTerm[76] + tmpFx[106]*tmpObjSEndTerm[90] + tmpFx[122]*tmpObjSEndTerm[104] + tmpFx[138]*tmpObjSEndTerm[118] + tmpFx[154]*tmpObjSEndTerm[132] + tmpFx[170]*tmpObjSEndTerm[146] + tmpFx[186]*tmpObjSEndTerm[160] + tmpFx[202]*tmpObjSEndTerm[174] + tmpFx[218]*tmpObjSEndTerm[188];
tmpQN2[147] = + tmpFx[10]*tmpObjSEndTerm[7] + tmpFx[26]*tmpObjSEndTerm[21] + tmpFx[42]*tmpObjSEndTerm[35] + tmpFx[58]*tmpObjSEndTerm[49] + tmpFx[74]*tmpObjSEndTerm[63] + tmpFx[90]*tmpObjSEndTerm[77] + tmpFx[106]*tmpObjSEndTerm[91] + tmpFx[122]*tmpObjSEndTerm[105] + tmpFx[138]*tmpObjSEndTerm[119] + tmpFx[154]*tmpObjSEndTerm[133] + tmpFx[170]*tmpObjSEndTerm[147] + tmpFx[186]*tmpObjSEndTerm[161] + tmpFx[202]*tmpObjSEndTerm[175] + tmpFx[218]*tmpObjSEndTerm[189];
tmpQN2[148] = + tmpFx[10]*tmpObjSEndTerm[8] + tmpFx[26]*tmpObjSEndTerm[22] + tmpFx[42]*tmpObjSEndTerm[36] + tmpFx[58]*tmpObjSEndTerm[50] + tmpFx[74]*tmpObjSEndTerm[64] + tmpFx[90]*tmpObjSEndTerm[78] + tmpFx[106]*tmpObjSEndTerm[92] + tmpFx[122]*tmpObjSEndTerm[106] + tmpFx[138]*tmpObjSEndTerm[120] + tmpFx[154]*tmpObjSEndTerm[134] + tmpFx[170]*tmpObjSEndTerm[148] + tmpFx[186]*tmpObjSEndTerm[162] + tmpFx[202]*tmpObjSEndTerm[176] + tmpFx[218]*tmpObjSEndTerm[190];
tmpQN2[149] = + tmpFx[10]*tmpObjSEndTerm[9] + tmpFx[26]*tmpObjSEndTerm[23] + tmpFx[42]*tmpObjSEndTerm[37] + tmpFx[58]*tmpObjSEndTerm[51] + tmpFx[74]*tmpObjSEndTerm[65] + tmpFx[90]*tmpObjSEndTerm[79] + tmpFx[106]*tmpObjSEndTerm[93] + tmpFx[122]*tmpObjSEndTerm[107] + tmpFx[138]*tmpObjSEndTerm[121] + tmpFx[154]*tmpObjSEndTerm[135] + tmpFx[170]*tmpObjSEndTerm[149] + tmpFx[186]*tmpObjSEndTerm[163] + tmpFx[202]*tmpObjSEndTerm[177] + tmpFx[218]*tmpObjSEndTerm[191];
tmpQN2[150] = + tmpFx[10]*tmpObjSEndTerm[10] + tmpFx[26]*tmpObjSEndTerm[24] + tmpFx[42]*tmpObjSEndTerm[38] + tmpFx[58]*tmpObjSEndTerm[52] + tmpFx[74]*tmpObjSEndTerm[66] + tmpFx[90]*tmpObjSEndTerm[80] + tmpFx[106]*tmpObjSEndTerm[94] + tmpFx[122]*tmpObjSEndTerm[108] + tmpFx[138]*tmpObjSEndTerm[122] + tmpFx[154]*tmpObjSEndTerm[136] + tmpFx[170]*tmpObjSEndTerm[150] + tmpFx[186]*tmpObjSEndTerm[164] + tmpFx[202]*tmpObjSEndTerm[178] + tmpFx[218]*tmpObjSEndTerm[192];
tmpQN2[151] = + tmpFx[10]*tmpObjSEndTerm[11] + tmpFx[26]*tmpObjSEndTerm[25] + tmpFx[42]*tmpObjSEndTerm[39] + tmpFx[58]*tmpObjSEndTerm[53] + tmpFx[74]*tmpObjSEndTerm[67] + tmpFx[90]*tmpObjSEndTerm[81] + tmpFx[106]*tmpObjSEndTerm[95] + tmpFx[122]*tmpObjSEndTerm[109] + tmpFx[138]*tmpObjSEndTerm[123] + tmpFx[154]*tmpObjSEndTerm[137] + tmpFx[170]*tmpObjSEndTerm[151] + tmpFx[186]*tmpObjSEndTerm[165] + tmpFx[202]*tmpObjSEndTerm[179] + tmpFx[218]*tmpObjSEndTerm[193];
tmpQN2[152] = + tmpFx[10]*tmpObjSEndTerm[12] + tmpFx[26]*tmpObjSEndTerm[26] + tmpFx[42]*tmpObjSEndTerm[40] + tmpFx[58]*tmpObjSEndTerm[54] + tmpFx[74]*tmpObjSEndTerm[68] + tmpFx[90]*tmpObjSEndTerm[82] + tmpFx[106]*tmpObjSEndTerm[96] + tmpFx[122]*tmpObjSEndTerm[110] + tmpFx[138]*tmpObjSEndTerm[124] + tmpFx[154]*tmpObjSEndTerm[138] + tmpFx[170]*tmpObjSEndTerm[152] + tmpFx[186]*tmpObjSEndTerm[166] + tmpFx[202]*tmpObjSEndTerm[180] + tmpFx[218]*tmpObjSEndTerm[194];
tmpQN2[153] = + tmpFx[10]*tmpObjSEndTerm[13] + tmpFx[26]*tmpObjSEndTerm[27] + tmpFx[42]*tmpObjSEndTerm[41] + tmpFx[58]*tmpObjSEndTerm[55] + tmpFx[74]*tmpObjSEndTerm[69] + tmpFx[90]*tmpObjSEndTerm[83] + tmpFx[106]*tmpObjSEndTerm[97] + tmpFx[122]*tmpObjSEndTerm[111] + tmpFx[138]*tmpObjSEndTerm[125] + tmpFx[154]*tmpObjSEndTerm[139] + tmpFx[170]*tmpObjSEndTerm[153] + tmpFx[186]*tmpObjSEndTerm[167] + tmpFx[202]*tmpObjSEndTerm[181] + tmpFx[218]*tmpObjSEndTerm[195];
tmpQN2[154] = + tmpFx[11]*tmpObjSEndTerm[0] + tmpFx[27]*tmpObjSEndTerm[14] + tmpFx[43]*tmpObjSEndTerm[28] + tmpFx[59]*tmpObjSEndTerm[42] + tmpFx[75]*tmpObjSEndTerm[56] + tmpFx[91]*tmpObjSEndTerm[70] + tmpFx[107]*tmpObjSEndTerm[84] + tmpFx[123]*tmpObjSEndTerm[98] + tmpFx[139]*tmpObjSEndTerm[112] + tmpFx[155]*tmpObjSEndTerm[126] + tmpFx[171]*tmpObjSEndTerm[140] + tmpFx[187]*tmpObjSEndTerm[154] + tmpFx[203]*tmpObjSEndTerm[168] + tmpFx[219]*tmpObjSEndTerm[182];
tmpQN2[155] = + tmpFx[11]*tmpObjSEndTerm[1] + tmpFx[27]*tmpObjSEndTerm[15] + tmpFx[43]*tmpObjSEndTerm[29] + tmpFx[59]*tmpObjSEndTerm[43] + tmpFx[75]*tmpObjSEndTerm[57] + tmpFx[91]*tmpObjSEndTerm[71] + tmpFx[107]*tmpObjSEndTerm[85] + tmpFx[123]*tmpObjSEndTerm[99] + tmpFx[139]*tmpObjSEndTerm[113] + tmpFx[155]*tmpObjSEndTerm[127] + tmpFx[171]*tmpObjSEndTerm[141] + tmpFx[187]*tmpObjSEndTerm[155] + tmpFx[203]*tmpObjSEndTerm[169] + tmpFx[219]*tmpObjSEndTerm[183];
tmpQN2[156] = + tmpFx[11]*tmpObjSEndTerm[2] + tmpFx[27]*tmpObjSEndTerm[16] + tmpFx[43]*tmpObjSEndTerm[30] + tmpFx[59]*tmpObjSEndTerm[44] + tmpFx[75]*tmpObjSEndTerm[58] + tmpFx[91]*tmpObjSEndTerm[72] + tmpFx[107]*tmpObjSEndTerm[86] + tmpFx[123]*tmpObjSEndTerm[100] + tmpFx[139]*tmpObjSEndTerm[114] + tmpFx[155]*tmpObjSEndTerm[128] + tmpFx[171]*tmpObjSEndTerm[142] + tmpFx[187]*tmpObjSEndTerm[156] + tmpFx[203]*tmpObjSEndTerm[170] + tmpFx[219]*tmpObjSEndTerm[184];
tmpQN2[157] = + tmpFx[11]*tmpObjSEndTerm[3] + tmpFx[27]*tmpObjSEndTerm[17] + tmpFx[43]*tmpObjSEndTerm[31] + tmpFx[59]*tmpObjSEndTerm[45] + tmpFx[75]*tmpObjSEndTerm[59] + tmpFx[91]*tmpObjSEndTerm[73] + tmpFx[107]*tmpObjSEndTerm[87] + tmpFx[123]*tmpObjSEndTerm[101] + tmpFx[139]*tmpObjSEndTerm[115] + tmpFx[155]*tmpObjSEndTerm[129] + tmpFx[171]*tmpObjSEndTerm[143] + tmpFx[187]*tmpObjSEndTerm[157] + tmpFx[203]*tmpObjSEndTerm[171] + tmpFx[219]*tmpObjSEndTerm[185];
tmpQN2[158] = + tmpFx[11]*tmpObjSEndTerm[4] + tmpFx[27]*tmpObjSEndTerm[18] + tmpFx[43]*tmpObjSEndTerm[32] + tmpFx[59]*tmpObjSEndTerm[46] + tmpFx[75]*tmpObjSEndTerm[60] + tmpFx[91]*tmpObjSEndTerm[74] + tmpFx[107]*tmpObjSEndTerm[88] + tmpFx[123]*tmpObjSEndTerm[102] + tmpFx[139]*tmpObjSEndTerm[116] + tmpFx[155]*tmpObjSEndTerm[130] + tmpFx[171]*tmpObjSEndTerm[144] + tmpFx[187]*tmpObjSEndTerm[158] + tmpFx[203]*tmpObjSEndTerm[172] + tmpFx[219]*tmpObjSEndTerm[186];
tmpQN2[159] = + tmpFx[11]*tmpObjSEndTerm[5] + tmpFx[27]*tmpObjSEndTerm[19] + tmpFx[43]*tmpObjSEndTerm[33] + tmpFx[59]*tmpObjSEndTerm[47] + tmpFx[75]*tmpObjSEndTerm[61] + tmpFx[91]*tmpObjSEndTerm[75] + tmpFx[107]*tmpObjSEndTerm[89] + tmpFx[123]*tmpObjSEndTerm[103] + tmpFx[139]*tmpObjSEndTerm[117] + tmpFx[155]*tmpObjSEndTerm[131] + tmpFx[171]*tmpObjSEndTerm[145] + tmpFx[187]*tmpObjSEndTerm[159] + tmpFx[203]*tmpObjSEndTerm[173] + tmpFx[219]*tmpObjSEndTerm[187];
tmpQN2[160] = + tmpFx[11]*tmpObjSEndTerm[6] + tmpFx[27]*tmpObjSEndTerm[20] + tmpFx[43]*tmpObjSEndTerm[34] + tmpFx[59]*tmpObjSEndTerm[48] + tmpFx[75]*tmpObjSEndTerm[62] + tmpFx[91]*tmpObjSEndTerm[76] + tmpFx[107]*tmpObjSEndTerm[90] + tmpFx[123]*tmpObjSEndTerm[104] + tmpFx[139]*tmpObjSEndTerm[118] + tmpFx[155]*tmpObjSEndTerm[132] + tmpFx[171]*tmpObjSEndTerm[146] + tmpFx[187]*tmpObjSEndTerm[160] + tmpFx[203]*tmpObjSEndTerm[174] + tmpFx[219]*tmpObjSEndTerm[188];
tmpQN2[161] = + tmpFx[11]*tmpObjSEndTerm[7] + tmpFx[27]*tmpObjSEndTerm[21] + tmpFx[43]*tmpObjSEndTerm[35] + tmpFx[59]*tmpObjSEndTerm[49] + tmpFx[75]*tmpObjSEndTerm[63] + tmpFx[91]*tmpObjSEndTerm[77] + tmpFx[107]*tmpObjSEndTerm[91] + tmpFx[123]*tmpObjSEndTerm[105] + tmpFx[139]*tmpObjSEndTerm[119] + tmpFx[155]*tmpObjSEndTerm[133] + tmpFx[171]*tmpObjSEndTerm[147] + tmpFx[187]*tmpObjSEndTerm[161] + tmpFx[203]*tmpObjSEndTerm[175] + tmpFx[219]*tmpObjSEndTerm[189];
tmpQN2[162] = + tmpFx[11]*tmpObjSEndTerm[8] + tmpFx[27]*tmpObjSEndTerm[22] + tmpFx[43]*tmpObjSEndTerm[36] + tmpFx[59]*tmpObjSEndTerm[50] + tmpFx[75]*tmpObjSEndTerm[64] + tmpFx[91]*tmpObjSEndTerm[78] + tmpFx[107]*tmpObjSEndTerm[92] + tmpFx[123]*tmpObjSEndTerm[106] + tmpFx[139]*tmpObjSEndTerm[120] + tmpFx[155]*tmpObjSEndTerm[134] + tmpFx[171]*tmpObjSEndTerm[148] + tmpFx[187]*tmpObjSEndTerm[162] + tmpFx[203]*tmpObjSEndTerm[176] + tmpFx[219]*tmpObjSEndTerm[190];
tmpQN2[163] = + tmpFx[11]*tmpObjSEndTerm[9] + tmpFx[27]*tmpObjSEndTerm[23] + tmpFx[43]*tmpObjSEndTerm[37] + tmpFx[59]*tmpObjSEndTerm[51] + tmpFx[75]*tmpObjSEndTerm[65] + tmpFx[91]*tmpObjSEndTerm[79] + tmpFx[107]*tmpObjSEndTerm[93] + tmpFx[123]*tmpObjSEndTerm[107] + tmpFx[139]*tmpObjSEndTerm[121] + tmpFx[155]*tmpObjSEndTerm[135] + tmpFx[171]*tmpObjSEndTerm[149] + tmpFx[187]*tmpObjSEndTerm[163] + tmpFx[203]*tmpObjSEndTerm[177] + tmpFx[219]*tmpObjSEndTerm[191];
tmpQN2[164] = + tmpFx[11]*tmpObjSEndTerm[10] + tmpFx[27]*tmpObjSEndTerm[24] + tmpFx[43]*tmpObjSEndTerm[38] + tmpFx[59]*tmpObjSEndTerm[52] + tmpFx[75]*tmpObjSEndTerm[66] + tmpFx[91]*tmpObjSEndTerm[80] + tmpFx[107]*tmpObjSEndTerm[94] + tmpFx[123]*tmpObjSEndTerm[108] + tmpFx[139]*tmpObjSEndTerm[122] + tmpFx[155]*tmpObjSEndTerm[136] + tmpFx[171]*tmpObjSEndTerm[150] + tmpFx[187]*tmpObjSEndTerm[164] + tmpFx[203]*tmpObjSEndTerm[178] + tmpFx[219]*tmpObjSEndTerm[192];
tmpQN2[165] = + tmpFx[11]*tmpObjSEndTerm[11] + tmpFx[27]*tmpObjSEndTerm[25] + tmpFx[43]*tmpObjSEndTerm[39] + tmpFx[59]*tmpObjSEndTerm[53] + tmpFx[75]*tmpObjSEndTerm[67] + tmpFx[91]*tmpObjSEndTerm[81] + tmpFx[107]*tmpObjSEndTerm[95] + tmpFx[123]*tmpObjSEndTerm[109] + tmpFx[139]*tmpObjSEndTerm[123] + tmpFx[155]*tmpObjSEndTerm[137] + tmpFx[171]*tmpObjSEndTerm[151] + tmpFx[187]*tmpObjSEndTerm[165] + tmpFx[203]*tmpObjSEndTerm[179] + tmpFx[219]*tmpObjSEndTerm[193];
tmpQN2[166] = + tmpFx[11]*tmpObjSEndTerm[12] + tmpFx[27]*tmpObjSEndTerm[26] + tmpFx[43]*tmpObjSEndTerm[40] + tmpFx[59]*tmpObjSEndTerm[54] + tmpFx[75]*tmpObjSEndTerm[68] + tmpFx[91]*tmpObjSEndTerm[82] + tmpFx[107]*tmpObjSEndTerm[96] + tmpFx[123]*tmpObjSEndTerm[110] + tmpFx[139]*tmpObjSEndTerm[124] + tmpFx[155]*tmpObjSEndTerm[138] + tmpFx[171]*tmpObjSEndTerm[152] + tmpFx[187]*tmpObjSEndTerm[166] + tmpFx[203]*tmpObjSEndTerm[180] + tmpFx[219]*tmpObjSEndTerm[194];
tmpQN2[167] = + tmpFx[11]*tmpObjSEndTerm[13] + tmpFx[27]*tmpObjSEndTerm[27] + tmpFx[43]*tmpObjSEndTerm[41] + tmpFx[59]*tmpObjSEndTerm[55] + tmpFx[75]*tmpObjSEndTerm[69] + tmpFx[91]*tmpObjSEndTerm[83] + tmpFx[107]*tmpObjSEndTerm[97] + tmpFx[123]*tmpObjSEndTerm[111] + tmpFx[139]*tmpObjSEndTerm[125] + tmpFx[155]*tmpObjSEndTerm[139] + tmpFx[171]*tmpObjSEndTerm[153] + tmpFx[187]*tmpObjSEndTerm[167] + tmpFx[203]*tmpObjSEndTerm[181] + tmpFx[219]*tmpObjSEndTerm[195];
tmpQN2[168] = + tmpFx[12]*tmpObjSEndTerm[0] + tmpFx[28]*tmpObjSEndTerm[14] + tmpFx[44]*tmpObjSEndTerm[28] + tmpFx[60]*tmpObjSEndTerm[42] + tmpFx[76]*tmpObjSEndTerm[56] + tmpFx[92]*tmpObjSEndTerm[70] + tmpFx[108]*tmpObjSEndTerm[84] + tmpFx[124]*tmpObjSEndTerm[98] + tmpFx[140]*tmpObjSEndTerm[112] + tmpFx[156]*tmpObjSEndTerm[126] + tmpFx[172]*tmpObjSEndTerm[140] + tmpFx[188]*tmpObjSEndTerm[154] + tmpFx[204]*tmpObjSEndTerm[168] + tmpFx[220]*tmpObjSEndTerm[182];
tmpQN2[169] = + tmpFx[12]*tmpObjSEndTerm[1] + tmpFx[28]*tmpObjSEndTerm[15] + tmpFx[44]*tmpObjSEndTerm[29] + tmpFx[60]*tmpObjSEndTerm[43] + tmpFx[76]*tmpObjSEndTerm[57] + tmpFx[92]*tmpObjSEndTerm[71] + tmpFx[108]*tmpObjSEndTerm[85] + tmpFx[124]*tmpObjSEndTerm[99] + tmpFx[140]*tmpObjSEndTerm[113] + tmpFx[156]*tmpObjSEndTerm[127] + tmpFx[172]*tmpObjSEndTerm[141] + tmpFx[188]*tmpObjSEndTerm[155] + tmpFx[204]*tmpObjSEndTerm[169] + tmpFx[220]*tmpObjSEndTerm[183];
tmpQN2[170] = + tmpFx[12]*tmpObjSEndTerm[2] + tmpFx[28]*tmpObjSEndTerm[16] + tmpFx[44]*tmpObjSEndTerm[30] + tmpFx[60]*tmpObjSEndTerm[44] + tmpFx[76]*tmpObjSEndTerm[58] + tmpFx[92]*tmpObjSEndTerm[72] + tmpFx[108]*tmpObjSEndTerm[86] + tmpFx[124]*tmpObjSEndTerm[100] + tmpFx[140]*tmpObjSEndTerm[114] + tmpFx[156]*tmpObjSEndTerm[128] + tmpFx[172]*tmpObjSEndTerm[142] + tmpFx[188]*tmpObjSEndTerm[156] + tmpFx[204]*tmpObjSEndTerm[170] + tmpFx[220]*tmpObjSEndTerm[184];
tmpQN2[171] = + tmpFx[12]*tmpObjSEndTerm[3] + tmpFx[28]*tmpObjSEndTerm[17] + tmpFx[44]*tmpObjSEndTerm[31] + tmpFx[60]*tmpObjSEndTerm[45] + tmpFx[76]*tmpObjSEndTerm[59] + tmpFx[92]*tmpObjSEndTerm[73] + tmpFx[108]*tmpObjSEndTerm[87] + tmpFx[124]*tmpObjSEndTerm[101] + tmpFx[140]*tmpObjSEndTerm[115] + tmpFx[156]*tmpObjSEndTerm[129] + tmpFx[172]*tmpObjSEndTerm[143] + tmpFx[188]*tmpObjSEndTerm[157] + tmpFx[204]*tmpObjSEndTerm[171] + tmpFx[220]*tmpObjSEndTerm[185];
tmpQN2[172] = + tmpFx[12]*tmpObjSEndTerm[4] + tmpFx[28]*tmpObjSEndTerm[18] + tmpFx[44]*tmpObjSEndTerm[32] + tmpFx[60]*tmpObjSEndTerm[46] + tmpFx[76]*tmpObjSEndTerm[60] + tmpFx[92]*tmpObjSEndTerm[74] + tmpFx[108]*tmpObjSEndTerm[88] + tmpFx[124]*tmpObjSEndTerm[102] + tmpFx[140]*tmpObjSEndTerm[116] + tmpFx[156]*tmpObjSEndTerm[130] + tmpFx[172]*tmpObjSEndTerm[144] + tmpFx[188]*tmpObjSEndTerm[158] + tmpFx[204]*tmpObjSEndTerm[172] + tmpFx[220]*tmpObjSEndTerm[186];
tmpQN2[173] = + tmpFx[12]*tmpObjSEndTerm[5] + tmpFx[28]*tmpObjSEndTerm[19] + tmpFx[44]*tmpObjSEndTerm[33] + tmpFx[60]*tmpObjSEndTerm[47] + tmpFx[76]*tmpObjSEndTerm[61] + tmpFx[92]*tmpObjSEndTerm[75] + tmpFx[108]*tmpObjSEndTerm[89] + tmpFx[124]*tmpObjSEndTerm[103] + tmpFx[140]*tmpObjSEndTerm[117] + tmpFx[156]*tmpObjSEndTerm[131] + tmpFx[172]*tmpObjSEndTerm[145] + tmpFx[188]*tmpObjSEndTerm[159] + tmpFx[204]*tmpObjSEndTerm[173] + tmpFx[220]*tmpObjSEndTerm[187];
tmpQN2[174] = + tmpFx[12]*tmpObjSEndTerm[6] + tmpFx[28]*tmpObjSEndTerm[20] + tmpFx[44]*tmpObjSEndTerm[34] + tmpFx[60]*tmpObjSEndTerm[48] + tmpFx[76]*tmpObjSEndTerm[62] + tmpFx[92]*tmpObjSEndTerm[76] + tmpFx[108]*tmpObjSEndTerm[90] + tmpFx[124]*tmpObjSEndTerm[104] + tmpFx[140]*tmpObjSEndTerm[118] + tmpFx[156]*tmpObjSEndTerm[132] + tmpFx[172]*tmpObjSEndTerm[146] + tmpFx[188]*tmpObjSEndTerm[160] + tmpFx[204]*tmpObjSEndTerm[174] + tmpFx[220]*tmpObjSEndTerm[188];
tmpQN2[175] = + tmpFx[12]*tmpObjSEndTerm[7] + tmpFx[28]*tmpObjSEndTerm[21] + tmpFx[44]*tmpObjSEndTerm[35] + tmpFx[60]*tmpObjSEndTerm[49] + tmpFx[76]*tmpObjSEndTerm[63] + tmpFx[92]*tmpObjSEndTerm[77] + tmpFx[108]*tmpObjSEndTerm[91] + tmpFx[124]*tmpObjSEndTerm[105] + tmpFx[140]*tmpObjSEndTerm[119] + tmpFx[156]*tmpObjSEndTerm[133] + tmpFx[172]*tmpObjSEndTerm[147] + tmpFx[188]*tmpObjSEndTerm[161] + tmpFx[204]*tmpObjSEndTerm[175] + tmpFx[220]*tmpObjSEndTerm[189];
tmpQN2[176] = + tmpFx[12]*tmpObjSEndTerm[8] + tmpFx[28]*tmpObjSEndTerm[22] + tmpFx[44]*tmpObjSEndTerm[36] + tmpFx[60]*tmpObjSEndTerm[50] + tmpFx[76]*tmpObjSEndTerm[64] + tmpFx[92]*tmpObjSEndTerm[78] + tmpFx[108]*tmpObjSEndTerm[92] + tmpFx[124]*tmpObjSEndTerm[106] + tmpFx[140]*tmpObjSEndTerm[120] + tmpFx[156]*tmpObjSEndTerm[134] + tmpFx[172]*tmpObjSEndTerm[148] + tmpFx[188]*tmpObjSEndTerm[162] + tmpFx[204]*tmpObjSEndTerm[176] + tmpFx[220]*tmpObjSEndTerm[190];
tmpQN2[177] = + tmpFx[12]*tmpObjSEndTerm[9] + tmpFx[28]*tmpObjSEndTerm[23] + tmpFx[44]*tmpObjSEndTerm[37] + tmpFx[60]*tmpObjSEndTerm[51] + tmpFx[76]*tmpObjSEndTerm[65] + tmpFx[92]*tmpObjSEndTerm[79] + tmpFx[108]*tmpObjSEndTerm[93] + tmpFx[124]*tmpObjSEndTerm[107] + tmpFx[140]*tmpObjSEndTerm[121] + tmpFx[156]*tmpObjSEndTerm[135] + tmpFx[172]*tmpObjSEndTerm[149] + tmpFx[188]*tmpObjSEndTerm[163] + tmpFx[204]*tmpObjSEndTerm[177] + tmpFx[220]*tmpObjSEndTerm[191];
tmpQN2[178] = + tmpFx[12]*tmpObjSEndTerm[10] + tmpFx[28]*tmpObjSEndTerm[24] + tmpFx[44]*tmpObjSEndTerm[38] + tmpFx[60]*tmpObjSEndTerm[52] + tmpFx[76]*tmpObjSEndTerm[66] + tmpFx[92]*tmpObjSEndTerm[80] + tmpFx[108]*tmpObjSEndTerm[94] + tmpFx[124]*tmpObjSEndTerm[108] + tmpFx[140]*tmpObjSEndTerm[122] + tmpFx[156]*tmpObjSEndTerm[136] + tmpFx[172]*tmpObjSEndTerm[150] + tmpFx[188]*tmpObjSEndTerm[164] + tmpFx[204]*tmpObjSEndTerm[178] + tmpFx[220]*tmpObjSEndTerm[192];
tmpQN2[179] = + tmpFx[12]*tmpObjSEndTerm[11] + tmpFx[28]*tmpObjSEndTerm[25] + tmpFx[44]*tmpObjSEndTerm[39] + tmpFx[60]*tmpObjSEndTerm[53] + tmpFx[76]*tmpObjSEndTerm[67] + tmpFx[92]*tmpObjSEndTerm[81] + tmpFx[108]*tmpObjSEndTerm[95] + tmpFx[124]*tmpObjSEndTerm[109] + tmpFx[140]*tmpObjSEndTerm[123] + tmpFx[156]*tmpObjSEndTerm[137] + tmpFx[172]*tmpObjSEndTerm[151] + tmpFx[188]*tmpObjSEndTerm[165] + tmpFx[204]*tmpObjSEndTerm[179] + tmpFx[220]*tmpObjSEndTerm[193];
tmpQN2[180] = + tmpFx[12]*tmpObjSEndTerm[12] + tmpFx[28]*tmpObjSEndTerm[26] + tmpFx[44]*tmpObjSEndTerm[40] + tmpFx[60]*tmpObjSEndTerm[54] + tmpFx[76]*tmpObjSEndTerm[68] + tmpFx[92]*tmpObjSEndTerm[82] + tmpFx[108]*tmpObjSEndTerm[96] + tmpFx[124]*tmpObjSEndTerm[110] + tmpFx[140]*tmpObjSEndTerm[124] + tmpFx[156]*tmpObjSEndTerm[138] + tmpFx[172]*tmpObjSEndTerm[152] + tmpFx[188]*tmpObjSEndTerm[166] + tmpFx[204]*tmpObjSEndTerm[180] + tmpFx[220]*tmpObjSEndTerm[194];
tmpQN2[181] = + tmpFx[12]*tmpObjSEndTerm[13] + tmpFx[28]*tmpObjSEndTerm[27] + tmpFx[44]*tmpObjSEndTerm[41] + tmpFx[60]*tmpObjSEndTerm[55] + tmpFx[76]*tmpObjSEndTerm[69] + tmpFx[92]*tmpObjSEndTerm[83] + tmpFx[108]*tmpObjSEndTerm[97] + tmpFx[124]*tmpObjSEndTerm[111] + tmpFx[140]*tmpObjSEndTerm[125] + tmpFx[156]*tmpObjSEndTerm[139] + tmpFx[172]*tmpObjSEndTerm[153] + tmpFx[188]*tmpObjSEndTerm[167] + tmpFx[204]*tmpObjSEndTerm[181] + tmpFx[220]*tmpObjSEndTerm[195];
tmpQN2[182] = + tmpFx[13]*tmpObjSEndTerm[0] + tmpFx[29]*tmpObjSEndTerm[14] + tmpFx[45]*tmpObjSEndTerm[28] + tmpFx[61]*tmpObjSEndTerm[42] + tmpFx[77]*tmpObjSEndTerm[56] + tmpFx[93]*tmpObjSEndTerm[70] + tmpFx[109]*tmpObjSEndTerm[84] + tmpFx[125]*tmpObjSEndTerm[98] + tmpFx[141]*tmpObjSEndTerm[112] + tmpFx[157]*tmpObjSEndTerm[126] + tmpFx[173]*tmpObjSEndTerm[140] + tmpFx[189]*tmpObjSEndTerm[154] + tmpFx[205]*tmpObjSEndTerm[168] + tmpFx[221]*tmpObjSEndTerm[182];
tmpQN2[183] = + tmpFx[13]*tmpObjSEndTerm[1] + tmpFx[29]*tmpObjSEndTerm[15] + tmpFx[45]*tmpObjSEndTerm[29] + tmpFx[61]*tmpObjSEndTerm[43] + tmpFx[77]*tmpObjSEndTerm[57] + tmpFx[93]*tmpObjSEndTerm[71] + tmpFx[109]*tmpObjSEndTerm[85] + tmpFx[125]*tmpObjSEndTerm[99] + tmpFx[141]*tmpObjSEndTerm[113] + tmpFx[157]*tmpObjSEndTerm[127] + tmpFx[173]*tmpObjSEndTerm[141] + tmpFx[189]*tmpObjSEndTerm[155] + tmpFx[205]*tmpObjSEndTerm[169] + tmpFx[221]*tmpObjSEndTerm[183];
tmpQN2[184] = + tmpFx[13]*tmpObjSEndTerm[2] + tmpFx[29]*tmpObjSEndTerm[16] + tmpFx[45]*tmpObjSEndTerm[30] + tmpFx[61]*tmpObjSEndTerm[44] + tmpFx[77]*tmpObjSEndTerm[58] + tmpFx[93]*tmpObjSEndTerm[72] + tmpFx[109]*tmpObjSEndTerm[86] + tmpFx[125]*tmpObjSEndTerm[100] + tmpFx[141]*tmpObjSEndTerm[114] + tmpFx[157]*tmpObjSEndTerm[128] + tmpFx[173]*tmpObjSEndTerm[142] + tmpFx[189]*tmpObjSEndTerm[156] + tmpFx[205]*tmpObjSEndTerm[170] + tmpFx[221]*tmpObjSEndTerm[184];
tmpQN2[185] = + tmpFx[13]*tmpObjSEndTerm[3] + tmpFx[29]*tmpObjSEndTerm[17] + tmpFx[45]*tmpObjSEndTerm[31] + tmpFx[61]*tmpObjSEndTerm[45] + tmpFx[77]*tmpObjSEndTerm[59] + tmpFx[93]*tmpObjSEndTerm[73] + tmpFx[109]*tmpObjSEndTerm[87] + tmpFx[125]*tmpObjSEndTerm[101] + tmpFx[141]*tmpObjSEndTerm[115] + tmpFx[157]*tmpObjSEndTerm[129] + tmpFx[173]*tmpObjSEndTerm[143] + tmpFx[189]*tmpObjSEndTerm[157] + tmpFx[205]*tmpObjSEndTerm[171] + tmpFx[221]*tmpObjSEndTerm[185];
tmpQN2[186] = + tmpFx[13]*tmpObjSEndTerm[4] + tmpFx[29]*tmpObjSEndTerm[18] + tmpFx[45]*tmpObjSEndTerm[32] + tmpFx[61]*tmpObjSEndTerm[46] + tmpFx[77]*tmpObjSEndTerm[60] + tmpFx[93]*tmpObjSEndTerm[74] + tmpFx[109]*tmpObjSEndTerm[88] + tmpFx[125]*tmpObjSEndTerm[102] + tmpFx[141]*tmpObjSEndTerm[116] + tmpFx[157]*tmpObjSEndTerm[130] + tmpFx[173]*tmpObjSEndTerm[144] + tmpFx[189]*tmpObjSEndTerm[158] + tmpFx[205]*tmpObjSEndTerm[172] + tmpFx[221]*tmpObjSEndTerm[186];
tmpQN2[187] = + tmpFx[13]*tmpObjSEndTerm[5] + tmpFx[29]*tmpObjSEndTerm[19] + tmpFx[45]*tmpObjSEndTerm[33] + tmpFx[61]*tmpObjSEndTerm[47] + tmpFx[77]*tmpObjSEndTerm[61] + tmpFx[93]*tmpObjSEndTerm[75] + tmpFx[109]*tmpObjSEndTerm[89] + tmpFx[125]*tmpObjSEndTerm[103] + tmpFx[141]*tmpObjSEndTerm[117] + tmpFx[157]*tmpObjSEndTerm[131] + tmpFx[173]*tmpObjSEndTerm[145] + tmpFx[189]*tmpObjSEndTerm[159] + tmpFx[205]*tmpObjSEndTerm[173] + tmpFx[221]*tmpObjSEndTerm[187];
tmpQN2[188] = + tmpFx[13]*tmpObjSEndTerm[6] + tmpFx[29]*tmpObjSEndTerm[20] + tmpFx[45]*tmpObjSEndTerm[34] + tmpFx[61]*tmpObjSEndTerm[48] + tmpFx[77]*tmpObjSEndTerm[62] + tmpFx[93]*tmpObjSEndTerm[76] + tmpFx[109]*tmpObjSEndTerm[90] + tmpFx[125]*tmpObjSEndTerm[104] + tmpFx[141]*tmpObjSEndTerm[118] + tmpFx[157]*tmpObjSEndTerm[132] + tmpFx[173]*tmpObjSEndTerm[146] + tmpFx[189]*tmpObjSEndTerm[160] + tmpFx[205]*tmpObjSEndTerm[174] + tmpFx[221]*tmpObjSEndTerm[188];
tmpQN2[189] = + tmpFx[13]*tmpObjSEndTerm[7] + tmpFx[29]*tmpObjSEndTerm[21] + tmpFx[45]*tmpObjSEndTerm[35] + tmpFx[61]*tmpObjSEndTerm[49] + tmpFx[77]*tmpObjSEndTerm[63] + tmpFx[93]*tmpObjSEndTerm[77] + tmpFx[109]*tmpObjSEndTerm[91] + tmpFx[125]*tmpObjSEndTerm[105] + tmpFx[141]*tmpObjSEndTerm[119] + tmpFx[157]*tmpObjSEndTerm[133] + tmpFx[173]*tmpObjSEndTerm[147] + tmpFx[189]*tmpObjSEndTerm[161] + tmpFx[205]*tmpObjSEndTerm[175] + tmpFx[221]*tmpObjSEndTerm[189];
tmpQN2[190] = + tmpFx[13]*tmpObjSEndTerm[8] + tmpFx[29]*tmpObjSEndTerm[22] + tmpFx[45]*tmpObjSEndTerm[36] + tmpFx[61]*tmpObjSEndTerm[50] + tmpFx[77]*tmpObjSEndTerm[64] + tmpFx[93]*tmpObjSEndTerm[78] + tmpFx[109]*tmpObjSEndTerm[92] + tmpFx[125]*tmpObjSEndTerm[106] + tmpFx[141]*tmpObjSEndTerm[120] + tmpFx[157]*tmpObjSEndTerm[134] + tmpFx[173]*tmpObjSEndTerm[148] + tmpFx[189]*tmpObjSEndTerm[162] + tmpFx[205]*tmpObjSEndTerm[176] + tmpFx[221]*tmpObjSEndTerm[190];
tmpQN2[191] = + tmpFx[13]*tmpObjSEndTerm[9] + tmpFx[29]*tmpObjSEndTerm[23] + tmpFx[45]*tmpObjSEndTerm[37] + tmpFx[61]*tmpObjSEndTerm[51] + tmpFx[77]*tmpObjSEndTerm[65] + tmpFx[93]*tmpObjSEndTerm[79] + tmpFx[109]*tmpObjSEndTerm[93] + tmpFx[125]*tmpObjSEndTerm[107] + tmpFx[141]*tmpObjSEndTerm[121] + tmpFx[157]*tmpObjSEndTerm[135] + tmpFx[173]*tmpObjSEndTerm[149] + tmpFx[189]*tmpObjSEndTerm[163] + tmpFx[205]*tmpObjSEndTerm[177] + tmpFx[221]*tmpObjSEndTerm[191];
tmpQN2[192] = + tmpFx[13]*tmpObjSEndTerm[10] + tmpFx[29]*tmpObjSEndTerm[24] + tmpFx[45]*tmpObjSEndTerm[38] + tmpFx[61]*tmpObjSEndTerm[52] + tmpFx[77]*tmpObjSEndTerm[66] + tmpFx[93]*tmpObjSEndTerm[80] + tmpFx[109]*tmpObjSEndTerm[94] + tmpFx[125]*tmpObjSEndTerm[108] + tmpFx[141]*tmpObjSEndTerm[122] + tmpFx[157]*tmpObjSEndTerm[136] + tmpFx[173]*tmpObjSEndTerm[150] + tmpFx[189]*tmpObjSEndTerm[164] + tmpFx[205]*tmpObjSEndTerm[178] + tmpFx[221]*tmpObjSEndTerm[192];
tmpQN2[193] = + tmpFx[13]*tmpObjSEndTerm[11] + tmpFx[29]*tmpObjSEndTerm[25] + tmpFx[45]*tmpObjSEndTerm[39] + tmpFx[61]*tmpObjSEndTerm[53] + tmpFx[77]*tmpObjSEndTerm[67] + tmpFx[93]*tmpObjSEndTerm[81] + tmpFx[109]*tmpObjSEndTerm[95] + tmpFx[125]*tmpObjSEndTerm[109] + tmpFx[141]*tmpObjSEndTerm[123] + tmpFx[157]*tmpObjSEndTerm[137] + tmpFx[173]*tmpObjSEndTerm[151] + tmpFx[189]*tmpObjSEndTerm[165] + tmpFx[205]*tmpObjSEndTerm[179] + tmpFx[221]*tmpObjSEndTerm[193];
tmpQN2[194] = + tmpFx[13]*tmpObjSEndTerm[12] + tmpFx[29]*tmpObjSEndTerm[26] + tmpFx[45]*tmpObjSEndTerm[40] + tmpFx[61]*tmpObjSEndTerm[54] + tmpFx[77]*tmpObjSEndTerm[68] + tmpFx[93]*tmpObjSEndTerm[82] + tmpFx[109]*tmpObjSEndTerm[96] + tmpFx[125]*tmpObjSEndTerm[110] + tmpFx[141]*tmpObjSEndTerm[124] + tmpFx[157]*tmpObjSEndTerm[138] + tmpFx[173]*tmpObjSEndTerm[152] + tmpFx[189]*tmpObjSEndTerm[166] + tmpFx[205]*tmpObjSEndTerm[180] + tmpFx[221]*tmpObjSEndTerm[194];
tmpQN2[195] = + tmpFx[13]*tmpObjSEndTerm[13] + tmpFx[29]*tmpObjSEndTerm[27] + tmpFx[45]*tmpObjSEndTerm[41] + tmpFx[61]*tmpObjSEndTerm[55] + tmpFx[77]*tmpObjSEndTerm[69] + tmpFx[93]*tmpObjSEndTerm[83] + tmpFx[109]*tmpObjSEndTerm[97] + tmpFx[125]*tmpObjSEndTerm[111] + tmpFx[141]*tmpObjSEndTerm[125] + tmpFx[157]*tmpObjSEndTerm[139] + tmpFx[173]*tmpObjSEndTerm[153] + tmpFx[189]*tmpObjSEndTerm[167] + tmpFx[205]*tmpObjSEndTerm[181] + tmpFx[221]*tmpObjSEndTerm[195];
tmpQN2[196] = + tmpFx[14]*tmpObjSEndTerm[0] + tmpFx[30]*tmpObjSEndTerm[14] + tmpFx[46]*tmpObjSEndTerm[28] + tmpFx[62]*tmpObjSEndTerm[42] + tmpFx[78]*tmpObjSEndTerm[56] + tmpFx[94]*tmpObjSEndTerm[70] + tmpFx[110]*tmpObjSEndTerm[84] + tmpFx[126]*tmpObjSEndTerm[98] + tmpFx[142]*tmpObjSEndTerm[112] + tmpFx[158]*tmpObjSEndTerm[126] + tmpFx[174]*tmpObjSEndTerm[140] + tmpFx[190]*tmpObjSEndTerm[154] + tmpFx[206]*tmpObjSEndTerm[168] + tmpFx[222]*tmpObjSEndTerm[182];
tmpQN2[197] = + tmpFx[14]*tmpObjSEndTerm[1] + tmpFx[30]*tmpObjSEndTerm[15] + tmpFx[46]*tmpObjSEndTerm[29] + tmpFx[62]*tmpObjSEndTerm[43] + tmpFx[78]*tmpObjSEndTerm[57] + tmpFx[94]*tmpObjSEndTerm[71] + tmpFx[110]*tmpObjSEndTerm[85] + tmpFx[126]*tmpObjSEndTerm[99] + tmpFx[142]*tmpObjSEndTerm[113] + tmpFx[158]*tmpObjSEndTerm[127] + tmpFx[174]*tmpObjSEndTerm[141] + tmpFx[190]*tmpObjSEndTerm[155] + tmpFx[206]*tmpObjSEndTerm[169] + tmpFx[222]*tmpObjSEndTerm[183];
tmpQN2[198] = + tmpFx[14]*tmpObjSEndTerm[2] + tmpFx[30]*tmpObjSEndTerm[16] + tmpFx[46]*tmpObjSEndTerm[30] + tmpFx[62]*tmpObjSEndTerm[44] + tmpFx[78]*tmpObjSEndTerm[58] + tmpFx[94]*tmpObjSEndTerm[72] + tmpFx[110]*tmpObjSEndTerm[86] + tmpFx[126]*tmpObjSEndTerm[100] + tmpFx[142]*tmpObjSEndTerm[114] + tmpFx[158]*tmpObjSEndTerm[128] + tmpFx[174]*tmpObjSEndTerm[142] + tmpFx[190]*tmpObjSEndTerm[156] + tmpFx[206]*tmpObjSEndTerm[170] + tmpFx[222]*tmpObjSEndTerm[184];
tmpQN2[199] = + tmpFx[14]*tmpObjSEndTerm[3] + tmpFx[30]*tmpObjSEndTerm[17] + tmpFx[46]*tmpObjSEndTerm[31] + tmpFx[62]*tmpObjSEndTerm[45] + tmpFx[78]*tmpObjSEndTerm[59] + tmpFx[94]*tmpObjSEndTerm[73] + tmpFx[110]*tmpObjSEndTerm[87] + tmpFx[126]*tmpObjSEndTerm[101] + tmpFx[142]*tmpObjSEndTerm[115] + tmpFx[158]*tmpObjSEndTerm[129] + tmpFx[174]*tmpObjSEndTerm[143] + tmpFx[190]*tmpObjSEndTerm[157] + tmpFx[206]*tmpObjSEndTerm[171] + tmpFx[222]*tmpObjSEndTerm[185];
tmpQN2[200] = + tmpFx[14]*tmpObjSEndTerm[4] + tmpFx[30]*tmpObjSEndTerm[18] + tmpFx[46]*tmpObjSEndTerm[32] + tmpFx[62]*tmpObjSEndTerm[46] + tmpFx[78]*tmpObjSEndTerm[60] + tmpFx[94]*tmpObjSEndTerm[74] + tmpFx[110]*tmpObjSEndTerm[88] + tmpFx[126]*tmpObjSEndTerm[102] + tmpFx[142]*tmpObjSEndTerm[116] + tmpFx[158]*tmpObjSEndTerm[130] + tmpFx[174]*tmpObjSEndTerm[144] + tmpFx[190]*tmpObjSEndTerm[158] + tmpFx[206]*tmpObjSEndTerm[172] + tmpFx[222]*tmpObjSEndTerm[186];
tmpQN2[201] = + tmpFx[14]*tmpObjSEndTerm[5] + tmpFx[30]*tmpObjSEndTerm[19] + tmpFx[46]*tmpObjSEndTerm[33] + tmpFx[62]*tmpObjSEndTerm[47] + tmpFx[78]*tmpObjSEndTerm[61] + tmpFx[94]*tmpObjSEndTerm[75] + tmpFx[110]*tmpObjSEndTerm[89] + tmpFx[126]*tmpObjSEndTerm[103] + tmpFx[142]*tmpObjSEndTerm[117] + tmpFx[158]*tmpObjSEndTerm[131] + tmpFx[174]*tmpObjSEndTerm[145] + tmpFx[190]*tmpObjSEndTerm[159] + tmpFx[206]*tmpObjSEndTerm[173] + tmpFx[222]*tmpObjSEndTerm[187];
tmpQN2[202] = + tmpFx[14]*tmpObjSEndTerm[6] + tmpFx[30]*tmpObjSEndTerm[20] + tmpFx[46]*tmpObjSEndTerm[34] + tmpFx[62]*tmpObjSEndTerm[48] + tmpFx[78]*tmpObjSEndTerm[62] + tmpFx[94]*tmpObjSEndTerm[76] + tmpFx[110]*tmpObjSEndTerm[90] + tmpFx[126]*tmpObjSEndTerm[104] + tmpFx[142]*tmpObjSEndTerm[118] + tmpFx[158]*tmpObjSEndTerm[132] + tmpFx[174]*tmpObjSEndTerm[146] + tmpFx[190]*tmpObjSEndTerm[160] + tmpFx[206]*tmpObjSEndTerm[174] + tmpFx[222]*tmpObjSEndTerm[188];
tmpQN2[203] = + tmpFx[14]*tmpObjSEndTerm[7] + tmpFx[30]*tmpObjSEndTerm[21] + tmpFx[46]*tmpObjSEndTerm[35] + tmpFx[62]*tmpObjSEndTerm[49] + tmpFx[78]*tmpObjSEndTerm[63] + tmpFx[94]*tmpObjSEndTerm[77] + tmpFx[110]*tmpObjSEndTerm[91] + tmpFx[126]*tmpObjSEndTerm[105] + tmpFx[142]*tmpObjSEndTerm[119] + tmpFx[158]*tmpObjSEndTerm[133] + tmpFx[174]*tmpObjSEndTerm[147] + tmpFx[190]*tmpObjSEndTerm[161] + tmpFx[206]*tmpObjSEndTerm[175] + tmpFx[222]*tmpObjSEndTerm[189];
tmpQN2[204] = + tmpFx[14]*tmpObjSEndTerm[8] + tmpFx[30]*tmpObjSEndTerm[22] + tmpFx[46]*tmpObjSEndTerm[36] + tmpFx[62]*tmpObjSEndTerm[50] + tmpFx[78]*tmpObjSEndTerm[64] + tmpFx[94]*tmpObjSEndTerm[78] + tmpFx[110]*tmpObjSEndTerm[92] + tmpFx[126]*tmpObjSEndTerm[106] + tmpFx[142]*tmpObjSEndTerm[120] + tmpFx[158]*tmpObjSEndTerm[134] + tmpFx[174]*tmpObjSEndTerm[148] + tmpFx[190]*tmpObjSEndTerm[162] + tmpFx[206]*tmpObjSEndTerm[176] + tmpFx[222]*tmpObjSEndTerm[190];
tmpQN2[205] = + tmpFx[14]*tmpObjSEndTerm[9] + tmpFx[30]*tmpObjSEndTerm[23] + tmpFx[46]*tmpObjSEndTerm[37] + tmpFx[62]*tmpObjSEndTerm[51] + tmpFx[78]*tmpObjSEndTerm[65] + tmpFx[94]*tmpObjSEndTerm[79] + tmpFx[110]*tmpObjSEndTerm[93] + tmpFx[126]*tmpObjSEndTerm[107] + tmpFx[142]*tmpObjSEndTerm[121] + tmpFx[158]*tmpObjSEndTerm[135] + tmpFx[174]*tmpObjSEndTerm[149] + tmpFx[190]*tmpObjSEndTerm[163] + tmpFx[206]*tmpObjSEndTerm[177] + tmpFx[222]*tmpObjSEndTerm[191];
tmpQN2[206] = + tmpFx[14]*tmpObjSEndTerm[10] + tmpFx[30]*tmpObjSEndTerm[24] + tmpFx[46]*tmpObjSEndTerm[38] + tmpFx[62]*tmpObjSEndTerm[52] + tmpFx[78]*tmpObjSEndTerm[66] + tmpFx[94]*tmpObjSEndTerm[80] + tmpFx[110]*tmpObjSEndTerm[94] + tmpFx[126]*tmpObjSEndTerm[108] + tmpFx[142]*tmpObjSEndTerm[122] + tmpFx[158]*tmpObjSEndTerm[136] + tmpFx[174]*tmpObjSEndTerm[150] + tmpFx[190]*tmpObjSEndTerm[164] + tmpFx[206]*tmpObjSEndTerm[178] + tmpFx[222]*tmpObjSEndTerm[192];
tmpQN2[207] = + tmpFx[14]*tmpObjSEndTerm[11] + tmpFx[30]*tmpObjSEndTerm[25] + tmpFx[46]*tmpObjSEndTerm[39] + tmpFx[62]*tmpObjSEndTerm[53] + tmpFx[78]*tmpObjSEndTerm[67] + tmpFx[94]*tmpObjSEndTerm[81] + tmpFx[110]*tmpObjSEndTerm[95] + tmpFx[126]*tmpObjSEndTerm[109] + tmpFx[142]*tmpObjSEndTerm[123] + tmpFx[158]*tmpObjSEndTerm[137] + tmpFx[174]*tmpObjSEndTerm[151] + tmpFx[190]*tmpObjSEndTerm[165] + tmpFx[206]*tmpObjSEndTerm[179] + tmpFx[222]*tmpObjSEndTerm[193];
tmpQN2[208] = + tmpFx[14]*tmpObjSEndTerm[12] + tmpFx[30]*tmpObjSEndTerm[26] + tmpFx[46]*tmpObjSEndTerm[40] + tmpFx[62]*tmpObjSEndTerm[54] + tmpFx[78]*tmpObjSEndTerm[68] + tmpFx[94]*tmpObjSEndTerm[82] + tmpFx[110]*tmpObjSEndTerm[96] + tmpFx[126]*tmpObjSEndTerm[110] + tmpFx[142]*tmpObjSEndTerm[124] + tmpFx[158]*tmpObjSEndTerm[138] + tmpFx[174]*tmpObjSEndTerm[152] + tmpFx[190]*tmpObjSEndTerm[166] + tmpFx[206]*tmpObjSEndTerm[180] + tmpFx[222]*tmpObjSEndTerm[194];
tmpQN2[209] = + tmpFx[14]*tmpObjSEndTerm[13] + tmpFx[30]*tmpObjSEndTerm[27] + tmpFx[46]*tmpObjSEndTerm[41] + tmpFx[62]*tmpObjSEndTerm[55] + tmpFx[78]*tmpObjSEndTerm[69] + tmpFx[94]*tmpObjSEndTerm[83] + tmpFx[110]*tmpObjSEndTerm[97] + tmpFx[126]*tmpObjSEndTerm[111] + tmpFx[142]*tmpObjSEndTerm[125] + tmpFx[158]*tmpObjSEndTerm[139] + tmpFx[174]*tmpObjSEndTerm[153] + tmpFx[190]*tmpObjSEndTerm[167] + tmpFx[206]*tmpObjSEndTerm[181] + tmpFx[222]*tmpObjSEndTerm[195];
tmpQN2[210] = + tmpFx[15]*tmpObjSEndTerm[0] + tmpFx[31]*tmpObjSEndTerm[14] + tmpFx[47]*tmpObjSEndTerm[28] + tmpFx[63]*tmpObjSEndTerm[42] + tmpFx[79]*tmpObjSEndTerm[56] + tmpFx[95]*tmpObjSEndTerm[70] + tmpFx[111]*tmpObjSEndTerm[84] + tmpFx[127]*tmpObjSEndTerm[98] + tmpFx[143]*tmpObjSEndTerm[112] + tmpFx[159]*tmpObjSEndTerm[126] + tmpFx[175]*tmpObjSEndTerm[140] + tmpFx[191]*tmpObjSEndTerm[154] + tmpFx[207]*tmpObjSEndTerm[168] + tmpFx[223]*tmpObjSEndTerm[182];
tmpQN2[211] = + tmpFx[15]*tmpObjSEndTerm[1] + tmpFx[31]*tmpObjSEndTerm[15] + tmpFx[47]*tmpObjSEndTerm[29] + tmpFx[63]*tmpObjSEndTerm[43] + tmpFx[79]*tmpObjSEndTerm[57] + tmpFx[95]*tmpObjSEndTerm[71] + tmpFx[111]*tmpObjSEndTerm[85] + tmpFx[127]*tmpObjSEndTerm[99] + tmpFx[143]*tmpObjSEndTerm[113] + tmpFx[159]*tmpObjSEndTerm[127] + tmpFx[175]*tmpObjSEndTerm[141] + tmpFx[191]*tmpObjSEndTerm[155] + tmpFx[207]*tmpObjSEndTerm[169] + tmpFx[223]*tmpObjSEndTerm[183];
tmpQN2[212] = + tmpFx[15]*tmpObjSEndTerm[2] + tmpFx[31]*tmpObjSEndTerm[16] + tmpFx[47]*tmpObjSEndTerm[30] + tmpFx[63]*tmpObjSEndTerm[44] + tmpFx[79]*tmpObjSEndTerm[58] + tmpFx[95]*tmpObjSEndTerm[72] + tmpFx[111]*tmpObjSEndTerm[86] + tmpFx[127]*tmpObjSEndTerm[100] + tmpFx[143]*tmpObjSEndTerm[114] + tmpFx[159]*tmpObjSEndTerm[128] + tmpFx[175]*tmpObjSEndTerm[142] + tmpFx[191]*tmpObjSEndTerm[156] + tmpFx[207]*tmpObjSEndTerm[170] + tmpFx[223]*tmpObjSEndTerm[184];
tmpQN2[213] = + tmpFx[15]*tmpObjSEndTerm[3] + tmpFx[31]*tmpObjSEndTerm[17] + tmpFx[47]*tmpObjSEndTerm[31] + tmpFx[63]*tmpObjSEndTerm[45] + tmpFx[79]*tmpObjSEndTerm[59] + tmpFx[95]*tmpObjSEndTerm[73] + tmpFx[111]*tmpObjSEndTerm[87] + tmpFx[127]*tmpObjSEndTerm[101] + tmpFx[143]*tmpObjSEndTerm[115] + tmpFx[159]*tmpObjSEndTerm[129] + tmpFx[175]*tmpObjSEndTerm[143] + tmpFx[191]*tmpObjSEndTerm[157] + tmpFx[207]*tmpObjSEndTerm[171] + tmpFx[223]*tmpObjSEndTerm[185];
tmpQN2[214] = + tmpFx[15]*tmpObjSEndTerm[4] + tmpFx[31]*tmpObjSEndTerm[18] + tmpFx[47]*tmpObjSEndTerm[32] + tmpFx[63]*tmpObjSEndTerm[46] + tmpFx[79]*tmpObjSEndTerm[60] + tmpFx[95]*tmpObjSEndTerm[74] + tmpFx[111]*tmpObjSEndTerm[88] + tmpFx[127]*tmpObjSEndTerm[102] + tmpFx[143]*tmpObjSEndTerm[116] + tmpFx[159]*tmpObjSEndTerm[130] + tmpFx[175]*tmpObjSEndTerm[144] + tmpFx[191]*tmpObjSEndTerm[158] + tmpFx[207]*tmpObjSEndTerm[172] + tmpFx[223]*tmpObjSEndTerm[186];
tmpQN2[215] = + tmpFx[15]*tmpObjSEndTerm[5] + tmpFx[31]*tmpObjSEndTerm[19] + tmpFx[47]*tmpObjSEndTerm[33] + tmpFx[63]*tmpObjSEndTerm[47] + tmpFx[79]*tmpObjSEndTerm[61] + tmpFx[95]*tmpObjSEndTerm[75] + tmpFx[111]*tmpObjSEndTerm[89] + tmpFx[127]*tmpObjSEndTerm[103] + tmpFx[143]*tmpObjSEndTerm[117] + tmpFx[159]*tmpObjSEndTerm[131] + tmpFx[175]*tmpObjSEndTerm[145] + tmpFx[191]*tmpObjSEndTerm[159] + tmpFx[207]*tmpObjSEndTerm[173] + tmpFx[223]*tmpObjSEndTerm[187];
tmpQN2[216] = + tmpFx[15]*tmpObjSEndTerm[6] + tmpFx[31]*tmpObjSEndTerm[20] + tmpFx[47]*tmpObjSEndTerm[34] + tmpFx[63]*tmpObjSEndTerm[48] + tmpFx[79]*tmpObjSEndTerm[62] + tmpFx[95]*tmpObjSEndTerm[76] + tmpFx[111]*tmpObjSEndTerm[90] + tmpFx[127]*tmpObjSEndTerm[104] + tmpFx[143]*tmpObjSEndTerm[118] + tmpFx[159]*tmpObjSEndTerm[132] + tmpFx[175]*tmpObjSEndTerm[146] + tmpFx[191]*tmpObjSEndTerm[160] + tmpFx[207]*tmpObjSEndTerm[174] + tmpFx[223]*tmpObjSEndTerm[188];
tmpQN2[217] = + tmpFx[15]*tmpObjSEndTerm[7] + tmpFx[31]*tmpObjSEndTerm[21] + tmpFx[47]*tmpObjSEndTerm[35] + tmpFx[63]*tmpObjSEndTerm[49] + tmpFx[79]*tmpObjSEndTerm[63] + tmpFx[95]*tmpObjSEndTerm[77] + tmpFx[111]*tmpObjSEndTerm[91] + tmpFx[127]*tmpObjSEndTerm[105] + tmpFx[143]*tmpObjSEndTerm[119] + tmpFx[159]*tmpObjSEndTerm[133] + tmpFx[175]*tmpObjSEndTerm[147] + tmpFx[191]*tmpObjSEndTerm[161] + tmpFx[207]*tmpObjSEndTerm[175] + tmpFx[223]*tmpObjSEndTerm[189];
tmpQN2[218] = + tmpFx[15]*tmpObjSEndTerm[8] + tmpFx[31]*tmpObjSEndTerm[22] + tmpFx[47]*tmpObjSEndTerm[36] + tmpFx[63]*tmpObjSEndTerm[50] + tmpFx[79]*tmpObjSEndTerm[64] + tmpFx[95]*tmpObjSEndTerm[78] + tmpFx[111]*tmpObjSEndTerm[92] + tmpFx[127]*tmpObjSEndTerm[106] + tmpFx[143]*tmpObjSEndTerm[120] + tmpFx[159]*tmpObjSEndTerm[134] + tmpFx[175]*tmpObjSEndTerm[148] + tmpFx[191]*tmpObjSEndTerm[162] + tmpFx[207]*tmpObjSEndTerm[176] + tmpFx[223]*tmpObjSEndTerm[190];
tmpQN2[219] = + tmpFx[15]*tmpObjSEndTerm[9] + tmpFx[31]*tmpObjSEndTerm[23] + tmpFx[47]*tmpObjSEndTerm[37] + tmpFx[63]*tmpObjSEndTerm[51] + tmpFx[79]*tmpObjSEndTerm[65] + tmpFx[95]*tmpObjSEndTerm[79] + tmpFx[111]*tmpObjSEndTerm[93] + tmpFx[127]*tmpObjSEndTerm[107] + tmpFx[143]*tmpObjSEndTerm[121] + tmpFx[159]*tmpObjSEndTerm[135] + tmpFx[175]*tmpObjSEndTerm[149] + tmpFx[191]*tmpObjSEndTerm[163] + tmpFx[207]*tmpObjSEndTerm[177] + tmpFx[223]*tmpObjSEndTerm[191];
tmpQN2[220] = + tmpFx[15]*tmpObjSEndTerm[10] + tmpFx[31]*tmpObjSEndTerm[24] + tmpFx[47]*tmpObjSEndTerm[38] + tmpFx[63]*tmpObjSEndTerm[52] + tmpFx[79]*tmpObjSEndTerm[66] + tmpFx[95]*tmpObjSEndTerm[80] + tmpFx[111]*tmpObjSEndTerm[94] + tmpFx[127]*tmpObjSEndTerm[108] + tmpFx[143]*tmpObjSEndTerm[122] + tmpFx[159]*tmpObjSEndTerm[136] + tmpFx[175]*tmpObjSEndTerm[150] + tmpFx[191]*tmpObjSEndTerm[164] + tmpFx[207]*tmpObjSEndTerm[178] + tmpFx[223]*tmpObjSEndTerm[192];
tmpQN2[221] = + tmpFx[15]*tmpObjSEndTerm[11] + tmpFx[31]*tmpObjSEndTerm[25] + tmpFx[47]*tmpObjSEndTerm[39] + tmpFx[63]*tmpObjSEndTerm[53] + tmpFx[79]*tmpObjSEndTerm[67] + tmpFx[95]*tmpObjSEndTerm[81] + tmpFx[111]*tmpObjSEndTerm[95] + tmpFx[127]*tmpObjSEndTerm[109] + tmpFx[143]*tmpObjSEndTerm[123] + tmpFx[159]*tmpObjSEndTerm[137] + tmpFx[175]*tmpObjSEndTerm[151] + tmpFx[191]*tmpObjSEndTerm[165] + tmpFx[207]*tmpObjSEndTerm[179] + tmpFx[223]*tmpObjSEndTerm[193];
tmpQN2[222] = + tmpFx[15]*tmpObjSEndTerm[12] + tmpFx[31]*tmpObjSEndTerm[26] + tmpFx[47]*tmpObjSEndTerm[40] + tmpFx[63]*tmpObjSEndTerm[54] + tmpFx[79]*tmpObjSEndTerm[68] + tmpFx[95]*tmpObjSEndTerm[82] + tmpFx[111]*tmpObjSEndTerm[96] + tmpFx[127]*tmpObjSEndTerm[110] + tmpFx[143]*tmpObjSEndTerm[124] + tmpFx[159]*tmpObjSEndTerm[138] + tmpFx[175]*tmpObjSEndTerm[152] + tmpFx[191]*tmpObjSEndTerm[166] + tmpFx[207]*tmpObjSEndTerm[180] + tmpFx[223]*tmpObjSEndTerm[194];
tmpQN2[223] = + tmpFx[15]*tmpObjSEndTerm[13] + tmpFx[31]*tmpObjSEndTerm[27] + tmpFx[47]*tmpObjSEndTerm[41] + tmpFx[63]*tmpObjSEndTerm[55] + tmpFx[79]*tmpObjSEndTerm[69] + tmpFx[95]*tmpObjSEndTerm[83] + tmpFx[111]*tmpObjSEndTerm[97] + tmpFx[127]*tmpObjSEndTerm[111] + tmpFx[143]*tmpObjSEndTerm[125] + tmpFx[159]*tmpObjSEndTerm[139] + tmpFx[175]*tmpObjSEndTerm[153] + tmpFx[191]*tmpObjSEndTerm[167] + tmpFx[207]*tmpObjSEndTerm[181] + tmpFx[223]*tmpObjSEndTerm[195];
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[16] + tmpQN2[2]*tmpFx[32] + tmpQN2[3]*tmpFx[48] + tmpQN2[4]*tmpFx[64] + tmpQN2[5]*tmpFx[80] + tmpQN2[6]*tmpFx[96] + tmpQN2[7]*tmpFx[112] + tmpQN2[8]*tmpFx[128] + tmpQN2[9]*tmpFx[144] + tmpQN2[10]*tmpFx[160] + tmpQN2[11]*tmpFx[176] + tmpQN2[12]*tmpFx[192] + tmpQN2[13]*tmpFx[208];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[17] + tmpQN2[2]*tmpFx[33] + tmpQN2[3]*tmpFx[49] + tmpQN2[4]*tmpFx[65] + tmpQN2[5]*tmpFx[81] + tmpQN2[6]*tmpFx[97] + tmpQN2[7]*tmpFx[113] + tmpQN2[8]*tmpFx[129] + tmpQN2[9]*tmpFx[145] + tmpQN2[10]*tmpFx[161] + tmpQN2[11]*tmpFx[177] + tmpQN2[12]*tmpFx[193] + tmpQN2[13]*tmpFx[209];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[18] + tmpQN2[2]*tmpFx[34] + tmpQN2[3]*tmpFx[50] + tmpQN2[4]*tmpFx[66] + tmpQN2[5]*tmpFx[82] + tmpQN2[6]*tmpFx[98] + tmpQN2[7]*tmpFx[114] + tmpQN2[8]*tmpFx[130] + tmpQN2[9]*tmpFx[146] + tmpQN2[10]*tmpFx[162] + tmpQN2[11]*tmpFx[178] + tmpQN2[12]*tmpFx[194] + tmpQN2[13]*tmpFx[210];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[19] + tmpQN2[2]*tmpFx[35] + tmpQN2[3]*tmpFx[51] + tmpQN2[4]*tmpFx[67] + tmpQN2[5]*tmpFx[83] + tmpQN2[6]*tmpFx[99] + tmpQN2[7]*tmpFx[115] + tmpQN2[8]*tmpFx[131] + tmpQN2[9]*tmpFx[147] + tmpQN2[10]*tmpFx[163] + tmpQN2[11]*tmpFx[179] + tmpQN2[12]*tmpFx[195] + tmpQN2[13]*tmpFx[211];
tmpQN1[4] = + tmpQN2[0]*tmpFx[4] + tmpQN2[1]*tmpFx[20] + tmpQN2[2]*tmpFx[36] + tmpQN2[3]*tmpFx[52] + tmpQN2[4]*tmpFx[68] + tmpQN2[5]*tmpFx[84] + tmpQN2[6]*tmpFx[100] + tmpQN2[7]*tmpFx[116] + tmpQN2[8]*tmpFx[132] + tmpQN2[9]*tmpFx[148] + tmpQN2[10]*tmpFx[164] + tmpQN2[11]*tmpFx[180] + tmpQN2[12]*tmpFx[196] + tmpQN2[13]*tmpFx[212];
tmpQN1[5] = + tmpQN2[0]*tmpFx[5] + tmpQN2[1]*tmpFx[21] + tmpQN2[2]*tmpFx[37] + tmpQN2[3]*tmpFx[53] + tmpQN2[4]*tmpFx[69] + tmpQN2[5]*tmpFx[85] + tmpQN2[6]*tmpFx[101] + tmpQN2[7]*tmpFx[117] + tmpQN2[8]*tmpFx[133] + tmpQN2[9]*tmpFx[149] + tmpQN2[10]*tmpFx[165] + tmpQN2[11]*tmpFx[181] + tmpQN2[12]*tmpFx[197] + tmpQN2[13]*tmpFx[213];
tmpQN1[6] = + tmpQN2[0]*tmpFx[6] + tmpQN2[1]*tmpFx[22] + tmpQN2[2]*tmpFx[38] + tmpQN2[3]*tmpFx[54] + tmpQN2[4]*tmpFx[70] + tmpQN2[5]*tmpFx[86] + tmpQN2[6]*tmpFx[102] + tmpQN2[7]*tmpFx[118] + tmpQN2[8]*tmpFx[134] + tmpQN2[9]*tmpFx[150] + tmpQN2[10]*tmpFx[166] + tmpQN2[11]*tmpFx[182] + tmpQN2[12]*tmpFx[198] + tmpQN2[13]*tmpFx[214];
tmpQN1[7] = + tmpQN2[0]*tmpFx[7] + tmpQN2[1]*tmpFx[23] + tmpQN2[2]*tmpFx[39] + tmpQN2[3]*tmpFx[55] + tmpQN2[4]*tmpFx[71] + tmpQN2[5]*tmpFx[87] + tmpQN2[6]*tmpFx[103] + tmpQN2[7]*tmpFx[119] + tmpQN2[8]*tmpFx[135] + tmpQN2[9]*tmpFx[151] + tmpQN2[10]*tmpFx[167] + tmpQN2[11]*tmpFx[183] + tmpQN2[12]*tmpFx[199] + tmpQN2[13]*tmpFx[215];
tmpQN1[8] = + tmpQN2[0]*tmpFx[8] + tmpQN2[1]*tmpFx[24] + tmpQN2[2]*tmpFx[40] + tmpQN2[3]*tmpFx[56] + tmpQN2[4]*tmpFx[72] + tmpQN2[5]*tmpFx[88] + tmpQN2[6]*tmpFx[104] + tmpQN2[7]*tmpFx[120] + tmpQN2[8]*tmpFx[136] + tmpQN2[9]*tmpFx[152] + tmpQN2[10]*tmpFx[168] + tmpQN2[11]*tmpFx[184] + tmpQN2[12]*tmpFx[200] + tmpQN2[13]*tmpFx[216];
tmpQN1[9] = + tmpQN2[0]*tmpFx[9] + tmpQN2[1]*tmpFx[25] + tmpQN2[2]*tmpFx[41] + tmpQN2[3]*tmpFx[57] + tmpQN2[4]*tmpFx[73] + tmpQN2[5]*tmpFx[89] + tmpQN2[6]*tmpFx[105] + tmpQN2[7]*tmpFx[121] + tmpQN2[8]*tmpFx[137] + tmpQN2[9]*tmpFx[153] + tmpQN2[10]*tmpFx[169] + tmpQN2[11]*tmpFx[185] + tmpQN2[12]*tmpFx[201] + tmpQN2[13]*tmpFx[217];
tmpQN1[10] = + tmpQN2[0]*tmpFx[10] + tmpQN2[1]*tmpFx[26] + tmpQN2[2]*tmpFx[42] + tmpQN2[3]*tmpFx[58] + tmpQN2[4]*tmpFx[74] + tmpQN2[5]*tmpFx[90] + tmpQN2[6]*tmpFx[106] + tmpQN2[7]*tmpFx[122] + tmpQN2[8]*tmpFx[138] + tmpQN2[9]*tmpFx[154] + tmpQN2[10]*tmpFx[170] + tmpQN2[11]*tmpFx[186] + tmpQN2[12]*tmpFx[202] + tmpQN2[13]*tmpFx[218];
tmpQN1[11] = + tmpQN2[0]*tmpFx[11] + tmpQN2[1]*tmpFx[27] + tmpQN2[2]*tmpFx[43] + tmpQN2[3]*tmpFx[59] + tmpQN2[4]*tmpFx[75] + tmpQN2[5]*tmpFx[91] + tmpQN2[6]*tmpFx[107] + tmpQN2[7]*tmpFx[123] + tmpQN2[8]*tmpFx[139] + tmpQN2[9]*tmpFx[155] + tmpQN2[10]*tmpFx[171] + tmpQN2[11]*tmpFx[187] + tmpQN2[12]*tmpFx[203] + tmpQN2[13]*tmpFx[219];
tmpQN1[12] = + tmpQN2[0]*tmpFx[12] + tmpQN2[1]*tmpFx[28] + tmpQN2[2]*tmpFx[44] + tmpQN2[3]*tmpFx[60] + tmpQN2[4]*tmpFx[76] + tmpQN2[5]*tmpFx[92] + tmpQN2[6]*tmpFx[108] + tmpQN2[7]*tmpFx[124] + tmpQN2[8]*tmpFx[140] + tmpQN2[9]*tmpFx[156] + tmpQN2[10]*tmpFx[172] + tmpQN2[11]*tmpFx[188] + tmpQN2[12]*tmpFx[204] + tmpQN2[13]*tmpFx[220];
tmpQN1[13] = + tmpQN2[0]*tmpFx[13] + tmpQN2[1]*tmpFx[29] + tmpQN2[2]*tmpFx[45] + tmpQN2[3]*tmpFx[61] + tmpQN2[4]*tmpFx[77] + tmpQN2[5]*tmpFx[93] + tmpQN2[6]*tmpFx[109] + tmpQN2[7]*tmpFx[125] + tmpQN2[8]*tmpFx[141] + tmpQN2[9]*tmpFx[157] + tmpQN2[10]*tmpFx[173] + tmpQN2[11]*tmpFx[189] + tmpQN2[12]*tmpFx[205] + tmpQN2[13]*tmpFx[221];
tmpQN1[14] = + tmpQN2[0]*tmpFx[14] + tmpQN2[1]*tmpFx[30] + tmpQN2[2]*tmpFx[46] + tmpQN2[3]*tmpFx[62] + tmpQN2[4]*tmpFx[78] + tmpQN2[5]*tmpFx[94] + tmpQN2[6]*tmpFx[110] + tmpQN2[7]*tmpFx[126] + tmpQN2[8]*tmpFx[142] + tmpQN2[9]*tmpFx[158] + tmpQN2[10]*tmpFx[174] + tmpQN2[11]*tmpFx[190] + tmpQN2[12]*tmpFx[206] + tmpQN2[13]*tmpFx[222];
tmpQN1[15] = + tmpQN2[0]*tmpFx[15] + tmpQN2[1]*tmpFx[31] + tmpQN2[2]*tmpFx[47] + tmpQN2[3]*tmpFx[63] + tmpQN2[4]*tmpFx[79] + tmpQN2[5]*tmpFx[95] + tmpQN2[6]*tmpFx[111] + tmpQN2[7]*tmpFx[127] + tmpQN2[8]*tmpFx[143] + tmpQN2[9]*tmpFx[159] + tmpQN2[10]*tmpFx[175] + tmpQN2[11]*tmpFx[191] + tmpQN2[12]*tmpFx[207] + tmpQN2[13]*tmpFx[223];
tmpQN1[16] = + tmpQN2[14]*tmpFx[0] + tmpQN2[15]*tmpFx[16] + tmpQN2[16]*tmpFx[32] + tmpQN2[17]*tmpFx[48] + tmpQN2[18]*tmpFx[64] + tmpQN2[19]*tmpFx[80] + tmpQN2[20]*tmpFx[96] + tmpQN2[21]*tmpFx[112] + tmpQN2[22]*tmpFx[128] + tmpQN2[23]*tmpFx[144] + tmpQN2[24]*tmpFx[160] + tmpQN2[25]*tmpFx[176] + tmpQN2[26]*tmpFx[192] + tmpQN2[27]*tmpFx[208];
tmpQN1[17] = + tmpQN2[14]*tmpFx[1] + tmpQN2[15]*tmpFx[17] + tmpQN2[16]*tmpFx[33] + tmpQN2[17]*tmpFx[49] + tmpQN2[18]*tmpFx[65] + tmpQN2[19]*tmpFx[81] + tmpQN2[20]*tmpFx[97] + tmpQN2[21]*tmpFx[113] + tmpQN2[22]*tmpFx[129] + tmpQN2[23]*tmpFx[145] + tmpQN2[24]*tmpFx[161] + tmpQN2[25]*tmpFx[177] + tmpQN2[26]*tmpFx[193] + tmpQN2[27]*tmpFx[209];
tmpQN1[18] = + tmpQN2[14]*tmpFx[2] + tmpQN2[15]*tmpFx[18] + tmpQN2[16]*tmpFx[34] + tmpQN2[17]*tmpFx[50] + tmpQN2[18]*tmpFx[66] + tmpQN2[19]*tmpFx[82] + tmpQN2[20]*tmpFx[98] + tmpQN2[21]*tmpFx[114] + tmpQN2[22]*tmpFx[130] + tmpQN2[23]*tmpFx[146] + tmpQN2[24]*tmpFx[162] + tmpQN2[25]*tmpFx[178] + tmpQN2[26]*tmpFx[194] + tmpQN2[27]*tmpFx[210];
tmpQN1[19] = + tmpQN2[14]*tmpFx[3] + tmpQN2[15]*tmpFx[19] + tmpQN2[16]*tmpFx[35] + tmpQN2[17]*tmpFx[51] + tmpQN2[18]*tmpFx[67] + tmpQN2[19]*tmpFx[83] + tmpQN2[20]*tmpFx[99] + tmpQN2[21]*tmpFx[115] + tmpQN2[22]*tmpFx[131] + tmpQN2[23]*tmpFx[147] + tmpQN2[24]*tmpFx[163] + tmpQN2[25]*tmpFx[179] + tmpQN2[26]*tmpFx[195] + tmpQN2[27]*tmpFx[211];
tmpQN1[20] = + tmpQN2[14]*tmpFx[4] + tmpQN2[15]*tmpFx[20] + tmpQN2[16]*tmpFx[36] + tmpQN2[17]*tmpFx[52] + tmpQN2[18]*tmpFx[68] + tmpQN2[19]*tmpFx[84] + tmpQN2[20]*tmpFx[100] + tmpQN2[21]*tmpFx[116] + tmpQN2[22]*tmpFx[132] + tmpQN2[23]*tmpFx[148] + tmpQN2[24]*tmpFx[164] + tmpQN2[25]*tmpFx[180] + tmpQN2[26]*tmpFx[196] + tmpQN2[27]*tmpFx[212];
tmpQN1[21] = + tmpQN2[14]*tmpFx[5] + tmpQN2[15]*tmpFx[21] + tmpQN2[16]*tmpFx[37] + tmpQN2[17]*tmpFx[53] + tmpQN2[18]*tmpFx[69] + tmpQN2[19]*tmpFx[85] + tmpQN2[20]*tmpFx[101] + tmpQN2[21]*tmpFx[117] + tmpQN2[22]*tmpFx[133] + tmpQN2[23]*tmpFx[149] + tmpQN2[24]*tmpFx[165] + tmpQN2[25]*tmpFx[181] + tmpQN2[26]*tmpFx[197] + tmpQN2[27]*tmpFx[213];
tmpQN1[22] = + tmpQN2[14]*tmpFx[6] + tmpQN2[15]*tmpFx[22] + tmpQN2[16]*tmpFx[38] + tmpQN2[17]*tmpFx[54] + tmpQN2[18]*tmpFx[70] + tmpQN2[19]*tmpFx[86] + tmpQN2[20]*tmpFx[102] + tmpQN2[21]*tmpFx[118] + tmpQN2[22]*tmpFx[134] + tmpQN2[23]*tmpFx[150] + tmpQN2[24]*tmpFx[166] + tmpQN2[25]*tmpFx[182] + tmpQN2[26]*tmpFx[198] + tmpQN2[27]*tmpFx[214];
tmpQN1[23] = + tmpQN2[14]*tmpFx[7] + tmpQN2[15]*tmpFx[23] + tmpQN2[16]*tmpFx[39] + tmpQN2[17]*tmpFx[55] + tmpQN2[18]*tmpFx[71] + tmpQN2[19]*tmpFx[87] + tmpQN2[20]*tmpFx[103] + tmpQN2[21]*tmpFx[119] + tmpQN2[22]*tmpFx[135] + tmpQN2[23]*tmpFx[151] + tmpQN2[24]*tmpFx[167] + tmpQN2[25]*tmpFx[183] + tmpQN2[26]*tmpFx[199] + tmpQN2[27]*tmpFx[215];
tmpQN1[24] = + tmpQN2[14]*tmpFx[8] + tmpQN2[15]*tmpFx[24] + tmpQN2[16]*tmpFx[40] + tmpQN2[17]*tmpFx[56] + tmpQN2[18]*tmpFx[72] + tmpQN2[19]*tmpFx[88] + tmpQN2[20]*tmpFx[104] + tmpQN2[21]*tmpFx[120] + tmpQN2[22]*tmpFx[136] + tmpQN2[23]*tmpFx[152] + tmpQN2[24]*tmpFx[168] + tmpQN2[25]*tmpFx[184] + tmpQN2[26]*tmpFx[200] + tmpQN2[27]*tmpFx[216];
tmpQN1[25] = + tmpQN2[14]*tmpFx[9] + tmpQN2[15]*tmpFx[25] + tmpQN2[16]*tmpFx[41] + tmpQN2[17]*tmpFx[57] + tmpQN2[18]*tmpFx[73] + tmpQN2[19]*tmpFx[89] + tmpQN2[20]*tmpFx[105] + tmpQN2[21]*tmpFx[121] + tmpQN2[22]*tmpFx[137] + tmpQN2[23]*tmpFx[153] + tmpQN2[24]*tmpFx[169] + tmpQN2[25]*tmpFx[185] + tmpQN2[26]*tmpFx[201] + tmpQN2[27]*tmpFx[217];
tmpQN1[26] = + tmpQN2[14]*tmpFx[10] + tmpQN2[15]*tmpFx[26] + tmpQN2[16]*tmpFx[42] + tmpQN2[17]*tmpFx[58] + tmpQN2[18]*tmpFx[74] + tmpQN2[19]*tmpFx[90] + tmpQN2[20]*tmpFx[106] + tmpQN2[21]*tmpFx[122] + tmpQN2[22]*tmpFx[138] + tmpQN2[23]*tmpFx[154] + tmpQN2[24]*tmpFx[170] + tmpQN2[25]*tmpFx[186] + tmpQN2[26]*tmpFx[202] + tmpQN2[27]*tmpFx[218];
tmpQN1[27] = + tmpQN2[14]*tmpFx[11] + tmpQN2[15]*tmpFx[27] + tmpQN2[16]*tmpFx[43] + tmpQN2[17]*tmpFx[59] + tmpQN2[18]*tmpFx[75] + tmpQN2[19]*tmpFx[91] + tmpQN2[20]*tmpFx[107] + tmpQN2[21]*tmpFx[123] + tmpQN2[22]*tmpFx[139] + tmpQN2[23]*tmpFx[155] + tmpQN2[24]*tmpFx[171] + tmpQN2[25]*tmpFx[187] + tmpQN2[26]*tmpFx[203] + tmpQN2[27]*tmpFx[219];
tmpQN1[28] = + tmpQN2[14]*tmpFx[12] + tmpQN2[15]*tmpFx[28] + tmpQN2[16]*tmpFx[44] + tmpQN2[17]*tmpFx[60] + tmpQN2[18]*tmpFx[76] + tmpQN2[19]*tmpFx[92] + tmpQN2[20]*tmpFx[108] + tmpQN2[21]*tmpFx[124] + tmpQN2[22]*tmpFx[140] + tmpQN2[23]*tmpFx[156] + tmpQN2[24]*tmpFx[172] + tmpQN2[25]*tmpFx[188] + tmpQN2[26]*tmpFx[204] + tmpQN2[27]*tmpFx[220];
tmpQN1[29] = + tmpQN2[14]*tmpFx[13] + tmpQN2[15]*tmpFx[29] + tmpQN2[16]*tmpFx[45] + tmpQN2[17]*tmpFx[61] + tmpQN2[18]*tmpFx[77] + tmpQN2[19]*tmpFx[93] + tmpQN2[20]*tmpFx[109] + tmpQN2[21]*tmpFx[125] + tmpQN2[22]*tmpFx[141] + tmpQN2[23]*tmpFx[157] + tmpQN2[24]*tmpFx[173] + tmpQN2[25]*tmpFx[189] + tmpQN2[26]*tmpFx[205] + tmpQN2[27]*tmpFx[221];
tmpQN1[30] = + tmpQN2[14]*tmpFx[14] + tmpQN2[15]*tmpFx[30] + tmpQN2[16]*tmpFx[46] + tmpQN2[17]*tmpFx[62] + tmpQN2[18]*tmpFx[78] + tmpQN2[19]*tmpFx[94] + tmpQN2[20]*tmpFx[110] + tmpQN2[21]*tmpFx[126] + tmpQN2[22]*tmpFx[142] + tmpQN2[23]*tmpFx[158] + tmpQN2[24]*tmpFx[174] + tmpQN2[25]*tmpFx[190] + tmpQN2[26]*tmpFx[206] + tmpQN2[27]*tmpFx[222];
tmpQN1[31] = + tmpQN2[14]*tmpFx[15] + tmpQN2[15]*tmpFx[31] + tmpQN2[16]*tmpFx[47] + tmpQN2[17]*tmpFx[63] + tmpQN2[18]*tmpFx[79] + tmpQN2[19]*tmpFx[95] + tmpQN2[20]*tmpFx[111] + tmpQN2[21]*tmpFx[127] + tmpQN2[22]*tmpFx[143] + tmpQN2[23]*tmpFx[159] + tmpQN2[24]*tmpFx[175] + tmpQN2[25]*tmpFx[191] + tmpQN2[26]*tmpFx[207] + tmpQN2[27]*tmpFx[223];
tmpQN1[32] = + tmpQN2[28]*tmpFx[0] + tmpQN2[29]*tmpFx[16] + tmpQN2[30]*tmpFx[32] + tmpQN2[31]*tmpFx[48] + tmpQN2[32]*tmpFx[64] + tmpQN2[33]*tmpFx[80] + tmpQN2[34]*tmpFx[96] + tmpQN2[35]*tmpFx[112] + tmpQN2[36]*tmpFx[128] + tmpQN2[37]*tmpFx[144] + tmpQN2[38]*tmpFx[160] + tmpQN2[39]*tmpFx[176] + tmpQN2[40]*tmpFx[192] + tmpQN2[41]*tmpFx[208];
tmpQN1[33] = + tmpQN2[28]*tmpFx[1] + tmpQN2[29]*tmpFx[17] + tmpQN2[30]*tmpFx[33] + tmpQN2[31]*tmpFx[49] + tmpQN2[32]*tmpFx[65] + tmpQN2[33]*tmpFx[81] + tmpQN2[34]*tmpFx[97] + tmpQN2[35]*tmpFx[113] + tmpQN2[36]*tmpFx[129] + tmpQN2[37]*tmpFx[145] + tmpQN2[38]*tmpFx[161] + tmpQN2[39]*tmpFx[177] + tmpQN2[40]*tmpFx[193] + tmpQN2[41]*tmpFx[209];
tmpQN1[34] = + tmpQN2[28]*tmpFx[2] + tmpQN2[29]*tmpFx[18] + tmpQN2[30]*tmpFx[34] + tmpQN2[31]*tmpFx[50] + tmpQN2[32]*tmpFx[66] + tmpQN2[33]*tmpFx[82] + tmpQN2[34]*tmpFx[98] + tmpQN2[35]*tmpFx[114] + tmpQN2[36]*tmpFx[130] + tmpQN2[37]*tmpFx[146] + tmpQN2[38]*tmpFx[162] + tmpQN2[39]*tmpFx[178] + tmpQN2[40]*tmpFx[194] + tmpQN2[41]*tmpFx[210];
tmpQN1[35] = + tmpQN2[28]*tmpFx[3] + tmpQN2[29]*tmpFx[19] + tmpQN2[30]*tmpFx[35] + tmpQN2[31]*tmpFx[51] + tmpQN2[32]*tmpFx[67] + tmpQN2[33]*tmpFx[83] + tmpQN2[34]*tmpFx[99] + tmpQN2[35]*tmpFx[115] + tmpQN2[36]*tmpFx[131] + tmpQN2[37]*tmpFx[147] + tmpQN2[38]*tmpFx[163] + tmpQN2[39]*tmpFx[179] + tmpQN2[40]*tmpFx[195] + tmpQN2[41]*tmpFx[211];
tmpQN1[36] = + tmpQN2[28]*tmpFx[4] + tmpQN2[29]*tmpFx[20] + tmpQN2[30]*tmpFx[36] + tmpQN2[31]*tmpFx[52] + tmpQN2[32]*tmpFx[68] + tmpQN2[33]*tmpFx[84] + tmpQN2[34]*tmpFx[100] + tmpQN2[35]*tmpFx[116] + tmpQN2[36]*tmpFx[132] + tmpQN2[37]*tmpFx[148] + tmpQN2[38]*tmpFx[164] + tmpQN2[39]*tmpFx[180] + tmpQN2[40]*tmpFx[196] + tmpQN2[41]*tmpFx[212];
tmpQN1[37] = + tmpQN2[28]*tmpFx[5] + tmpQN2[29]*tmpFx[21] + tmpQN2[30]*tmpFx[37] + tmpQN2[31]*tmpFx[53] + tmpQN2[32]*tmpFx[69] + tmpQN2[33]*tmpFx[85] + tmpQN2[34]*tmpFx[101] + tmpQN2[35]*tmpFx[117] + tmpQN2[36]*tmpFx[133] + tmpQN2[37]*tmpFx[149] + tmpQN2[38]*tmpFx[165] + tmpQN2[39]*tmpFx[181] + tmpQN2[40]*tmpFx[197] + tmpQN2[41]*tmpFx[213];
tmpQN1[38] = + tmpQN2[28]*tmpFx[6] + tmpQN2[29]*tmpFx[22] + tmpQN2[30]*tmpFx[38] + tmpQN2[31]*tmpFx[54] + tmpQN2[32]*tmpFx[70] + tmpQN2[33]*tmpFx[86] + tmpQN2[34]*tmpFx[102] + tmpQN2[35]*tmpFx[118] + tmpQN2[36]*tmpFx[134] + tmpQN2[37]*tmpFx[150] + tmpQN2[38]*tmpFx[166] + tmpQN2[39]*tmpFx[182] + tmpQN2[40]*tmpFx[198] + tmpQN2[41]*tmpFx[214];
tmpQN1[39] = + tmpQN2[28]*tmpFx[7] + tmpQN2[29]*tmpFx[23] + tmpQN2[30]*tmpFx[39] + tmpQN2[31]*tmpFx[55] + tmpQN2[32]*tmpFx[71] + tmpQN2[33]*tmpFx[87] + tmpQN2[34]*tmpFx[103] + tmpQN2[35]*tmpFx[119] + tmpQN2[36]*tmpFx[135] + tmpQN2[37]*tmpFx[151] + tmpQN2[38]*tmpFx[167] + tmpQN2[39]*tmpFx[183] + tmpQN2[40]*tmpFx[199] + tmpQN2[41]*tmpFx[215];
tmpQN1[40] = + tmpQN2[28]*tmpFx[8] + tmpQN2[29]*tmpFx[24] + tmpQN2[30]*tmpFx[40] + tmpQN2[31]*tmpFx[56] + tmpQN2[32]*tmpFx[72] + tmpQN2[33]*tmpFx[88] + tmpQN2[34]*tmpFx[104] + tmpQN2[35]*tmpFx[120] + tmpQN2[36]*tmpFx[136] + tmpQN2[37]*tmpFx[152] + tmpQN2[38]*tmpFx[168] + tmpQN2[39]*tmpFx[184] + tmpQN2[40]*tmpFx[200] + tmpQN2[41]*tmpFx[216];
tmpQN1[41] = + tmpQN2[28]*tmpFx[9] + tmpQN2[29]*tmpFx[25] + tmpQN2[30]*tmpFx[41] + tmpQN2[31]*tmpFx[57] + tmpQN2[32]*tmpFx[73] + tmpQN2[33]*tmpFx[89] + tmpQN2[34]*tmpFx[105] + tmpQN2[35]*tmpFx[121] + tmpQN2[36]*tmpFx[137] + tmpQN2[37]*tmpFx[153] + tmpQN2[38]*tmpFx[169] + tmpQN2[39]*tmpFx[185] + tmpQN2[40]*tmpFx[201] + tmpQN2[41]*tmpFx[217];
tmpQN1[42] = + tmpQN2[28]*tmpFx[10] + tmpQN2[29]*tmpFx[26] + tmpQN2[30]*tmpFx[42] + tmpQN2[31]*tmpFx[58] + tmpQN2[32]*tmpFx[74] + tmpQN2[33]*tmpFx[90] + tmpQN2[34]*tmpFx[106] + tmpQN2[35]*tmpFx[122] + tmpQN2[36]*tmpFx[138] + tmpQN2[37]*tmpFx[154] + tmpQN2[38]*tmpFx[170] + tmpQN2[39]*tmpFx[186] + tmpQN2[40]*tmpFx[202] + tmpQN2[41]*tmpFx[218];
tmpQN1[43] = + tmpQN2[28]*tmpFx[11] + tmpQN2[29]*tmpFx[27] + tmpQN2[30]*tmpFx[43] + tmpQN2[31]*tmpFx[59] + tmpQN2[32]*tmpFx[75] + tmpQN2[33]*tmpFx[91] + tmpQN2[34]*tmpFx[107] + tmpQN2[35]*tmpFx[123] + tmpQN2[36]*tmpFx[139] + tmpQN2[37]*tmpFx[155] + tmpQN2[38]*tmpFx[171] + tmpQN2[39]*tmpFx[187] + tmpQN2[40]*tmpFx[203] + tmpQN2[41]*tmpFx[219];
tmpQN1[44] = + tmpQN2[28]*tmpFx[12] + tmpQN2[29]*tmpFx[28] + tmpQN2[30]*tmpFx[44] + tmpQN2[31]*tmpFx[60] + tmpQN2[32]*tmpFx[76] + tmpQN2[33]*tmpFx[92] + tmpQN2[34]*tmpFx[108] + tmpQN2[35]*tmpFx[124] + tmpQN2[36]*tmpFx[140] + tmpQN2[37]*tmpFx[156] + tmpQN2[38]*tmpFx[172] + tmpQN2[39]*tmpFx[188] + tmpQN2[40]*tmpFx[204] + tmpQN2[41]*tmpFx[220];
tmpQN1[45] = + tmpQN2[28]*tmpFx[13] + tmpQN2[29]*tmpFx[29] + tmpQN2[30]*tmpFx[45] + tmpQN2[31]*tmpFx[61] + tmpQN2[32]*tmpFx[77] + tmpQN2[33]*tmpFx[93] + tmpQN2[34]*tmpFx[109] + tmpQN2[35]*tmpFx[125] + tmpQN2[36]*tmpFx[141] + tmpQN2[37]*tmpFx[157] + tmpQN2[38]*tmpFx[173] + tmpQN2[39]*tmpFx[189] + tmpQN2[40]*tmpFx[205] + tmpQN2[41]*tmpFx[221];
tmpQN1[46] = + tmpQN2[28]*tmpFx[14] + tmpQN2[29]*tmpFx[30] + tmpQN2[30]*tmpFx[46] + tmpQN2[31]*tmpFx[62] + tmpQN2[32]*tmpFx[78] + tmpQN2[33]*tmpFx[94] + tmpQN2[34]*tmpFx[110] + tmpQN2[35]*tmpFx[126] + tmpQN2[36]*tmpFx[142] + tmpQN2[37]*tmpFx[158] + tmpQN2[38]*tmpFx[174] + tmpQN2[39]*tmpFx[190] + tmpQN2[40]*tmpFx[206] + tmpQN2[41]*tmpFx[222];
tmpQN1[47] = + tmpQN2[28]*tmpFx[15] + tmpQN2[29]*tmpFx[31] + tmpQN2[30]*tmpFx[47] + tmpQN2[31]*tmpFx[63] + tmpQN2[32]*tmpFx[79] + tmpQN2[33]*tmpFx[95] + tmpQN2[34]*tmpFx[111] + tmpQN2[35]*tmpFx[127] + tmpQN2[36]*tmpFx[143] + tmpQN2[37]*tmpFx[159] + tmpQN2[38]*tmpFx[175] + tmpQN2[39]*tmpFx[191] + tmpQN2[40]*tmpFx[207] + tmpQN2[41]*tmpFx[223];
tmpQN1[48] = + tmpQN2[42]*tmpFx[0] + tmpQN2[43]*tmpFx[16] + tmpQN2[44]*tmpFx[32] + tmpQN2[45]*tmpFx[48] + tmpQN2[46]*tmpFx[64] + tmpQN2[47]*tmpFx[80] + tmpQN2[48]*tmpFx[96] + tmpQN2[49]*tmpFx[112] + tmpQN2[50]*tmpFx[128] + tmpQN2[51]*tmpFx[144] + tmpQN2[52]*tmpFx[160] + tmpQN2[53]*tmpFx[176] + tmpQN2[54]*tmpFx[192] + tmpQN2[55]*tmpFx[208];
tmpQN1[49] = + tmpQN2[42]*tmpFx[1] + tmpQN2[43]*tmpFx[17] + tmpQN2[44]*tmpFx[33] + tmpQN2[45]*tmpFx[49] + tmpQN2[46]*tmpFx[65] + tmpQN2[47]*tmpFx[81] + tmpQN2[48]*tmpFx[97] + tmpQN2[49]*tmpFx[113] + tmpQN2[50]*tmpFx[129] + tmpQN2[51]*tmpFx[145] + tmpQN2[52]*tmpFx[161] + tmpQN2[53]*tmpFx[177] + tmpQN2[54]*tmpFx[193] + tmpQN2[55]*tmpFx[209];
tmpQN1[50] = + tmpQN2[42]*tmpFx[2] + tmpQN2[43]*tmpFx[18] + tmpQN2[44]*tmpFx[34] + tmpQN2[45]*tmpFx[50] + tmpQN2[46]*tmpFx[66] + tmpQN2[47]*tmpFx[82] + tmpQN2[48]*tmpFx[98] + tmpQN2[49]*tmpFx[114] + tmpQN2[50]*tmpFx[130] + tmpQN2[51]*tmpFx[146] + tmpQN2[52]*tmpFx[162] + tmpQN2[53]*tmpFx[178] + tmpQN2[54]*tmpFx[194] + tmpQN2[55]*tmpFx[210];
tmpQN1[51] = + tmpQN2[42]*tmpFx[3] + tmpQN2[43]*tmpFx[19] + tmpQN2[44]*tmpFx[35] + tmpQN2[45]*tmpFx[51] + tmpQN2[46]*tmpFx[67] + tmpQN2[47]*tmpFx[83] + tmpQN2[48]*tmpFx[99] + tmpQN2[49]*tmpFx[115] + tmpQN2[50]*tmpFx[131] + tmpQN2[51]*tmpFx[147] + tmpQN2[52]*tmpFx[163] + tmpQN2[53]*tmpFx[179] + tmpQN2[54]*tmpFx[195] + tmpQN2[55]*tmpFx[211];
tmpQN1[52] = + tmpQN2[42]*tmpFx[4] + tmpQN2[43]*tmpFx[20] + tmpQN2[44]*tmpFx[36] + tmpQN2[45]*tmpFx[52] + tmpQN2[46]*tmpFx[68] + tmpQN2[47]*tmpFx[84] + tmpQN2[48]*tmpFx[100] + tmpQN2[49]*tmpFx[116] + tmpQN2[50]*tmpFx[132] + tmpQN2[51]*tmpFx[148] + tmpQN2[52]*tmpFx[164] + tmpQN2[53]*tmpFx[180] + tmpQN2[54]*tmpFx[196] + tmpQN2[55]*tmpFx[212];
tmpQN1[53] = + tmpQN2[42]*tmpFx[5] + tmpQN2[43]*tmpFx[21] + tmpQN2[44]*tmpFx[37] + tmpQN2[45]*tmpFx[53] + tmpQN2[46]*tmpFx[69] + tmpQN2[47]*tmpFx[85] + tmpQN2[48]*tmpFx[101] + tmpQN2[49]*tmpFx[117] + tmpQN2[50]*tmpFx[133] + tmpQN2[51]*tmpFx[149] + tmpQN2[52]*tmpFx[165] + tmpQN2[53]*tmpFx[181] + tmpQN2[54]*tmpFx[197] + tmpQN2[55]*tmpFx[213];
tmpQN1[54] = + tmpQN2[42]*tmpFx[6] + tmpQN2[43]*tmpFx[22] + tmpQN2[44]*tmpFx[38] + tmpQN2[45]*tmpFx[54] + tmpQN2[46]*tmpFx[70] + tmpQN2[47]*tmpFx[86] + tmpQN2[48]*tmpFx[102] + tmpQN2[49]*tmpFx[118] + tmpQN2[50]*tmpFx[134] + tmpQN2[51]*tmpFx[150] + tmpQN2[52]*tmpFx[166] + tmpQN2[53]*tmpFx[182] + tmpQN2[54]*tmpFx[198] + tmpQN2[55]*tmpFx[214];
tmpQN1[55] = + tmpQN2[42]*tmpFx[7] + tmpQN2[43]*tmpFx[23] + tmpQN2[44]*tmpFx[39] + tmpQN2[45]*tmpFx[55] + tmpQN2[46]*tmpFx[71] + tmpQN2[47]*tmpFx[87] + tmpQN2[48]*tmpFx[103] + tmpQN2[49]*tmpFx[119] + tmpQN2[50]*tmpFx[135] + tmpQN2[51]*tmpFx[151] + tmpQN2[52]*tmpFx[167] + tmpQN2[53]*tmpFx[183] + tmpQN2[54]*tmpFx[199] + tmpQN2[55]*tmpFx[215];
tmpQN1[56] = + tmpQN2[42]*tmpFx[8] + tmpQN2[43]*tmpFx[24] + tmpQN2[44]*tmpFx[40] + tmpQN2[45]*tmpFx[56] + tmpQN2[46]*tmpFx[72] + tmpQN2[47]*tmpFx[88] + tmpQN2[48]*tmpFx[104] + tmpQN2[49]*tmpFx[120] + tmpQN2[50]*tmpFx[136] + tmpQN2[51]*tmpFx[152] + tmpQN2[52]*tmpFx[168] + tmpQN2[53]*tmpFx[184] + tmpQN2[54]*tmpFx[200] + tmpQN2[55]*tmpFx[216];
tmpQN1[57] = + tmpQN2[42]*tmpFx[9] + tmpQN2[43]*tmpFx[25] + tmpQN2[44]*tmpFx[41] + tmpQN2[45]*tmpFx[57] + tmpQN2[46]*tmpFx[73] + tmpQN2[47]*tmpFx[89] + tmpQN2[48]*tmpFx[105] + tmpQN2[49]*tmpFx[121] + tmpQN2[50]*tmpFx[137] + tmpQN2[51]*tmpFx[153] + tmpQN2[52]*tmpFx[169] + tmpQN2[53]*tmpFx[185] + tmpQN2[54]*tmpFx[201] + tmpQN2[55]*tmpFx[217];
tmpQN1[58] = + tmpQN2[42]*tmpFx[10] + tmpQN2[43]*tmpFx[26] + tmpQN2[44]*tmpFx[42] + tmpQN2[45]*tmpFx[58] + tmpQN2[46]*tmpFx[74] + tmpQN2[47]*tmpFx[90] + tmpQN2[48]*tmpFx[106] + tmpQN2[49]*tmpFx[122] + tmpQN2[50]*tmpFx[138] + tmpQN2[51]*tmpFx[154] + tmpQN2[52]*tmpFx[170] + tmpQN2[53]*tmpFx[186] + tmpQN2[54]*tmpFx[202] + tmpQN2[55]*tmpFx[218];
tmpQN1[59] = + tmpQN2[42]*tmpFx[11] + tmpQN2[43]*tmpFx[27] + tmpQN2[44]*tmpFx[43] + tmpQN2[45]*tmpFx[59] + tmpQN2[46]*tmpFx[75] + tmpQN2[47]*tmpFx[91] + tmpQN2[48]*tmpFx[107] + tmpQN2[49]*tmpFx[123] + tmpQN2[50]*tmpFx[139] + tmpQN2[51]*tmpFx[155] + tmpQN2[52]*tmpFx[171] + tmpQN2[53]*tmpFx[187] + tmpQN2[54]*tmpFx[203] + tmpQN2[55]*tmpFx[219];
tmpQN1[60] = + tmpQN2[42]*tmpFx[12] + tmpQN2[43]*tmpFx[28] + tmpQN2[44]*tmpFx[44] + tmpQN2[45]*tmpFx[60] + tmpQN2[46]*tmpFx[76] + tmpQN2[47]*tmpFx[92] + tmpQN2[48]*tmpFx[108] + tmpQN2[49]*tmpFx[124] + tmpQN2[50]*tmpFx[140] + tmpQN2[51]*tmpFx[156] + tmpQN2[52]*tmpFx[172] + tmpQN2[53]*tmpFx[188] + tmpQN2[54]*tmpFx[204] + tmpQN2[55]*tmpFx[220];
tmpQN1[61] = + tmpQN2[42]*tmpFx[13] + tmpQN2[43]*tmpFx[29] + tmpQN2[44]*tmpFx[45] + tmpQN2[45]*tmpFx[61] + tmpQN2[46]*tmpFx[77] + tmpQN2[47]*tmpFx[93] + tmpQN2[48]*tmpFx[109] + tmpQN2[49]*tmpFx[125] + tmpQN2[50]*tmpFx[141] + tmpQN2[51]*tmpFx[157] + tmpQN2[52]*tmpFx[173] + tmpQN2[53]*tmpFx[189] + tmpQN2[54]*tmpFx[205] + tmpQN2[55]*tmpFx[221];
tmpQN1[62] = + tmpQN2[42]*tmpFx[14] + tmpQN2[43]*tmpFx[30] + tmpQN2[44]*tmpFx[46] + tmpQN2[45]*tmpFx[62] + tmpQN2[46]*tmpFx[78] + tmpQN2[47]*tmpFx[94] + tmpQN2[48]*tmpFx[110] + tmpQN2[49]*tmpFx[126] + tmpQN2[50]*tmpFx[142] + tmpQN2[51]*tmpFx[158] + tmpQN2[52]*tmpFx[174] + tmpQN2[53]*tmpFx[190] + tmpQN2[54]*tmpFx[206] + tmpQN2[55]*tmpFx[222];
tmpQN1[63] = + tmpQN2[42]*tmpFx[15] + tmpQN2[43]*tmpFx[31] + tmpQN2[44]*tmpFx[47] + tmpQN2[45]*tmpFx[63] + tmpQN2[46]*tmpFx[79] + tmpQN2[47]*tmpFx[95] + tmpQN2[48]*tmpFx[111] + tmpQN2[49]*tmpFx[127] + tmpQN2[50]*tmpFx[143] + tmpQN2[51]*tmpFx[159] + tmpQN2[52]*tmpFx[175] + tmpQN2[53]*tmpFx[191] + tmpQN2[54]*tmpFx[207] + tmpQN2[55]*tmpFx[223];
tmpQN1[64] = + tmpQN2[56]*tmpFx[0] + tmpQN2[57]*tmpFx[16] + tmpQN2[58]*tmpFx[32] + tmpQN2[59]*tmpFx[48] + tmpQN2[60]*tmpFx[64] + tmpQN2[61]*tmpFx[80] + tmpQN2[62]*tmpFx[96] + tmpQN2[63]*tmpFx[112] + tmpQN2[64]*tmpFx[128] + tmpQN2[65]*tmpFx[144] + tmpQN2[66]*tmpFx[160] + tmpQN2[67]*tmpFx[176] + tmpQN2[68]*tmpFx[192] + tmpQN2[69]*tmpFx[208];
tmpQN1[65] = + tmpQN2[56]*tmpFx[1] + tmpQN2[57]*tmpFx[17] + tmpQN2[58]*tmpFx[33] + tmpQN2[59]*tmpFx[49] + tmpQN2[60]*tmpFx[65] + tmpQN2[61]*tmpFx[81] + tmpQN2[62]*tmpFx[97] + tmpQN2[63]*tmpFx[113] + tmpQN2[64]*tmpFx[129] + tmpQN2[65]*tmpFx[145] + tmpQN2[66]*tmpFx[161] + tmpQN2[67]*tmpFx[177] + tmpQN2[68]*tmpFx[193] + tmpQN2[69]*tmpFx[209];
tmpQN1[66] = + tmpQN2[56]*tmpFx[2] + tmpQN2[57]*tmpFx[18] + tmpQN2[58]*tmpFx[34] + tmpQN2[59]*tmpFx[50] + tmpQN2[60]*tmpFx[66] + tmpQN2[61]*tmpFx[82] + tmpQN2[62]*tmpFx[98] + tmpQN2[63]*tmpFx[114] + tmpQN2[64]*tmpFx[130] + tmpQN2[65]*tmpFx[146] + tmpQN2[66]*tmpFx[162] + tmpQN2[67]*tmpFx[178] + tmpQN2[68]*tmpFx[194] + tmpQN2[69]*tmpFx[210];
tmpQN1[67] = + tmpQN2[56]*tmpFx[3] + tmpQN2[57]*tmpFx[19] + tmpQN2[58]*tmpFx[35] + tmpQN2[59]*tmpFx[51] + tmpQN2[60]*tmpFx[67] + tmpQN2[61]*tmpFx[83] + tmpQN2[62]*tmpFx[99] + tmpQN2[63]*tmpFx[115] + tmpQN2[64]*tmpFx[131] + tmpQN2[65]*tmpFx[147] + tmpQN2[66]*tmpFx[163] + tmpQN2[67]*tmpFx[179] + tmpQN2[68]*tmpFx[195] + tmpQN2[69]*tmpFx[211];
tmpQN1[68] = + tmpQN2[56]*tmpFx[4] + tmpQN2[57]*tmpFx[20] + tmpQN2[58]*tmpFx[36] + tmpQN2[59]*tmpFx[52] + tmpQN2[60]*tmpFx[68] + tmpQN2[61]*tmpFx[84] + tmpQN2[62]*tmpFx[100] + tmpQN2[63]*tmpFx[116] + tmpQN2[64]*tmpFx[132] + tmpQN2[65]*tmpFx[148] + tmpQN2[66]*tmpFx[164] + tmpQN2[67]*tmpFx[180] + tmpQN2[68]*tmpFx[196] + tmpQN2[69]*tmpFx[212];
tmpQN1[69] = + tmpQN2[56]*tmpFx[5] + tmpQN2[57]*tmpFx[21] + tmpQN2[58]*tmpFx[37] + tmpQN2[59]*tmpFx[53] + tmpQN2[60]*tmpFx[69] + tmpQN2[61]*tmpFx[85] + tmpQN2[62]*tmpFx[101] + tmpQN2[63]*tmpFx[117] + tmpQN2[64]*tmpFx[133] + tmpQN2[65]*tmpFx[149] + tmpQN2[66]*tmpFx[165] + tmpQN2[67]*tmpFx[181] + tmpQN2[68]*tmpFx[197] + tmpQN2[69]*tmpFx[213];
tmpQN1[70] = + tmpQN2[56]*tmpFx[6] + tmpQN2[57]*tmpFx[22] + tmpQN2[58]*tmpFx[38] + tmpQN2[59]*tmpFx[54] + tmpQN2[60]*tmpFx[70] + tmpQN2[61]*tmpFx[86] + tmpQN2[62]*tmpFx[102] + tmpQN2[63]*tmpFx[118] + tmpQN2[64]*tmpFx[134] + tmpQN2[65]*tmpFx[150] + tmpQN2[66]*tmpFx[166] + tmpQN2[67]*tmpFx[182] + tmpQN2[68]*tmpFx[198] + tmpQN2[69]*tmpFx[214];
tmpQN1[71] = + tmpQN2[56]*tmpFx[7] + tmpQN2[57]*tmpFx[23] + tmpQN2[58]*tmpFx[39] + tmpQN2[59]*tmpFx[55] + tmpQN2[60]*tmpFx[71] + tmpQN2[61]*tmpFx[87] + tmpQN2[62]*tmpFx[103] + tmpQN2[63]*tmpFx[119] + tmpQN2[64]*tmpFx[135] + tmpQN2[65]*tmpFx[151] + tmpQN2[66]*tmpFx[167] + tmpQN2[67]*tmpFx[183] + tmpQN2[68]*tmpFx[199] + tmpQN2[69]*tmpFx[215];
tmpQN1[72] = + tmpQN2[56]*tmpFx[8] + tmpQN2[57]*tmpFx[24] + tmpQN2[58]*tmpFx[40] + tmpQN2[59]*tmpFx[56] + tmpQN2[60]*tmpFx[72] + tmpQN2[61]*tmpFx[88] + tmpQN2[62]*tmpFx[104] + tmpQN2[63]*tmpFx[120] + tmpQN2[64]*tmpFx[136] + tmpQN2[65]*tmpFx[152] + tmpQN2[66]*tmpFx[168] + tmpQN2[67]*tmpFx[184] + tmpQN2[68]*tmpFx[200] + tmpQN2[69]*tmpFx[216];
tmpQN1[73] = + tmpQN2[56]*tmpFx[9] + tmpQN2[57]*tmpFx[25] + tmpQN2[58]*tmpFx[41] + tmpQN2[59]*tmpFx[57] + tmpQN2[60]*tmpFx[73] + tmpQN2[61]*tmpFx[89] + tmpQN2[62]*tmpFx[105] + tmpQN2[63]*tmpFx[121] + tmpQN2[64]*tmpFx[137] + tmpQN2[65]*tmpFx[153] + tmpQN2[66]*tmpFx[169] + tmpQN2[67]*tmpFx[185] + tmpQN2[68]*tmpFx[201] + tmpQN2[69]*tmpFx[217];
tmpQN1[74] = + tmpQN2[56]*tmpFx[10] + tmpQN2[57]*tmpFx[26] + tmpQN2[58]*tmpFx[42] + tmpQN2[59]*tmpFx[58] + tmpQN2[60]*tmpFx[74] + tmpQN2[61]*tmpFx[90] + tmpQN2[62]*tmpFx[106] + tmpQN2[63]*tmpFx[122] + tmpQN2[64]*tmpFx[138] + tmpQN2[65]*tmpFx[154] + tmpQN2[66]*tmpFx[170] + tmpQN2[67]*tmpFx[186] + tmpQN2[68]*tmpFx[202] + tmpQN2[69]*tmpFx[218];
tmpQN1[75] = + tmpQN2[56]*tmpFx[11] + tmpQN2[57]*tmpFx[27] + tmpQN2[58]*tmpFx[43] + tmpQN2[59]*tmpFx[59] + tmpQN2[60]*tmpFx[75] + tmpQN2[61]*tmpFx[91] + tmpQN2[62]*tmpFx[107] + tmpQN2[63]*tmpFx[123] + tmpQN2[64]*tmpFx[139] + tmpQN2[65]*tmpFx[155] + tmpQN2[66]*tmpFx[171] + tmpQN2[67]*tmpFx[187] + tmpQN2[68]*tmpFx[203] + tmpQN2[69]*tmpFx[219];
tmpQN1[76] = + tmpQN2[56]*tmpFx[12] + tmpQN2[57]*tmpFx[28] + tmpQN2[58]*tmpFx[44] + tmpQN2[59]*tmpFx[60] + tmpQN2[60]*tmpFx[76] + tmpQN2[61]*tmpFx[92] + tmpQN2[62]*tmpFx[108] + tmpQN2[63]*tmpFx[124] + tmpQN2[64]*tmpFx[140] + tmpQN2[65]*tmpFx[156] + tmpQN2[66]*tmpFx[172] + tmpQN2[67]*tmpFx[188] + tmpQN2[68]*tmpFx[204] + tmpQN2[69]*tmpFx[220];
tmpQN1[77] = + tmpQN2[56]*tmpFx[13] + tmpQN2[57]*tmpFx[29] + tmpQN2[58]*tmpFx[45] + tmpQN2[59]*tmpFx[61] + tmpQN2[60]*tmpFx[77] + tmpQN2[61]*tmpFx[93] + tmpQN2[62]*tmpFx[109] + tmpQN2[63]*tmpFx[125] + tmpQN2[64]*tmpFx[141] + tmpQN2[65]*tmpFx[157] + tmpQN2[66]*tmpFx[173] + tmpQN2[67]*tmpFx[189] + tmpQN2[68]*tmpFx[205] + tmpQN2[69]*tmpFx[221];
tmpQN1[78] = + tmpQN2[56]*tmpFx[14] + tmpQN2[57]*tmpFx[30] + tmpQN2[58]*tmpFx[46] + tmpQN2[59]*tmpFx[62] + tmpQN2[60]*tmpFx[78] + tmpQN2[61]*tmpFx[94] + tmpQN2[62]*tmpFx[110] + tmpQN2[63]*tmpFx[126] + tmpQN2[64]*tmpFx[142] + tmpQN2[65]*tmpFx[158] + tmpQN2[66]*tmpFx[174] + tmpQN2[67]*tmpFx[190] + tmpQN2[68]*tmpFx[206] + tmpQN2[69]*tmpFx[222];
tmpQN1[79] = + tmpQN2[56]*tmpFx[15] + tmpQN2[57]*tmpFx[31] + tmpQN2[58]*tmpFx[47] + tmpQN2[59]*tmpFx[63] + tmpQN2[60]*tmpFx[79] + tmpQN2[61]*tmpFx[95] + tmpQN2[62]*tmpFx[111] + tmpQN2[63]*tmpFx[127] + tmpQN2[64]*tmpFx[143] + tmpQN2[65]*tmpFx[159] + tmpQN2[66]*tmpFx[175] + tmpQN2[67]*tmpFx[191] + tmpQN2[68]*tmpFx[207] + tmpQN2[69]*tmpFx[223];
tmpQN1[80] = + tmpQN2[70]*tmpFx[0] + tmpQN2[71]*tmpFx[16] + tmpQN2[72]*tmpFx[32] + tmpQN2[73]*tmpFx[48] + tmpQN2[74]*tmpFx[64] + tmpQN2[75]*tmpFx[80] + tmpQN2[76]*tmpFx[96] + tmpQN2[77]*tmpFx[112] + tmpQN2[78]*tmpFx[128] + tmpQN2[79]*tmpFx[144] + tmpQN2[80]*tmpFx[160] + tmpQN2[81]*tmpFx[176] + tmpQN2[82]*tmpFx[192] + tmpQN2[83]*tmpFx[208];
tmpQN1[81] = + tmpQN2[70]*tmpFx[1] + tmpQN2[71]*tmpFx[17] + tmpQN2[72]*tmpFx[33] + tmpQN2[73]*tmpFx[49] + tmpQN2[74]*tmpFx[65] + tmpQN2[75]*tmpFx[81] + tmpQN2[76]*tmpFx[97] + tmpQN2[77]*tmpFx[113] + tmpQN2[78]*tmpFx[129] + tmpQN2[79]*tmpFx[145] + tmpQN2[80]*tmpFx[161] + tmpQN2[81]*tmpFx[177] + tmpQN2[82]*tmpFx[193] + tmpQN2[83]*tmpFx[209];
tmpQN1[82] = + tmpQN2[70]*tmpFx[2] + tmpQN2[71]*tmpFx[18] + tmpQN2[72]*tmpFx[34] + tmpQN2[73]*tmpFx[50] + tmpQN2[74]*tmpFx[66] + tmpQN2[75]*tmpFx[82] + tmpQN2[76]*tmpFx[98] + tmpQN2[77]*tmpFx[114] + tmpQN2[78]*tmpFx[130] + tmpQN2[79]*tmpFx[146] + tmpQN2[80]*tmpFx[162] + tmpQN2[81]*tmpFx[178] + tmpQN2[82]*tmpFx[194] + tmpQN2[83]*tmpFx[210];
tmpQN1[83] = + tmpQN2[70]*tmpFx[3] + tmpQN2[71]*tmpFx[19] + tmpQN2[72]*tmpFx[35] + tmpQN2[73]*tmpFx[51] + tmpQN2[74]*tmpFx[67] + tmpQN2[75]*tmpFx[83] + tmpQN2[76]*tmpFx[99] + tmpQN2[77]*tmpFx[115] + tmpQN2[78]*tmpFx[131] + tmpQN2[79]*tmpFx[147] + tmpQN2[80]*tmpFx[163] + tmpQN2[81]*tmpFx[179] + tmpQN2[82]*tmpFx[195] + tmpQN2[83]*tmpFx[211];
tmpQN1[84] = + tmpQN2[70]*tmpFx[4] + tmpQN2[71]*tmpFx[20] + tmpQN2[72]*tmpFx[36] + tmpQN2[73]*tmpFx[52] + tmpQN2[74]*tmpFx[68] + tmpQN2[75]*tmpFx[84] + tmpQN2[76]*tmpFx[100] + tmpQN2[77]*tmpFx[116] + tmpQN2[78]*tmpFx[132] + tmpQN2[79]*tmpFx[148] + tmpQN2[80]*tmpFx[164] + tmpQN2[81]*tmpFx[180] + tmpQN2[82]*tmpFx[196] + tmpQN2[83]*tmpFx[212];
tmpQN1[85] = + tmpQN2[70]*tmpFx[5] + tmpQN2[71]*tmpFx[21] + tmpQN2[72]*tmpFx[37] + tmpQN2[73]*tmpFx[53] + tmpQN2[74]*tmpFx[69] + tmpQN2[75]*tmpFx[85] + tmpQN2[76]*tmpFx[101] + tmpQN2[77]*tmpFx[117] + tmpQN2[78]*tmpFx[133] + tmpQN2[79]*tmpFx[149] + tmpQN2[80]*tmpFx[165] + tmpQN2[81]*tmpFx[181] + tmpQN2[82]*tmpFx[197] + tmpQN2[83]*tmpFx[213];
tmpQN1[86] = + tmpQN2[70]*tmpFx[6] + tmpQN2[71]*tmpFx[22] + tmpQN2[72]*tmpFx[38] + tmpQN2[73]*tmpFx[54] + tmpQN2[74]*tmpFx[70] + tmpQN2[75]*tmpFx[86] + tmpQN2[76]*tmpFx[102] + tmpQN2[77]*tmpFx[118] + tmpQN2[78]*tmpFx[134] + tmpQN2[79]*tmpFx[150] + tmpQN2[80]*tmpFx[166] + tmpQN2[81]*tmpFx[182] + tmpQN2[82]*tmpFx[198] + tmpQN2[83]*tmpFx[214];
tmpQN1[87] = + tmpQN2[70]*tmpFx[7] + tmpQN2[71]*tmpFx[23] + tmpQN2[72]*tmpFx[39] + tmpQN2[73]*tmpFx[55] + tmpQN2[74]*tmpFx[71] + tmpQN2[75]*tmpFx[87] + tmpQN2[76]*tmpFx[103] + tmpQN2[77]*tmpFx[119] + tmpQN2[78]*tmpFx[135] + tmpQN2[79]*tmpFx[151] + tmpQN2[80]*tmpFx[167] + tmpQN2[81]*tmpFx[183] + tmpQN2[82]*tmpFx[199] + tmpQN2[83]*tmpFx[215];
tmpQN1[88] = + tmpQN2[70]*tmpFx[8] + tmpQN2[71]*tmpFx[24] + tmpQN2[72]*tmpFx[40] + tmpQN2[73]*tmpFx[56] + tmpQN2[74]*tmpFx[72] + tmpQN2[75]*tmpFx[88] + tmpQN2[76]*tmpFx[104] + tmpQN2[77]*tmpFx[120] + tmpQN2[78]*tmpFx[136] + tmpQN2[79]*tmpFx[152] + tmpQN2[80]*tmpFx[168] + tmpQN2[81]*tmpFx[184] + tmpQN2[82]*tmpFx[200] + tmpQN2[83]*tmpFx[216];
tmpQN1[89] = + tmpQN2[70]*tmpFx[9] + tmpQN2[71]*tmpFx[25] + tmpQN2[72]*tmpFx[41] + tmpQN2[73]*tmpFx[57] + tmpQN2[74]*tmpFx[73] + tmpQN2[75]*tmpFx[89] + tmpQN2[76]*tmpFx[105] + tmpQN2[77]*tmpFx[121] + tmpQN2[78]*tmpFx[137] + tmpQN2[79]*tmpFx[153] + tmpQN2[80]*tmpFx[169] + tmpQN2[81]*tmpFx[185] + tmpQN2[82]*tmpFx[201] + tmpQN2[83]*tmpFx[217];
tmpQN1[90] = + tmpQN2[70]*tmpFx[10] + tmpQN2[71]*tmpFx[26] + tmpQN2[72]*tmpFx[42] + tmpQN2[73]*tmpFx[58] + tmpQN2[74]*tmpFx[74] + tmpQN2[75]*tmpFx[90] + tmpQN2[76]*tmpFx[106] + tmpQN2[77]*tmpFx[122] + tmpQN2[78]*tmpFx[138] + tmpQN2[79]*tmpFx[154] + tmpQN2[80]*tmpFx[170] + tmpQN2[81]*tmpFx[186] + tmpQN2[82]*tmpFx[202] + tmpQN2[83]*tmpFx[218];
tmpQN1[91] = + tmpQN2[70]*tmpFx[11] + tmpQN2[71]*tmpFx[27] + tmpQN2[72]*tmpFx[43] + tmpQN2[73]*tmpFx[59] + tmpQN2[74]*tmpFx[75] + tmpQN2[75]*tmpFx[91] + tmpQN2[76]*tmpFx[107] + tmpQN2[77]*tmpFx[123] + tmpQN2[78]*tmpFx[139] + tmpQN2[79]*tmpFx[155] + tmpQN2[80]*tmpFx[171] + tmpQN2[81]*tmpFx[187] + tmpQN2[82]*tmpFx[203] + tmpQN2[83]*tmpFx[219];
tmpQN1[92] = + tmpQN2[70]*tmpFx[12] + tmpQN2[71]*tmpFx[28] + tmpQN2[72]*tmpFx[44] + tmpQN2[73]*tmpFx[60] + tmpQN2[74]*tmpFx[76] + tmpQN2[75]*tmpFx[92] + tmpQN2[76]*tmpFx[108] + tmpQN2[77]*tmpFx[124] + tmpQN2[78]*tmpFx[140] + tmpQN2[79]*tmpFx[156] + tmpQN2[80]*tmpFx[172] + tmpQN2[81]*tmpFx[188] + tmpQN2[82]*tmpFx[204] + tmpQN2[83]*tmpFx[220];
tmpQN1[93] = + tmpQN2[70]*tmpFx[13] + tmpQN2[71]*tmpFx[29] + tmpQN2[72]*tmpFx[45] + tmpQN2[73]*tmpFx[61] + tmpQN2[74]*tmpFx[77] + tmpQN2[75]*tmpFx[93] + tmpQN2[76]*tmpFx[109] + tmpQN2[77]*tmpFx[125] + tmpQN2[78]*tmpFx[141] + tmpQN2[79]*tmpFx[157] + tmpQN2[80]*tmpFx[173] + tmpQN2[81]*tmpFx[189] + tmpQN2[82]*tmpFx[205] + tmpQN2[83]*tmpFx[221];
tmpQN1[94] = + tmpQN2[70]*tmpFx[14] + tmpQN2[71]*tmpFx[30] + tmpQN2[72]*tmpFx[46] + tmpQN2[73]*tmpFx[62] + tmpQN2[74]*tmpFx[78] + tmpQN2[75]*tmpFx[94] + tmpQN2[76]*tmpFx[110] + tmpQN2[77]*tmpFx[126] + tmpQN2[78]*tmpFx[142] + tmpQN2[79]*tmpFx[158] + tmpQN2[80]*tmpFx[174] + tmpQN2[81]*tmpFx[190] + tmpQN2[82]*tmpFx[206] + tmpQN2[83]*tmpFx[222];
tmpQN1[95] = + tmpQN2[70]*tmpFx[15] + tmpQN2[71]*tmpFx[31] + tmpQN2[72]*tmpFx[47] + tmpQN2[73]*tmpFx[63] + tmpQN2[74]*tmpFx[79] + tmpQN2[75]*tmpFx[95] + tmpQN2[76]*tmpFx[111] + tmpQN2[77]*tmpFx[127] + tmpQN2[78]*tmpFx[143] + tmpQN2[79]*tmpFx[159] + tmpQN2[80]*tmpFx[175] + tmpQN2[81]*tmpFx[191] + tmpQN2[82]*tmpFx[207] + tmpQN2[83]*tmpFx[223];
tmpQN1[96] = + tmpQN2[84]*tmpFx[0] + tmpQN2[85]*tmpFx[16] + tmpQN2[86]*tmpFx[32] + tmpQN2[87]*tmpFx[48] + tmpQN2[88]*tmpFx[64] + tmpQN2[89]*tmpFx[80] + tmpQN2[90]*tmpFx[96] + tmpQN2[91]*tmpFx[112] + tmpQN2[92]*tmpFx[128] + tmpQN2[93]*tmpFx[144] + tmpQN2[94]*tmpFx[160] + tmpQN2[95]*tmpFx[176] + tmpQN2[96]*tmpFx[192] + tmpQN2[97]*tmpFx[208];
tmpQN1[97] = + tmpQN2[84]*tmpFx[1] + tmpQN2[85]*tmpFx[17] + tmpQN2[86]*tmpFx[33] + tmpQN2[87]*tmpFx[49] + tmpQN2[88]*tmpFx[65] + tmpQN2[89]*tmpFx[81] + tmpQN2[90]*tmpFx[97] + tmpQN2[91]*tmpFx[113] + tmpQN2[92]*tmpFx[129] + tmpQN2[93]*tmpFx[145] + tmpQN2[94]*tmpFx[161] + tmpQN2[95]*tmpFx[177] + tmpQN2[96]*tmpFx[193] + tmpQN2[97]*tmpFx[209];
tmpQN1[98] = + tmpQN2[84]*tmpFx[2] + tmpQN2[85]*tmpFx[18] + tmpQN2[86]*tmpFx[34] + tmpQN2[87]*tmpFx[50] + tmpQN2[88]*tmpFx[66] + tmpQN2[89]*tmpFx[82] + tmpQN2[90]*tmpFx[98] + tmpQN2[91]*tmpFx[114] + tmpQN2[92]*tmpFx[130] + tmpQN2[93]*tmpFx[146] + tmpQN2[94]*tmpFx[162] + tmpQN2[95]*tmpFx[178] + tmpQN2[96]*tmpFx[194] + tmpQN2[97]*tmpFx[210];
tmpQN1[99] = + tmpQN2[84]*tmpFx[3] + tmpQN2[85]*tmpFx[19] + tmpQN2[86]*tmpFx[35] + tmpQN2[87]*tmpFx[51] + tmpQN2[88]*tmpFx[67] + tmpQN2[89]*tmpFx[83] + tmpQN2[90]*tmpFx[99] + tmpQN2[91]*tmpFx[115] + tmpQN2[92]*tmpFx[131] + tmpQN2[93]*tmpFx[147] + tmpQN2[94]*tmpFx[163] + tmpQN2[95]*tmpFx[179] + tmpQN2[96]*tmpFx[195] + tmpQN2[97]*tmpFx[211];
tmpQN1[100] = + tmpQN2[84]*tmpFx[4] + tmpQN2[85]*tmpFx[20] + tmpQN2[86]*tmpFx[36] + tmpQN2[87]*tmpFx[52] + tmpQN2[88]*tmpFx[68] + tmpQN2[89]*tmpFx[84] + tmpQN2[90]*tmpFx[100] + tmpQN2[91]*tmpFx[116] + tmpQN2[92]*tmpFx[132] + tmpQN2[93]*tmpFx[148] + tmpQN2[94]*tmpFx[164] + tmpQN2[95]*tmpFx[180] + tmpQN2[96]*tmpFx[196] + tmpQN2[97]*tmpFx[212];
tmpQN1[101] = + tmpQN2[84]*tmpFx[5] + tmpQN2[85]*tmpFx[21] + tmpQN2[86]*tmpFx[37] + tmpQN2[87]*tmpFx[53] + tmpQN2[88]*tmpFx[69] + tmpQN2[89]*tmpFx[85] + tmpQN2[90]*tmpFx[101] + tmpQN2[91]*tmpFx[117] + tmpQN2[92]*tmpFx[133] + tmpQN2[93]*tmpFx[149] + tmpQN2[94]*tmpFx[165] + tmpQN2[95]*tmpFx[181] + tmpQN2[96]*tmpFx[197] + tmpQN2[97]*tmpFx[213];
tmpQN1[102] = + tmpQN2[84]*tmpFx[6] + tmpQN2[85]*tmpFx[22] + tmpQN2[86]*tmpFx[38] + tmpQN2[87]*tmpFx[54] + tmpQN2[88]*tmpFx[70] + tmpQN2[89]*tmpFx[86] + tmpQN2[90]*tmpFx[102] + tmpQN2[91]*tmpFx[118] + tmpQN2[92]*tmpFx[134] + tmpQN2[93]*tmpFx[150] + tmpQN2[94]*tmpFx[166] + tmpQN2[95]*tmpFx[182] + tmpQN2[96]*tmpFx[198] + tmpQN2[97]*tmpFx[214];
tmpQN1[103] = + tmpQN2[84]*tmpFx[7] + tmpQN2[85]*tmpFx[23] + tmpQN2[86]*tmpFx[39] + tmpQN2[87]*tmpFx[55] + tmpQN2[88]*tmpFx[71] + tmpQN2[89]*tmpFx[87] + tmpQN2[90]*tmpFx[103] + tmpQN2[91]*tmpFx[119] + tmpQN2[92]*tmpFx[135] + tmpQN2[93]*tmpFx[151] + tmpQN2[94]*tmpFx[167] + tmpQN2[95]*tmpFx[183] + tmpQN2[96]*tmpFx[199] + tmpQN2[97]*tmpFx[215];
tmpQN1[104] = + tmpQN2[84]*tmpFx[8] + tmpQN2[85]*tmpFx[24] + tmpQN2[86]*tmpFx[40] + tmpQN2[87]*tmpFx[56] + tmpQN2[88]*tmpFx[72] + tmpQN2[89]*tmpFx[88] + tmpQN2[90]*tmpFx[104] + tmpQN2[91]*tmpFx[120] + tmpQN2[92]*tmpFx[136] + tmpQN2[93]*tmpFx[152] + tmpQN2[94]*tmpFx[168] + tmpQN2[95]*tmpFx[184] + tmpQN2[96]*tmpFx[200] + tmpQN2[97]*tmpFx[216];
tmpQN1[105] = + tmpQN2[84]*tmpFx[9] + tmpQN2[85]*tmpFx[25] + tmpQN2[86]*tmpFx[41] + tmpQN2[87]*tmpFx[57] + tmpQN2[88]*tmpFx[73] + tmpQN2[89]*tmpFx[89] + tmpQN2[90]*tmpFx[105] + tmpQN2[91]*tmpFx[121] + tmpQN2[92]*tmpFx[137] + tmpQN2[93]*tmpFx[153] + tmpQN2[94]*tmpFx[169] + tmpQN2[95]*tmpFx[185] + tmpQN2[96]*tmpFx[201] + tmpQN2[97]*tmpFx[217];
tmpQN1[106] = + tmpQN2[84]*tmpFx[10] + tmpQN2[85]*tmpFx[26] + tmpQN2[86]*tmpFx[42] + tmpQN2[87]*tmpFx[58] + tmpQN2[88]*tmpFx[74] + tmpQN2[89]*tmpFx[90] + tmpQN2[90]*tmpFx[106] + tmpQN2[91]*tmpFx[122] + tmpQN2[92]*tmpFx[138] + tmpQN2[93]*tmpFx[154] + tmpQN2[94]*tmpFx[170] + tmpQN2[95]*tmpFx[186] + tmpQN2[96]*tmpFx[202] + tmpQN2[97]*tmpFx[218];
tmpQN1[107] = + tmpQN2[84]*tmpFx[11] + tmpQN2[85]*tmpFx[27] + tmpQN2[86]*tmpFx[43] + tmpQN2[87]*tmpFx[59] + tmpQN2[88]*tmpFx[75] + tmpQN2[89]*tmpFx[91] + tmpQN2[90]*tmpFx[107] + tmpQN2[91]*tmpFx[123] + tmpQN2[92]*tmpFx[139] + tmpQN2[93]*tmpFx[155] + tmpQN2[94]*tmpFx[171] + tmpQN2[95]*tmpFx[187] + tmpQN2[96]*tmpFx[203] + tmpQN2[97]*tmpFx[219];
tmpQN1[108] = + tmpQN2[84]*tmpFx[12] + tmpQN2[85]*tmpFx[28] + tmpQN2[86]*tmpFx[44] + tmpQN2[87]*tmpFx[60] + tmpQN2[88]*tmpFx[76] + tmpQN2[89]*tmpFx[92] + tmpQN2[90]*tmpFx[108] + tmpQN2[91]*tmpFx[124] + tmpQN2[92]*tmpFx[140] + tmpQN2[93]*tmpFx[156] + tmpQN2[94]*tmpFx[172] + tmpQN2[95]*tmpFx[188] + tmpQN2[96]*tmpFx[204] + tmpQN2[97]*tmpFx[220];
tmpQN1[109] = + tmpQN2[84]*tmpFx[13] + tmpQN2[85]*tmpFx[29] + tmpQN2[86]*tmpFx[45] + tmpQN2[87]*tmpFx[61] + tmpQN2[88]*tmpFx[77] + tmpQN2[89]*tmpFx[93] + tmpQN2[90]*tmpFx[109] + tmpQN2[91]*tmpFx[125] + tmpQN2[92]*tmpFx[141] + tmpQN2[93]*tmpFx[157] + tmpQN2[94]*tmpFx[173] + tmpQN2[95]*tmpFx[189] + tmpQN2[96]*tmpFx[205] + tmpQN2[97]*tmpFx[221];
tmpQN1[110] = + tmpQN2[84]*tmpFx[14] + tmpQN2[85]*tmpFx[30] + tmpQN2[86]*tmpFx[46] + tmpQN2[87]*tmpFx[62] + tmpQN2[88]*tmpFx[78] + tmpQN2[89]*tmpFx[94] + tmpQN2[90]*tmpFx[110] + tmpQN2[91]*tmpFx[126] + tmpQN2[92]*tmpFx[142] + tmpQN2[93]*tmpFx[158] + tmpQN2[94]*tmpFx[174] + tmpQN2[95]*tmpFx[190] + tmpQN2[96]*tmpFx[206] + tmpQN2[97]*tmpFx[222];
tmpQN1[111] = + tmpQN2[84]*tmpFx[15] + tmpQN2[85]*tmpFx[31] + tmpQN2[86]*tmpFx[47] + tmpQN2[87]*tmpFx[63] + tmpQN2[88]*tmpFx[79] + tmpQN2[89]*tmpFx[95] + tmpQN2[90]*tmpFx[111] + tmpQN2[91]*tmpFx[127] + tmpQN2[92]*tmpFx[143] + tmpQN2[93]*tmpFx[159] + tmpQN2[94]*tmpFx[175] + tmpQN2[95]*tmpFx[191] + tmpQN2[96]*tmpFx[207] + tmpQN2[97]*tmpFx[223];
tmpQN1[112] = + tmpQN2[98]*tmpFx[0] + tmpQN2[99]*tmpFx[16] + tmpQN2[100]*tmpFx[32] + tmpQN2[101]*tmpFx[48] + tmpQN2[102]*tmpFx[64] + tmpQN2[103]*tmpFx[80] + tmpQN2[104]*tmpFx[96] + tmpQN2[105]*tmpFx[112] + tmpQN2[106]*tmpFx[128] + tmpQN2[107]*tmpFx[144] + tmpQN2[108]*tmpFx[160] + tmpQN2[109]*tmpFx[176] + tmpQN2[110]*tmpFx[192] + tmpQN2[111]*tmpFx[208];
tmpQN1[113] = + tmpQN2[98]*tmpFx[1] + tmpQN2[99]*tmpFx[17] + tmpQN2[100]*tmpFx[33] + tmpQN2[101]*tmpFx[49] + tmpQN2[102]*tmpFx[65] + tmpQN2[103]*tmpFx[81] + tmpQN2[104]*tmpFx[97] + tmpQN2[105]*tmpFx[113] + tmpQN2[106]*tmpFx[129] + tmpQN2[107]*tmpFx[145] + tmpQN2[108]*tmpFx[161] + tmpQN2[109]*tmpFx[177] + tmpQN2[110]*tmpFx[193] + tmpQN2[111]*tmpFx[209];
tmpQN1[114] = + tmpQN2[98]*tmpFx[2] + tmpQN2[99]*tmpFx[18] + tmpQN2[100]*tmpFx[34] + tmpQN2[101]*tmpFx[50] + tmpQN2[102]*tmpFx[66] + tmpQN2[103]*tmpFx[82] + tmpQN2[104]*tmpFx[98] + tmpQN2[105]*tmpFx[114] + tmpQN2[106]*tmpFx[130] + tmpQN2[107]*tmpFx[146] + tmpQN2[108]*tmpFx[162] + tmpQN2[109]*tmpFx[178] + tmpQN2[110]*tmpFx[194] + tmpQN2[111]*tmpFx[210];
tmpQN1[115] = + tmpQN2[98]*tmpFx[3] + tmpQN2[99]*tmpFx[19] + tmpQN2[100]*tmpFx[35] + tmpQN2[101]*tmpFx[51] + tmpQN2[102]*tmpFx[67] + tmpQN2[103]*tmpFx[83] + tmpQN2[104]*tmpFx[99] + tmpQN2[105]*tmpFx[115] + tmpQN2[106]*tmpFx[131] + tmpQN2[107]*tmpFx[147] + tmpQN2[108]*tmpFx[163] + tmpQN2[109]*tmpFx[179] + tmpQN2[110]*tmpFx[195] + tmpQN2[111]*tmpFx[211];
tmpQN1[116] = + tmpQN2[98]*tmpFx[4] + tmpQN2[99]*tmpFx[20] + tmpQN2[100]*tmpFx[36] + tmpQN2[101]*tmpFx[52] + tmpQN2[102]*tmpFx[68] + tmpQN2[103]*tmpFx[84] + tmpQN2[104]*tmpFx[100] + tmpQN2[105]*tmpFx[116] + tmpQN2[106]*tmpFx[132] + tmpQN2[107]*tmpFx[148] + tmpQN2[108]*tmpFx[164] + tmpQN2[109]*tmpFx[180] + tmpQN2[110]*tmpFx[196] + tmpQN2[111]*tmpFx[212];
tmpQN1[117] = + tmpQN2[98]*tmpFx[5] + tmpQN2[99]*tmpFx[21] + tmpQN2[100]*tmpFx[37] + tmpQN2[101]*tmpFx[53] + tmpQN2[102]*tmpFx[69] + tmpQN2[103]*tmpFx[85] + tmpQN2[104]*tmpFx[101] + tmpQN2[105]*tmpFx[117] + tmpQN2[106]*tmpFx[133] + tmpQN2[107]*tmpFx[149] + tmpQN2[108]*tmpFx[165] + tmpQN2[109]*tmpFx[181] + tmpQN2[110]*tmpFx[197] + tmpQN2[111]*tmpFx[213];
tmpQN1[118] = + tmpQN2[98]*tmpFx[6] + tmpQN2[99]*tmpFx[22] + tmpQN2[100]*tmpFx[38] + tmpQN2[101]*tmpFx[54] + tmpQN2[102]*tmpFx[70] + tmpQN2[103]*tmpFx[86] + tmpQN2[104]*tmpFx[102] + tmpQN2[105]*tmpFx[118] + tmpQN2[106]*tmpFx[134] + tmpQN2[107]*tmpFx[150] + tmpQN2[108]*tmpFx[166] + tmpQN2[109]*tmpFx[182] + tmpQN2[110]*tmpFx[198] + tmpQN2[111]*tmpFx[214];
tmpQN1[119] = + tmpQN2[98]*tmpFx[7] + tmpQN2[99]*tmpFx[23] + tmpQN2[100]*tmpFx[39] + tmpQN2[101]*tmpFx[55] + tmpQN2[102]*tmpFx[71] + tmpQN2[103]*tmpFx[87] + tmpQN2[104]*tmpFx[103] + tmpQN2[105]*tmpFx[119] + tmpQN2[106]*tmpFx[135] + tmpQN2[107]*tmpFx[151] + tmpQN2[108]*tmpFx[167] + tmpQN2[109]*tmpFx[183] + tmpQN2[110]*tmpFx[199] + tmpQN2[111]*tmpFx[215];
tmpQN1[120] = + tmpQN2[98]*tmpFx[8] + tmpQN2[99]*tmpFx[24] + tmpQN2[100]*tmpFx[40] + tmpQN2[101]*tmpFx[56] + tmpQN2[102]*tmpFx[72] + tmpQN2[103]*tmpFx[88] + tmpQN2[104]*tmpFx[104] + tmpQN2[105]*tmpFx[120] + tmpQN2[106]*tmpFx[136] + tmpQN2[107]*tmpFx[152] + tmpQN2[108]*tmpFx[168] + tmpQN2[109]*tmpFx[184] + tmpQN2[110]*tmpFx[200] + tmpQN2[111]*tmpFx[216];
tmpQN1[121] = + tmpQN2[98]*tmpFx[9] + tmpQN2[99]*tmpFx[25] + tmpQN2[100]*tmpFx[41] + tmpQN2[101]*tmpFx[57] + tmpQN2[102]*tmpFx[73] + tmpQN2[103]*tmpFx[89] + tmpQN2[104]*tmpFx[105] + tmpQN2[105]*tmpFx[121] + tmpQN2[106]*tmpFx[137] + tmpQN2[107]*tmpFx[153] + tmpQN2[108]*tmpFx[169] + tmpQN2[109]*tmpFx[185] + tmpQN2[110]*tmpFx[201] + tmpQN2[111]*tmpFx[217];
tmpQN1[122] = + tmpQN2[98]*tmpFx[10] + tmpQN2[99]*tmpFx[26] + tmpQN2[100]*tmpFx[42] + tmpQN2[101]*tmpFx[58] + tmpQN2[102]*tmpFx[74] + tmpQN2[103]*tmpFx[90] + tmpQN2[104]*tmpFx[106] + tmpQN2[105]*tmpFx[122] + tmpQN2[106]*tmpFx[138] + tmpQN2[107]*tmpFx[154] + tmpQN2[108]*tmpFx[170] + tmpQN2[109]*tmpFx[186] + tmpQN2[110]*tmpFx[202] + tmpQN2[111]*tmpFx[218];
tmpQN1[123] = + tmpQN2[98]*tmpFx[11] + tmpQN2[99]*tmpFx[27] + tmpQN2[100]*tmpFx[43] + tmpQN2[101]*tmpFx[59] + tmpQN2[102]*tmpFx[75] + tmpQN2[103]*tmpFx[91] + tmpQN2[104]*tmpFx[107] + tmpQN2[105]*tmpFx[123] + tmpQN2[106]*tmpFx[139] + tmpQN2[107]*tmpFx[155] + tmpQN2[108]*tmpFx[171] + tmpQN2[109]*tmpFx[187] + tmpQN2[110]*tmpFx[203] + tmpQN2[111]*tmpFx[219];
tmpQN1[124] = + tmpQN2[98]*tmpFx[12] + tmpQN2[99]*tmpFx[28] + tmpQN2[100]*tmpFx[44] + tmpQN2[101]*tmpFx[60] + tmpQN2[102]*tmpFx[76] + tmpQN2[103]*tmpFx[92] + tmpQN2[104]*tmpFx[108] + tmpQN2[105]*tmpFx[124] + tmpQN2[106]*tmpFx[140] + tmpQN2[107]*tmpFx[156] + tmpQN2[108]*tmpFx[172] + tmpQN2[109]*tmpFx[188] + tmpQN2[110]*tmpFx[204] + tmpQN2[111]*tmpFx[220];
tmpQN1[125] = + tmpQN2[98]*tmpFx[13] + tmpQN2[99]*tmpFx[29] + tmpQN2[100]*tmpFx[45] + tmpQN2[101]*tmpFx[61] + tmpQN2[102]*tmpFx[77] + tmpQN2[103]*tmpFx[93] + tmpQN2[104]*tmpFx[109] + tmpQN2[105]*tmpFx[125] + tmpQN2[106]*tmpFx[141] + tmpQN2[107]*tmpFx[157] + tmpQN2[108]*tmpFx[173] + tmpQN2[109]*tmpFx[189] + tmpQN2[110]*tmpFx[205] + tmpQN2[111]*tmpFx[221];
tmpQN1[126] = + tmpQN2[98]*tmpFx[14] + tmpQN2[99]*tmpFx[30] + tmpQN2[100]*tmpFx[46] + tmpQN2[101]*tmpFx[62] + tmpQN2[102]*tmpFx[78] + tmpQN2[103]*tmpFx[94] + tmpQN2[104]*tmpFx[110] + tmpQN2[105]*tmpFx[126] + tmpQN2[106]*tmpFx[142] + tmpQN2[107]*tmpFx[158] + tmpQN2[108]*tmpFx[174] + tmpQN2[109]*tmpFx[190] + tmpQN2[110]*tmpFx[206] + tmpQN2[111]*tmpFx[222];
tmpQN1[127] = + tmpQN2[98]*tmpFx[15] + tmpQN2[99]*tmpFx[31] + tmpQN2[100]*tmpFx[47] + tmpQN2[101]*tmpFx[63] + tmpQN2[102]*tmpFx[79] + tmpQN2[103]*tmpFx[95] + tmpQN2[104]*tmpFx[111] + tmpQN2[105]*tmpFx[127] + tmpQN2[106]*tmpFx[143] + tmpQN2[107]*tmpFx[159] + tmpQN2[108]*tmpFx[175] + tmpQN2[109]*tmpFx[191] + tmpQN2[110]*tmpFx[207] + tmpQN2[111]*tmpFx[223];
tmpQN1[128] = + tmpQN2[112]*tmpFx[0] + tmpQN2[113]*tmpFx[16] + tmpQN2[114]*tmpFx[32] + tmpQN2[115]*tmpFx[48] + tmpQN2[116]*tmpFx[64] + tmpQN2[117]*tmpFx[80] + tmpQN2[118]*tmpFx[96] + tmpQN2[119]*tmpFx[112] + tmpQN2[120]*tmpFx[128] + tmpQN2[121]*tmpFx[144] + tmpQN2[122]*tmpFx[160] + tmpQN2[123]*tmpFx[176] + tmpQN2[124]*tmpFx[192] + tmpQN2[125]*tmpFx[208];
tmpQN1[129] = + tmpQN2[112]*tmpFx[1] + tmpQN2[113]*tmpFx[17] + tmpQN2[114]*tmpFx[33] + tmpQN2[115]*tmpFx[49] + tmpQN2[116]*tmpFx[65] + tmpQN2[117]*tmpFx[81] + tmpQN2[118]*tmpFx[97] + tmpQN2[119]*tmpFx[113] + tmpQN2[120]*tmpFx[129] + tmpQN2[121]*tmpFx[145] + tmpQN2[122]*tmpFx[161] + tmpQN2[123]*tmpFx[177] + tmpQN2[124]*tmpFx[193] + tmpQN2[125]*tmpFx[209];
tmpQN1[130] = + tmpQN2[112]*tmpFx[2] + tmpQN2[113]*tmpFx[18] + tmpQN2[114]*tmpFx[34] + tmpQN2[115]*tmpFx[50] + tmpQN2[116]*tmpFx[66] + tmpQN2[117]*tmpFx[82] + tmpQN2[118]*tmpFx[98] + tmpQN2[119]*tmpFx[114] + tmpQN2[120]*tmpFx[130] + tmpQN2[121]*tmpFx[146] + tmpQN2[122]*tmpFx[162] + tmpQN2[123]*tmpFx[178] + tmpQN2[124]*tmpFx[194] + tmpQN2[125]*tmpFx[210];
tmpQN1[131] = + tmpQN2[112]*tmpFx[3] + tmpQN2[113]*tmpFx[19] + tmpQN2[114]*tmpFx[35] + tmpQN2[115]*tmpFx[51] + tmpQN2[116]*tmpFx[67] + tmpQN2[117]*tmpFx[83] + tmpQN2[118]*tmpFx[99] + tmpQN2[119]*tmpFx[115] + tmpQN2[120]*tmpFx[131] + tmpQN2[121]*tmpFx[147] + tmpQN2[122]*tmpFx[163] + tmpQN2[123]*tmpFx[179] + tmpQN2[124]*tmpFx[195] + tmpQN2[125]*tmpFx[211];
tmpQN1[132] = + tmpQN2[112]*tmpFx[4] + tmpQN2[113]*tmpFx[20] + tmpQN2[114]*tmpFx[36] + tmpQN2[115]*tmpFx[52] + tmpQN2[116]*tmpFx[68] + tmpQN2[117]*tmpFx[84] + tmpQN2[118]*tmpFx[100] + tmpQN2[119]*tmpFx[116] + tmpQN2[120]*tmpFx[132] + tmpQN2[121]*tmpFx[148] + tmpQN2[122]*tmpFx[164] + tmpQN2[123]*tmpFx[180] + tmpQN2[124]*tmpFx[196] + tmpQN2[125]*tmpFx[212];
tmpQN1[133] = + tmpQN2[112]*tmpFx[5] + tmpQN2[113]*tmpFx[21] + tmpQN2[114]*tmpFx[37] + tmpQN2[115]*tmpFx[53] + tmpQN2[116]*tmpFx[69] + tmpQN2[117]*tmpFx[85] + tmpQN2[118]*tmpFx[101] + tmpQN2[119]*tmpFx[117] + tmpQN2[120]*tmpFx[133] + tmpQN2[121]*tmpFx[149] + tmpQN2[122]*tmpFx[165] + tmpQN2[123]*tmpFx[181] + tmpQN2[124]*tmpFx[197] + tmpQN2[125]*tmpFx[213];
tmpQN1[134] = + tmpQN2[112]*tmpFx[6] + tmpQN2[113]*tmpFx[22] + tmpQN2[114]*tmpFx[38] + tmpQN2[115]*tmpFx[54] + tmpQN2[116]*tmpFx[70] + tmpQN2[117]*tmpFx[86] + tmpQN2[118]*tmpFx[102] + tmpQN2[119]*tmpFx[118] + tmpQN2[120]*tmpFx[134] + tmpQN2[121]*tmpFx[150] + tmpQN2[122]*tmpFx[166] + tmpQN2[123]*tmpFx[182] + tmpQN2[124]*tmpFx[198] + tmpQN2[125]*tmpFx[214];
tmpQN1[135] = + tmpQN2[112]*tmpFx[7] + tmpQN2[113]*tmpFx[23] + tmpQN2[114]*tmpFx[39] + tmpQN2[115]*tmpFx[55] + tmpQN2[116]*tmpFx[71] + tmpQN2[117]*tmpFx[87] + tmpQN2[118]*tmpFx[103] + tmpQN2[119]*tmpFx[119] + tmpQN2[120]*tmpFx[135] + tmpQN2[121]*tmpFx[151] + tmpQN2[122]*tmpFx[167] + tmpQN2[123]*tmpFx[183] + tmpQN2[124]*tmpFx[199] + tmpQN2[125]*tmpFx[215];
tmpQN1[136] = + tmpQN2[112]*tmpFx[8] + tmpQN2[113]*tmpFx[24] + tmpQN2[114]*tmpFx[40] + tmpQN2[115]*tmpFx[56] + tmpQN2[116]*tmpFx[72] + tmpQN2[117]*tmpFx[88] + tmpQN2[118]*tmpFx[104] + tmpQN2[119]*tmpFx[120] + tmpQN2[120]*tmpFx[136] + tmpQN2[121]*tmpFx[152] + tmpQN2[122]*tmpFx[168] + tmpQN2[123]*tmpFx[184] + tmpQN2[124]*tmpFx[200] + tmpQN2[125]*tmpFx[216];
tmpQN1[137] = + tmpQN2[112]*tmpFx[9] + tmpQN2[113]*tmpFx[25] + tmpQN2[114]*tmpFx[41] + tmpQN2[115]*tmpFx[57] + tmpQN2[116]*tmpFx[73] + tmpQN2[117]*tmpFx[89] + tmpQN2[118]*tmpFx[105] + tmpQN2[119]*tmpFx[121] + tmpQN2[120]*tmpFx[137] + tmpQN2[121]*tmpFx[153] + tmpQN2[122]*tmpFx[169] + tmpQN2[123]*tmpFx[185] + tmpQN2[124]*tmpFx[201] + tmpQN2[125]*tmpFx[217];
tmpQN1[138] = + tmpQN2[112]*tmpFx[10] + tmpQN2[113]*tmpFx[26] + tmpQN2[114]*tmpFx[42] + tmpQN2[115]*tmpFx[58] + tmpQN2[116]*tmpFx[74] + tmpQN2[117]*tmpFx[90] + tmpQN2[118]*tmpFx[106] + tmpQN2[119]*tmpFx[122] + tmpQN2[120]*tmpFx[138] + tmpQN2[121]*tmpFx[154] + tmpQN2[122]*tmpFx[170] + tmpQN2[123]*tmpFx[186] + tmpQN2[124]*tmpFx[202] + tmpQN2[125]*tmpFx[218];
tmpQN1[139] = + tmpQN2[112]*tmpFx[11] + tmpQN2[113]*tmpFx[27] + tmpQN2[114]*tmpFx[43] + tmpQN2[115]*tmpFx[59] + tmpQN2[116]*tmpFx[75] + tmpQN2[117]*tmpFx[91] + tmpQN2[118]*tmpFx[107] + tmpQN2[119]*tmpFx[123] + tmpQN2[120]*tmpFx[139] + tmpQN2[121]*tmpFx[155] + tmpQN2[122]*tmpFx[171] + tmpQN2[123]*tmpFx[187] + tmpQN2[124]*tmpFx[203] + tmpQN2[125]*tmpFx[219];
tmpQN1[140] = + tmpQN2[112]*tmpFx[12] + tmpQN2[113]*tmpFx[28] + tmpQN2[114]*tmpFx[44] + tmpQN2[115]*tmpFx[60] + tmpQN2[116]*tmpFx[76] + tmpQN2[117]*tmpFx[92] + tmpQN2[118]*tmpFx[108] + tmpQN2[119]*tmpFx[124] + tmpQN2[120]*tmpFx[140] + tmpQN2[121]*tmpFx[156] + tmpQN2[122]*tmpFx[172] + tmpQN2[123]*tmpFx[188] + tmpQN2[124]*tmpFx[204] + tmpQN2[125]*tmpFx[220];
tmpQN1[141] = + tmpQN2[112]*tmpFx[13] + tmpQN2[113]*tmpFx[29] + tmpQN2[114]*tmpFx[45] + tmpQN2[115]*tmpFx[61] + tmpQN2[116]*tmpFx[77] + tmpQN2[117]*tmpFx[93] + tmpQN2[118]*tmpFx[109] + tmpQN2[119]*tmpFx[125] + tmpQN2[120]*tmpFx[141] + tmpQN2[121]*tmpFx[157] + tmpQN2[122]*tmpFx[173] + tmpQN2[123]*tmpFx[189] + tmpQN2[124]*tmpFx[205] + tmpQN2[125]*tmpFx[221];
tmpQN1[142] = + tmpQN2[112]*tmpFx[14] + tmpQN2[113]*tmpFx[30] + tmpQN2[114]*tmpFx[46] + tmpQN2[115]*tmpFx[62] + tmpQN2[116]*tmpFx[78] + tmpQN2[117]*tmpFx[94] + tmpQN2[118]*tmpFx[110] + tmpQN2[119]*tmpFx[126] + tmpQN2[120]*tmpFx[142] + tmpQN2[121]*tmpFx[158] + tmpQN2[122]*tmpFx[174] + tmpQN2[123]*tmpFx[190] + tmpQN2[124]*tmpFx[206] + tmpQN2[125]*tmpFx[222];
tmpQN1[143] = + tmpQN2[112]*tmpFx[15] + tmpQN2[113]*tmpFx[31] + tmpQN2[114]*tmpFx[47] + tmpQN2[115]*tmpFx[63] + tmpQN2[116]*tmpFx[79] + tmpQN2[117]*tmpFx[95] + tmpQN2[118]*tmpFx[111] + tmpQN2[119]*tmpFx[127] + tmpQN2[120]*tmpFx[143] + tmpQN2[121]*tmpFx[159] + tmpQN2[122]*tmpFx[175] + tmpQN2[123]*tmpFx[191] + tmpQN2[124]*tmpFx[207] + tmpQN2[125]*tmpFx[223];
tmpQN1[144] = + tmpQN2[126]*tmpFx[0] + tmpQN2[127]*tmpFx[16] + tmpQN2[128]*tmpFx[32] + tmpQN2[129]*tmpFx[48] + tmpQN2[130]*tmpFx[64] + tmpQN2[131]*tmpFx[80] + tmpQN2[132]*tmpFx[96] + tmpQN2[133]*tmpFx[112] + tmpQN2[134]*tmpFx[128] + tmpQN2[135]*tmpFx[144] + tmpQN2[136]*tmpFx[160] + tmpQN2[137]*tmpFx[176] + tmpQN2[138]*tmpFx[192] + tmpQN2[139]*tmpFx[208];
tmpQN1[145] = + tmpQN2[126]*tmpFx[1] + tmpQN2[127]*tmpFx[17] + tmpQN2[128]*tmpFx[33] + tmpQN2[129]*tmpFx[49] + tmpQN2[130]*tmpFx[65] + tmpQN2[131]*tmpFx[81] + tmpQN2[132]*tmpFx[97] + tmpQN2[133]*tmpFx[113] + tmpQN2[134]*tmpFx[129] + tmpQN2[135]*tmpFx[145] + tmpQN2[136]*tmpFx[161] + tmpQN2[137]*tmpFx[177] + tmpQN2[138]*tmpFx[193] + tmpQN2[139]*tmpFx[209];
tmpQN1[146] = + tmpQN2[126]*tmpFx[2] + tmpQN2[127]*tmpFx[18] + tmpQN2[128]*tmpFx[34] + tmpQN2[129]*tmpFx[50] + tmpQN2[130]*tmpFx[66] + tmpQN2[131]*tmpFx[82] + tmpQN2[132]*tmpFx[98] + tmpQN2[133]*tmpFx[114] + tmpQN2[134]*tmpFx[130] + tmpQN2[135]*tmpFx[146] + tmpQN2[136]*tmpFx[162] + tmpQN2[137]*tmpFx[178] + tmpQN2[138]*tmpFx[194] + tmpQN2[139]*tmpFx[210];
tmpQN1[147] = + tmpQN2[126]*tmpFx[3] + tmpQN2[127]*tmpFx[19] + tmpQN2[128]*tmpFx[35] + tmpQN2[129]*tmpFx[51] + tmpQN2[130]*tmpFx[67] + tmpQN2[131]*tmpFx[83] + tmpQN2[132]*tmpFx[99] + tmpQN2[133]*tmpFx[115] + tmpQN2[134]*tmpFx[131] + tmpQN2[135]*tmpFx[147] + tmpQN2[136]*tmpFx[163] + tmpQN2[137]*tmpFx[179] + tmpQN2[138]*tmpFx[195] + tmpQN2[139]*tmpFx[211];
tmpQN1[148] = + tmpQN2[126]*tmpFx[4] + tmpQN2[127]*tmpFx[20] + tmpQN2[128]*tmpFx[36] + tmpQN2[129]*tmpFx[52] + tmpQN2[130]*tmpFx[68] + tmpQN2[131]*tmpFx[84] + tmpQN2[132]*tmpFx[100] + tmpQN2[133]*tmpFx[116] + tmpQN2[134]*tmpFx[132] + tmpQN2[135]*tmpFx[148] + tmpQN2[136]*tmpFx[164] + tmpQN2[137]*tmpFx[180] + tmpQN2[138]*tmpFx[196] + tmpQN2[139]*tmpFx[212];
tmpQN1[149] = + tmpQN2[126]*tmpFx[5] + tmpQN2[127]*tmpFx[21] + tmpQN2[128]*tmpFx[37] + tmpQN2[129]*tmpFx[53] + tmpQN2[130]*tmpFx[69] + tmpQN2[131]*tmpFx[85] + tmpQN2[132]*tmpFx[101] + tmpQN2[133]*tmpFx[117] + tmpQN2[134]*tmpFx[133] + tmpQN2[135]*tmpFx[149] + tmpQN2[136]*tmpFx[165] + tmpQN2[137]*tmpFx[181] + tmpQN2[138]*tmpFx[197] + tmpQN2[139]*tmpFx[213];
tmpQN1[150] = + tmpQN2[126]*tmpFx[6] + tmpQN2[127]*tmpFx[22] + tmpQN2[128]*tmpFx[38] + tmpQN2[129]*tmpFx[54] + tmpQN2[130]*tmpFx[70] + tmpQN2[131]*tmpFx[86] + tmpQN2[132]*tmpFx[102] + tmpQN2[133]*tmpFx[118] + tmpQN2[134]*tmpFx[134] + tmpQN2[135]*tmpFx[150] + tmpQN2[136]*tmpFx[166] + tmpQN2[137]*tmpFx[182] + tmpQN2[138]*tmpFx[198] + tmpQN2[139]*tmpFx[214];
tmpQN1[151] = + tmpQN2[126]*tmpFx[7] + tmpQN2[127]*tmpFx[23] + tmpQN2[128]*tmpFx[39] + tmpQN2[129]*tmpFx[55] + tmpQN2[130]*tmpFx[71] + tmpQN2[131]*tmpFx[87] + tmpQN2[132]*tmpFx[103] + tmpQN2[133]*tmpFx[119] + tmpQN2[134]*tmpFx[135] + tmpQN2[135]*tmpFx[151] + tmpQN2[136]*tmpFx[167] + tmpQN2[137]*tmpFx[183] + tmpQN2[138]*tmpFx[199] + tmpQN2[139]*tmpFx[215];
tmpQN1[152] = + tmpQN2[126]*tmpFx[8] + tmpQN2[127]*tmpFx[24] + tmpQN2[128]*tmpFx[40] + tmpQN2[129]*tmpFx[56] + tmpQN2[130]*tmpFx[72] + tmpQN2[131]*tmpFx[88] + tmpQN2[132]*tmpFx[104] + tmpQN2[133]*tmpFx[120] + tmpQN2[134]*tmpFx[136] + tmpQN2[135]*tmpFx[152] + tmpQN2[136]*tmpFx[168] + tmpQN2[137]*tmpFx[184] + tmpQN2[138]*tmpFx[200] + tmpQN2[139]*tmpFx[216];
tmpQN1[153] = + tmpQN2[126]*tmpFx[9] + tmpQN2[127]*tmpFx[25] + tmpQN2[128]*tmpFx[41] + tmpQN2[129]*tmpFx[57] + tmpQN2[130]*tmpFx[73] + tmpQN2[131]*tmpFx[89] + tmpQN2[132]*tmpFx[105] + tmpQN2[133]*tmpFx[121] + tmpQN2[134]*tmpFx[137] + tmpQN2[135]*tmpFx[153] + tmpQN2[136]*tmpFx[169] + tmpQN2[137]*tmpFx[185] + tmpQN2[138]*tmpFx[201] + tmpQN2[139]*tmpFx[217];
tmpQN1[154] = + tmpQN2[126]*tmpFx[10] + tmpQN2[127]*tmpFx[26] + tmpQN2[128]*tmpFx[42] + tmpQN2[129]*tmpFx[58] + tmpQN2[130]*tmpFx[74] + tmpQN2[131]*tmpFx[90] + tmpQN2[132]*tmpFx[106] + tmpQN2[133]*tmpFx[122] + tmpQN2[134]*tmpFx[138] + tmpQN2[135]*tmpFx[154] + tmpQN2[136]*tmpFx[170] + tmpQN2[137]*tmpFx[186] + tmpQN2[138]*tmpFx[202] + tmpQN2[139]*tmpFx[218];
tmpQN1[155] = + tmpQN2[126]*tmpFx[11] + tmpQN2[127]*tmpFx[27] + tmpQN2[128]*tmpFx[43] + tmpQN2[129]*tmpFx[59] + tmpQN2[130]*tmpFx[75] + tmpQN2[131]*tmpFx[91] + tmpQN2[132]*tmpFx[107] + tmpQN2[133]*tmpFx[123] + tmpQN2[134]*tmpFx[139] + tmpQN2[135]*tmpFx[155] + tmpQN2[136]*tmpFx[171] + tmpQN2[137]*tmpFx[187] + tmpQN2[138]*tmpFx[203] + tmpQN2[139]*tmpFx[219];
tmpQN1[156] = + tmpQN2[126]*tmpFx[12] + tmpQN2[127]*tmpFx[28] + tmpQN2[128]*tmpFx[44] + tmpQN2[129]*tmpFx[60] + tmpQN2[130]*tmpFx[76] + tmpQN2[131]*tmpFx[92] + tmpQN2[132]*tmpFx[108] + tmpQN2[133]*tmpFx[124] + tmpQN2[134]*tmpFx[140] + tmpQN2[135]*tmpFx[156] + tmpQN2[136]*tmpFx[172] + tmpQN2[137]*tmpFx[188] + tmpQN2[138]*tmpFx[204] + tmpQN2[139]*tmpFx[220];
tmpQN1[157] = + tmpQN2[126]*tmpFx[13] + tmpQN2[127]*tmpFx[29] + tmpQN2[128]*tmpFx[45] + tmpQN2[129]*tmpFx[61] + tmpQN2[130]*tmpFx[77] + tmpQN2[131]*tmpFx[93] + tmpQN2[132]*tmpFx[109] + tmpQN2[133]*tmpFx[125] + tmpQN2[134]*tmpFx[141] + tmpQN2[135]*tmpFx[157] + tmpQN2[136]*tmpFx[173] + tmpQN2[137]*tmpFx[189] + tmpQN2[138]*tmpFx[205] + tmpQN2[139]*tmpFx[221];
tmpQN1[158] = + tmpQN2[126]*tmpFx[14] + tmpQN2[127]*tmpFx[30] + tmpQN2[128]*tmpFx[46] + tmpQN2[129]*tmpFx[62] + tmpQN2[130]*tmpFx[78] + tmpQN2[131]*tmpFx[94] + tmpQN2[132]*tmpFx[110] + tmpQN2[133]*tmpFx[126] + tmpQN2[134]*tmpFx[142] + tmpQN2[135]*tmpFx[158] + tmpQN2[136]*tmpFx[174] + tmpQN2[137]*tmpFx[190] + tmpQN2[138]*tmpFx[206] + tmpQN2[139]*tmpFx[222];
tmpQN1[159] = + tmpQN2[126]*tmpFx[15] + tmpQN2[127]*tmpFx[31] + tmpQN2[128]*tmpFx[47] + tmpQN2[129]*tmpFx[63] + tmpQN2[130]*tmpFx[79] + tmpQN2[131]*tmpFx[95] + tmpQN2[132]*tmpFx[111] + tmpQN2[133]*tmpFx[127] + tmpQN2[134]*tmpFx[143] + tmpQN2[135]*tmpFx[159] + tmpQN2[136]*tmpFx[175] + tmpQN2[137]*tmpFx[191] + tmpQN2[138]*tmpFx[207] + tmpQN2[139]*tmpFx[223];
tmpQN1[160] = + tmpQN2[140]*tmpFx[0] + tmpQN2[141]*tmpFx[16] + tmpQN2[142]*tmpFx[32] + tmpQN2[143]*tmpFx[48] + tmpQN2[144]*tmpFx[64] + tmpQN2[145]*tmpFx[80] + tmpQN2[146]*tmpFx[96] + tmpQN2[147]*tmpFx[112] + tmpQN2[148]*tmpFx[128] + tmpQN2[149]*tmpFx[144] + tmpQN2[150]*tmpFx[160] + tmpQN2[151]*tmpFx[176] + tmpQN2[152]*tmpFx[192] + tmpQN2[153]*tmpFx[208];
tmpQN1[161] = + tmpQN2[140]*tmpFx[1] + tmpQN2[141]*tmpFx[17] + tmpQN2[142]*tmpFx[33] + tmpQN2[143]*tmpFx[49] + tmpQN2[144]*tmpFx[65] + tmpQN2[145]*tmpFx[81] + tmpQN2[146]*tmpFx[97] + tmpQN2[147]*tmpFx[113] + tmpQN2[148]*tmpFx[129] + tmpQN2[149]*tmpFx[145] + tmpQN2[150]*tmpFx[161] + tmpQN2[151]*tmpFx[177] + tmpQN2[152]*tmpFx[193] + tmpQN2[153]*tmpFx[209];
tmpQN1[162] = + tmpQN2[140]*tmpFx[2] + tmpQN2[141]*tmpFx[18] + tmpQN2[142]*tmpFx[34] + tmpQN2[143]*tmpFx[50] + tmpQN2[144]*tmpFx[66] + tmpQN2[145]*tmpFx[82] + tmpQN2[146]*tmpFx[98] + tmpQN2[147]*tmpFx[114] + tmpQN2[148]*tmpFx[130] + tmpQN2[149]*tmpFx[146] + tmpQN2[150]*tmpFx[162] + tmpQN2[151]*tmpFx[178] + tmpQN2[152]*tmpFx[194] + tmpQN2[153]*tmpFx[210];
tmpQN1[163] = + tmpQN2[140]*tmpFx[3] + tmpQN2[141]*tmpFx[19] + tmpQN2[142]*tmpFx[35] + tmpQN2[143]*tmpFx[51] + tmpQN2[144]*tmpFx[67] + tmpQN2[145]*tmpFx[83] + tmpQN2[146]*tmpFx[99] + tmpQN2[147]*tmpFx[115] + tmpQN2[148]*tmpFx[131] + tmpQN2[149]*tmpFx[147] + tmpQN2[150]*tmpFx[163] + tmpQN2[151]*tmpFx[179] + tmpQN2[152]*tmpFx[195] + tmpQN2[153]*tmpFx[211];
tmpQN1[164] = + tmpQN2[140]*tmpFx[4] + tmpQN2[141]*tmpFx[20] + tmpQN2[142]*tmpFx[36] + tmpQN2[143]*tmpFx[52] + tmpQN2[144]*tmpFx[68] + tmpQN2[145]*tmpFx[84] + tmpQN2[146]*tmpFx[100] + tmpQN2[147]*tmpFx[116] + tmpQN2[148]*tmpFx[132] + tmpQN2[149]*tmpFx[148] + tmpQN2[150]*tmpFx[164] + tmpQN2[151]*tmpFx[180] + tmpQN2[152]*tmpFx[196] + tmpQN2[153]*tmpFx[212];
tmpQN1[165] = + tmpQN2[140]*tmpFx[5] + tmpQN2[141]*tmpFx[21] + tmpQN2[142]*tmpFx[37] + tmpQN2[143]*tmpFx[53] + tmpQN2[144]*tmpFx[69] + tmpQN2[145]*tmpFx[85] + tmpQN2[146]*tmpFx[101] + tmpQN2[147]*tmpFx[117] + tmpQN2[148]*tmpFx[133] + tmpQN2[149]*tmpFx[149] + tmpQN2[150]*tmpFx[165] + tmpQN2[151]*tmpFx[181] + tmpQN2[152]*tmpFx[197] + tmpQN2[153]*tmpFx[213];
tmpQN1[166] = + tmpQN2[140]*tmpFx[6] + tmpQN2[141]*tmpFx[22] + tmpQN2[142]*tmpFx[38] + tmpQN2[143]*tmpFx[54] + tmpQN2[144]*tmpFx[70] + tmpQN2[145]*tmpFx[86] + tmpQN2[146]*tmpFx[102] + tmpQN2[147]*tmpFx[118] + tmpQN2[148]*tmpFx[134] + tmpQN2[149]*tmpFx[150] + tmpQN2[150]*tmpFx[166] + tmpQN2[151]*tmpFx[182] + tmpQN2[152]*tmpFx[198] + tmpQN2[153]*tmpFx[214];
tmpQN1[167] = + tmpQN2[140]*tmpFx[7] + tmpQN2[141]*tmpFx[23] + tmpQN2[142]*tmpFx[39] + tmpQN2[143]*tmpFx[55] + tmpQN2[144]*tmpFx[71] + tmpQN2[145]*tmpFx[87] + tmpQN2[146]*tmpFx[103] + tmpQN2[147]*tmpFx[119] + tmpQN2[148]*tmpFx[135] + tmpQN2[149]*tmpFx[151] + tmpQN2[150]*tmpFx[167] + tmpQN2[151]*tmpFx[183] + tmpQN2[152]*tmpFx[199] + tmpQN2[153]*tmpFx[215];
tmpQN1[168] = + tmpQN2[140]*tmpFx[8] + tmpQN2[141]*tmpFx[24] + tmpQN2[142]*tmpFx[40] + tmpQN2[143]*tmpFx[56] + tmpQN2[144]*tmpFx[72] + tmpQN2[145]*tmpFx[88] + tmpQN2[146]*tmpFx[104] + tmpQN2[147]*tmpFx[120] + tmpQN2[148]*tmpFx[136] + tmpQN2[149]*tmpFx[152] + tmpQN2[150]*tmpFx[168] + tmpQN2[151]*tmpFx[184] + tmpQN2[152]*tmpFx[200] + tmpQN2[153]*tmpFx[216];
tmpQN1[169] = + tmpQN2[140]*tmpFx[9] + tmpQN2[141]*tmpFx[25] + tmpQN2[142]*tmpFx[41] + tmpQN2[143]*tmpFx[57] + tmpQN2[144]*tmpFx[73] + tmpQN2[145]*tmpFx[89] + tmpQN2[146]*tmpFx[105] + tmpQN2[147]*tmpFx[121] + tmpQN2[148]*tmpFx[137] + tmpQN2[149]*tmpFx[153] + tmpQN2[150]*tmpFx[169] + tmpQN2[151]*tmpFx[185] + tmpQN2[152]*tmpFx[201] + tmpQN2[153]*tmpFx[217];
tmpQN1[170] = + tmpQN2[140]*tmpFx[10] + tmpQN2[141]*tmpFx[26] + tmpQN2[142]*tmpFx[42] + tmpQN2[143]*tmpFx[58] + tmpQN2[144]*tmpFx[74] + tmpQN2[145]*tmpFx[90] + tmpQN2[146]*tmpFx[106] + tmpQN2[147]*tmpFx[122] + tmpQN2[148]*tmpFx[138] + tmpQN2[149]*tmpFx[154] + tmpQN2[150]*tmpFx[170] + tmpQN2[151]*tmpFx[186] + tmpQN2[152]*tmpFx[202] + tmpQN2[153]*tmpFx[218];
tmpQN1[171] = + tmpQN2[140]*tmpFx[11] + tmpQN2[141]*tmpFx[27] + tmpQN2[142]*tmpFx[43] + tmpQN2[143]*tmpFx[59] + tmpQN2[144]*tmpFx[75] + tmpQN2[145]*tmpFx[91] + tmpQN2[146]*tmpFx[107] + tmpQN2[147]*tmpFx[123] + tmpQN2[148]*tmpFx[139] + tmpQN2[149]*tmpFx[155] + tmpQN2[150]*tmpFx[171] + tmpQN2[151]*tmpFx[187] + tmpQN2[152]*tmpFx[203] + tmpQN2[153]*tmpFx[219];
tmpQN1[172] = + tmpQN2[140]*tmpFx[12] + tmpQN2[141]*tmpFx[28] + tmpQN2[142]*tmpFx[44] + tmpQN2[143]*tmpFx[60] + tmpQN2[144]*tmpFx[76] + tmpQN2[145]*tmpFx[92] + tmpQN2[146]*tmpFx[108] + tmpQN2[147]*tmpFx[124] + tmpQN2[148]*tmpFx[140] + tmpQN2[149]*tmpFx[156] + tmpQN2[150]*tmpFx[172] + tmpQN2[151]*tmpFx[188] + tmpQN2[152]*tmpFx[204] + tmpQN2[153]*tmpFx[220];
tmpQN1[173] = + tmpQN2[140]*tmpFx[13] + tmpQN2[141]*tmpFx[29] + tmpQN2[142]*tmpFx[45] + tmpQN2[143]*tmpFx[61] + tmpQN2[144]*tmpFx[77] + tmpQN2[145]*tmpFx[93] + tmpQN2[146]*tmpFx[109] + tmpQN2[147]*tmpFx[125] + tmpQN2[148]*tmpFx[141] + tmpQN2[149]*tmpFx[157] + tmpQN2[150]*tmpFx[173] + tmpQN2[151]*tmpFx[189] + tmpQN2[152]*tmpFx[205] + tmpQN2[153]*tmpFx[221];
tmpQN1[174] = + tmpQN2[140]*tmpFx[14] + tmpQN2[141]*tmpFx[30] + tmpQN2[142]*tmpFx[46] + tmpQN2[143]*tmpFx[62] + tmpQN2[144]*tmpFx[78] + tmpQN2[145]*tmpFx[94] + tmpQN2[146]*tmpFx[110] + tmpQN2[147]*tmpFx[126] + tmpQN2[148]*tmpFx[142] + tmpQN2[149]*tmpFx[158] + tmpQN2[150]*tmpFx[174] + tmpQN2[151]*tmpFx[190] + tmpQN2[152]*tmpFx[206] + tmpQN2[153]*tmpFx[222];
tmpQN1[175] = + tmpQN2[140]*tmpFx[15] + tmpQN2[141]*tmpFx[31] + tmpQN2[142]*tmpFx[47] + tmpQN2[143]*tmpFx[63] + tmpQN2[144]*tmpFx[79] + tmpQN2[145]*tmpFx[95] + tmpQN2[146]*tmpFx[111] + tmpQN2[147]*tmpFx[127] + tmpQN2[148]*tmpFx[143] + tmpQN2[149]*tmpFx[159] + tmpQN2[150]*tmpFx[175] + tmpQN2[151]*tmpFx[191] + tmpQN2[152]*tmpFx[207] + tmpQN2[153]*tmpFx[223];
tmpQN1[176] = + tmpQN2[154]*tmpFx[0] + tmpQN2[155]*tmpFx[16] + tmpQN2[156]*tmpFx[32] + tmpQN2[157]*tmpFx[48] + tmpQN2[158]*tmpFx[64] + tmpQN2[159]*tmpFx[80] + tmpQN2[160]*tmpFx[96] + tmpQN2[161]*tmpFx[112] + tmpQN2[162]*tmpFx[128] + tmpQN2[163]*tmpFx[144] + tmpQN2[164]*tmpFx[160] + tmpQN2[165]*tmpFx[176] + tmpQN2[166]*tmpFx[192] + tmpQN2[167]*tmpFx[208];
tmpQN1[177] = + tmpQN2[154]*tmpFx[1] + tmpQN2[155]*tmpFx[17] + tmpQN2[156]*tmpFx[33] + tmpQN2[157]*tmpFx[49] + tmpQN2[158]*tmpFx[65] + tmpQN2[159]*tmpFx[81] + tmpQN2[160]*tmpFx[97] + tmpQN2[161]*tmpFx[113] + tmpQN2[162]*tmpFx[129] + tmpQN2[163]*tmpFx[145] + tmpQN2[164]*tmpFx[161] + tmpQN2[165]*tmpFx[177] + tmpQN2[166]*tmpFx[193] + tmpQN2[167]*tmpFx[209];
tmpQN1[178] = + tmpQN2[154]*tmpFx[2] + tmpQN2[155]*tmpFx[18] + tmpQN2[156]*tmpFx[34] + tmpQN2[157]*tmpFx[50] + tmpQN2[158]*tmpFx[66] + tmpQN2[159]*tmpFx[82] + tmpQN2[160]*tmpFx[98] + tmpQN2[161]*tmpFx[114] + tmpQN2[162]*tmpFx[130] + tmpQN2[163]*tmpFx[146] + tmpQN2[164]*tmpFx[162] + tmpQN2[165]*tmpFx[178] + tmpQN2[166]*tmpFx[194] + tmpQN2[167]*tmpFx[210];
tmpQN1[179] = + tmpQN2[154]*tmpFx[3] + tmpQN2[155]*tmpFx[19] + tmpQN2[156]*tmpFx[35] + tmpQN2[157]*tmpFx[51] + tmpQN2[158]*tmpFx[67] + tmpQN2[159]*tmpFx[83] + tmpQN2[160]*tmpFx[99] + tmpQN2[161]*tmpFx[115] + tmpQN2[162]*tmpFx[131] + tmpQN2[163]*tmpFx[147] + tmpQN2[164]*tmpFx[163] + tmpQN2[165]*tmpFx[179] + tmpQN2[166]*tmpFx[195] + tmpQN2[167]*tmpFx[211];
tmpQN1[180] = + tmpQN2[154]*tmpFx[4] + tmpQN2[155]*tmpFx[20] + tmpQN2[156]*tmpFx[36] + tmpQN2[157]*tmpFx[52] + tmpQN2[158]*tmpFx[68] + tmpQN2[159]*tmpFx[84] + tmpQN2[160]*tmpFx[100] + tmpQN2[161]*tmpFx[116] + tmpQN2[162]*tmpFx[132] + tmpQN2[163]*tmpFx[148] + tmpQN2[164]*tmpFx[164] + tmpQN2[165]*tmpFx[180] + tmpQN2[166]*tmpFx[196] + tmpQN2[167]*tmpFx[212];
tmpQN1[181] = + tmpQN2[154]*tmpFx[5] + tmpQN2[155]*tmpFx[21] + tmpQN2[156]*tmpFx[37] + tmpQN2[157]*tmpFx[53] + tmpQN2[158]*tmpFx[69] + tmpQN2[159]*tmpFx[85] + tmpQN2[160]*tmpFx[101] + tmpQN2[161]*tmpFx[117] + tmpQN2[162]*tmpFx[133] + tmpQN2[163]*tmpFx[149] + tmpQN2[164]*tmpFx[165] + tmpQN2[165]*tmpFx[181] + tmpQN2[166]*tmpFx[197] + tmpQN2[167]*tmpFx[213];
tmpQN1[182] = + tmpQN2[154]*tmpFx[6] + tmpQN2[155]*tmpFx[22] + tmpQN2[156]*tmpFx[38] + tmpQN2[157]*tmpFx[54] + tmpQN2[158]*tmpFx[70] + tmpQN2[159]*tmpFx[86] + tmpQN2[160]*tmpFx[102] + tmpQN2[161]*tmpFx[118] + tmpQN2[162]*tmpFx[134] + tmpQN2[163]*tmpFx[150] + tmpQN2[164]*tmpFx[166] + tmpQN2[165]*tmpFx[182] + tmpQN2[166]*tmpFx[198] + tmpQN2[167]*tmpFx[214];
tmpQN1[183] = + tmpQN2[154]*tmpFx[7] + tmpQN2[155]*tmpFx[23] + tmpQN2[156]*tmpFx[39] + tmpQN2[157]*tmpFx[55] + tmpQN2[158]*tmpFx[71] + tmpQN2[159]*tmpFx[87] + tmpQN2[160]*tmpFx[103] + tmpQN2[161]*tmpFx[119] + tmpQN2[162]*tmpFx[135] + tmpQN2[163]*tmpFx[151] + tmpQN2[164]*tmpFx[167] + tmpQN2[165]*tmpFx[183] + tmpQN2[166]*tmpFx[199] + tmpQN2[167]*tmpFx[215];
tmpQN1[184] = + tmpQN2[154]*tmpFx[8] + tmpQN2[155]*tmpFx[24] + tmpQN2[156]*tmpFx[40] + tmpQN2[157]*tmpFx[56] + tmpQN2[158]*tmpFx[72] + tmpQN2[159]*tmpFx[88] + tmpQN2[160]*tmpFx[104] + tmpQN2[161]*tmpFx[120] + tmpQN2[162]*tmpFx[136] + tmpQN2[163]*tmpFx[152] + tmpQN2[164]*tmpFx[168] + tmpQN2[165]*tmpFx[184] + tmpQN2[166]*tmpFx[200] + tmpQN2[167]*tmpFx[216];
tmpQN1[185] = + tmpQN2[154]*tmpFx[9] + tmpQN2[155]*tmpFx[25] + tmpQN2[156]*tmpFx[41] + tmpQN2[157]*tmpFx[57] + tmpQN2[158]*tmpFx[73] + tmpQN2[159]*tmpFx[89] + tmpQN2[160]*tmpFx[105] + tmpQN2[161]*tmpFx[121] + tmpQN2[162]*tmpFx[137] + tmpQN2[163]*tmpFx[153] + tmpQN2[164]*tmpFx[169] + tmpQN2[165]*tmpFx[185] + tmpQN2[166]*tmpFx[201] + tmpQN2[167]*tmpFx[217];
tmpQN1[186] = + tmpQN2[154]*tmpFx[10] + tmpQN2[155]*tmpFx[26] + tmpQN2[156]*tmpFx[42] + tmpQN2[157]*tmpFx[58] + tmpQN2[158]*tmpFx[74] + tmpQN2[159]*tmpFx[90] + tmpQN2[160]*tmpFx[106] + tmpQN2[161]*tmpFx[122] + tmpQN2[162]*tmpFx[138] + tmpQN2[163]*tmpFx[154] + tmpQN2[164]*tmpFx[170] + tmpQN2[165]*tmpFx[186] + tmpQN2[166]*tmpFx[202] + tmpQN2[167]*tmpFx[218];
tmpQN1[187] = + tmpQN2[154]*tmpFx[11] + tmpQN2[155]*tmpFx[27] + tmpQN2[156]*tmpFx[43] + tmpQN2[157]*tmpFx[59] + tmpQN2[158]*tmpFx[75] + tmpQN2[159]*tmpFx[91] + tmpQN2[160]*tmpFx[107] + tmpQN2[161]*tmpFx[123] + tmpQN2[162]*tmpFx[139] + tmpQN2[163]*tmpFx[155] + tmpQN2[164]*tmpFx[171] + tmpQN2[165]*tmpFx[187] + tmpQN2[166]*tmpFx[203] + tmpQN2[167]*tmpFx[219];
tmpQN1[188] = + tmpQN2[154]*tmpFx[12] + tmpQN2[155]*tmpFx[28] + tmpQN2[156]*tmpFx[44] + tmpQN2[157]*tmpFx[60] + tmpQN2[158]*tmpFx[76] + tmpQN2[159]*tmpFx[92] + tmpQN2[160]*tmpFx[108] + tmpQN2[161]*tmpFx[124] + tmpQN2[162]*tmpFx[140] + tmpQN2[163]*tmpFx[156] + tmpQN2[164]*tmpFx[172] + tmpQN2[165]*tmpFx[188] + tmpQN2[166]*tmpFx[204] + tmpQN2[167]*tmpFx[220];
tmpQN1[189] = + tmpQN2[154]*tmpFx[13] + tmpQN2[155]*tmpFx[29] + tmpQN2[156]*tmpFx[45] + tmpQN2[157]*tmpFx[61] + tmpQN2[158]*tmpFx[77] + tmpQN2[159]*tmpFx[93] + tmpQN2[160]*tmpFx[109] + tmpQN2[161]*tmpFx[125] + tmpQN2[162]*tmpFx[141] + tmpQN2[163]*tmpFx[157] + tmpQN2[164]*tmpFx[173] + tmpQN2[165]*tmpFx[189] + tmpQN2[166]*tmpFx[205] + tmpQN2[167]*tmpFx[221];
tmpQN1[190] = + tmpQN2[154]*tmpFx[14] + tmpQN2[155]*tmpFx[30] + tmpQN2[156]*tmpFx[46] + tmpQN2[157]*tmpFx[62] + tmpQN2[158]*tmpFx[78] + tmpQN2[159]*tmpFx[94] + tmpQN2[160]*tmpFx[110] + tmpQN2[161]*tmpFx[126] + tmpQN2[162]*tmpFx[142] + tmpQN2[163]*tmpFx[158] + tmpQN2[164]*tmpFx[174] + tmpQN2[165]*tmpFx[190] + tmpQN2[166]*tmpFx[206] + tmpQN2[167]*tmpFx[222];
tmpQN1[191] = + tmpQN2[154]*tmpFx[15] + tmpQN2[155]*tmpFx[31] + tmpQN2[156]*tmpFx[47] + tmpQN2[157]*tmpFx[63] + tmpQN2[158]*tmpFx[79] + tmpQN2[159]*tmpFx[95] + tmpQN2[160]*tmpFx[111] + tmpQN2[161]*tmpFx[127] + tmpQN2[162]*tmpFx[143] + tmpQN2[163]*tmpFx[159] + tmpQN2[164]*tmpFx[175] + tmpQN2[165]*tmpFx[191] + tmpQN2[166]*tmpFx[207] + tmpQN2[167]*tmpFx[223];
tmpQN1[192] = + tmpQN2[168]*tmpFx[0] + tmpQN2[169]*tmpFx[16] + tmpQN2[170]*tmpFx[32] + tmpQN2[171]*tmpFx[48] + tmpQN2[172]*tmpFx[64] + tmpQN2[173]*tmpFx[80] + tmpQN2[174]*tmpFx[96] + tmpQN2[175]*tmpFx[112] + tmpQN2[176]*tmpFx[128] + tmpQN2[177]*tmpFx[144] + tmpQN2[178]*tmpFx[160] + tmpQN2[179]*tmpFx[176] + tmpQN2[180]*tmpFx[192] + tmpQN2[181]*tmpFx[208];
tmpQN1[193] = + tmpQN2[168]*tmpFx[1] + tmpQN2[169]*tmpFx[17] + tmpQN2[170]*tmpFx[33] + tmpQN2[171]*tmpFx[49] + tmpQN2[172]*tmpFx[65] + tmpQN2[173]*tmpFx[81] + tmpQN2[174]*tmpFx[97] + tmpQN2[175]*tmpFx[113] + tmpQN2[176]*tmpFx[129] + tmpQN2[177]*tmpFx[145] + tmpQN2[178]*tmpFx[161] + tmpQN2[179]*tmpFx[177] + tmpQN2[180]*tmpFx[193] + tmpQN2[181]*tmpFx[209];
tmpQN1[194] = + tmpQN2[168]*tmpFx[2] + tmpQN2[169]*tmpFx[18] + tmpQN2[170]*tmpFx[34] + tmpQN2[171]*tmpFx[50] + tmpQN2[172]*tmpFx[66] + tmpQN2[173]*tmpFx[82] + tmpQN2[174]*tmpFx[98] + tmpQN2[175]*tmpFx[114] + tmpQN2[176]*tmpFx[130] + tmpQN2[177]*tmpFx[146] + tmpQN2[178]*tmpFx[162] + tmpQN2[179]*tmpFx[178] + tmpQN2[180]*tmpFx[194] + tmpQN2[181]*tmpFx[210];
tmpQN1[195] = + tmpQN2[168]*tmpFx[3] + tmpQN2[169]*tmpFx[19] + tmpQN2[170]*tmpFx[35] + tmpQN2[171]*tmpFx[51] + tmpQN2[172]*tmpFx[67] + tmpQN2[173]*tmpFx[83] + tmpQN2[174]*tmpFx[99] + tmpQN2[175]*tmpFx[115] + tmpQN2[176]*tmpFx[131] + tmpQN2[177]*tmpFx[147] + tmpQN2[178]*tmpFx[163] + tmpQN2[179]*tmpFx[179] + tmpQN2[180]*tmpFx[195] + tmpQN2[181]*tmpFx[211];
tmpQN1[196] = + tmpQN2[168]*tmpFx[4] + tmpQN2[169]*tmpFx[20] + tmpQN2[170]*tmpFx[36] + tmpQN2[171]*tmpFx[52] + tmpQN2[172]*tmpFx[68] + tmpQN2[173]*tmpFx[84] + tmpQN2[174]*tmpFx[100] + tmpQN2[175]*tmpFx[116] + tmpQN2[176]*tmpFx[132] + tmpQN2[177]*tmpFx[148] + tmpQN2[178]*tmpFx[164] + tmpQN2[179]*tmpFx[180] + tmpQN2[180]*tmpFx[196] + tmpQN2[181]*tmpFx[212];
tmpQN1[197] = + tmpQN2[168]*tmpFx[5] + tmpQN2[169]*tmpFx[21] + tmpQN2[170]*tmpFx[37] + tmpQN2[171]*tmpFx[53] + tmpQN2[172]*tmpFx[69] + tmpQN2[173]*tmpFx[85] + tmpQN2[174]*tmpFx[101] + tmpQN2[175]*tmpFx[117] + tmpQN2[176]*tmpFx[133] + tmpQN2[177]*tmpFx[149] + tmpQN2[178]*tmpFx[165] + tmpQN2[179]*tmpFx[181] + tmpQN2[180]*tmpFx[197] + tmpQN2[181]*tmpFx[213];
tmpQN1[198] = + tmpQN2[168]*tmpFx[6] + tmpQN2[169]*tmpFx[22] + tmpQN2[170]*tmpFx[38] + tmpQN2[171]*tmpFx[54] + tmpQN2[172]*tmpFx[70] + tmpQN2[173]*tmpFx[86] + tmpQN2[174]*tmpFx[102] + tmpQN2[175]*tmpFx[118] + tmpQN2[176]*tmpFx[134] + tmpQN2[177]*tmpFx[150] + tmpQN2[178]*tmpFx[166] + tmpQN2[179]*tmpFx[182] + tmpQN2[180]*tmpFx[198] + tmpQN2[181]*tmpFx[214];
tmpQN1[199] = + tmpQN2[168]*tmpFx[7] + tmpQN2[169]*tmpFx[23] + tmpQN2[170]*tmpFx[39] + tmpQN2[171]*tmpFx[55] + tmpQN2[172]*tmpFx[71] + tmpQN2[173]*tmpFx[87] + tmpQN2[174]*tmpFx[103] + tmpQN2[175]*tmpFx[119] + tmpQN2[176]*tmpFx[135] + tmpQN2[177]*tmpFx[151] + tmpQN2[178]*tmpFx[167] + tmpQN2[179]*tmpFx[183] + tmpQN2[180]*tmpFx[199] + tmpQN2[181]*tmpFx[215];
tmpQN1[200] = + tmpQN2[168]*tmpFx[8] + tmpQN2[169]*tmpFx[24] + tmpQN2[170]*tmpFx[40] + tmpQN2[171]*tmpFx[56] + tmpQN2[172]*tmpFx[72] + tmpQN2[173]*tmpFx[88] + tmpQN2[174]*tmpFx[104] + tmpQN2[175]*tmpFx[120] + tmpQN2[176]*tmpFx[136] + tmpQN2[177]*tmpFx[152] + tmpQN2[178]*tmpFx[168] + tmpQN2[179]*tmpFx[184] + tmpQN2[180]*tmpFx[200] + tmpQN2[181]*tmpFx[216];
tmpQN1[201] = + tmpQN2[168]*tmpFx[9] + tmpQN2[169]*tmpFx[25] + tmpQN2[170]*tmpFx[41] + tmpQN2[171]*tmpFx[57] + tmpQN2[172]*tmpFx[73] + tmpQN2[173]*tmpFx[89] + tmpQN2[174]*tmpFx[105] + tmpQN2[175]*tmpFx[121] + tmpQN2[176]*tmpFx[137] + tmpQN2[177]*tmpFx[153] + tmpQN2[178]*tmpFx[169] + tmpQN2[179]*tmpFx[185] + tmpQN2[180]*tmpFx[201] + tmpQN2[181]*tmpFx[217];
tmpQN1[202] = + tmpQN2[168]*tmpFx[10] + tmpQN2[169]*tmpFx[26] + tmpQN2[170]*tmpFx[42] + tmpQN2[171]*tmpFx[58] + tmpQN2[172]*tmpFx[74] + tmpQN2[173]*tmpFx[90] + tmpQN2[174]*tmpFx[106] + tmpQN2[175]*tmpFx[122] + tmpQN2[176]*tmpFx[138] + tmpQN2[177]*tmpFx[154] + tmpQN2[178]*tmpFx[170] + tmpQN2[179]*tmpFx[186] + tmpQN2[180]*tmpFx[202] + tmpQN2[181]*tmpFx[218];
tmpQN1[203] = + tmpQN2[168]*tmpFx[11] + tmpQN2[169]*tmpFx[27] + tmpQN2[170]*tmpFx[43] + tmpQN2[171]*tmpFx[59] + tmpQN2[172]*tmpFx[75] + tmpQN2[173]*tmpFx[91] + tmpQN2[174]*tmpFx[107] + tmpQN2[175]*tmpFx[123] + tmpQN2[176]*tmpFx[139] + tmpQN2[177]*tmpFx[155] + tmpQN2[178]*tmpFx[171] + tmpQN2[179]*tmpFx[187] + tmpQN2[180]*tmpFx[203] + tmpQN2[181]*tmpFx[219];
tmpQN1[204] = + tmpQN2[168]*tmpFx[12] + tmpQN2[169]*tmpFx[28] + tmpQN2[170]*tmpFx[44] + tmpQN2[171]*tmpFx[60] + tmpQN2[172]*tmpFx[76] + tmpQN2[173]*tmpFx[92] + tmpQN2[174]*tmpFx[108] + tmpQN2[175]*tmpFx[124] + tmpQN2[176]*tmpFx[140] + tmpQN2[177]*tmpFx[156] + tmpQN2[178]*tmpFx[172] + tmpQN2[179]*tmpFx[188] + tmpQN2[180]*tmpFx[204] + tmpQN2[181]*tmpFx[220];
tmpQN1[205] = + tmpQN2[168]*tmpFx[13] + tmpQN2[169]*tmpFx[29] + tmpQN2[170]*tmpFx[45] + tmpQN2[171]*tmpFx[61] + tmpQN2[172]*tmpFx[77] + tmpQN2[173]*tmpFx[93] + tmpQN2[174]*tmpFx[109] + tmpQN2[175]*tmpFx[125] + tmpQN2[176]*tmpFx[141] + tmpQN2[177]*tmpFx[157] + tmpQN2[178]*tmpFx[173] + tmpQN2[179]*tmpFx[189] + tmpQN2[180]*tmpFx[205] + tmpQN2[181]*tmpFx[221];
tmpQN1[206] = + tmpQN2[168]*tmpFx[14] + tmpQN2[169]*tmpFx[30] + tmpQN2[170]*tmpFx[46] + tmpQN2[171]*tmpFx[62] + tmpQN2[172]*tmpFx[78] + tmpQN2[173]*tmpFx[94] + tmpQN2[174]*tmpFx[110] + tmpQN2[175]*tmpFx[126] + tmpQN2[176]*tmpFx[142] + tmpQN2[177]*tmpFx[158] + tmpQN2[178]*tmpFx[174] + tmpQN2[179]*tmpFx[190] + tmpQN2[180]*tmpFx[206] + tmpQN2[181]*tmpFx[222];
tmpQN1[207] = + tmpQN2[168]*tmpFx[15] + tmpQN2[169]*tmpFx[31] + tmpQN2[170]*tmpFx[47] + tmpQN2[171]*tmpFx[63] + tmpQN2[172]*tmpFx[79] + tmpQN2[173]*tmpFx[95] + tmpQN2[174]*tmpFx[111] + tmpQN2[175]*tmpFx[127] + tmpQN2[176]*tmpFx[143] + tmpQN2[177]*tmpFx[159] + tmpQN2[178]*tmpFx[175] + tmpQN2[179]*tmpFx[191] + tmpQN2[180]*tmpFx[207] + tmpQN2[181]*tmpFx[223];
tmpQN1[208] = + tmpQN2[182]*tmpFx[0] + tmpQN2[183]*tmpFx[16] + tmpQN2[184]*tmpFx[32] + tmpQN2[185]*tmpFx[48] + tmpQN2[186]*tmpFx[64] + tmpQN2[187]*tmpFx[80] + tmpQN2[188]*tmpFx[96] + tmpQN2[189]*tmpFx[112] + tmpQN2[190]*tmpFx[128] + tmpQN2[191]*tmpFx[144] + tmpQN2[192]*tmpFx[160] + tmpQN2[193]*tmpFx[176] + tmpQN2[194]*tmpFx[192] + tmpQN2[195]*tmpFx[208];
tmpQN1[209] = + tmpQN2[182]*tmpFx[1] + tmpQN2[183]*tmpFx[17] + tmpQN2[184]*tmpFx[33] + tmpQN2[185]*tmpFx[49] + tmpQN2[186]*tmpFx[65] + tmpQN2[187]*tmpFx[81] + tmpQN2[188]*tmpFx[97] + tmpQN2[189]*tmpFx[113] + tmpQN2[190]*tmpFx[129] + tmpQN2[191]*tmpFx[145] + tmpQN2[192]*tmpFx[161] + tmpQN2[193]*tmpFx[177] + tmpQN2[194]*tmpFx[193] + tmpQN2[195]*tmpFx[209];
tmpQN1[210] = + tmpQN2[182]*tmpFx[2] + tmpQN2[183]*tmpFx[18] + tmpQN2[184]*tmpFx[34] + tmpQN2[185]*tmpFx[50] + tmpQN2[186]*tmpFx[66] + tmpQN2[187]*tmpFx[82] + tmpQN2[188]*tmpFx[98] + tmpQN2[189]*tmpFx[114] + tmpQN2[190]*tmpFx[130] + tmpQN2[191]*tmpFx[146] + tmpQN2[192]*tmpFx[162] + tmpQN2[193]*tmpFx[178] + tmpQN2[194]*tmpFx[194] + tmpQN2[195]*tmpFx[210];
tmpQN1[211] = + tmpQN2[182]*tmpFx[3] + tmpQN2[183]*tmpFx[19] + tmpQN2[184]*tmpFx[35] + tmpQN2[185]*tmpFx[51] + tmpQN2[186]*tmpFx[67] + tmpQN2[187]*tmpFx[83] + tmpQN2[188]*tmpFx[99] + tmpQN2[189]*tmpFx[115] + tmpQN2[190]*tmpFx[131] + tmpQN2[191]*tmpFx[147] + tmpQN2[192]*tmpFx[163] + tmpQN2[193]*tmpFx[179] + tmpQN2[194]*tmpFx[195] + tmpQN2[195]*tmpFx[211];
tmpQN1[212] = + tmpQN2[182]*tmpFx[4] + tmpQN2[183]*tmpFx[20] + tmpQN2[184]*tmpFx[36] + tmpQN2[185]*tmpFx[52] + tmpQN2[186]*tmpFx[68] + tmpQN2[187]*tmpFx[84] + tmpQN2[188]*tmpFx[100] + tmpQN2[189]*tmpFx[116] + tmpQN2[190]*tmpFx[132] + tmpQN2[191]*tmpFx[148] + tmpQN2[192]*tmpFx[164] + tmpQN2[193]*tmpFx[180] + tmpQN2[194]*tmpFx[196] + tmpQN2[195]*tmpFx[212];
tmpQN1[213] = + tmpQN2[182]*tmpFx[5] + tmpQN2[183]*tmpFx[21] + tmpQN2[184]*tmpFx[37] + tmpQN2[185]*tmpFx[53] + tmpQN2[186]*tmpFx[69] + tmpQN2[187]*tmpFx[85] + tmpQN2[188]*tmpFx[101] + tmpQN2[189]*tmpFx[117] + tmpQN2[190]*tmpFx[133] + tmpQN2[191]*tmpFx[149] + tmpQN2[192]*tmpFx[165] + tmpQN2[193]*tmpFx[181] + tmpQN2[194]*tmpFx[197] + tmpQN2[195]*tmpFx[213];
tmpQN1[214] = + tmpQN2[182]*tmpFx[6] + tmpQN2[183]*tmpFx[22] + tmpQN2[184]*tmpFx[38] + tmpQN2[185]*tmpFx[54] + tmpQN2[186]*tmpFx[70] + tmpQN2[187]*tmpFx[86] + tmpQN2[188]*tmpFx[102] + tmpQN2[189]*tmpFx[118] + tmpQN2[190]*tmpFx[134] + tmpQN2[191]*tmpFx[150] + tmpQN2[192]*tmpFx[166] + tmpQN2[193]*tmpFx[182] + tmpQN2[194]*tmpFx[198] + tmpQN2[195]*tmpFx[214];
tmpQN1[215] = + tmpQN2[182]*tmpFx[7] + tmpQN2[183]*tmpFx[23] + tmpQN2[184]*tmpFx[39] + tmpQN2[185]*tmpFx[55] + tmpQN2[186]*tmpFx[71] + tmpQN2[187]*tmpFx[87] + tmpQN2[188]*tmpFx[103] + tmpQN2[189]*tmpFx[119] + tmpQN2[190]*tmpFx[135] + tmpQN2[191]*tmpFx[151] + tmpQN2[192]*tmpFx[167] + tmpQN2[193]*tmpFx[183] + tmpQN2[194]*tmpFx[199] + tmpQN2[195]*tmpFx[215];
tmpQN1[216] = + tmpQN2[182]*tmpFx[8] + tmpQN2[183]*tmpFx[24] + tmpQN2[184]*tmpFx[40] + tmpQN2[185]*tmpFx[56] + tmpQN2[186]*tmpFx[72] + tmpQN2[187]*tmpFx[88] + tmpQN2[188]*tmpFx[104] + tmpQN2[189]*tmpFx[120] + tmpQN2[190]*tmpFx[136] + tmpQN2[191]*tmpFx[152] + tmpQN2[192]*tmpFx[168] + tmpQN2[193]*tmpFx[184] + tmpQN2[194]*tmpFx[200] + tmpQN2[195]*tmpFx[216];
tmpQN1[217] = + tmpQN2[182]*tmpFx[9] + tmpQN2[183]*tmpFx[25] + tmpQN2[184]*tmpFx[41] + tmpQN2[185]*tmpFx[57] + tmpQN2[186]*tmpFx[73] + tmpQN2[187]*tmpFx[89] + tmpQN2[188]*tmpFx[105] + tmpQN2[189]*tmpFx[121] + tmpQN2[190]*tmpFx[137] + tmpQN2[191]*tmpFx[153] + tmpQN2[192]*tmpFx[169] + tmpQN2[193]*tmpFx[185] + tmpQN2[194]*tmpFx[201] + tmpQN2[195]*tmpFx[217];
tmpQN1[218] = + tmpQN2[182]*tmpFx[10] + tmpQN2[183]*tmpFx[26] + tmpQN2[184]*tmpFx[42] + tmpQN2[185]*tmpFx[58] + tmpQN2[186]*tmpFx[74] + tmpQN2[187]*tmpFx[90] + tmpQN2[188]*tmpFx[106] + tmpQN2[189]*tmpFx[122] + tmpQN2[190]*tmpFx[138] + tmpQN2[191]*tmpFx[154] + tmpQN2[192]*tmpFx[170] + tmpQN2[193]*tmpFx[186] + tmpQN2[194]*tmpFx[202] + tmpQN2[195]*tmpFx[218];
tmpQN1[219] = + tmpQN2[182]*tmpFx[11] + tmpQN2[183]*tmpFx[27] + tmpQN2[184]*tmpFx[43] + tmpQN2[185]*tmpFx[59] + tmpQN2[186]*tmpFx[75] + tmpQN2[187]*tmpFx[91] + tmpQN2[188]*tmpFx[107] + tmpQN2[189]*tmpFx[123] + tmpQN2[190]*tmpFx[139] + tmpQN2[191]*tmpFx[155] + tmpQN2[192]*tmpFx[171] + tmpQN2[193]*tmpFx[187] + tmpQN2[194]*tmpFx[203] + tmpQN2[195]*tmpFx[219];
tmpQN1[220] = + tmpQN2[182]*tmpFx[12] + tmpQN2[183]*tmpFx[28] + tmpQN2[184]*tmpFx[44] + tmpQN2[185]*tmpFx[60] + tmpQN2[186]*tmpFx[76] + tmpQN2[187]*tmpFx[92] + tmpQN2[188]*tmpFx[108] + tmpQN2[189]*tmpFx[124] + tmpQN2[190]*tmpFx[140] + tmpQN2[191]*tmpFx[156] + tmpQN2[192]*tmpFx[172] + tmpQN2[193]*tmpFx[188] + tmpQN2[194]*tmpFx[204] + tmpQN2[195]*tmpFx[220];
tmpQN1[221] = + tmpQN2[182]*tmpFx[13] + tmpQN2[183]*tmpFx[29] + tmpQN2[184]*tmpFx[45] + tmpQN2[185]*tmpFx[61] + tmpQN2[186]*tmpFx[77] + tmpQN2[187]*tmpFx[93] + tmpQN2[188]*tmpFx[109] + tmpQN2[189]*tmpFx[125] + tmpQN2[190]*tmpFx[141] + tmpQN2[191]*tmpFx[157] + tmpQN2[192]*tmpFx[173] + tmpQN2[193]*tmpFx[189] + tmpQN2[194]*tmpFx[205] + tmpQN2[195]*tmpFx[221];
tmpQN1[222] = + tmpQN2[182]*tmpFx[14] + tmpQN2[183]*tmpFx[30] + tmpQN2[184]*tmpFx[46] + tmpQN2[185]*tmpFx[62] + tmpQN2[186]*tmpFx[78] + tmpQN2[187]*tmpFx[94] + tmpQN2[188]*tmpFx[110] + tmpQN2[189]*tmpFx[126] + tmpQN2[190]*tmpFx[142] + tmpQN2[191]*tmpFx[158] + tmpQN2[192]*tmpFx[174] + tmpQN2[193]*tmpFx[190] + tmpQN2[194]*tmpFx[206] + tmpQN2[195]*tmpFx[222];
tmpQN1[223] = + tmpQN2[182]*tmpFx[15] + tmpQN2[183]*tmpFx[31] + tmpQN2[184]*tmpFx[47] + tmpQN2[185]*tmpFx[63] + tmpQN2[186]*tmpFx[79] + tmpQN2[187]*tmpFx[95] + tmpQN2[188]*tmpFx[111] + tmpQN2[189]*tmpFx[127] + tmpQN2[190]*tmpFx[143] + tmpQN2[191]*tmpFx[159] + tmpQN2[192]*tmpFx[175] + tmpQN2[193]*tmpFx[191] + tmpQN2[194]*tmpFx[207] + tmpQN2[195]*tmpFx[223];
tmpQN1[224] = + tmpQN2[196]*tmpFx[0] + tmpQN2[197]*tmpFx[16] + tmpQN2[198]*tmpFx[32] + tmpQN2[199]*tmpFx[48] + tmpQN2[200]*tmpFx[64] + tmpQN2[201]*tmpFx[80] + tmpQN2[202]*tmpFx[96] + tmpQN2[203]*tmpFx[112] + tmpQN2[204]*tmpFx[128] + tmpQN2[205]*tmpFx[144] + tmpQN2[206]*tmpFx[160] + tmpQN2[207]*tmpFx[176] + tmpQN2[208]*tmpFx[192] + tmpQN2[209]*tmpFx[208];
tmpQN1[225] = + tmpQN2[196]*tmpFx[1] + tmpQN2[197]*tmpFx[17] + tmpQN2[198]*tmpFx[33] + tmpQN2[199]*tmpFx[49] + tmpQN2[200]*tmpFx[65] + tmpQN2[201]*tmpFx[81] + tmpQN2[202]*tmpFx[97] + tmpQN2[203]*tmpFx[113] + tmpQN2[204]*tmpFx[129] + tmpQN2[205]*tmpFx[145] + tmpQN2[206]*tmpFx[161] + tmpQN2[207]*tmpFx[177] + tmpQN2[208]*tmpFx[193] + tmpQN2[209]*tmpFx[209];
tmpQN1[226] = + tmpQN2[196]*tmpFx[2] + tmpQN2[197]*tmpFx[18] + tmpQN2[198]*tmpFx[34] + tmpQN2[199]*tmpFx[50] + tmpQN2[200]*tmpFx[66] + tmpQN2[201]*tmpFx[82] + tmpQN2[202]*tmpFx[98] + tmpQN2[203]*tmpFx[114] + tmpQN2[204]*tmpFx[130] + tmpQN2[205]*tmpFx[146] + tmpQN2[206]*tmpFx[162] + tmpQN2[207]*tmpFx[178] + tmpQN2[208]*tmpFx[194] + tmpQN2[209]*tmpFx[210];
tmpQN1[227] = + tmpQN2[196]*tmpFx[3] + tmpQN2[197]*tmpFx[19] + tmpQN2[198]*tmpFx[35] + tmpQN2[199]*tmpFx[51] + tmpQN2[200]*tmpFx[67] + tmpQN2[201]*tmpFx[83] + tmpQN2[202]*tmpFx[99] + tmpQN2[203]*tmpFx[115] + tmpQN2[204]*tmpFx[131] + tmpQN2[205]*tmpFx[147] + tmpQN2[206]*tmpFx[163] + tmpQN2[207]*tmpFx[179] + tmpQN2[208]*tmpFx[195] + tmpQN2[209]*tmpFx[211];
tmpQN1[228] = + tmpQN2[196]*tmpFx[4] + tmpQN2[197]*tmpFx[20] + tmpQN2[198]*tmpFx[36] + tmpQN2[199]*tmpFx[52] + tmpQN2[200]*tmpFx[68] + tmpQN2[201]*tmpFx[84] + tmpQN2[202]*tmpFx[100] + tmpQN2[203]*tmpFx[116] + tmpQN2[204]*tmpFx[132] + tmpQN2[205]*tmpFx[148] + tmpQN2[206]*tmpFx[164] + tmpQN2[207]*tmpFx[180] + tmpQN2[208]*tmpFx[196] + tmpQN2[209]*tmpFx[212];
tmpQN1[229] = + tmpQN2[196]*tmpFx[5] + tmpQN2[197]*tmpFx[21] + tmpQN2[198]*tmpFx[37] + tmpQN2[199]*tmpFx[53] + tmpQN2[200]*tmpFx[69] + tmpQN2[201]*tmpFx[85] + tmpQN2[202]*tmpFx[101] + tmpQN2[203]*tmpFx[117] + tmpQN2[204]*tmpFx[133] + tmpQN2[205]*tmpFx[149] + tmpQN2[206]*tmpFx[165] + tmpQN2[207]*tmpFx[181] + tmpQN2[208]*tmpFx[197] + tmpQN2[209]*tmpFx[213];
tmpQN1[230] = + tmpQN2[196]*tmpFx[6] + tmpQN2[197]*tmpFx[22] + tmpQN2[198]*tmpFx[38] + tmpQN2[199]*tmpFx[54] + tmpQN2[200]*tmpFx[70] + tmpQN2[201]*tmpFx[86] + tmpQN2[202]*tmpFx[102] + tmpQN2[203]*tmpFx[118] + tmpQN2[204]*tmpFx[134] + tmpQN2[205]*tmpFx[150] + tmpQN2[206]*tmpFx[166] + tmpQN2[207]*tmpFx[182] + tmpQN2[208]*tmpFx[198] + tmpQN2[209]*tmpFx[214];
tmpQN1[231] = + tmpQN2[196]*tmpFx[7] + tmpQN2[197]*tmpFx[23] + tmpQN2[198]*tmpFx[39] + tmpQN2[199]*tmpFx[55] + tmpQN2[200]*tmpFx[71] + tmpQN2[201]*tmpFx[87] + tmpQN2[202]*tmpFx[103] + tmpQN2[203]*tmpFx[119] + tmpQN2[204]*tmpFx[135] + tmpQN2[205]*tmpFx[151] + tmpQN2[206]*tmpFx[167] + tmpQN2[207]*tmpFx[183] + tmpQN2[208]*tmpFx[199] + tmpQN2[209]*tmpFx[215];
tmpQN1[232] = + tmpQN2[196]*tmpFx[8] + tmpQN2[197]*tmpFx[24] + tmpQN2[198]*tmpFx[40] + tmpQN2[199]*tmpFx[56] + tmpQN2[200]*tmpFx[72] + tmpQN2[201]*tmpFx[88] + tmpQN2[202]*tmpFx[104] + tmpQN2[203]*tmpFx[120] + tmpQN2[204]*tmpFx[136] + tmpQN2[205]*tmpFx[152] + tmpQN2[206]*tmpFx[168] + tmpQN2[207]*tmpFx[184] + tmpQN2[208]*tmpFx[200] + tmpQN2[209]*tmpFx[216];
tmpQN1[233] = + tmpQN2[196]*tmpFx[9] + tmpQN2[197]*tmpFx[25] + tmpQN2[198]*tmpFx[41] + tmpQN2[199]*tmpFx[57] + tmpQN2[200]*tmpFx[73] + tmpQN2[201]*tmpFx[89] + tmpQN2[202]*tmpFx[105] + tmpQN2[203]*tmpFx[121] + tmpQN2[204]*tmpFx[137] + tmpQN2[205]*tmpFx[153] + tmpQN2[206]*tmpFx[169] + tmpQN2[207]*tmpFx[185] + tmpQN2[208]*tmpFx[201] + tmpQN2[209]*tmpFx[217];
tmpQN1[234] = + tmpQN2[196]*tmpFx[10] + tmpQN2[197]*tmpFx[26] + tmpQN2[198]*tmpFx[42] + tmpQN2[199]*tmpFx[58] + tmpQN2[200]*tmpFx[74] + tmpQN2[201]*tmpFx[90] + tmpQN2[202]*tmpFx[106] + tmpQN2[203]*tmpFx[122] + tmpQN2[204]*tmpFx[138] + tmpQN2[205]*tmpFx[154] + tmpQN2[206]*tmpFx[170] + tmpQN2[207]*tmpFx[186] + tmpQN2[208]*tmpFx[202] + tmpQN2[209]*tmpFx[218];
tmpQN1[235] = + tmpQN2[196]*tmpFx[11] + tmpQN2[197]*tmpFx[27] + tmpQN2[198]*tmpFx[43] + tmpQN2[199]*tmpFx[59] + tmpQN2[200]*tmpFx[75] + tmpQN2[201]*tmpFx[91] + tmpQN2[202]*tmpFx[107] + tmpQN2[203]*tmpFx[123] + tmpQN2[204]*tmpFx[139] + tmpQN2[205]*tmpFx[155] + tmpQN2[206]*tmpFx[171] + tmpQN2[207]*tmpFx[187] + tmpQN2[208]*tmpFx[203] + tmpQN2[209]*tmpFx[219];
tmpQN1[236] = + tmpQN2[196]*tmpFx[12] + tmpQN2[197]*tmpFx[28] + tmpQN2[198]*tmpFx[44] + tmpQN2[199]*tmpFx[60] + tmpQN2[200]*tmpFx[76] + tmpQN2[201]*tmpFx[92] + tmpQN2[202]*tmpFx[108] + tmpQN2[203]*tmpFx[124] + tmpQN2[204]*tmpFx[140] + tmpQN2[205]*tmpFx[156] + tmpQN2[206]*tmpFx[172] + tmpQN2[207]*tmpFx[188] + tmpQN2[208]*tmpFx[204] + tmpQN2[209]*tmpFx[220];
tmpQN1[237] = + tmpQN2[196]*tmpFx[13] + tmpQN2[197]*tmpFx[29] + tmpQN2[198]*tmpFx[45] + tmpQN2[199]*tmpFx[61] + tmpQN2[200]*tmpFx[77] + tmpQN2[201]*tmpFx[93] + tmpQN2[202]*tmpFx[109] + tmpQN2[203]*tmpFx[125] + tmpQN2[204]*tmpFx[141] + tmpQN2[205]*tmpFx[157] + tmpQN2[206]*tmpFx[173] + tmpQN2[207]*tmpFx[189] + tmpQN2[208]*tmpFx[205] + tmpQN2[209]*tmpFx[221];
tmpQN1[238] = + tmpQN2[196]*tmpFx[14] + tmpQN2[197]*tmpFx[30] + tmpQN2[198]*tmpFx[46] + tmpQN2[199]*tmpFx[62] + tmpQN2[200]*tmpFx[78] + tmpQN2[201]*tmpFx[94] + tmpQN2[202]*tmpFx[110] + tmpQN2[203]*tmpFx[126] + tmpQN2[204]*tmpFx[142] + tmpQN2[205]*tmpFx[158] + tmpQN2[206]*tmpFx[174] + tmpQN2[207]*tmpFx[190] + tmpQN2[208]*tmpFx[206] + tmpQN2[209]*tmpFx[222];
tmpQN1[239] = + tmpQN2[196]*tmpFx[15] + tmpQN2[197]*tmpFx[31] + tmpQN2[198]*tmpFx[47] + tmpQN2[199]*tmpFx[63] + tmpQN2[200]*tmpFx[79] + tmpQN2[201]*tmpFx[95] + tmpQN2[202]*tmpFx[111] + tmpQN2[203]*tmpFx[127] + tmpQN2[204]*tmpFx[143] + tmpQN2[205]*tmpFx[159] + tmpQN2[206]*tmpFx[175] + tmpQN2[207]*tmpFx[191] + tmpQN2[208]*tmpFx[207] + tmpQN2[209]*tmpFx[223];
tmpQN1[240] = + tmpQN2[210]*tmpFx[0] + tmpQN2[211]*tmpFx[16] + tmpQN2[212]*tmpFx[32] + tmpQN2[213]*tmpFx[48] + tmpQN2[214]*tmpFx[64] + tmpQN2[215]*tmpFx[80] + tmpQN2[216]*tmpFx[96] + tmpQN2[217]*tmpFx[112] + tmpQN2[218]*tmpFx[128] + tmpQN2[219]*tmpFx[144] + tmpQN2[220]*tmpFx[160] + tmpQN2[221]*tmpFx[176] + tmpQN2[222]*tmpFx[192] + tmpQN2[223]*tmpFx[208];
tmpQN1[241] = + tmpQN2[210]*tmpFx[1] + tmpQN2[211]*tmpFx[17] + tmpQN2[212]*tmpFx[33] + tmpQN2[213]*tmpFx[49] + tmpQN2[214]*tmpFx[65] + tmpQN2[215]*tmpFx[81] + tmpQN2[216]*tmpFx[97] + tmpQN2[217]*tmpFx[113] + tmpQN2[218]*tmpFx[129] + tmpQN2[219]*tmpFx[145] + tmpQN2[220]*tmpFx[161] + tmpQN2[221]*tmpFx[177] + tmpQN2[222]*tmpFx[193] + tmpQN2[223]*tmpFx[209];
tmpQN1[242] = + tmpQN2[210]*tmpFx[2] + tmpQN2[211]*tmpFx[18] + tmpQN2[212]*tmpFx[34] + tmpQN2[213]*tmpFx[50] + tmpQN2[214]*tmpFx[66] + tmpQN2[215]*tmpFx[82] + tmpQN2[216]*tmpFx[98] + tmpQN2[217]*tmpFx[114] + tmpQN2[218]*tmpFx[130] + tmpQN2[219]*tmpFx[146] + tmpQN2[220]*tmpFx[162] + tmpQN2[221]*tmpFx[178] + tmpQN2[222]*tmpFx[194] + tmpQN2[223]*tmpFx[210];
tmpQN1[243] = + tmpQN2[210]*tmpFx[3] + tmpQN2[211]*tmpFx[19] + tmpQN2[212]*tmpFx[35] + tmpQN2[213]*tmpFx[51] + tmpQN2[214]*tmpFx[67] + tmpQN2[215]*tmpFx[83] + tmpQN2[216]*tmpFx[99] + tmpQN2[217]*tmpFx[115] + tmpQN2[218]*tmpFx[131] + tmpQN2[219]*tmpFx[147] + tmpQN2[220]*tmpFx[163] + tmpQN2[221]*tmpFx[179] + tmpQN2[222]*tmpFx[195] + tmpQN2[223]*tmpFx[211];
tmpQN1[244] = + tmpQN2[210]*tmpFx[4] + tmpQN2[211]*tmpFx[20] + tmpQN2[212]*tmpFx[36] + tmpQN2[213]*tmpFx[52] + tmpQN2[214]*tmpFx[68] + tmpQN2[215]*tmpFx[84] + tmpQN2[216]*tmpFx[100] + tmpQN2[217]*tmpFx[116] + tmpQN2[218]*tmpFx[132] + tmpQN2[219]*tmpFx[148] + tmpQN2[220]*tmpFx[164] + tmpQN2[221]*tmpFx[180] + tmpQN2[222]*tmpFx[196] + tmpQN2[223]*tmpFx[212];
tmpQN1[245] = + tmpQN2[210]*tmpFx[5] + tmpQN2[211]*tmpFx[21] + tmpQN2[212]*tmpFx[37] + tmpQN2[213]*tmpFx[53] + tmpQN2[214]*tmpFx[69] + tmpQN2[215]*tmpFx[85] + tmpQN2[216]*tmpFx[101] + tmpQN2[217]*tmpFx[117] + tmpQN2[218]*tmpFx[133] + tmpQN2[219]*tmpFx[149] + tmpQN2[220]*tmpFx[165] + tmpQN2[221]*tmpFx[181] + tmpQN2[222]*tmpFx[197] + tmpQN2[223]*tmpFx[213];
tmpQN1[246] = + tmpQN2[210]*tmpFx[6] + tmpQN2[211]*tmpFx[22] + tmpQN2[212]*tmpFx[38] + tmpQN2[213]*tmpFx[54] + tmpQN2[214]*tmpFx[70] + tmpQN2[215]*tmpFx[86] + tmpQN2[216]*tmpFx[102] + tmpQN2[217]*tmpFx[118] + tmpQN2[218]*tmpFx[134] + tmpQN2[219]*tmpFx[150] + tmpQN2[220]*tmpFx[166] + tmpQN2[221]*tmpFx[182] + tmpQN2[222]*tmpFx[198] + tmpQN2[223]*tmpFx[214];
tmpQN1[247] = + tmpQN2[210]*tmpFx[7] + tmpQN2[211]*tmpFx[23] + tmpQN2[212]*tmpFx[39] + tmpQN2[213]*tmpFx[55] + tmpQN2[214]*tmpFx[71] + tmpQN2[215]*tmpFx[87] + tmpQN2[216]*tmpFx[103] + tmpQN2[217]*tmpFx[119] + tmpQN2[218]*tmpFx[135] + tmpQN2[219]*tmpFx[151] + tmpQN2[220]*tmpFx[167] + tmpQN2[221]*tmpFx[183] + tmpQN2[222]*tmpFx[199] + tmpQN2[223]*tmpFx[215];
tmpQN1[248] = + tmpQN2[210]*tmpFx[8] + tmpQN2[211]*tmpFx[24] + tmpQN2[212]*tmpFx[40] + tmpQN2[213]*tmpFx[56] + tmpQN2[214]*tmpFx[72] + tmpQN2[215]*tmpFx[88] + tmpQN2[216]*tmpFx[104] + tmpQN2[217]*tmpFx[120] + tmpQN2[218]*tmpFx[136] + tmpQN2[219]*tmpFx[152] + tmpQN2[220]*tmpFx[168] + tmpQN2[221]*tmpFx[184] + tmpQN2[222]*tmpFx[200] + tmpQN2[223]*tmpFx[216];
tmpQN1[249] = + tmpQN2[210]*tmpFx[9] + tmpQN2[211]*tmpFx[25] + tmpQN2[212]*tmpFx[41] + tmpQN2[213]*tmpFx[57] + tmpQN2[214]*tmpFx[73] + tmpQN2[215]*tmpFx[89] + tmpQN2[216]*tmpFx[105] + tmpQN2[217]*tmpFx[121] + tmpQN2[218]*tmpFx[137] + tmpQN2[219]*tmpFx[153] + tmpQN2[220]*tmpFx[169] + tmpQN2[221]*tmpFx[185] + tmpQN2[222]*tmpFx[201] + tmpQN2[223]*tmpFx[217];
tmpQN1[250] = + tmpQN2[210]*tmpFx[10] + tmpQN2[211]*tmpFx[26] + tmpQN2[212]*tmpFx[42] + tmpQN2[213]*tmpFx[58] + tmpQN2[214]*tmpFx[74] + tmpQN2[215]*tmpFx[90] + tmpQN2[216]*tmpFx[106] + tmpQN2[217]*tmpFx[122] + tmpQN2[218]*tmpFx[138] + tmpQN2[219]*tmpFx[154] + tmpQN2[220]*tmpFx[170] + tmpQN2[221]*tmpFx[186] + tmpQN2[222]*tmpFx[202] + tmpQN2[223]*tmpFx[218];
tmpQN1[251] = + tmpQN2[210]*tmpFx[11] + tmpQN2[211]*tmpFx[27] + tmpQN2[212]*tmpFx[43] + tmpQN2[213]*tmpFx[59] + tmpQN2[214]*tmpFx[75] + tmpQN2[215]*tmpFx[91] + tmpQN2[216]*tmpFx[107] + tmpQN2[217]*tmpFx[123] + tmpQN2[218]*tmpFx[139] + tmpQN2[219]*tmpFx[155] + tmpQN2[220]*tmpFx[171] + tmpQN2[221]*tmpFx[187] + tmpQN2[222]*tmpFx[203] + tmpQN2[223]*tmpFx[219];
tmpQN1[252] = + tmpQN2[210]*tmpFx[12] + tmpQN2[211]*tmpFx[28] + tmpQN2[212]*tmpFx[44] + tmpQN2[213]*tmpFx[60] + tmpQN2[214]*tmpFx[76] + tmpQN2[215]*tmpFx[92] + tmpQN2[216]*tmpFx[108] + tmpQN2[217]*tmpFx[124] + tmpQN2[218]*tmpFx[140] + tmpQN2[219]*tmpFx[156] + tmpQN2[220]*tmpFx[172] + tmpQN2[221]*tmpFx[188] + tmpQN2[222]*tmpFx[204] + tmpQN2[223]*tmpFx[220];
tmpQN1[253] = + tmpQN2[210]*tmpFx[13] + tmpQN2[211]*tmpFx[29] + tmpQN2[212]*tmpFx[45] + tmpQN2[213]*tmpFx[61] + tmpQN2[214]*tmpFx[77] + tmpQN2[215]*tmpFx[93] + tmpQN2[216]*tmpFx[109] + tmpQN2[217]*tmpFx[125] + tmpQN2[218]*tmpFx[141] + tmpQN2[219]*tmpFx[157] + tmpQN2[220]*tmpFx[173] + tmpQN2[221]*tmpFx[189] + tmpQN2[222]*tmpFx[205] + tmpQN2[223]*tmpFx[221];
tmpQN1[254] = + tmpQN2[210]*tmpFx[14] + tmpQN2[211]*tmpFx[30] + tmpQN2[212]*tmpFx[46] + tmpQN2[213]*tmpFx[62] + tmpQN2[214]*tmpFx[78] + tmpQN2[215]*tmpFx[94] + tmpQN2[216]*tmpFx[110] + tmpQN2[217]*tmpFx[126] + tmpQN2[218]*tmpFx[142] + tmpQN2[219]*tmpFx[158] + tmpQN2[220]*tmpFx[174] + tmpQN2[221]*tmpFx[190] + tmpQN2[222]*tmpFx[206] + tmpQN2[223]*tmpFx[222];
tmpQN1[255] = + tmpQN2[210]*tmpFx[15] + tmpQN2[211]*tmpFx[31] + tmpQN2[212]*tmpFx[47] + tmpQN2[213]*tmpFx[63] + tmpQN2[214]*tmpFx[79] + tmpQN2[215]*tmpFx[95] + tmpQN2[216]*tmpFx[111] + tmpQN2[217]*tmpFx[127] + tmpQN2[218]*tmpFx[143] + tmpQN2[219]*tmpFx[159] + tmpQN2[220]*tmpFx[175] + tmpQN2[221]*tmpFx[191] + tmpQN2[222]*tmpFx[207] + tmpQN2[223]*tmpFx[223];
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 16];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 16 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 16 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 16 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 16 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 16 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 16 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 16 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[runObj * 16 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[runObj * 16 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[runObj * 16 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[runObj * 16 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[runObj * 16 + 12];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[runObj * 16 + 13];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[runObj * 16 + 14];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[runObj * 16 + 15];
nmpcWorkspace.objValueIn[16] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[17] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[runObj * 9];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[runObj * 9 + 1];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[runObj * 9 + 2];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[runObj * 9 + 3];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[runObj * 9 + 4];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[runObj * 9 + 5];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[runObj * 9 + 6];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[runObj * 9 + 7];
nmpcWorkspace.objValueIn[28] = nmpcVariables.od[runObj * 9 + 8];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 18] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 18 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 18 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 18 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 18 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 18 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 18 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 18 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 18 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 18 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 18 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 18 + 11] = nmpcWorkspace.objValueOut[11];
nmpcWorkspace.Dy[runObj * 18 + 12] = nmpcWorkspace.objValueOut[12];
nmpcWorkspace.Dy[runObj * 18 + 13] = nmpcWorkspace.objValueOut[13];
nmpcWorkspace.Dy[runObj * 18 + 14] = nmpcWorkspace.objValueOut[14];
nmpcWorkspace.Dy[runObj * 18 + 15] = nmpcWorkspace.objValueOut[15];
nmpcWorkspace.Dy[runObj * 18 + 16] = nmpcWorkspace.objValueOut[16];
nmpcWorkspace.Dy[runObj * 18 + 17] = nmpcWorkspace.objValueOut[17];

nmpc_setObjQ1Q2( &(nmpcWorkspace.objValueOut[ 18 ]), nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 256 ]), &(nmpcWorkspace.Q2[ runObj * 288 ]) );

nmpc_setObjR1R2( &(nmpcWorkspace.objValueOut[ 306 ]), nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 72 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[480];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[481];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[482];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[483];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[484];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[485];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[486];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[487];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[488];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[489];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[490];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[491];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[492];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[493];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[494];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[495];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[270];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[271];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[272];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[273];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[274];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[275];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[276];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[277];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.DyN[9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.DyN[10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.DyN[11] = nmpcWorkspace.objValueOut[11];
nmpcWorkspace.DyN[12] = nmpcWorkspace.objValueOut[12];
nmpcWorkspace.DyN[13] = nmpcWorkspace.objValueOut[13];

nmpc_setObjQN1QN2( &(nmpcWorkspace.objValueOut[ 14 ]), nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11] + Gx1[12]*dOld[12] + Gx1[13]*dOld[13] + Gx1[14]*dOld[14] + Gx1[15]*dOld[15];
dNew[1] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7] + Gx1[24]*dOld[8] + Gx1[25]*dOld[9] + Gx1[26]*dOld[10] + Gx1[27]*dOld[11] + Gx1[28]*dOld[12] + Gx1[29]*dOld[13] + Gx1[30]*dOld[14] + Gx1[31]*dOld[15];
dNew[2] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7] + Gx1[40]*dOld[8] + Gx1[41]*dOld[9] + Gx1[42]*dOld[10] + Gx1[43]*dOld[11] + Gx1[44]*dOld[12] + Gx1[45]*dOld[13] + Gx1[46]*dOld[14] + Gx1[47]*dOld[15];
dNew[3] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11] + Gx1[60]*dOld[12] + Gx1[61]*dOld[13] + Gx1[62]*dOld[14] + Gx1[63]*dOld[15];
dNew[4] += + Gx1[64]*dOld[0] + Gx1[65]*dOld[1] + Gx1[66]*dOld[2] + Gx1[67]*dOld[3] + Gx1[68]*dOld[4] + Gx1[69]*dOld[5] + Gx1[70]*dOld[6] + Gx1[71]*dOld[7] + Gx1[72]*dOld[8] + Gx1[73]*dOld[9] + Gx1[74]*dOld[10] + Gx1[75]*dOld[11] + Gx1[76]*dOld[12] + Gx1[77]*dOld[13] + Gx1[78]*dOld[14] + Gx1[79]*dOld[15];
dNew[5] += + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9] + Gx1[90]*dOld[10] + Gx1[91]*dOld[11] + Gx1[92]*dOld[12] + Gx1[93]*dOld[13] + Gx1[94]*dOld[14] + Gx1[95]*dOld[15];
dNew[6] += + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11] + Gx1[108]*dOld[12] + Gx1[109]*dOld[13] + Gx1[110]*dOld[14] + Gx1[111]*dOld[15];
dNew[7] += + Gx1[112]*dOld[0] + Gx1[113]*dOld[1] + Gx1[114]*dOld[2] + Gx1[115]*dOld[3] + Gx1[116]*dOld[4] + Gx1[117]*dOld[5] + Gx1[118]*dOld[6] + Gx1[119]*dOld[7] + Gx1[120]*dOld[8] + Gx1[121]*dOld[9] + Gx1[122]*dOld[10] + Gx1[123]*dOld[11] + Gx1[124]*dOld[12] + Gx1[125]*dOld[13] + Gx1[126]*dOld[14] + Gx1[127]*dOld[15];
dNew[8] += + Gx1[128]*dOld[0] + Gx1[129]*dOld[1] + Gx1[130]*dOld[2] + Gx1[131]*dOld[3] + Gx1[132]*dOld[4] + Gx1[133]*dOld[5] + Gx1[134]*dOld[6] + Gx1[135]*dOld[7] + Gx1[136]*dOld[8] + Gx1[137]*dOld[9] + Gx1[138]*dOld[10] + Gx1[139]*dOld[11] + Gx1[140]*dOld[12] + Gx1[141]*dOld[13] + Gx1[142]*dOld[14] + Gx1[143]*dOld[15];
dNew[9] += + Gx1[144]*dOld[0] + Gx1[145]*dOld[1] + Gx1[146]*dOld[2] + Gx1[147]*dOld[3] + Gx1[148]*dOld[4] + Gx1[149]*dOld[5] + Gx1[150]*dOld[6] + Gx1[151]*dOld[7] + Gx1[152]*dOld[8] + Gx1[153]*dOld[9] + Gx1[154]*dOld[10] + Gx1[155]*dOld[11] + Gx1[156]*dOld[12] + Gx1[157]*dOld[13] + Gx1[158]*dOld[14] + Gx1[159]*dOld[15];
dNew[10] += + Gx1[160]*dOld[0] + Gx1[161]*dOld[1] + Gx1[162]*dOld[2] + Gx1[163]*dOld[3] + Gx1[164]*dOld[4] + Gx1[165]*dOld[5] + Gx1[166]*dOld[6] + Gx1[167]*dOld[7] + Gx1[168]*dOld[8] + Gx1[169]*dOld[9] + Gx1[170]*dOld[10] + Gx1[171]*dOld[11] + Gx1[172]*dOld[12] + Gx1[173]*dOld[13] + Gx1[174]*dOld[14] + Gx1[175]*dOld[15];
dNew[11] += + Gx1[176]*dOld[0] + Gx1[177]*dOld[1] + Gx1[178]*dOld[2] + Gx1[179]*dOld[3] + Gx1[180]*dOld[4] + Gx1[181]*dOld[5] + Gx1[182]*dOld[6] + Gx1[183]*dOld[7] + Gx1[184]*dOld[8] + Gx1[185]*dOld[9] + Gx1[186]*dOld[10] + Gx1[187]*dOld[11] + Gx1[188]*dOld[12] + Gx1[189]*dOld[13] + Gx1[190]*dOld[14] + Gx1[191]*dOld[15];
dNew[12] += + Gx1[192]*dOld[0] + Gx1[193]*dOld[1] + Gx1[194]*dOld[2] + Gx1[195]*dOld[3] + Gx1[196]*dOld[4] + Gx1[197]*dOld[5] + Gx1[198]*dOld[6] + Gx1[199]*dOld[7] + Gx1[200]*dOld[8] + Gx1[201]*dOld[9] + Gx1[202]*dOld[10] + Gx1[203]*dOld[11] + Gx1[204]*dOld[12] + Gx1[205]*dOld[13] + Gx1[206]*dOld[14] + Gx1[207]*dOld[15];
dNew[13] += + Gx1[208]*dOld[0] + Gx1[209]*dOld[1] + Gx1[210]*dOld[2] + Gx1[211]*dOld[3] + Gx1[212]*dOld[4] + Gx1[213]*dOld[5] + Gx1[214]*dOld[6] + Gx1[215]*dOld[7] + Gx1[216]*dOld[8] + Gx1[217]*dOld[9] + Gx1[218]*dOld[10] + Gx1[219]*dOld[11] + Gx1[220]*dOld[12] + Gx1[221]*dOld[13] + Gx1[222]*dOld[14] + Gx1[223]*dOld[15];
dNew[14] += + Gx1[224]*dOld[0] + Gx1[225]*dOld[1] + Gx1[226]*dOld[2] + Gx1[227]*dOld[3] + Gx1[228]*dOld[4] + Gx1[229]*dOld[5] + Gx1[230]*dOld[6] + Gx1[231]*dOld[7] + Gx1[232]*dOld[8] + Gx1[233]*dOld[9] + Gx1[234]*dOld[10] + Gx1[235]*dOld[11] + Gx1[236]*dOld[12] + Gx1[237]*dOld[13] + Gx1[238]*dOld[14] + Gx1[239]*dOld[15];
dNew[15] += + Gx1[240]*dOld[0] + Gx1[241]*dOld[1] + Gx1[242]*dOld[2] + Gx1[243]*dOld[3] + Gx1[244]*dOld[4] + Gx1[245]*dOld[5] + Gx1[246]*dOld[6] + Gx1[247]*dOld[7] + Gx1[248]*dOld[8] + Gx1[249]*dOld[9] + Gx1[250]*dOld[10] + Gx1[251]*dOld[11] + Gx1[252]*dOld[12] + Gx1[253]*dOld[13] + Gx1[254]*dOld[14] + Gx1[255]*dOld[15];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 16; ++lRun1)
for (lRun2 = 0;lRun2 < 16; ++lRun2)
Gx2[(lRun1 * 16) + (lRun2)] = Gx1[(lRun1 * 16) + (lRun2)];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + Gx1[(lRun1 * 16) + (lRun3)]*Gx2[(lRun3 * 16) + (lRun2)];
}
Gx3[(lRun1 * 16) + (lRun2)] = t;
}
}
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44] + Gx1[12]*Gu1[48] + Gx1[13]*Gu1[52] + Gx1[14]*Gu1[56] + Gx1[15]*Gu1[60];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45] + Gx1[12]*Gu1[49] + Gx1[13]*Gu1[53] + Gx1[14]*Gu1[57] + Gx1[15]*Gu1[61];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46] + Gx1[12]*Gu1[50] + Gx1[13]*Gu1[54] + Gx1[14]*Gu1[58] + Gx1[15]*Gu1[62];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47] + Gx1[12]*Gu1[51] + Gx1[13]*Gu1[55] + Gx1[14]*Gu1[59] + Gx1[15]*Gu1[63];
Gu2[4] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[8] + Gx1[19]*Gu1[12] + Gx1[20]*Gu1[16] + Gx1[21]*Gu1[20] + Gx1[22]*Gu1[24] + Gx1[23]*Gu1[28] + Gx1[24]*Gu1[32] + Gx1[25]*Gu1[36] + Gx1[26]*Gu1[40] + Gx1[27]*Gu1[44] + Gx1[28]*Gu1[48] + Gx1[29]*Gu1[52] + Gx1[30]*Gu1[56] + Gx1[31]*Gu1[60];
Gu2[5] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[9] + Gx1[19]*Gu1[13] + Gx1[20]*Gu1[17] + Gx1[21]*Gu1[21] + Gx1[22]*Gu1[25] + Gx1[23]*Gu1[29] + Gx1[24]*Gu1[33] + Gx1[25]*Gu1[37] + Gx1[26]*Gu1[41] + Gx1[27]*Gu1[45] + Gx1[28]*Gu1[49] + Gx1[29]*Gu1[53] + Gx1[30]*Gu1[57] + Gx1[31]*Gu1[61];
Gu2[6] = + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[18]*Gu1[10] + Gx1[19]*Gu1[14] + Gx1[20]*Gu1[18] + Gx1[21]*Gu1[22] + Gx1[22]*Gu1[26] + Gx1[23]*Gu1[30] + Gx1[24]*Gu1[34] + Gx1[25]*Gu1[38] + Gx1[26]*Gu1[42] + Gx1[27]*Gu1[46] + Gx1[28]*Gu1[50] + Gx1[29]*Gu1[54] + Gx1[30]*Gu1[58] + Gx1[31]*Gu1[62];
Gu2[7] = + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[18]*Gu1[11] + Gx1[19]*Gu1[15] + Gx1[20]*Gu1[19] + Gx1[21]*Gu1[23] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[31] + Gx1[24]*Gu1[35] + Gx1[25]*Gu1[39] + Gx1[26]*Gu1[43] + Gx1[27]*Gu1[47] + Gx1[28]*Gu1[51] + Gx1[29]*Gu1[55] + Gx1[30]*Gu1[59] + Gx1[31]*Gu1[63];
Gu2[8] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[36]*Gu1[16] + Gx1[37]*Gu1[20] + Gx1[38]*Gu1[24] + Gx1[39]*Gu1[28] + Gx1[40]*Gu1[32] + Gx1[41]*Gu1[36] + Gx1[42]*Gu1[40] + Gx1[43]*Gu1[44] + Gx1[44]*Gu1[48] + Gx1[45]*Gu1[52] + Gx1[46]*Gu1[56] + Gx1[47]*Gu1[60];
Gu2[9] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[36]*Gu1[17] + Gx1[37]*Gu1[21] + Gx1[38]*Gu1[25] + Gx1[39]*Gu1[29] + Gx1[40]*Gu1[33] + Gx1[41]*Gu1[37] + Gx1[42]*Gu1[41] + Gx1[43]*Gu1[45] + Gx1[44]*Gu1[49] + Gx1[45]*Gu1[53] + Gx1[46]*Gu1[57] + Gx1[47]*Gu1[61];
Gu2[10] = + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[36]*Gu1[18] + Gx1[37]*Gu1[22] + Gx1[38]*Gu1[26] + Gx1[39]*Gu1[30] + Gx1[40]*Gu1[34] + Gx1[41]*Gu1[38] + Gx1[42]*Gu1[42] + Gx1[43]*Gu1[46] + Gx1[44]*Gu1[50] + Gx1[45]*Gu1[54] + Gx1[46]*Gu1[58] + Gx1[47]*Gu1[62];
Gu2[11] = + Gx1[32]*Gu1[3] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[36]*Gu1[19] + Gx1[37]*Gu1[23] + Gx1[38]*Gu1[27] + Gx1[39]*Gu1[31] + Gx1[40]*Gu1[35] + Gx1[41]*Gu1[39] + Gx1[42]*Gu1[43] + Gx1[43]*Gu1[47] + Gx1[44]*Gu1[51] + Gx1[45]*Gu1[55] + Gx1[46]*Gu1[59] + Gx1[47]*Gu1[63];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44] + Gx1[60]*Gu1[48] + Gx1[61]*Gu1[52] + Gx1[62]*Gu1[56] + Gx1[63]*Gu1[60];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45] + Gx1[60]*Gu1[49] + Gx1[61]*Gu1[53] + Gx1[62]*Gu1[57] + Gx1[63]*Gu1[61];
Gu2[14] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46] + Gx1[60]*Gu1[50] + Gx1[61]*Gu1[54] + Gx1[62]*Gu1[58] + Gx1[63]*Gu1[62];
Gu2[15] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47] + Gx1[60]*Gu1[51] + Gx1[61]*Gu1[55] + Gx1[62]*Gu1[59] + Gx1[63]*Gu1[63];
Gu2[16] = + Gx1[64]*Gu1[0] + Gx1[65]*Gu1[4] + Gx1[66]*Gu1[8] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[24] + Gx1[71]*Gu1[28] + Gx1[72]*Gu1[32] + Gx1[73]*Gu1[36] + Gx1[74]*Gu1[40] + Gx1[75]*Gu1[44] + Gx1[76]*Gu1[48] + Gx1[77]*Gu1[52] + Gx1[78]*Gu1[56] + Gx1[79]*Gu1[60];
Gu2[17] = + Gx1[64]*Gu1[1] + Gx1[65]*Gu1[5] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[21] + Gx1[70]*Gu1[25] + Gx1[71]*Gu1[29] + Gx1[72]*Gu1[33] + Gx1[73]*Gu1[37] + Gx1[74]*Gu1[41] + Gx1[75]*Gu1[45] + Gx1[76]*Gu1[49] + Gx1[77]*Gu1[53] + Gx1[78]*Gu1[57] + Gx1[79]*Gu1[61];
Gu2[18] = + Gx1[64]*Gu1[2] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[18] + Gx1[69]*Gu1[22] + Gx1[70]*Gu1[26] + Gx1[71]*Gu1[30] + Gx1[72]*Gu1[34] + Gx1[73]*Gu1[38] + Gx1[74]*Gu1[42] + Gx1[75]*Gu1[46] + Gx1[76]*Gu1[50] + Gx1[77]*Gu1[54] + Gx1[78]*Gu1[58] + Gx1[79]*Gu1[62];
Gu2[19] = + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[15] + Gx1[68]*Gu1[19] + Gx1[69]*Gu1[23] + Gx1[70]*Gu1[27] + Gx1[71]*Gu1[31] + Gx1[72]*Gu1[35] + Gx1[73]*Gu1[39] + Gx1[74]*Gu1[43] + Gx1[75]*Gu1[47] + Gx1[76]*Gu1[51] + Gx1[77]*Gu1[55] + Gx1[78]*Gu1[59] + Gx1[79]*Gu1[63];
Gu2[20] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[12] + Gx1[84]*Gu1[16] + Gx1[85]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[36] + Gx1[90]*Gu1[40] + Gx1[91]*Gu1[44] + Gx1[92]*Gu1[48] + Gx1[93]*Gu1[52] + Gx1[94]*Gu1[56] + Gx1[95]*Gu1[60];
Gu2[21] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[9] + Gx1[83]*Gu1[13] + Gx1[84]*Gu1[17] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[89]*Gu1[37] + Gx1[90]*Gu1[41] + Gx1[91]*Gu1[45] + Gx1[92]*Gu1[49] + Gx1[93]*Gu1[53] + Gx1[94]*Gu1[57] + Gx1[95]*Gu1[61];
Gu2[22] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[6] + Gx1[82]*Gu1[10] + Gx1[83]*Gu1[14] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[89]*Gu1[38] + Gx1[90]*Gu1[42] + Gx1[91]*Gu1[46] + Gx1[92]*Gu1[50] + Gx1[93]*Gu1[54] + Gx1[94]*Gu1[58] + Gx1[95]*Gu1[62];
Gu2[23] = + Gx1[80]*Gu1[3] + Gx1[81]*Gu1[7] + Gx1[82]*Gu1[11] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[89]*Gu1[39] + Gx1[90]*Gu1[43] + Gx1[91]*Gu1[47] + Gx1[92]*Gu1[51] + Gx1[93]*Gu1[55] + Gx1[94]*Gu1[59] + Gx1[95]*Gu1[63];
Gu2[24] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44] + Gx1[108]*Gu1[48] + Gx1[109]*Gu1[52] + Gx1[110]*Gu1[56] + Gx1[111]*Gu1[60];
Gu2[25] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45] + Gx1[108]*Gu1[49] + Gx1[109]*Gu1[53] + Gx1[110]*Gu1[57] + Gx1[111]*Gu1[61];
Gu2[26] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46] + Gx1[108]*Gu1[50] + Gx1[109]*Gu1[54] + Gx1[110]*Gu1[58] + Gx1[111]*Gu1[62];
Gu2[27] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47] + Gx1[108]*Gu1[51] + Gx1[109]*Gu1[55] + Gx1[110]*Gu1[59] + Gx1[111]*Gu1[63];
Gu2[28] = + Gx1[112]*Gu1[0] + Gx1[113]*Gu1[4] + Gx1[114]*Gu1[8] + Gx1[115]*Gu1[12] + Gx1[116]*Gu1[16] + Gx1[117]*Gu1[20] + Gx1[118]*Gu1[24] + Gx1[119]*Gu1[28] + Gx1[120]*Gu1[32] + Gx1[121]*Gu1[36] + Gx1[122]*Gu1[40] + Gx1[123]*Gu1[44] + Gx1[124]*Gu1[48] + Gx1[125]*Gu1[52] + Gx1[126]*Gu1[56] + Gx1[127]*Gu1[60];
Gu2[29] = + Gx1[112]*Gu1[1] + Gx1[113]*Gu1[5] + Gx1[114]*Gu1[9] + Gx1[115]*Gu1[13] + Gx1[116]*Gu1[17] + Gx1[117]*Gu1[21] + Gx1[118]*Gu1[25] + Gx1[119]*Gu1[29] + Gx1[120]*Gu1[33] + Gx1[121]*Gu1[37] + Gx1[122]*Gu1[41] + Gx1[123]*Gu1[45] + Gx1[124]*Gu1[49] + Gx1[125]*Gu1[53] + Gx1[126]*Gu1[57] + Gx1[127]*Gu1[61];
Gu2[30] = + Gx1[112]*Gu1[2] + Gx1[113]*Gu1[6] + Gx1[114]*Gu1[10] + Gx1[115]*Gu1[14] + Gx1[116]*Gu1[18] + Gx1[117]*Gu1[22] + Gx1[118]*Gu1[26] + Gx1[119]*Gu1[30] + Gx1[120]*Gu1[34] + Gx1[121]*Gu1[38] + Gx1[122]*Gu1[42] + Gx1[123]*Gu1[46] + Gx1[124]*Gu1[50] + Gx1[125]*Gu1[54] + Gx1[126]*Gu1[58] + Gx1[127]*Gu1[62];
Gu2[31] = + Gx1[112]*Gu1[3] + Gx1[113]*Gu1[7] + Gx1[114]*Gu1[11] + Gx1[115]*Gu1[15] + Gx1[116]*Gu1[19] + Gx1[117]*Gu1[23] + Gx1[118]*Gu1[27] + Gx1[119]*Gu1[31] + Gx1[120]*Gu1[35] + Gx1[121]*Gu1[39] + Gx1[122]*Gu1[43] + Gx1[123]*Gu1[47] + Gx1[124]*Gu1[51] + Gx1[125]*Gu1[55] + Gx1[126]*Gu1[59] + Gx1[127]*Gu1[63];
Gu2[32] = + Gx1[128]*Gu1[0] + Gx1[129]*Gu1[4] + Gx1[130]*Gu1[8] + Gx1[131]*Gu1[12] + Gx1[132]*Gu1[16] + Gx1[133]*Gu1[20] + Gx1[134]*Gu1[24] + Gx1[135]*Gu1[28] + Gx1[136]*Gu1[32] + Gx1[137]*Gu1[36] + Gx1[138]*Gu1[40] + Gx1[139]*Gu1[44] + Gx1[140]*Gu1[48] + Gx1[141]*Gu1[52] + Gx1[142]*Gu1[56] + Gx1[143]*Gu1[60];
Gu2[33] = + Gx1[128]*Gu1[1] + Gx1[129]*Gu1[5] + Gx1[130]*Gu1[9] + Gx1[131]*Gu1[13] + Gx1[132]*Gu1[17] + Gx1[133]*Gu1[21] + Gx1[134]*Gu1[25] + Gx1[135]*Gu1[29] + Gx1[136]*Gu1[33] + Gx1[137]*Gu1[37] + Gx1[138]*Gu1[41] + Gx1[139]*Gu1[45] + Gx1[140]*Gu1[49] + Gx1[141]*Gu1[53] + Gx1[142]*Gu1[57] + Gx1[143]*Gu1[61];
Gu2[34] = + Gx1[128]*Gu1[2] + Gx1[129]*Gu1[6] + Gx1[130]*Gu1[10] + Gx1[131]*Gu1[14] + Gx1[132]*Gu1[18] + Gx1[133]*Gu1[22] + Gx1[134]*Gu1[26] + Gx1[135]*Gu1[30] + Gx1[136]*Gu1[34] + Gx1[137]*Gu1[38] + Gx1[138]*Gu1[42] + Gx1[139]*Gu1[46] + Gx1[140]*Gu1[50] + Gx1[141]*Gu1[54] + Gx1[142]*Gu1[58] + Gx1[143]*Gu1[62];
Gu2[35] = + Gx1[128]*Gu1[3] + Gx1[129]*Gu1[7] + Gx1[130]*Gu1[11] + Gx1[131]*Gu1[15] + Gx1[132]*Gu1[19] + Gx1[133]*Gu1[23] + Gx1[134]*Gu1[27] + Gx1[135]*Gu1[31] + Gx1[136]*Gu1[35] + Gx1[137]*Gu1[39] + Gx1[138]*Gu1[43] + Gx1[139]*Gu1[47] + Gx1[140]*Gu1[51] + Gx1[141]*Gu1[55] + Gx1[142]*Gu1[59] + Gx1[143]*Gu1[63];
Gu2[36] = + Gx1[144]*Gu1[0] + Gx1[145]*Gu1[4] + Gx1[146]*Gu1[8] + Gx1[147]*Gu1[12] + Gx1[148]*Gu1[16] + Gx1[149]*Gu1[20] + Gx1[150]*Gu1[24] + Gx1[151]*Gu1[28] + Gx1[152]*Gu1[32] + Gx1[153]*Gu1[36] + Gx1[154]*Gu1[40] + Gx1[155]*Gu1[44] + Gx1[156]*Gu1[48] + Gx1[157]*Gu1[52] + Gx1[158]*Gu1[56] + Gx1[159]*Gu1[60];
Gu2[37] = + Gx1[144]*Gu1[1] + Gx1[145]*Gu1[5] + Gx1[146]*Gu1[9] + Gx1[147]*Gu1[13] + Gx1[148]*Gu1[17] + Gx1[149]*Gu1[21] + Gx1[150]*Gu1[25] + Gx1[151]*Gu1[29] + Gx1[152]*Gu1[33] + Gx1[153]*Gu1[37] + Gx1[154]*Gu1[41] + Gx1[155]*Gu1[45] + Gx1[156]*Gu1[49] + Gx1[157]*Gu1[53] + Gx1[158]*Gu1[57] + Gx1[159]*Gu1[61];
Gu2[38] = + Gx1[144]*Gu1[2] + Gx1[145]*Gu1[6] + Gx1[146]*Gu1[10] + Gx1[147]*Gu1[14] + Gx1[148]*Gu1[18] + Gx1[149]*Gu1[22] + Gx1[150]*Gu1[26] + Gx1[151]*Gu1[30] + Gx1[152]*Gu1[34] + Gx1[153]*Gu1[38] + Gx1[154]*Gu1[42] + Gx1[155]*Gu1[46] + Gx1[156]*Gu1[50] + Gx1[157]*Gu1[54] + Gx1[158]*Gu1[58] + Gx1[159]*Gu1[62];
Gu2[39] = + Gx1[144]*Gu1[3] + Gx1[145]*Gu1[7] + Gx1[146]*Gu1[11] + Gx1[147]*Gu1[15] + Gx1[148]*Gu1[19] + Gx1[149]*Gu1[23] + Gx1[150]*Gu1[27] + Gx1[151]*Gu1[31] + Gx1[152]*Gu1[35] + Gx1[153]*Gu1[39] + Gx1[154]*Gu1[43] + Gx1[155]*Gu1[47] + Gx1[156]*Gu1[51] + Gx1[157]*Gu1[55] + Gx1[158]*Gu1[59] + Gx1[159]*Gu1[63];
Gu2[40] = + Gx1[160]*Gu1[0] + Gx1[161]*Gu1[4] + Gx1[162]*Gu1[8] + Gx1[163]*Gu1[12] + Gx1[164]*Gu1[16] + Gx1[165]*Gu1[20] + Gx1[166]*Gu1[24] + Gx1[167]*Gu1[28] + Gx1[168]*Gu1[32] + Gx1[169]*Gu1[36] + Gx1[170]*Gu1[40] + Gx1[171]*Gu1[44] + Gx1[172]*Gu1[48] + Gx1[173]*Gu1[52] + Gx1[174]*Gu1[56] + Gx1[175]*Gu1[60];
Gu2[41] = + Gx1[160]*Gu1[1] + Gx1[161]*Gu1[5] + Gx1[162]*Gu1[9] + Gx1[163]*Gu1[13] + Gx1[164]*Gu1[17] + Gx1[165]*Gu1[21] + Gx1[166]*Gu1[25] + Gx1[167]*Gu1[29] + Gx1[168]*Gu1[33] + Gx1[169]*Gu1[37] + Gx1[170]*Gu1[41] + Gx1[171]*Gu1[45] + Gx1[172]*Gu1[49] + Gx1[173]*Gu1[53] + Gx1[174]*Gu1[57] + Gx1[175]*Gu1[61];
Gu2[42] = + Gx1[160]*Gu1[2] + Gx1[161]*Gu1[6] + Gx1[162]*Gu1[10] + Gx1[163]*Gu1[14] + Gx1[164]*Gu1[18] + Gx1[165]*Gu1[22] + Gx1[166]*Gu1[26] + Gx1[167]*Gu1[30] + Gx1[168]*Gu1[34] + Gx1[169]*Gu1[38] + Gx1[170]*Gu1[42] + Gx1[171]*Gu1[46] + Gx1[172]*Gu1[50] + Gx1[173]*Gu1[54] + Gx1[174]*Gu1[58] + Gx1[175]*Gu1[62];
Gu2[43] = + Gx1[160]*Gu1[3] + Gx1[161]*Gu1[7] + Gx1[162]*Gu1[11] + Gx1[163]*Gu1[15] + Gx1[164]*Gu1[19] + Gx1[165]*Gu1[23] + Gx1[166]*Gu1[27] + Gx1[167]*Gu1[31] + Gx1[168]*Gu1[35] + Gx1[169]*Gu1[39] + Gx1[170]*Gu1[43] + Gx1[171]*Gu1[47] + Gx1[172]*Gu1[51] + Gx1[173]*Gu1[55] + Gx1[174]*Gu1[59] + Gx1[175]*Gu1[63];
Gu2[44] = + Gx1[176]*Gu1[0] + Gx1[177]*Gu1[4] + Gx1[178]*Gu1[8] + Gx1[179]*Gu1[12] + Gx1[180]*Gu1[16] + Gx1[181]*Gu1[20] + Gx1[182]*Gu1[24] + Gx1[183]*Gu1[28] + Gx1[184]*Gu1[32] + Gx1[185]*Gu1[36] + Gx1[186]*Gu1[40] + Gx1[187]*Gu1[44] + Gx1[188]*Gu1[48] + Gx1[189]*Gu1[52] + Gx1[190]*Gu1[56] + Gx1[191]*Gu1[60];
Gu2[45] = + Gx1[176]*Gu1[1] + Gx1[177]*Gu1[5] + Gx1[178]*Gu1[9] + Gx1[179]*Gu1[13] + Gx1[180]*Gu1[17] + Gx1[181]*Gu1[21] + Gx1[182]*Gu1[25] + Gx1[183]*Gu1[29] + Gx1[184]*Gu1[33] + Gx1[185]*Gu1[37] + Gx1[186]*Gu1[41] + Gx1[187]*Gu1[45] + Gx1[188]*Gu1[49] + Gx1[189]*Gu1[53] + Gx1[190]*Gu1[57] + Gx1[191]*Gu1[61];
Gu2[46] = + Gx1[176]*Gu1[2] + Gx1[177]*Gu1[6] + Gx1[178]*Gu1[10] + Gx1[179]*Gu1[14] + Gx1[180]*Gu1[18] + Gx1[181]*Gu1[22] + Gx1[182]*Gu1[26] + Gx1[183]*Gu1[30] + Gx1[184]*Gu1[34] + Gx1[185]*Gu1[38] + Gx1[186]*Gu1[42] + Gx1[187]*Gu1[46] + Gx1[188]*Gu1[50] + Gx1[189]*Gu1[54] + Gx1[190]*Gu1[58] + Gx1[191]*Gu1[62];
Gu2[47] = + Gx1[176]*Gu1[3] + Gx1[177]*Gu1[7] + Gx1[178]*Gu1[11] + Gx1[179]*Gu1[15] + Gx1[180]*Gu1[19] + Gx1[181]*Gu1[23] + Gx1[182]*Gu1[27] + Gx1[183]*Gu1[31] + Gx1[184]*Gu1[35] + Gx1[185]*Gu1[39] + Gx1[186]*Gu1[43] + Gx1[187]*Gu1[47] + Gx1[188]*Gu1[51] + Gx1[189]*Gu1[55] + Gx1[190]*Gu1[59] + Gx1[191]*Gu1[63];
Gu2[48] = + Gx1[192]*Gu1[0] + Gx1[193]*Gu1[4] + Gx1[194]*Gu1[8] + Gx1[195]*Gu1[12] + Gx1[196]*Gu1[16] + Gx1[197]*Gu1[20] + Gx1[198]*Gu1[24] + Gx1[199]*Gu1[28] + Gx1[200]*Gu1[32] + Gx1[201]*Gu1[36] + Gx1[202]*Gu1[40] + Gx1[203]*Gu1[44] + Gx1[204]*Gu1[48] + Gx1[205]*Gu1[52] + Gx1[206]*Gu1[56] + Gx1[207]*Gu1[60];
Gu2[49] = + Gx1[192]*Gu1[1] + Gx1[193]*Gu1[5] + Gx1[194]*Gu1[9] + Gx1[195]*Gu1[13] + Gx1[196]*Gu1[17] + Gx1[197]*Gu1[21] + Gx1[198]*Gu1[25] + Gx1[199]*Gu1[29] + Gx1[200]*Gu1[33] + Gx1[201]*Gu1[37] + Gx1[202]*Gu1[41] + Gx1[203]*Gu1[45] + Gx1[204]*Gu1[49] + Gx1[205]*Gu1[53] + Gx1[206]*Gu1[57] + Gx1[207]*Gu1[61];
Gu2[50] = + Gx1[192]*Gu1[2] + Gx1[193]*Gu1[6] + Gx1[194]*Gu1[10] + Gx1[195]*Gu1[14] + Gx1[196]*Gu1[18] + Gx1[197]*Gu1[22] + Gx1[198]*Gu1[26] + Gx1[199]*Gu1[30] + Gx1[200]*Gu1[34] + Gx1[201]*Gu1[38] + Gx1[202]*Gu1[42] + Gx1[203]*Gu1[46] + Gx1[204]*Gu1[50] + Gx1[205]*Gu1[54] + Gx1[206]*Gu1[58] + Gx1[207]*Gu1[62];
Gu2[51] = + Gx1[192]*Gu1[3] + Gx1[193]*Gu1[7] + Gx1[194]*Gu1[11] + Gx1[195]*Gu1[15] + Gx1[196]*Gu1[19] + Gx1[197]*Gu1[23] + Gx1[198]*Gu1[27] + Gx1[199]*Gu1[31] + Gx1[200]*Gu1[35] + Gx1[201]*Gu1[39] + Gx1[202]*Gu1[43] + Gx1[203]*Gu1[47] + Gx1[204]*Gu1[51] + Gx1[205]*Gu1[55] + Gx1[206]*Gu1[59] + Gx1[207]*Gu1[63];
Gu2[52] = + Gx1[208]*Gu1[0] + Gx1[209]*Gu1[4] + Gx1[210]*Gu1[8] + Gx1[211]*Gu1[12] + Gx1[212]*Gu1[16] + Gx1[213]*Gu1[20] + Gx1[214]*Gu1[24] + Gx1[215]*Gu1[28] + Gx1[216]*Gu1[32] + Gx1[217]*Gu1[36] + Gx1[218]*Gu1[40] + Gx1[219]*Gu1[44] + Gx1[220]*Gu1[48] + Gx1[221]*Gu1[52] + Gx1[222]*Gu1[56] + Gx1[223]*Gu1[60];
Gu2[53] = + Gx1[208]*Gu1[1] + Gx1[209]*Gu1[5] + Gx1[210]*Gu1[9] + Gx1[211]*Gu1[13] + Gx1[212]*Gu1[17] + Gx1[213]*Gu1[21] + Gx1[214]*Gu1[25] + Gx1[215]*Gu1[29] + Gx1[216]*Gu1[33] + Gx1[217]*Gu1[37] + Gx1[218]*Gu1[41] + Gx1[219]*Gu1[45] + Gx1[220]*Gu1[49] + Gx1[221]*Gu1[53] + Gx1[222]*Gu1[57] + Gx1[223]*Gu1[61];
Gu2[54] = + Gx1[208]*Gu1[2] + Gx1[209]*Gu1[6] + Gx1[210]*Gu1[10] + Gx1[211]*Gu1[14] + Gx1[212]*Gu1[18] + Gx1[213]*Gu1[22] + Gx1[214]*Gu1[26] + Gx1[215]*Gu1[30] + Gx1[216]*Gu1[34] + Gx1[217]*Gu1[38] + Gx1[218]*Gu1[42] + Gx1[219]*Gu1[46] + Gx1[220]*Gu1[50] + Gx1[221]*Gu1[54] + Gx1[222]*Gu1[58] + Gx1[223]*Gu1[62];
Gu2[55] = + Gx1[208]*Gu1[3] + Gx1[209]*Gu1[7] + Gx1[210]*Gu1[11] + Gx1[211]*Gu1[15] + Gx1[212]*Gu1[19] + Gx1[213]*Gu1[23] + Gx1[214]*Gu1[27] + Gx1[215]*Gu1[31] + Gx1[216]*Gu1[35] + Gx1[217]*Gu1[39] + Gx1[218]*Gu1[43] + Gx1[219]*Gu1[47] + Gx1[220]*Gu1[51] + Gx1[221]*Gu1[55] + Gx1[222]*Gu1[59] + Gx1[223]*Gu1[63];
Gu2[56] = + Gx1[224]*Gu1[0] + Gx1[225]*Gu1[4] + Gx1[226]*Gu1[8] + Gx1[227]*Gu1[12] + Gx1[228]*Gu1[16] + Gx1[229]*Gu1[20] + Gx1[230]*Gu1[24] + Gx1[231]*Gu1[28] + Gx1[232]*Gu1[32] + Gx1[233]*Gu1[36] + Gx1[234]*Gu1[40] + Gx1[235]*Gu1[44] + Gx1[236]*Gu1[48] + Gx1[237]*Gu1[52] + Gx1[238]*Gu1[56] + Gx1[239]*Gu1[60];
Gu2[57] = + Gx1[224]*Gu1[1] + Gx1[225]*Gu1[5] + Gx1[226]*Gu1[9] + Gx1[227]*Gu1[13] + Gx1[228]*Gu1[17] + Gx1[229]*Gu1[21] + Gx1[230]*Gu1[25] + Gx1[231]*Gu1[29] + Gx1[232]*Gu1[33] + Gx1[233]*Gu1[37] + Gx1[234]*Gu1[41] + Gx1[235]*Gu1[45] + Gx1[236]*Gu1[49] + Gx1[237]*Gu1[53] + Gx1[238]*Gu1[57] + Gx1[239]*Gu1[61];
Gu2[58] = + Gx1[224]*Gu1[2] + Gx1[225]*Gu1[6] + Gx1[226]*Gu1[10] + Gx1[227]*Gu1[14] + Gx1[228]*Gu1[18] + Gx1[229]*Gu1[22] + Gx1[230]*Gu1[26] + Gx1[231]*Gu1[30] + Gx1[232]*Gu1[34] + Gx1[233]*Gu1[38] + Gx1[234]*Gu1[42] + Gx1[235]*Gu1[46] + Gx1[236]*Gu1[50] + Gx1[237]*Gu1[54] + Gx1[238]*Gu1[58] + Gx1[239]*Gu1[62];
Gu2[59] = + Gx1[224]*Gu1[3] + Gx1[225]*Gu1[7] + Gx1[226]*Gu1[11] + Gx1[227]*Gu1[15] + Gx1[228]*Gu1[19] + Gx1[229]*Gu1[23] + Gx1[230]*Gu1[27] + Gx1[231]*Gu1[31] + Gx1[232]*Gu1[35] + Gx1[233]*Gu1[39] + Gx1[234]*Gu1[43] + Gx1[235]*Gu1[47] + Gx1[236]*Gu1[51] + Gx1[237]*Gu1[55] + Gx1[238]*Gu1[59] + Gx1[239]*Gu1[63];
Gu2[60] = + Gx1[240]*Gu1[0] + Gx1[241]*Gu1[4] + Gx1[242]*Gu1[8] + Gx1[243]*Gu1[12] + Gx1[244]*Gu1[16] + Gx1[245]*Gu1[20] + Gx1[246]*Gu1[24] + Gx1[247]*Gu1[28] + Gx1[248]*Gu1[32] + Gx1[249]*Gu1[36] + Gx1[250]*Gu1[40] + Gx1[251]*Gu1[44] + Gx1[252]*Gu1[48] + Gx1[253]*Gu1[52] + Gx1[254]*Gu1[56] + Gx1[255]*Gu1[60];
Gu2[61] = + Gx1[240]*Gu1[1] + Gx1[241]*Gu1[5] + Gx1[242]*Gu1[9] + Gx1[243]*Gu1[13] + Gx1[244]*Gu1[17] + Gx1[245]*Gu1[21] + Gx1[246]*Gu1[25] + Gx1[247]*Gu1[29] + Gx1[248]*Gu1[33] + Gx1[249]*Gu1[37] + Gx1[250]*Gu1[41] + Gx1[251]*Gu1[45] + Gx1[252]*Gu1[49] + Gx1[253]*Gu1[53] + Gx1[254]*Gu1[57] + Gx1[255]*Gu1[61];
Gu2[62] = + Gx1[240]*Gu1[2] + Gx1[241]*Gu1[6] + Gx1[242]*Gu1[10] + Gx1[243]*Gu1[14] + Gx1[244]*Gu1[18] + Gx1[245]*Gu1[22] + Gx1[246]*Gu1[26] + Gx1[247]*Gu1[30] + Gx1[248]*Gu1[34] + Gx1[249]*Gu1[38] + Gx1[250]*Gu1[42] + Gx1[251]*Gu1[46] + Gx1[252]*Gu1[50] + Gx1[253]*Gu1[54] + Gx1[254]*Gu1[58] + Gx1[255]*Gu1[62];
Gu2[63] = + Gx1[240]*Gu1[3] + Gx1[241]*Gu1[7] + Gx1[242]*Gu1[11] + Gx1[243]*Gu1[15] + Gx1[244]*Gu1[19] + Gx1[245]*Gu1[23] + Gx1[246]*Gu1[27] + Gx1[247]*Gu1[31] + Gx1[248]*Gu1[35] + Gx1[249]*Gu1[39] + Gx1[250]*Gu1[43] + Gx1[251]*Gu1[47] + Gx1[252]*Gu1[51] + Gx1[253]*Gu1[55] + Gx1[254]*Gu1[59] + Gx1[255]*Gu1[63];
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
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
Gu2[48] = Gu1[48];
Gu2[49] = Gu1[49];
Gu2[50] = Gu1[50];
Gu2[51] = Gu1[51];
Gu2[52] = Gu1[52];
Gu2[53] = Gu1[53];
Gu2[54] = Gu1[54];
Gu2[55] = Gu1[55];
Gu2[56] = Gu1[56];
Gu2[57] = Gu1[57];
Gu2[58] = Gu1[58];
Gu2[59] = Gu1[59];
Gu2[60] = Gu1[60];
Gu2[61] = Gu1[61];
Gu2[62] = Gu1[62];
Gu2[63] = Gu1[63];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + Gu1[48]*Gu2[48] + Gu1[52]*Gu2[52] + Gu1[56]*Gu2[56] + Gu1[60]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45] + Gu1[48]*Gu2[49] + Gu1[52]*Gu2[53] + Gu1[56]*Gu2[57] + Gu1[60]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46] + Gu1[48]*Gu2[50] + Gu1[52]*Gu2[54] + Gu1[56]*Gu2[58] + Gu1[60]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47] + Gu1[48]*Gu2[51] + Gu1[52]*Gu2[55] + Gu1[56]*Gu2[59] + Gu1[60]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44] + Gu1[49]*Gu2[48] + Gu1[53]*Gu2[52] + Gu1[57]*Gu2[56] + Gu1[61]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + Gu1[49]*Gu2[49] + Gu1[53]*Gu2[53] + Gu1[57]*Gu2[57] + Gu1[61]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46] + Gu1[49]*Gu2[50] + Gu1[53]*Gu2[54] + Gu1[57]*Gu2[58] + Gu1[61]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47] + Gu1[49]*Gu2[51] + Gu1[53]*Gu2[55] + Gu1[57]*Gu2[59] + Gu1[61]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44] + Gu1[50]*Gu2[48] + Gu1[54]*Gu2[52] + Gu1[58]*Gu2[56] + Gu1[62]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45] + Gu1[50]*Gu2[49] + Gu1[54]*Gu2[53] + Gu1[58]*Gu2[57] + Gu1[62]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + Gu1[50]*Gu2[50] + Gu1[54]*Gu2[54] + Gu1[58]*Gu2[58] + Gu1[62]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47] + Gu1[50]*Gu2[51] + Gu1[54]*Gu2[55] + Gu1[58]*Gu2[59] + Gu1[62]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44] + Gu1[51]*Gu2[48] + Gu1[55]*Gu2[52] + Gu1[59]*Gu2[56] + Gu1[63]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45] + Gu1[51]*Gu2[49] + Gu1[55]*Gu2[53] + Gu1[59]*Gu2[57] + Gu1[63]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46] + Gu1[51]*Gu2[50] + Gu1[55]*Gu2[54] + Gu1[59]*Gu2[58] + Gu1[63]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + Gu1[51]*Gu2[51] + Gu1[55]*Gu2[55] + Gu1[59]*Gu2[59] + Gu1[63]*Gu2[63];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = R11[0];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = R11[1];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = R11[2];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = R11[3];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = R11[4];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = R11[5];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = R11[6];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = R11[7];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = R11[8];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = R11[9];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = R11[10];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = R11[11];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = R11[12];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = R11[13];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = R11[14];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 19)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11] + Gx1[12]*dOld[12] + Gx1[13]*dOld[13] + Gx1[14]*dOld[14] + Gx1[15]*dOld[15];
dNew[1] = + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7] + Gx1[24]*dOld[8] + Gx1[25]*dOld[9] + Gx1[26]*dOld[10] + Gx1[27]*dOld[11] + Gx1[28]*dOld[12] + Gx1[29]*dOld[13] + Gx1[30]*dOld[14] + Gx1[31]*dOld[15];
dNew[2] = + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7] + Gx1[40]*dOld[8] + Gx1[41]*dOld[9] + Gx1[42]*dOld[10] + Gx1[43]*dOld[11] + Gx1[44]*dOld[12] + Gx1[45]*dOld[13] + Gx1[46]*dOld[14] + Gx1[47]*dOld[15];
dNew[3] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11] + Gx1[60]*dOld[12] + Gx1[61]*dOld[13] + Gx1[62]*dOld[14] + Gx1[63]*dOld[15];
dNew[4] = + Gx1[64]*dOld[0] + Gx1[65]*dOld[1] + Gx1[66]*dOld[2] + Gx1[67]*dOld[3] + Gx1[68]*dOld[4] + Gx1[69]*dOld[5] + Gx1[70]*dOld[6] + Gx1[71]*dOld[7] + Gx1[72]*dOld[8] + Gx1[73]*dOld[9] + Gx1[74]*dOld[10] + Gx1[75]*dOld[11] + Gx1[76]*dOld[12] + Gx1[77]*dOld[13] + Gx1[78]*dOld[14] + Gx1[79]*dOld[15];
dNew[5] = + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9] + Gx1[90]*dOld[10] + Gx1[91]*dOld[11] + Gx1[92]*dOld[12] + Gx1[93]*dOld[13] + Gx1[94]*dOld[14] + Gx1[95]*dOld[15];
dNew[6] = + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11] + Gx1[108]*dOld[12] + Gx1[109]*dOld[13] + Gx1[110]*dOld[14] + Gx1[111]*dOld[15];
dNew[7] = + Gx1[112]*dOld[0] + Gx1[113]*dOld[1] + Gx1[114]*dOld[2] + Gx1[115]*dOld[3] + Gx1[116]*dOld[4] + Gx1[117]*dOld[5] + Gx1[118]*dOld[6] + Gx1[119]*dOld[7] + Gx1[120]*dOld[8] + Gx1[121]*dOld[9] + Gx1[122]*dOld[10] + Gx1[123]*dOld[11] + Gx1[124]*dOld[12] + Gx1[125]*dOld[13] + Gx1[126]*dOld[14] + Gx1[127]*dOld[15];
dNew[8] = + Gx1[128]*dOld[0] + Gx1[129]*dOld[1] + Gx1[130]*dOld[2] + Gx1[131]*dOld[3] + Gx1[132]*dOld[4] + Gx1[133]*dOld[5] + Gx1[134]*dOld[6] + Gx1[135]*dOld[7] + Gx1[136]*dOld[8] + Gx1[137]*dOld[9] + Gx1[138]*dOld[10] + Gx1[139]*dOld[11] + Gx1[140]*dOld[12] + Gx1[141]*dOld[13] + Gx1[142]*dOld[14] + Gx1[143]*dOld[15];
dNew[9] = + Gx1[144]*dOld[0] + Gx1[145]*dOld[1] + Gx1[146]*dOld[2] + Gx1[147]*dOld[3] + Gx1[148]*dOld[4] + Gx1[149]*dOld[5] + Gx1[150]*dOld[6] + Gx1[151]*dOld[7] + Gx1[152]*dOld[8] + Gx1[153]*dOld[9] + Gx1[154]*dOld[10] + Gx1[155]*dOld[11] + Gx1[156]*dOld[12] + Gx1[157]*dOld[13] + Gx1[158]*dOld[14] + Gx1[159]*dOld[15];
dNew[10] = + Gx1[160]*dOld[0] + Gx1[161]*dOld[1] + Gx1[162]*dOld[2] + Gx1[163]*dOld[3] + Gx1[164]*dOld[4] + Gx1[165]*dOld[5] + Gx1[166]*dOld[6] + Gx1[167]*dOld[7] + Gx1[168]*dOld[8] + Gx1[169]*dOld[9] + Gx1[170]*dOld[10] + Gx1[171]*dOld[11] + Gx1[172]*dOld[12] + Gx1[173]*dOld[13] + Gx1[174]*dOld[14] + Gx1[175]*dOld[15];
dNew[11] = + Gx1[176]*dOld[0] + Gx1[177]*dOld[1] + Gx1[178]*dOld[2] + Gx1[179]*dOld[3] + Gx1[180]*dOld[4] + Gx1[181]*dOld[5] + Gx1[182]*dOld[6] + Gx1[183]*dOld[7] + Gx1[184]*dOld[8] + Gx1[185]*dOld[9] + Gx1[186]*dOld[10] + Gx1[187]*dOld[11] + Gx1[188]*dOld[12] + Gx1[189]*dOld[13] + Gx1[190]*dOld[14] + Gx1[191]*dOld[15];
dNew[12] = + Gx1[192]*dOld[0] + Gx1[193]*dOld[1] + Gx1[194]*dOld[2] + Gx1[195]*dOld[3] + Gx1[196]*dOld[4] + Gx1[197]*dOld[5] + Gx1[198]*dOld[6] + Gx1[199]*dOld[7] + Gx1[200]*dOld[8] + Gx1[201]*dOld[9] + Gx1[202]*dOld[10] + Gx1[203]*dOld[11] + Gx1[204]*dOld[12] + Gx1[205]*dOld[13] + Gx1[206]*dOld[14] + Gx1[207]*dOld[15];
dNew[13] = + Gx1[208]*dOld[0] + Gx1[209]*dOld[1] + Gx1[210]*dOld[2] + Gx1[211]*dOld[3] + Gx1[212]*dOld[4] + Gx1[213]*dOld[5] + Gx1[214]*dOld[6] + Gx1[215]*dOld[7] + Gx1[216]*dOld[8] + Gx1[217]*dOld[9] + Gx1[218]*dOld[10] + Gx1[219]*dOld[11] + Gx1[220]*dOld[12] + Gx1[221]*dOld[13] + Gx1[222]*dOld[14] + Gx1[223]*dOld[15];
dNew[14] = + Gx1[224]*dOld[0] + Gx1[225]*dOld[1] + Gx1[226]*dOld[2] + Gx1[227]*dOld[3] + Gx1[228]*dOld[4] + Gx1[229]*dOld[5] + Gx1[230]*dOld[6] + Gx1[231]*dOld[7] + Gx1[232]*dOld[8] + Gx1[233]*dOld[9] + Gx1[234]*dOld[10] + Gx1[235]*dOld[11] + Gx1[236]*dOld[12] + Gx1[237]*dOld[13] + Gx1[238]*dOld[14] + Gx1[239]*dOld[15];
dNew[15] = + Gx1[240]*dOld[0] + Gx1[241]*dOld[1] + Gx1[242]*dOld[2] + Gx1[243]*dOld[3] + Gx1[244]*dOld[4] + Gx1[245]*dOld[5] + Gx1[246]*dOld[6] + Gx1[247]*dOld[7] + Gx1[248]*dOld[8] + Gx1[249]*dOld[9] + Gx1[250]*dOld[10] + Gx1[251]*dOld[11] + Gx1[252]*dOld[12] + Gx1[253]*dOld[13] + Gx1[254]*dOld[14] + Gx1[255]*dOld[15];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7] + nmpcWorkspace.QN1[8]*dOld[8] + nmpcWorkspace.QN1[9]*dOld[9] + nmpcWorkspace.QN1[10]*dOld[10] + nmpcWorkspace.QN1[11]*dOld[11] + nmpcWorkspace.QN1[12]*dOld[12] + nmpcWorkspace.QN1[13]*dOld[13] + nmpcWorkspace.QN1[14]*dOld[14] + nmpcWorkspace.QN1[15]*dOld[15];
dNew[1] = + nmpcWorkspace.QN1[16]*dOld[0] + nmpcWorkspace.QN1[17]*dOld[1] + nmpcWorkspace.QN1[18]*dOld[2] + nmpcWorkspace.QN1[19]*dOld[3] + nmpcWorkspace.QN1[20]*dOld[4] + nmpcWorkspace.QN1[21]*dOld[5] + nmpcWorkspace.QN1[22]*dOld[6] + nmpcWorkspace.QN1[23]*dOld[7] + nmpcWorkspace.QN1[24]*dOld[8] + nmpcWorkspace.QN1[25]*dOld[9] + nmpcWorkspace.QN1[26]*dOld[10] + nmpcWorkspace.QN1[27]*dOld[11] + nmpcWorkspace.QN1[28]*dOld[12] + nmpcWorkspace.QN1[29]*dOld[13] + nmpcWorkspace.QN1[30]*dOld[14] + nmpcWorkspace.QN1[31]*dOld[15];
dNew[2] = + nmpcWorkspace.QN1[32]*dOld[0] + nmpcWorkspace.QN1[33]*dOld[1] + nmpcWorkspace.QN1[34]*dOld[2] + nmpcWorkspace.QN1[35]*dOld[3] + nmpcWorkspace.QN1[36]*dOld[4] + nmpcWorkspace.QN1[37]*dOld[5] + nmpcWorkspace.QN1[38]*dOld[6] + nmpcWorkspace.QN1[39]*dOld[7] + nmpcWorkspace.QN1[40]*dOld[8] + nmpcWorkspace.QN1[41]*dOld[9] + nmpcWorkspace.QN1[42]*dOld[10] + nmpcWorkspace.QN1[43]*dOld[11] + nmpcWorkspace.QN1[44]*dOld[12] + nmpcWorkspace.QN1[45]*dOld[13] + nmpcWorkspace.QN1[46]*dOld[14] + nmpcWorkspace.QN1[47]*dOld[15];
dNew[3] = + nmpcWorkspace.QN1[48]*dOld[0] + nmpcWorkspace.QN1[49]*dOld[1] + nmpcWorkspace.QN1[50]*dOld[2] + nmpcWorkspace.QN1[51]*dOld[3] + nmpcWorkspace.QN1[52]*dOld[4] + nmpcWorkspace.QN1[53]*dOld[5] + nmpcWorkspace.QN1[54]*dOld[6] + nmpcWorkspace.QN1[55]*dOld[7] + nmpcWorkspace.QN1[56]*dOld[8] + nmpcWorkspace.QN1[57]*dOld[9] + nmpcWorkspace.QN1[58]*dOld[10] + nmpcWorkspace.QN1[59]*dOld[11] + nmpcWorkspace.QN1[60]*dOld[12] + nmpcWorkspace.QN1[61]*dOld[13] + nmpcWorkspace.QN1[62]*dOld[14] + nmpcWorkspace.QN1[63]*dOld[15];
dNew[4] = + nmpcWorkspace.QN1[64]*dOld[0] + nmpcWorkspace.QN1[65]*dOld[1] + nmpcWorkspace.QN1[66]*dOld[2] + nmpcWorkspace.QN1[67]*dOld[3] + nmpcWorkspace.QN1[68]*dOld[4] + nmpcWorkspace.QN1[69]*dOld[5] + nmpcWorkspace.QN1[70]*dOld[6] + nmpcWorkspace.QN1[71]*dOld[7] + nmpcWorkspace.QN1[72]*dOld[8] + nmpcWorkspace.QN1[73]*dOld[9] + nmpcWorkspace.QN1[74]*dOld[10] + nmpcWorkspace.QN1[75]*dOld[11] + nmpcWorkspace.QN1[76]*dOld[12] + nmpcWorkspace.QN1[77]*dOld[13] + nmpcWorkspace.QN1[78]*dOld[14] + nmpcWorkspace.QN1[79]*dOld[15];
dNew[5] = + nmpcWorkspace.QN1[80]*dOld[0] + nmpcWorkspace.QN1[81]*dOld[1] + nmpcWorkspace.QN1[82]*dOld[2] + nmpcWorkspace.QN1[83]*dOld[3] + nmpcWorkspace.QN1[84]*dOld[4] + nmpcWorkspace.QN1[85]*dOld[5] + nmpcWorkspace.QN1[86]*dOld[6] + nmpcWorkspace.QN1[87]*dOld[7] + nmpcWorkspace.QN1[88]*dOld[8] + nmpcWorkspace.QN1[89]*dOld[9] + nmpcWorkspace.QN1[90]*dOld[10] + nmpcWorkspace.QN1[91]*dOld[11] + nmpcWorkspace.QN1[92]*dOld[12] + nmpcWorkspace.QN1[93]*dOld[13] + nmpcWorkspace.QN1[94]*dOld[14] + nmpcWorkspace.QN1[95]*dOld[15];
dNew[6] = + nmpcWorkspace.QN1[96]*dOld[0] + nmpcWorkspace.QN1[97]*dOld[1] + nmpcWorkspace.QN1[98]*dOld[2] + nmpcWorkspace.QN1[99]*dOld[3] + nmpcWorkspace.QN1[100]*dOld[4] + nmpcWorkspace.QN1[101]*dOld[5] + nmpcWorkspace.QN1[102]*dOld[6] + nmpcWorkspace.QN1[103]*dOld[7] + nmpcWorkspace.QN1[104]*dOld[8] + nmpcWorkspace.QN1[105]*dOld[9] + nmpcWorkspace.QN1[106]*dOld[10] + nmpcWorkspace.QN1[107]*dOld[11] + nmpcWorkspace.QN1[108]*dOld[12] + nmpcWorkspace.QN1[109]*dOld[13] + nmpcWorkspace.QN1[110]*dOld[14] + nmpcWorkspace.QN1[111]*dOld[15];
dNew[7] = + nmpcWorkspace.QN1[112]*dOld[0] + nmpcWorkspace.QN1[113]*dOld[1] + nmpcWorkspace.QN1[114]*dOld[2] + nmpcWorkspace.QN1[115]*dOld[3] + nmpcWorkspace.QN1[116]*dOld[4] + nmpcWorkspace.QN1[117]*dOld[5] + nmpcWorkspace.QN1[118]*dOld[6] + nmpcWorkspace.QN1[119]*dOld[7] + nmpcWorkspace.QN1[120]*dOld[8] + nmpcWorkspace.QN1[121]*dOld[9] + nmpcWorkspace.QN1[122]*dOld[10] + nmpcWorkspace.QN1[123]*dOld[11] + nmpcWorkspace.QN1[124]*dOld[12] + nmpcWorkspace.QN1[125]*dOld[13] + nmpcWorkspace.QN1[126]*dOld[14] + nmpcWorkspace.QN1[127]*dOld[15];
dNew[8] = + nmpcWorkspace.QN1[128]*dOld[0] + nmpcWorkspace.QN1[129]*dOld[1] + nmpcWorkspace.QN1[130]*dOld[2] + nmpcWorkspace.QN1[131]*dOld[3] + nmpcWorkspace.QN1[132]*dOld[4] + nmpcWorkspace.QN1[133]*dOld[5] + nmpcWorkspace.QN1[134]*dOld[6] + nmpcWorkspace.QN1[135]*dOld[7] + nmpcWorkspace.QN1[136]*dOld[8] + nmpcWorkspace.QN1[137]*dOld[9] + nmpcWorkspace.QN1[138]*dOld[10] + nmpcWorkspace.QN1[139]*dOld[11] + nmpcWorkspace.QN1[140]*dOld[12] + nmpcWorkspace.QN1[141]*dOld[13] + nmpcWorkspace.QN1[142]*dOld[14] + nmpcWorkspace.QN1[143]*dOld[15];
dNew[9] = + nmpcWorkspace.QN1[144]*dOld[0] + nmpcWorkspace.QN1[145]*dOld[1] + nmpcWorkspace.QN1[146]*dOld[2] + nmpcWorkspace.QN1[147]*dOld[3] + nmpcWorkspace.QN1[148]*dOld[4] + nmpcWorkspace.QN1[149]*dOld[5] + nmpcWorkspace.QN1[150]*dOld[6] + nmpcWorkspace.QN1[151]*dOld[7] + nmpcWorkspace.QN1[152]*dOld[8] + nmpcWorkspace.QN1[153]*dOld[9] + nmpcWorkspace.QN1[154]*dOld[10] + nmpcWorkspace.QN1[155]*dOld[11] + nmpcWorkspace.QN1[156]*dOld[12] + nmpcWorkspace.QN1[157]*dOld[13] + nmpcWorkspace.QN1[158]*dOld[14] + nmpcWorkspace.QN1[159]*dOld[15];
dNew[10] = + nmpcWorkspace.QN1[160]*dOld[0] + nmpcWorkspace.QN1[161]*dOld[1] + nmpcWorkspace.QN1[162]*dOld[2] + nmpcWorkspace.QN1[163]*dOld[3] + nmpcWorkspace.QN1[164]*dOld[4] + nmpcWorkspace.QN1[165]*dOld[5] + nmpcWorkspace.QN1[166]*dOld[6] + nmpcWorkspace.QN1[167]*dOld[7] + nmpcWorkspace.QN1[168]*dOld[8] + nmpcWorkspace.QN1[169]*dOld[9] + nmpcWorkspace.QN1[170]*dOld[10] + nmpcWorkspace.QN1[171]*dOld[11] + nmpcWorkspace.QN1[172]*dOld[12] + nmpcWorkspace.QN1[173]*dOld[13] + nmpcWorkspace.QN1[174]*dOld[14] + nmpcWorkspace.QN1[175]*dOld[15];
dNew[11] = + nmpcWorkspace.QN1[176]*dOld[0] + nmpcWorkspace.QN1[177]*dOld[1] + nmpcWorkspace.QN1[178]*dOld[2] + nmpcWorkspace.QN1[179]*dOld[3] + nmpcWorkspace.QN1[180]*dOld[4] + nmpcWorkspace.QN1[181]*dOld[5] + nmpcWorkspace.QN1[182]*dOld[6] + nmpcWorkspace.QN1[183]*dOld[7] + nmpcWorkspace.QN1[184]*dOld[8] + nmpcWorkspace.QN1[185]*dOld[9] + nmpcWorkspace.QN1[186]*dOld[10] + nmpcWorkspace.QN1[187]*dOld[11] + nmpcWorkspace.QN1[188]*dOld[12] + nmpcWorkspace.QN1[189]*dOld[13] + nmpcWorkspace.QN1[190]*dOld[14] + nmpcWorkspace.QN1[191]*dOld[15];
dNew[12] = + nmpcWorkspace.QN1[192]*dOld[0] + nmpcWorkspace.QN1[193]*dOld[1] + nmpcWorkspace.QN1[194]*dOld[2] + nmpcWorkspace.QN1[195]*dOld[3] + nmpcWorkspace.QN1[196]*dOld[4] + nmpcWorkspace.QN1[197]*dOld[5] + nmpcWorkspace.QN1[198]*dOld[6] + nmpcWorkspace.QN1[199]*dOld[7] + nmpcWorkspace.QN1[200]*dOld[8] + nmpcWorkspace.QN1[201]*dOld[9] + nmpcWorkspace.QN1[202]*dOld[10] + nmpcWorkspace.QN1[203]*dOld[11] + nmpcWorkspace.QN1[204]*dOld[12] + nmpcWorkspace.QN1[205]*dOld[13] + nmpcWorkspace.QN1[206]*dOld[14] + nmpcWorkspace.QN1[207]*dOld[15];
dNew[13] = + nmpcWorkspace.QN1[208]*dOld[0] + nmpcWorkspace.QN1[209]*dOld[1] + nmpcWorkspace.QN1[210]*dOld[2] + nmpcWorkspace.QN1[211]*dOld[3] + nmpcWorkspace.QN1[212]*dOld[4] + nmpcWorkspace.QN1[213]*dOld[5] + nmpcWorkspace.QN1[214]*dOld[6] + nmpcWorkspace.QN1[215]*dOld[7] + nmpcWorkspace.QN1[216]*dOld[8] + nmpcWorkspace.QN1[217]*dOld[9] + nmpcWorkspace.QN1[218]*dOld[10] + nmpcWorkspace.QN1[219]*dOld[11] + nmpcWorkspace.QN1[220]*dOld[12] + nmpcWorkspace.QN1[221]*dOld[13] + nmpcWorkspace.QN1[222]*dOld[14] + nmpcWorkspace.QN1[223]*dOld[15];
dNew[14] = + nmpcWorkspace.QN1[224]*dOld[0] + nmpcWorkspace.QN1[225]*dOld[1] + nmpcWorkspace.QN1[226]*dOld[2] + nmpcWorkspace.QN1[227]*dOld[3] + nmpcWorkspace.QN1[228]*dOld[4] + nmpcWorkspace.QN1[229]*dOld[5] + nmpcWorkspace.QN1[230]*dOld[6] + nmpcWorkspace.QN1[231]*dOld[7] + nmpcWorkspace.QN1[232]*dOld[8] + nmpcWorkspace.QN1[233]*dOld[9] + nmpcWorkspace.QN1[234]*dOld[10] + nmpcWorkspace.QN1[235]*dOld[11] + nmpcWorkspace.QN1[236]*dOld[12] + nmpcWorkspace.QN1[237]*dOld[13] + nmpcWorkspace.QN1[238]*dOld[14] + nmpcWorkspace.QN1[239]*dOld[15];
dNew[15] = + nmpcWorkspace.QN1[240]*dOld[0] + nmpcWorkspace.QN1[241]*dOld[1] + nmpcWorkspace.QN1[242]*dOld[2] + nmpcWorkspace.QN1[243]*dOld[3] + nmpcWorkspace.QN1[244]*dOld[4] + nmpcWorkspace.QN1[245]*dOld[5] + nmpcWorkspace.QN1[246]*dOld[6] + nmpcWorkspace.QN1[247]*dOld[7] + nmpcWorkspace.QN1[248]*dOld[8] + nmpcWorkspace.QN1[249]*dOld[9] + nmpcWorkspace.QN1[250]*dOld[10] + nmpcWorkspace.QN1[251]*dOld[11] + nmpcWorkspace.QN1[252]*dOld[12] + nmpcWorkspace.QN1[253]*dOld[13] + nmpcWorkspace.QN1[254]*dOld[14] + nmpcWorkspace.QN1[255]*dOld[15];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15] + R2[16]*Dy1[16] + R2[17]*Dy1[17];
RDy1[1] = + R2[18]*Dy1[0] + R2[19]*Dy1[1] + R2[20]*Dy1[2] + R2[21]*Dy1[3] + R2[22]*Dy1[4] + R2[23]*Dy1[5] + R2[24]*Dy1[6] + R2[25]*Dy1[7] + R2[26]*Dy1[8] + R2[27]*Dy1[9] + R2[28]*Dy1[10] + R2[29]*Dy1[11] + R2[30]*Dy1[12] + R2[31]*Dy1[13] + R2[32]*Dy1[14] + R2[33]*Dy1[15] + R2[34]*Dy1[16] + R2[35]*Dy1[17];
RDy1[2] = + R2[36]*Dy1[0] + R2[37]*Dy1[1] + R2[38]*Dy1[2] + R2[39]*Dy1[3] + R2[40]*Dy1[4] + R2[41]*Dy1[5] + R2[42]*Dy1[6] + R2[43]*Dy1[7] + R2[44]*Dy1[8] + R2[45]*Dy1[9] + R2[46]*Dy1[10] + R2[47]*Dy1[11] + R2[48]*Dy1[12] + R2[49]*Dy1[13] + R2[50]*Dy1[14] + R2[51]*Dy1[15] + R2[52]*Dy1[16] + R2[53]*Dy1[17];
RDy1[3] = + R2[54]*Dy1[0] + R2[55]*Dy1[1] + R2[56]*Dy1[2] + R2[57]*Dy1[3] + R2[58]*Dy1[4] + R2[59]*Dy1[5] + R2[60]*Dy1[6] + R2[61]*Dy1[7] + R2[62]*Dy1[8] + R2[63]*Dy1[9] + R2[64]*Dy1[10] + R2[65]*Dy1[11] + R2[66]*Dy1[12] + R2[67]*Dy1[13] + R2[68]*Dy1[14] + R2[69]*Dy1[15] + R2[70]*Dy1[16] + R2[71]*Dy1[17];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15] + Q2[16]*Dy1[16] + Q2[17]*Dy1[17];
QDy1[1] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5] + Q2[24]*Dy1[6] + Q2[25]*Dy1[7] + Q2[26]*Dy1[8] + Q2[27]*Dy1[9] + Q2[28]*Dy1[10] + Q2[29]*Dy1[11] + Q2[30]*Dy1[12] + Q2[31]*Dy1[13] + Q2[32]*Dy1[14] + Q2[33]*Dy1[15] + Q2[34]*Dy1[16] + Q2[35]*Dy1[17];
QDy1[2] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11] + Q2[48]*Dy1[12] + Q2[49]*Dy1[13] + Q2[50]*Dy1[14] + Q2[51]*Dy1[15] + Q2[52]*Dy1[16] + Q2[53]*Dy1[17];
QDy1[3] = + Q2[54]*Dy1[0] + Q2[55]*Dy1[1] + Q2[56]*Dy1[2] + Q2[57]*Dy1[3] + Q2[58]*Dy1[4] + Q2[59]*Dy1[5] + Q2[60]*Dy1[6] + Q2[61]*Dy1[7] + Q2[62]*Dy1[8] + Q2[63]*Dy1[9] + Q2[64]*Dy1[10] + Q2[65]*Dy1[11] + Q2[66]*Dy1[12] + Q2[67]*Dy1[13] + Q2[68]*Dy1[14] + Q2[69]*Dy1[15] + Q2[70]*Dy1[16] + Q2[71]*Dy1[17];
QDy1[4] = + Q2[72]*Dy1[0] + Q2[73]*Dy1[1] + Q2[74]*Dy1[2] + Q2[75]*Dy1[3] + Q2[76]*Dy1[4] + Q2[77]*Dy1[5] + Q2[78]*Dy1[6] + Q2[79]*Dy1[7] + Q2[80]*Dy1[8] + Q2[81]*Dy1[9] + Q2[82]*Dy1[10] + Q2[83]*Dy1[11] + Q2[84]*Dy1[12] + Q2[85]*Dy1[13] + Q2[86]*Dy1[14] + Q2[87]*Dy1[15] + Q2[88]*Dy1[16] + Q2[89]*Dy1[17];
QDy1[5] = + Q2[90]*Dy1[0] + Q2[91]*Dy1[1] + Q2[92]*Dy1[2] + Q2[93]*Dy1[3] + Q2[94]*Dy1[4] + Q2[95]*Dy1[5] + Q2[96]*Dy1[6] + Q2[97]*Dy1[7] + Q2[98]*Dy1[8] + Q2[99]*Dy1[9] + Q2[100]*Dy1[10] + Q2[101]*Dy1[11] + Q2[102]*Dy1[12] + Q2[103]*Dy1[13] + Q2[104]*Dy1[14] + Q2[105]*Dy1[15] + Q2[106]*Dy1[16] + Q2[107]*Dy1[17];
QDy1[6] = + Q2[108]*Dy1[0] + Q2[109]*Dy1[1] + Q2[110]*Dy1[2] + Q2[111]*Dy1[3] + Q2[112]*Dy1[4] + Q2[113]*Dy1[5] + Q2[114]*Dy1[6] + Q2[115]*Dy1[7] + Q2[116]*Dy1[8] + Q2[117]*Dy1[9] + Q2[118]*Dy1[10] + Q2[119]*Dy1[11] + Q2[120]*Dy1[12] + Q2[121]*Dy1[13] + Q2[122]*Dy1[14] + Q2[123]*Dy1[15] + Q2[124]*Dy1[16] + Q2[125]*Dy1[17];
QDy1[7] = + Q2[126]*Dy1[0] + Q2[127]*Dy1[1] + Q2[128]*Dy1[2] + Q2[129]*Dy1[3] + Q2[130]*Dy1[4] + Q2[131]*Dy1[5] + Q2[132]*Dy1[6] + Q2[133]*Dy1[7] + Q2[134]*Dy1[8] + Q2[135]*Dy1[9] + Q2[136]*Dy1[10] + Q2[137]*Dy1[11] + Q2[138]*Dy1[12] + Q2[139]*Dy1[13] + Q2[140]*Dy1[14] + Q2[141]*Dy1[15] + Q2[142]*Dy1[16] + Q2[143]*Dy1[17];
QDy1[8] = + Q2[144]*Dy1[0] + Q2[145]*Dy1[1] + Q2[146]*Dy1[2] + Q2[147]*Dy1[3] + Q2[148]*Dy1[4] + Q2[149]*Dy1[5] + Q2[150]*Dy1[6] + Q2[151]*Dy1[7] + Q2[152]*Dy1[8] + Q2[153]*Dy1[9] + Q2[154]*Dy1[10] + Q2[155]*Dy1[11] + Q2[156]*Dy1[12] + Q2[157]*Dy1[13] + Q2[158]*Dy1[14] + Q2[159]*Dy1[15] + Q2[160]*Dy1[16] + Q2[161]*Dy1[17];
QDy1[9] = + Q2[162]*Dy1[0] + Q2[163]*Dy1[1] + Q2[164]*Dy1[2] + Q2[165]*Dy1[3] + Q2[166]*Dy1[4] + Q2[167]*Dy1[5] + Q2[168]*Dy1[6] + Q2[169]*Dy1[7] + Q2[170]*Dy1[8] + Q2[171]*Dy1[9] + Q2[172]*Dy1[10] + Q2[173]*Dy1[11] + Q2[174]*Dy1[12] + Q2[175]*Dy1[13] + Q2[176]*Dy1[14] + Q2[177]*Dy1[15] + Q2[178]*Dy1[16] + Q2[179]*Dy1[17];
QDy1[10] = + Q2[180]*Dy1[0] + Q2[181]*Dy1[1] + Q2[182]*Dy1[2] + Q2[183]*Dy1[3] + Q2[184]*Dy1[4] + Q2[185]*Dy1[5] + Q2[186]*Dy1[6] + Q2[187]*Dy1[7] + Q2[188]*Dy1[8] + Q2[189]*Dy1[9] + Q2[190]*Dy1[10] + Q2[191]*Dy1[11] + Q2[192]*Dy1[12] + Q2[193]*Dy1[13] + Q2[194]*Dy1[14] + Q2[195]*Dy1[15] + Q2[196]*Dy1[16] + Q2[197]*Dy1[17];
QDy1[11] = + Q2[198]*Dy1[0] + Q2[199]*Dy1[1] + Q2[200]*Dy1[2] + Q2[201]*Dy1[3] + Q2[202]*Dy1[4] + Q2[203]*Dy1[5] + Q2[204]*Dy1[6] + Q2[205]*Dy1[7] + Q2[206]*Dy1[8] + Q2[207]*Dy1[9] + Q2[208]*Dy1[10] + Q2[209]*Dy1[11] + Q2[210]*Dy1[12] + Q2[211]*Dy1[13] + Q2[212]*Dy1[14] + Q2[213]*Dy1[15] + Q2[214]*Dy1[16] + Q2[215]*Dy1[17];
QDy1[12] = + Q2[216]*Dy1[0] + Q2[217]*Dy1[1] + Q2[218]*Dy1[2] + Q2[219]*Dy1[3] + Q2[220]*Dy1[4] + Q2[221]*Dy1[5] + Q2[222]*Dy1[6] + Q2[223]*Dy1[7] + Q2[224]*Dy1[8] + Q2[225]*Dy1[9] + Q2[226]*Dy1[10] + Q2[227]*Dy1[11] + Q2[228]*Dy1[12] + Q2[229]*Dy1[13] + Q2[230]*Dy1[14] + Q2[231]*Dy1[15] + Q2[232]*Dy1[16] + Q2[233]*Dy1[17];
QDy1[13] = + Q2[234]*Dy1[0] + Q2[235]*Dy1[1] + Q2[236]*Dy1[2] + Q2[237]*Dy1[3] + Q2[238]*Dy1[4] + Q2[239]*Dy1[5] + Q2[240]*Dy1[6] + Q2[241]*Dy1[7] + Q2[242]*Dy1[8] + Q2[243]*Dy1[9] + Q2[244]*Dy1[10] + Q2[245]*Dy1[11] + Q2[246]*Dy1[12] + Q2[247]*Dy1[13] + Q2[248]*Dy1[14] + Q2[249]*Dy1[15] + Q2[250]*Dy1[16] + Q2[251]*Dy1[17];
QDy1[14] = + Q2[252]*Dy1[0] + Q2[253]*Dy1[1] + Q2[254]*Dy1[2] + Q2[255]*Dy1[3] + Q2[256]*Dy1[4] + Q2[257]*Dy1[5] + Q2[258]*Dy1[6] + Q2[259]*Dy1[7] + Q2[260]*Dy1[8] + Q2[261]*Dy1[9] + Q2[262]*Dy1[10] + Q2[263]*Dy1[11] + Q2[264]*Dy1[12] + Q2[265]*Dy1[13] + Q2[266]*Dy1[14] + Q2[267]*Dy1[15] + Q2[268]*Dy1[16] + Q2[269]*Dy1[17];
QDy1[15] = + Q2[270]*Dy1[0] + Q2[271]*Dy1[1] + Q2[272]*Dy1[2] + Q2[273]*Dy1[3] + Q2[274]*Dy1[4] + Q2[275]*Dy1[5] + Q2[276]*Dy1[6] + Q2[277]*Dy1[7] + Q2[278]*Dy1[8] + Q2[279]*Dy1[9] + Q2[280]*Dy1[10] + Q2[281]*Dy1[11] + Q2[282]*Dy1[12] + Q2[283]*Dy1[13] + Q2[284]*Dy1[14] + Q2[285]*Dy1[15] + Q2[286]*Dy1[16] + Q2[287]*Dy1[17];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8] + E1[36]*QDy1[9] + E1[40]*QDy1[10] + E1[44]*QDy1[11] + E1[48]*QDy1[12] + E1[52]*QDy1[13] + E1[56]*QDy1[14] + E1[60]*QDy1[15];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8] + E1[37]*QDy1[9] + E1[41]*QDy1[10] + E1[45]*QDy1[11] + E1[49]*QDy1[12] + E1[53]*QDy1[13] + E1[57]*QDy1[14] + E1[61]*QDy1[15];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8] + E1[38]*QDy1[9] + E1[42]*QDy1[10] + E1[46]*QDy1[11] + E1[50]*QDy1[12] + E1[54]*QDy1[13] + E1[58]*QDy1[14] + E1[62]*QDy1[15];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8] + E1[39]*QDy1[9] + E1[43]*QDy1[10] + E1[47]*QDy1[11] + E1[51]*QDy1[12] + E1[55]*QDy1[13] + E1[59]*QDy1[14] + E1[63]*QDy1[15];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[16] + E1[8]*Gx1[32] + E1[12]*Gx1[48] + E1[16]*Gx1[64] + E1[20]*Gx1[80] + E1[24]*Gx1[96] + E1[28]*Gx1[112] + E1[32]*Gx1[128] + E1[36]*Gx1[144] + E1[40]*Gx1[160] + E1[44]*Gx1[176] + E1[48]*Gx1[192] + E1[52]*Gx1[208] + E1[56]*Gx1[224] + E1[60]*Gx1[240];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[17] + E1[8]*Gx1[33] + E1[12]*Gx1[49] + E1[16]*Gx1[65] + E1[20]*Gx1[81] + E1[24]*Gx1[97] + E1[28]*Gx1[113] + E1[32]*Gx1[129] + E1[36]*Gx1[145] + E1[40]*Gx1[161] + E1[44]*Gx1[177] + E1[48]*Gx1[193] + E1[52]*Gx1[209] + E1[56]*Gx1[225] + E1[60]*Gx1[241];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[18] + E1[8]*Gx1[34] + E1[12]*Gx1[50] + E1[16]*Gx1[66] + E1[20]*Gx1[82] + E1[24]*Gx1[98] + E1[28]*Gx1[114] + E1[32]*Gx1[130] + E1[36]*Gx1[146] + E1[40]*Gx1[162] + E1[44]*Gx1[178] + E1[48]*Gx1[194] + E1[52]*Gx1[210] + E1[56]*Gx1[226] + E1[60]*Gx1[242];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[19] + E1[8]*Gx1[35] + E1[12]*Gx1[51] + E1[16]*Gx1[67] + E1[20]*Gx1[83] + E1[24]*Gx1[99] + E1[28]*Gx1[115] + E1[32]*Gx1[131] + E1[36]*Gx1[147] + E1[40]*Gx1[163] + E1[44]*Gx1[179] + E1[48]*Gx1[195] + E1[52]*Gx1[211] + E1[56]*Gx1[227] + E1[60]*Gx1[243];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[20] + E1[8]*Gx1[36] + E1[12]*Gx1[52] + E1[16]*Gx1[68] + E1[20]*Gx1[84] + E1[24]*Gx1[100] + E1[28]*Gx1[116] + E1[32]*Gx1[132] + E1[36]*Gx1[148] + E1[40]*Gx1[164] + E1[44]*Gx1[180] + E1[48]*Gx1[196] + E1[52]*Gx1[212] + E1[56]*Gx1[228] + E1[60]*Gx1[244];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[21] + E1[8]*Gx1[37] + E1[12]*Gx1[53] + E1[16]*Gx1[69] + E1[20]*Gx1[85] + E1[24]*Gx1[101] + E1[28]*Gx1[117] + E1[32]*Gx1[133] + E1[36]*Gx1[149] + E1[40]*Gx1[165] + E1[44]*Gx1[181] + E1[48]*Gx1[197] + E1[52]*Gx1[213] + E1[56]*Gx1[229] + E1[60]*Gx1[245];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[22] + E1[8]*Gx1[38] + E1[12]*Gx1[54] + E1[16]*Gx1[70] + E1[20]*Gx1[86] + E1[24]*Gx1[102] + E1[28]*Gx1[118] + E1[32]*Gx1[134] + E1[36]*Gx1[150] + E1[40]*Gx1[166] + E1[44]*Gx1[182] + E1[48]*Gx1[198] + E1[52]*Gx1[214] + E1[56]*Gx1[230] + E1[60]*Gx1[246];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[23] + E1[8]*Gx1[39] + E1[12]*Gx1[55] + E1[16]*Gx1[71] + E1[20]*Gx1[87] + E1[24]*Gx1[103] + E1[28]*Gx1[119] + E1[32]*Gx1[135] + E1[36]*Gx1[151] + E1[40]*Gx1[167] + E1[44]*Gx1[183] + E1[48]*Gx1[199] + E1[52]*Gx1[215] + E1[56]*Gx1[231] + E1[60]*Gx1[247];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[24] + E1[8]*Gx1[40] + E1[12]*Gx1[56] + E1[16]*Gx1[72] + E1[20]*Gx1[88] + E1[24]*Gx1[104] + E1[28]*Gx1[120] + E1[32]*Gx1[136] + E1[36]*Gx1[152] + E1[40]*Gx1[168] + E1[44]*Gx1[184] + E1[48]*Gx1[200] + E1[52]*Gx1[216] + E1[56]*Gx1[232] + E1[60]*Gx1[248];
H101[9] += + E1[0]*Gx1[9] + E1[4]*Gx1[25] + E1[8]*Gx1[41] + E1[12]*Gx1[57] + E1[16]*Gx1[73] + E1[20]*Gx1[89] + E1[24]*Gx1[105] + E1[28]*Gx1[121] + E1[32]*Gx1[137] + E1[36]*Gx1[153] + E1[40]*Gx1[169] + E1[44]*Gx1[185] + E1[48]*Gx1[201] + E1[52]*Gx1[217] + E1[56]*Gx1[233] + E1[60]*Gx1[249];
H101[10] += + E1[0]*Gx1[10] + E1[4]*Gx1[26] + E1[8]*Gx1[42] + E1[12]*Gx1[58] + E1[16]*Gx1[74] + E1[20]*Gx1[90] + E1[24]*Gx1[106] + E1[28]*Gx1[122] + E1[32]*Gx1[138] + E1[36]*Gx1[154] + E1[40]*Gx1[170] + E1[44]*Gx1[186] + E1[48]*Gx1[202] + E1[52]*Gx1[218] + E1[56]*Gx1[234] + E1[60]*Gx1[250];
H101[11] += + E1[0]*Gx1[11] + E1[4]*Gx1[27] + E1[8]*Gx1[43] + E1[12]*Gx1[59] + E1[16]*Gx1[75] + E1[20]*Gx1[91] + E1[24]*Gx1[107] + E1[28]*Gx1[123] + E1[32]*Gx1[139] + E1[36]*Gx1[155] + E1[40]*Gx1[171] + E1[44]*Gx1[187] + E1[48]*Gx1[203] + E1[52]*Gx1[219] + E1[56]*Gx1[235] + E1[60]*Gx1[251];
H101[12] += + E1[0]*Gx1[12] + E1[4]*Gx1[28] + E1[8]*Gx1[44] + E1[12]*Gx1[60] + E1[16]*Gx1[76] + E1[20]*Gx1[92] + E1[24]*Gx1[108] + E1[28]*Gx1[124] + E1[32]*Gx1[140] + E1[36]*Gx1[156] + E1[40]*Gx1[172] + E1[44]*Gx1[188] + E1[48]*Gx1[204] + E1[52]*Gx1[220] + E1[56]*Gx1[236] + E1[60]*Gx1[252];
H101[13] += + E1[0]*Gx1[13] + E1[4]*Gx1[29] + E1[8]*Gx1[45] + E1[12]*Gx1[61] + E1[16]*Gx1[77] + E1[20]*Gx1[93] + E1[24]*Gx1[109] + E1[28]*Gx1[125] + E1[32]*Gx1[141] + E1[36]*Gx1[157] + E1[40]*Gx1[173] + E1[44]*Gx1[189] + E1[48]*Gx1[205] + E1[52]*Gx1[221] + E1[56]*Gx1[237] + E1[60]*Gx1[253];
H101[14] += + E1[0]*Gx1[14] + E1[4]*Gx1[30] + E1[8]*Gx1[46] + E1[12]*Gx1[62] + E1[16]*Gx1[78] + E1[20]*Gx1[94] + E1[24]*Gx1[110] + E1[28]*Gx1[126] + E1[32]*Gx1[142] + E1[36]*Gx1[158] + E1[40]*Gx1[174] + E1[44]*Gx1[190] + E1[48]*Gx1[206] + E1[52]*Gx1[222] + E1[56]*Gx1[238] + E1[60]*Gx1[254];
H101[15] += + E1[0]*Gx1[15] + E1[4]*Gx1[31] + E1[8]*Gx1[47] + E1[12]*Gx1[63] + E1[16]*Gx1[79] + E1[20]*Gx1[95] + E1[24]*Gx1[111] + E1[28]*Gx1[127] + E1[32]*Gx1[143] + E1[36]*Gx1[159] + E1[40]*Gx1[175] + E1[44]*Gx1[191] + E1[48]*Gx1[207] + E1[52]*Gx1[223] + E1[56]*Gx1[239] + E1[60]*Gx1[255];
H101[16] += + E1[1]*Gx1[0] + E1[5]*Gx1[16] + E1[9]*Gx1[32] + E1[13]*Gx1[48] + E1[17]*Gx1[64] + E1[21]*Gx1[80] + E1[25]*Gx1[96] + E1[29]*Gx1[112] + E1[33]*Gx1[128] + E1[37]*Gx1[144] + E1[41]*Gx1[160] + E1[45]*Gx1[176] + E1[49]*Gx1[192] + E1[53]*Gx1[208] + E1[57]*Gx1[224] + E1[61]*Gx1[240];
H101[17] += + E1[1]*Gx1[1] + E1[5]*Gx1[17] + E1[9]*Gx1[33] + E1[13]*Gx1[49] + E1[17]*Gx1[65] + E1[21]*Gx1[81] + E1[25]*Gx1[97] + E1[29]*Gx1[113] + E1[33]*Gx1[129] + E1[37]*Gx1[145] + E1[41]*Gx1[161] + E1[45]*Gx1[177] + E1[49]*Gx1[193] + E1[53]*Gx1[209] + E1[57]*Gx1[225] + E1[61]*Gx1[241];
H101[18] += + E1[1]*Gx1[2] + E1[5]*Gx1[18] + E1[9]*Gx1[34] + E1[13]*Gx1[50] + E1[17]*Gx1[66] + E1[21]*Gx1[82] + E1[25]*Gx1[98] + E1[29]*Gx1[114] + E1[33]*Gx1[130] + E1[37]*Gx1[146] + E1[41]*Gx1[162] + E1[45]*Gx1[178] + E1[49]*Gx1[194] + E1[53]*Gx1[210] + E1[57]*Gx1[226] + E1[61]*Gx1[242];
H101[19] += + E1[1]*Gx1[3] + E1[5]*Gx1[19] + E1[9]*Gx1[35] + E1[13]*Gx1[51] + E1[17]*Gx1[67] + E1[21]*Gx1[83] + E1[25]*Gx1[99] + E1[29]*Gx1[115] + E1[33]*Gx1[131] + E1[37]*Gx1[147] + E1[41]*Gx1[163] + E1[45]*Gx1[179] + E1[49]*Gx1[195] + E1[53]*Gx1[211] + E1[57]*Gx1[227] + E1[61]*Gx1[243];
H101[20] += + E1[1]*Gx1[4] + E1[5]*Gx1[20] + E1[9]*Gx1[36] + E1[13]*Gx1[52] + E1[17]*Gx1[68] + E1[21]*Gx1[84] + E1[25]*Gx1[100] + E1[29]*Gx1[116] + E1[33]*Gx1[132] + E1[37]*Gx1[148] + E1[41]*Gx1[164] + E1[45]*Gx1[180] + E1[49]*Gx1[196] + E1[53]*Gx1[212] + E1[57]*Gx1[228] + E1[61]*Gx1[244];
H101[21] += + E1[1]*Gx1[5] + E1[5]*Gx1[21] + E1[9]*Gx1[37] + E1[13]*Gx1[53] + E1[17]*Gx1[69] + E1[21]*Gx1[85] + E1[25]*Gx1[101] + E1[29]*Gx1[117] + E1[33]*Gx1[133] + E1[37]*Gx1[149] + E1[41]*Gx1[165] + E1[45]*Gx1[181] + E1[49]*Gx1[197] + E1[53]*Gx1[213] + E1[57]*Gx1[229] + E1[61]*Gx1[245];
H101[22] += + E1[1]*Gx1[6] + E1[5]*Gx1[22] + E1[9]*Gx1[38] + E1[13]*Gx1[54] + E1[17]*Gx1[70] + E1[21]*Gx1[86] + E1[25]*Gx1[102] + E1[29]*Gx1[118] + E1[33]*Gx1[134] + E1[37]*Gx1[150] + E1[41]*Gx1[166] + E1[45]*Gx1[182] + E1[49]*Gx1[198] + E1[53]*Gx1[214] + E1[57]*Gx1[230] + E1[61]*Gx1[246];
H101[23] += + E1[1]*Gx1[7] + E1[5]*Gx1[23] + E1[9]*Gx1[39] + E1[13]*Gx1[55] + E1[17]*Gx1[71] + E1[21]*Gx1[87] + E1[25]*Gx1[103] + E1[29]*Gx1[119] + E1[33]*Gx1[135] + E1[37]*Gx1[151] + E1[41]*Gx1[167] + E1[45]*Gx1[183] + E1[49]*Gx1[199] + E1[53]*Gx1[215] + E1[57]*Gx1[231] + E1[61]*Gx1[247];
H101[24] += + E1[1]*Gx1[8] + E1[5]*Gx1[24] + E1[9]*Gx1[40] + E1[13]*Gx1[56] + E1[17]*Gx1[72] + E1[21]*Gx1[88] + E1[25]*Gx1[104] + E1[29]*Gx1[120] + E1[33]*Gx1[136] + E1[37]*Gx1[152] + E1[41]*Gx1[168] + E1[45]*Gx1[184] + E1[49]*Gx1[200] + E1[53]*Gx1[216] + E1[57]*Gx1[232] + E1[61]*Gx1[248];
H101[25] += + E1[1]*Gx1[9] + E1[5]*Gx1[25] + E1[9]*Gx1[41] + E1[13]*Gx1[57] + E1[17]*Gx1[73] + E1[21]*Gx1[89] + E1[25]*Gx1[105] + E1[29]*Gx1[121] + E1[33]*Gx1[137] + E1[37]*Gx1[153] + E1[41]*Gx1[169] + E1[45]*Gx1[185] + E1[49]*Gx1[201] + E1[53]*Gx1[217] + E1[57]*Gx1[233] + E1[61]*Gx1[249];
H101[26] += + E1[1]*Gx1[10] + E1[5]*Gx1[26] + E1[9]*Gx1[42] + E1[13]*Gx1[58] + E1[17]*Gx1[74] + E1[21]*Gx1[90] + E1[25]*Gx1[106] + E1[29]*Gx1[122] + E1[33]*Gx1[138] + E1[37]*Gx1[154] + E1[41]*Gx1[170] + E1[45]*Gx1[186] + E1[49]*Gx1[202] + E1[53]*Gx1[218] + E1[57]*Gx1[234] + E1[61]*Gx1[250];
H101[27] += + E1[1]*Gx1[11] + E1[5]*Gx1[27] + E1[9]*Gx1[43] + E1[13]*Gx1[59] + E1[17]*Gx1[75] + E1[21]*Gx1[91] + E1[25]*Gx1[107] + E1[29]*Gx1[123] + E1[33]*Gx1[139] + E1[37]*Gx1[155] + E1[41]*Gx1[171] + E1[45]*Gx1[187] + E1[49]*Gx1[203] + E1[53]*Gx1[219] + E1[57]*Gx1[235] + E1[61]*Gx1[251];
H101[28] += + E1[1]*Gx1[12] + E1[5]*Gx1[28] + E1[9]*Gx1[44] + E1[13]*Gx1[60] + E1[17]*Gx1[76] + E1[21]*Gx1[92] + E1[25]*Gx1[108] + E1[29]*Gx1[124] + E1[33]*Gx1[140] + E1[37]*Gx1[156] + E1[41]*Gx1[172] + E1[45]*Gx1[188] + E1[49]*Gx1[204] + E1[53]*Gx1[220] + E1[57]*Gx1[236] + E1[61]*Gx1[252];
H101[29] += + E1[1]*Gx1[13] + E1[5]*Gx1[29] + E1[9]*Gx1[45] + E1[13]*Gx1[61] + E1[17]*Gx1[77] + E1[21]*Gx1[93] + E1[25]*Gx1[109] + E1[29]*Gx1[125] + E1[33]*Gx1[141] + E1[37]*Gx1[157] + E1[41]*Gx1[173] + E1[45]*Gx1[189] + E1[49]*Gx1[205] + E1[53]*Gx1[221] + E1[57]*Gx1[237] + E1[61]*Gx1[253];
H101[30] += + E1[1]*Gx1[14] + E1[5]*Gx1[30] + E1[9]*Gx1[46] + E1[13]*Gx1[62] + E1[17]*Gx1[78] + E1[21]*Gx1[94] + E1[25]*Gx1[110] + E1[29]*Gx1[126] + E1[33]*Gx1[142] + E1[37]*Gx1[158] + E1[41]*Gx1[174] + E1[45]*Gx1[190] + E1[49]*Gx1[206] + E1[53]*Gx1[222] + E1[57]*Gx1[238] + E1[61]*Gx1[254];
H101[31] += + E1[1]*Gx1[15] + E1[5]*Gx1[31] + E1[9]*Gx1[47] + E1[13]*Gx1[63] + E1[17]*Gx1[79] + E1[21]*Gx1[95] + E1[25]*Gx1[111] + E1[29]*Gx1[127] + E1[33]*Gx1[143] + E1[37]*Gx1[159] + E1[41]*Gx1[175] + E1[45]*Gx1[191] + E1[49]*Gx1[207] + E1[53]*Gx1[223] + E1[57]*Gx1[239] + E1[61]*Gx1[255];
H101[32] += + E1[2]*Gx1[0] + E1[6]*Gx1[16] + E1[10]*Gx1[32] + E1[14]*Gx1[48] + E1[18]*Gx1[64] + E1[22]*Gx1[80] + E1[26]*Gx1[96] + E1[30]*Gx1[112] + E1[34]*Gx1[128] + E1[38]*Gx1[144] + E1[42]*Gx1[160] + E1[46]*Gx1[176] + E1[50]*Gx1[192] + E1[54]*Gx1[208] + E1[58]*Gx1[224] + E1[62]*Gx1[240];
H101[33] += + E1[2]*Gx1[1] + E1[6]*Gx1[17] + E1[10]*Gx1[33] + E1[14]*Gx1[49] + E1[18]*Gx1[65] + E1[22]*Gx1[81] + E1[26]*Gx1[97] + E1[30]*Gx1[113] + E1[34]*Gx1[129] + E1[38]*Gx1[145] + E1[42]*Gx1[161] + E1[46]*Gx1[177] + E1[50]*Gx1[193] + E1[54]*Gx1[209] + E1[58]*Gx1[225] + E1[62]*Gx1[241];
H101[34] += + E1[2]*Gx1[2] + E1[6]*Gx1[18] + E1[10]*Gx1[34] + E1[14]*Gx1[50] + E1[18]*Gx1[66] + E1[22]*Gx1[82] + E1[26]*Gx1[98] + E1[30]*Gx1[114] + E1[34]*Gx1[130] + E1[38]*Gx1[146] + E1[42]*Gx1[162] + E1[46]*Gx1[178] + E1[50]*Gx1[194] + E1[54]*Gx1[210] + E1[58]*Gx1[226] + E1[62]*Gx1[242];
H101[35] += + E1[2]*Gx1[3] + E1[6]*Gx1[19] + E1[10]*Gx1[35] + E1[14]*Gx1[51] + E1[18]*Gx1[67] + E1[22]*Gx1[83] + E1[26]*Gx1[99] + E1[30]*Gx1[115] + E1[34]*Gx1[131] + E1[38]*Gx1[147] + E1[42]*Gx1[163] + E1[46]*Gx1[179] + E1[50]*Gx1[195] + E1[54]*Gx1[211] + E1[58]*Gx1[227] + E1[62]*Gx1[243];
H101[36] += + E1[2]*Gx1[4] + E1[6]*Gx1[20] + E1[10]*Gx1[36] + E1[14]*Gx1[52] + E1[18]*Gx1[68] + E1[22]*Gx1[84] + E1[26]*Gx1[100] + E1[30]*Gx1[116] + E1[34]*Gx1[132] + E1[38]*Gx1[148] + E1[42]*Gx1[164] + E1[46]*Gx1[180] + E1[50]*Gx1[196] + E1[54]*Gx1[212] + E1[58]*Gx1[228] + E1[62]*Gx1[244];
H101[37] += + E1[2]*Gx1[5] + E1[6]*Gx1[21] + E1[10]*Gx1[37] + E1[14]*Gx1[53] + E1[18]*Gx1[69] + E1[22]*Gx1[85] + E1[26]*Gx1[101] + E1[30]*Gx1[117] + E1[34]*Gx1[133] + E1[38]*Gx1[149] + E1[42]*Gx1[165] + E1[46]*Gx1[181] + E1[50]*Gx1[197] + E1[54]*Gx1[213] + E1[58]*Gx1[229] + E1[62]*Gx1[245];
H101[38] += + E1[2]*Gx1[6] + E1[6]*Gx1[22] + E1[10]*Gx1[38] + E1[14]*Gx1[54] + E1[18]*Gx1[70] + E1[22]*Gx1[86] + E1[26]*Gx1[102] + E1[30]*Gx1[118] + E1[34]*Gx1[134] + E1[38]*Gx1[150] + E1[42]*Gx1[166] + E1[46]*Gx1[182] + E1[50]*Gx1[198] + E1[54]*Gx1[214] + E1[58]*Gx1[230] + E1[62]*Gx1[246];
H101[39] += + E1[2]*Gx1[7] + E1[6]*Gx1[23] + E1[10]*Gx1[39] + E1[14]*Gx1[55] + E1[18]*Gx1[71] + E1[22]*Gx1[87] + E1[26]*Gx1[103] + E1[30]*Gx1[119] + E1[34]*Gx1[135] + E1[38]*Gx1[151] + E1[42]*Gx1[167] + E1[46]*Gx1[183] + E1[50]*Gx1[199] + E1[54]*Gx1[215] + E1[58]*Gx1[231] + E1[62]*Gx1[247];
H101[40] += + E1[2]*Gx1[8] + E1[6]*Gx1[24] + E1[10]*Gx1[40] + E1[14]*Gx1[56] + E1[18]*Gx1[72] + E1[22]*Gx1[88] + E1[26]*Gx1[104] + E1[30]*Gx1[120] + E1[34]*Gx1[136] + E1[38]*Gx1[152] + E1[42]*Gx1[168] + E1[46]*Gx1[184] + E1[50]*Gx1[200] + E1[54]*Gx1[216] + E1[58]*Gx1[232] + E1[62]*Gx1[248];
H101[41] += + E1[2]*Gx1[9] + E1[6]*Gx1[25] + E1[10]*Gx1[41] + E1[14]*Gx1[57] + E1[18]*Gx1[73] + E1[22]*Gx1[89] + E1[26]*Gx1[105] + E1[30]*Gx1[121] + E1[34]*Gx1[137] + E1[38]*Gx1[153] + E1[42]*Gx1[169] + E1[46]*Gx1[185] + E1[50]*Gx1[201] + E1[54]*Gx1[217] + E1[58]*Gx1[233] + E1[62]*Gx1[249];
H101[42] += + E1[2]*Gx1[10] + E1[6]*Gx1[26] + E1[10]*Gx1[42] + E1[14]*Gx1[58] + E1[18]*Gx1[74] + E1[22]*Gx1[90] + E1[26]*Gx1[106] + E1[30]*Gx1[122] + E1[34]*Gx1[138] + E1[38]*Gx1[154] + E1[42]*Gx1[170] + E1[46]*Gx1[186] + E1[50]*Gx1[202] + E1[54]*Gx1[218] + E1[58]*Gx1[234] + E1[62]*Gx1[250];
H101[43] += + E1[2]*Gx1[11] + E1[6]*Gx1[27] + E1[10]*Gx1[43] + E1[14]*Gx1[59] + E1[18]*Gx1[75] + E1[22]*Gx1[91] + E1[26]*Gx1[107] + E1[30]*Gx1[123] + E1[34]*Gx1[139] + E1[38]*Gx1[155] + E1[42]*Gx1[171] + E1[46]*Gx1[187] + E1[50]*Gx1[203] + E1[54]*Gx1[219] + E1[58]*Gx1[235] + E1[62]*Gx1[251];
H101[44] += + E1[2]*Gx1[12] + E1[6]*Gx1[28] + E1[10]*Gx1[44] + E1[14]*Gx1[60] + E1[18]*Gx1[76] + E1[22]*Gx1[92] + E1[26]*Gx1[108] + E1[30]*Gx1[124] + E1[34]*Gx1[140] + E1[38]*Gx1[156] + E1[42]*Gx1[172] + E1[46]*Gx1[188] + E1[50]*Gx1[204] + E1[54]*Gx1[220] + E1[58]*Gx1[236] + E1[62]*Gx1[252];
H101[45] += + E1[2]*Gx1[13] + E1[6]*Gx1[29] + E1[10]*Gx1[45] + E1[14]*Gx1[61] + E1[18]*Gx1[77] + E1[22]*Gx1[93] + E1[26]*Gx1[109] + E1[30]*Gx1[125] + E1[34]*Gx1[141] + E1[38]*Gx1[157] + E1[42]*Gx1[173] + E1[46]*Gx1[189] + E1[50]*Gx1[205] + E1[54]*Gx1[221] + E1[58]*Gx1[237] + E1[62]*Gx1[253];
H101[46] += + E1[2]*Gx1[14] + E1[6]*Gx1[30] + E1[10]*Gx1[46] + E1[14]*Gx1[62] + E1[18]*Gx1[78] + E1[22]*Gx1[94] + E1[26]*Gx1[110] + E1[30]*Gx1[126] + E1[34]*Gx1[142] + E1[38]*Gx1[158] + E1[42]*Gx1[174] + E1[46]*Gx1[190] + E1[50]*Gx1[206] + E1[54]*Gx1[222] + E1[58]*Gx1[238] + E1[62]*Gx1[254];
H101[47] += + E1[2]*Gx1[15] + E1[6]*Gx1[31] + E1[10]*Gx1[47] + E1[14]*Gx1[63] + E1[18]*Gx1[79] + E1[22]*Gx1[95] + E1[26]*Gx1[111] + E1[30]*Gx1[127] + E1[34]*Gx1[143] + E1[38]*Gx1[159] + E1[42]*Gx1[175] + E1[46]*Gx1[191] + E1[50]*Gx1[207] + E1[54]*Gx1[223] + E1[58]*Gx1[239] + E1[62]*Gx1[255];
H101[48] += + E1[3]*Gx1[0] + E1[7]*Gx1[16] + E1[11]*Gx1[32] + E1[15]*Gx1[48] + E1[19]*Gx1[64] + E1[23]*Gx1[80] + E1[27]*Gx1[96] + E1[31]*Gx1[112] + E1[35]*Gx1[128] + E1[39]*Gx1[144] + E1[43]*Gx1[160] + E1[47]*Gx1[176] + E1[51]*Gx1[192] + E1[55]*Gx1[208] + E1[59]*Gx1[224] + E1[63]*Gx1[240];
H101[49] += + E1[3]*Gx1[1] + E1[7]*Gx1[17] + E1[11]*Gx1[33] + E1[15]*Gx1[49] + E1[19]*Gx1[65] + E1[23]*Gx1[81] + E1[27]*Gx1[97] + E1[31]*Gx1[113] + E1[35]*Gx1[129] + E1[39]*Gx1[145] + E1[43]*Gx1[161] + E1[47]*Gx1[177] + E1[51]*Gx1[193] + E1[55]*Gx1[209] + E1[59]*Gx1[225] + E1[63]*Gx1[241];
H101[50] += + E1[3]*Gx1[2] + E1[7]*Gx1[18] + E1[11]*Gx1[34] + E1[15]*Gx1[50] + E1[19]*Gx1[66] + E1[23]*Gx1[82] + E1[27]*Gx1[98] + E1[31]*Gx1[114] + E1[35]*Gx1[130] + E1[39]*Gx1[146] + E1[43]*Gx1[162] + E1[47]*Gx1[178] + E1[51]*Gx1[194] + E1[55]*Gx1[210] + E1[59]*Gx1[226] + E1[63]*Gx1[242];
H101[51] += + E1[3]*Gx1[3] + E1[7]*Gx1[19] + E1[11]*Gx1[35] + E1[15]*Gx1[51] + E1[19]*Gx1[67] + E1[23]*Gx1[83] + E1[27]*Gx1[99] + E1[31]*Gx1[115] + E1[35]*Gx1[131] + E1[39]*Gx1[147] + E1[43]*Gx1[163] + E1[47]*Gx1[179] + E1[51]*Gx1[195] + E1[55]*Gx1[211] + E1[59]*Gx1[227] + E1[63]*Gx1[243];
H101[52] += + E1[3]*Gx1[4] + E1[7]*Gx1[20] + E1[11]*Gx1[36] + E1[15]*Gx1[52] + E1[19]*Gx1[68] + E1[23]*Gx1[84] + E1[27]*Gx1[100] + E1[31]*Gx1[116] + E1[35]*Gx1[132] + E1[39]*Gx1[148] + E1[43]*Gx1[164] + E1[47]*Gx1[180] + E1[51]*Gx1[196] + E1[55]*Gx1[212] + E1[59]*Gx1[228] + E1[63]*Gx1[244];
H101[53] += + E1[3]*Gx1[5] + E1[7]*Gx1[21] + E1[11]*Gx1[37] + E1[15]*Gx1[53] + E1[19]*Gx1[69] + E1[23]*Gx1[85] + E1[27]*Gx1[101] + E1[31]*Gx1[117] + E1[35]*Gx1[133] + E1[39]*Gx1[149] + E1[43]*Gx1[165] + E1[47]*Gx1[181] + E1[51]*Gx1[197] + E1[55]*Gx1[213] + E1[59]*Gx1[229] + E1[63]*Gx1[245];
H101[54] += + E1[3]*Gx1[6] + E1[7]*Gx1[22] + E1[11]*Gx1[38] + E1[15]*Gx1[54] + E1[19]*Gx1[70] + E1[23]*Gx1[86] + E1[27]*Gx1[102] + E1[31]*Gx1[118] + E1[35]*Gx1[134] + E1[39]*Gx1[150] + E1[43]*Gx1[166] + E1[47]*Gx1[182] + E1[51]*Gx1[198] + E1[55]*Gx1[214] + E1[59]*Gx1[230] + E1[63]*Gx1[246];
H101[55] += + E1[3]*Gx1[7] + E1[7]*Gx1[23] + E1[11]*Gx1[39] + E1[15]*Gx1[55] + E1[19]*Gx1[71] + E1[23]*Gx1[87] + E1[27]*Gx1[103] + E1[31]*Gx1[119] + E1[35]*Gx1[135] + E1[39]*Gx1[151] + E1[43]*Gx1[167] + E1[47]*Gx1[183] + E1[51]*Gx1[199] + E1[55]*Gx1[215] + E1[59]*Gx1[231] + E1[63]*Gx1[247];
H101[56] += + E1[3]*Gx1[8] + E1[7]*Gx1[24] + E1[11]*Gx1[40] + E1[15]*Gx1[56] + E1[19]*Gx1[72] + E1[23]*Gx1[88] + E1[27]*Gx1[104] + E1[31]*Gx1[120] + E1[35]*Gx1[136] + E1[39]*Gx1[152] + E1[43]*Gx1[168] + E1[47]*Gx1[184] + E1[51]*Gx1[200] + E1[55]*Gx1[216] + E1[59]*Gx1[232] + E1[63]*Gx1[248];
H101[57] += + E1[3]*Gx1[9] + E1[7]*Gx1[25] + E1[11]*Gx1[41] + E1[15]*Gx1[57] + E1[19]*Gx1[73] + E1[23]*Gx1[89] + E1[27]*Gx1[105] + E1[31]*Gx1[121] + E1[35]*Gx1[137] + E1[39]*Gx1[153] + E1[43]*Gx1[169] + E1[47]*Gx1[185] + E1[51]*Gx1[201] + E1[55]*Gx1[217] + E1[59]*Gx1[233] + E1[63]*Gx1[249];
H101[58] += + E1[3]*Gx1[10] + E1[7]*Gx1[26] + E1[11]*Gx1[42] + E1[15]*Gx1[58] + E1[19]*Gx1[74] + E1[23]*Gx1[90] + E1[27]*Gx1[106] + E1[31]*Gx1[122] + E1[35]*Gx1[138] + E1[39]*Gx1[154] + E1[43]*Gx1[170] + E1[47]*Gx1[186] + E1[51]*Gx1[202] + E1[55]*Gx1[218] + E1[59]*Gx1[234] + E1[63]*Gx1[250];
H101[59] += + E1[3]*Gx1[11] + E1[7]*Gx1[27] + E1[11]*Gx1[43] + E1[15]*Gx1[59] + E1[19]*Gx1[75] + E1[23]*Gx1[91] + E1[27]*Gx1[107] + E1[31]*Gx1[123] + E1[35]*Gx1[139] + E1[39]*Gx1[155] + E1[43]*Gx1[171] + E1[47]*Gx1[187] + E1[51]*Gx1[203] + E1[55]*Gx1[219] + E1[59]*Gx1[235] + E1[63]*Gx1[251];
H101[60] += + E1[3]*Gx1[12] + E1[7]*Gx1[28] + E1[11]*Gx1[44] + E1[15]*Gx1[60] + E1[19]*Gx1[76] + E1[23]*Gx1[92] + E1[27]*Gx1[108] + E1[31]*Gx1[124] + E1[35]*Gx1[140] + E1[39]*Gx1[156] + E1[43]*Gx1[172] + E1[47]*Gx1[188] + E1[51]*Gx1[204] + E1[55]*Gx1[220] + E1[59]*Gx1[236] + E1[63]*Gx1[252];
H101[61] += + E1[3]*Gx1[13] + E1[7]*Gx1[29] + E1[11]*Gx1[45] + E1[15]*Gx1[61] + E1[19]*Gx1[77] + E1[23]*Gx1[93] + E1[27]*Gx1[109] + E1[31]*Gx1[125] + E1[35]*Gx1[141] + E1[39]*Gx1[157] + E1[43]*Gx1[173] + E1[47]*Gx1[189] + E1[51]*Gx1[205] + E1[55]*Gx1[221] + E1[59]*Gx1[237] + E1[63]*Gx1[253];
H101[62] += + E1[3]*Gx1[14] + E1[7]*Gx1[30] + E1[11]*Gx1[46] + E1[15]*Gx1[62] + E1[19]*Gx1[78] + E1[23]*Gx1[94] + E1[27]*Gx1[110] + E1[31]*Gx1[126] + E1[35]*Gx1[142] + E1[39]*Gx1[158] + E1[43]*Gx1[174] + E1[47]*Gx1[190] + E1[51]*Gx1[206] + E1[55]*Gx1[222] + E1[59]*Gx1[238] + E1[63]*Gx1[254];
H101[63] += + E1[3]*Gx1[15] + E1[7]*Gx1[31] + E1[11]*Gx1[47] + E1[15]*Gx1[63] + E1[19]*Gx1[79] + E1[23]*Gx1[95] + E1[27]*Gx1[111] + E1[31]*Gx1[127] + E1[35]*Gx1[143] + E1[39]*Gx1[159] + E1[43]*Gx1[175] + E1[47]*Gx1[191] + E1[51]*Gx1[207] + E1[55]*Gx1[223] + E1[59]*Gx1[239] + E1[63]*Gx1[255];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 64; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
dNew[9] += + E1[36]*U1[0] + E1[37]*U1[1] + E1[38]*U1[2] + E1[39]*U1[3];
dNew[10] += + E1[40]*U1[0] + E1[41]*U1[1] + E1[42]*U1[2] + E1[43]*U1[3];
dNew[11] += + E1[44]*U1[0] + E1[45]*U1[1] + E1[46]*U1[2] + E1[47]*U1[3];
dNew[12] += + E1[48]*U1[0] + E1[49]*U1[1] + E1[50]*U1[2] + E1[51]*U1[3];
dNew[13] += + E1[52]*U1[0] + E1[53]*U1[1] + E1[54]*U1[2] + E1[55]*U1[3];
dNew[14] += + E1[56]*U1[0] + E1[57]*U1[1] + E1[58]*U1[2] + E1[59]*U1[3];
dNew[15] += + E1[60]*U1[0] + E1[61]*U1[1] + E1[62]*U1[2] + E1[63]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[6] = 0.0000000000000000e+00;
nmpcWorkspace.H[7] = 0.0000000000000000e+00;
nmpcWorkspace.H[8] = 0.0000000000000000e+00;
nmpcWorkspace.H[9] = 0.0000000000000000e+00;
nmpcWorkspace.H[10] = 0.0000000000000000e+00;
nmpcWorkspace.H[11] = 0.0000000000000000e+00;
nmpcWorkspace.H[12] = 0.0000000000000000e+00;
nmpcWorkspace.H[13] = 0.0000000000000000e+00;
nmpcWorkspace.H[14] = 0.0000000000000000e+00;
nmpcWorkspace.H[15] = 0.0000000000000000e+00;
nmpcWorkspace.H[136] = 0.0000000000000000e+00;
nmpcWorkspace.H[137] = 0.0000000000000000e+00;
nmpcWorkspace.H[138] = 0.0000000000000000e+00;
nmpcWorkspace.H[139] = 0.0000000000000000e+00;
nmpcWorkspace.H[140] = 0.0000000000000000e+00;
nmpcWorkspace.H[141] = 0.0000000000000000e+00;
nmpcWorkspace.H[142] = 0.0000000000000000e+00;
nmpcWorkspace.H[143] = 0.0000000000000000e+00;
nmpcWorkspace.H[144] = 0.0000000000000000e+00;
nmpcWorkspace.H[145] = 0.0000000000000000e+00;
nmpcWorkspace.H[146] = 0.0000000000000000e+00;
nmpcWorkspace.H[147] = 0.0000000000000000e+00;
nmpcWorkspace.H[148] = 0.0000000000000000e+00;
nmpcWorkspace.H[149] = 0.0000000000000000e+00;
nmpcWorkspace.H[150] = 0.0000000000000000e+00;
nmpcWorkspace.H[151] = 0.0000000000000000e+00;
nmpcWorkspace.H[272] = 0.0000000000000000e+00;
nmpcWorkspace.H[273] = 0.0000000000000000e+00;
nmpcWorkspace.H[274] = 0.0000000000000000e+00;
nmpcWorkspace.H[275] = 0.0000000000000000e+00;
nmpcWorkspace.H[276] = 0.0000000000000000e+00;
nmpcWorkspace.H[277] = 0.0000000000000000e+00;
nmpcWorkspace.H[278] = 0.0000000000000000e+00;
nmpcWorkspace.H[279] = 0.0000000000000000e+00;
nmpcWorkspace.H[280] = 0.0000000000000000e+00;
nmpcWorkspace.H[281] = 0.0000000000000000e+00;
nmpcWorkspace.H[282] = 0.0000000000000000e+00;
nmpcWorkspace.H[283] = 0.0000000000000000e+00;
nmpcWorkspace.H[284] = 0.0000000000000000e+00;
nmpcWorkspace.H[285] = 0.0000000000000000e+00;
nmpcWorkspace.H[286] = 0.0000000000000000e+00;
nmpcWorkspace.H[287] = 0.0000000000000000e+00;
nmpcWorkspace.H[408] = 0.0000000000000000e+00;
nmpcWorkspace.H[409] = 0.0000000000000000e+00;
nmpcWorkspace.H[410] = 0.0000000000000000e+00;
nmpcWorkspace.H[411] = 0.0000000000000000e+00;
nmpcWorkspace.H[412] = 0.0000000000000000e+00;
nmpcWorkspace.H[413] = 0.0000000000000000e+00;
nmpcWorkspace.H[414] = 0.0000000000000000e+00;
nmpcWorkspace.H[415] = 0.0000000000000000e+00;
nmpcWorkspace.H[416] = 0.0000000000000000e+00;
nmpcWorkspace.H[417] = 0.0000000000000000e+00;
nmpcWorkspace.H[418] = 0.0000000000000000e+00;
nmpcWorkspace.H[419] = 0.0000000000000000e+00;
nmpcWorkspace.H[420] = 0.0000000000000000e+00;
nmpcWorkspace.H[421] = 0.0000000000000000e+00;
nmpcWorkspace.H[422] = 0.0000000000000000e+00;
nmpcWorkspace.H[423] = 0.0000000000000000e+00;
nmpcWorkspace.H[544] = 0.0000000000000000e+00;
nmpcWorkspace.H[545] = 0.0000000000000000e+00;
nmpcWorkspace.H[546] = 0.0000000000000000e+00;
nmpcWorkspace.H[547] = 0.0000000000000000e+00;
nmpcWorkspace.H[548] = 0.0000000000000000e+00;
nmpcWorkspace.H[549] = 0.0000000000000000e+00;
nmpcWorkspace.H[550] = 0.0000000000000000e+00;
nmpcWorkspace.H[551] = 0.0000000000000000e+00;
nmpcWorkspace.H[552] = 0.0000000000000000e+00;
nmpcWorkspace.H[553] = 0.0000000000000000e+00;
nmpcWorkspace.H[554] = 0.0000000000000000e+00;
nmpcWorkspace.H[555] = 0.0000000000000000e+00;
nmpcWorkspace.H[556] = 0.0000000000000000e+00;
nmpcWorkspace.H[557] = 0.0000000000000000e+00;
nmpcWorkspace.H[558] = 0.0000000000000000e+00;
nmpcWorkspace.H[559] = 0.0000000000000000e+00;
nmpcWorkspace.H[680] = 0.0000000000000000e+00;
nmpcWorkspace.H[681] = 0.0000000000000000e+00;
nmpcWorkspace.H[682] = 0.0000000000000000e+00;
nmpcWorkspace.H[683] = 0.0000000000000000e+00;
nmpcWorkspace.H[684] = 0.0000000000000000e+00;
nmpcWorkspace.H[685] = 0.0000000000000000e+00;
nmpcWorkspace.H[686] = 0.0000000000000000e+00;
nmpcWorkspace.H[687] = 0.0000000000000000e+00;
nmpcWorkspace.H[688] = 0.0000000000000000e+00;
nmpcWorkspace.H[689] = 0.0000000000000000e+00;
nmpcWorkspace.H[690] = 0.0000000000000000e+00;
nmpcWorkspace.H[691] = 0.0000000000000000e+00;
nmpcWorkspace.H[692] = 0.0000000000000000e+00;
nmpcWorkspace.H[693] = 0.0000000000000000e+00;
nmpcWorkspace.H[694] = 0.0000000000000000e+00;
nmpcWorkspace.H[695] = 0.0000000000000000e+00;
nmpcWorkspace.H[816] = 0.0000000000000000e+00;
nmpcWorkspace.H[817] = 0.0000000000000000e+00;
nmpcWorkspace.H[818] = 0.0000000000000000e+00;
nmpcWorkspace.H[819] = 0.0000000000000000e+00;
nmpcWorkspace.H[820] = 0.0000000000000000e+00;
nmpcWorkspace.H[821] = 0.0000000000000000e+00;
nmpcWorkspace.H[822] = 0.0000000000000000e+00;
nmpcWorkspace.H[823] = 0.0000000000000000e+00;
nmpcWorkspace.H[824] = 0.0000000000000000e+00;
nmpcWorkspace.H[825] = 0.0000000000000000e+00;
nmpcWorkspace.H[826] = 0.0000000000000000e+00;
nmpcWorkspace.H[827] = 0.0000000000000000e+00;
nmpcWorkspace.H[828] = 0.0000000000000000e+00;
nmpcWorkspace.H[829] = 0.0000000000000000e+00;
nmpcWorkspace.H[830] = 0.0000000000000000e+00;
nmpcWorkspace.H[831] = 0.0000000000000000e+00;
nmpcWorkspace.H[952] = 0.0000000000000000e+00;
nmpcWorkspace.H[953] = 0.0000000000000000e+00;
nmpcWorkspace.H[954] = 0.0000000000000000e+00;
nmpcWorkspace.H[955] = 0.0000000000000000e+00;
nmpcWorkspace.H[956] = 0.0000000000000000e+00;
nmpcWorkspace.H[957] = 0.0000000000000000e+00;
nmpcWorkspace.H[958] = 0.0000000000000000e+00;
nmpcWorkspace.H[959] = 0.0000000000000000e+00;
nmpcWorkspace.H[960] = 0.0000000000000000e+00;
nmpcWorkspace.H[961] = 0.0000000000000000e+00;
nmpcWorkspace.H[962] = 0.0000000000000000e+00;
nmpcWorkspace.H[963] = 0.0000000000000000e+00;
nmpcWorkspace.H[964] = 0.0000000000000000e+00;
nmpcWorkspace.H[965] = 0.0000000000000000e+00;
nmpcWorkspace.H[966] = 0.0000000000000000e+00;
nmpcWorkspace.H[967] = 0.0000000000000000e+00;
nmpcWorkspace.H[1088] = 0.0000000000000000e+00;
nmpcWorkspace.H[1089] = 0.0000000000000000e+00;
nmpcWorkspace.H[1090] = 0.0000000000000000e+00;
nmpcWorkspace.H[1091] = 0.0000000000000000e+00;
nmpcWorkspace.H[1092] = 0.0000000000000000e+00;
nmpcWorkspace.H[1093] = 0.0000000000000000e+00;
nmpcWorkspace.H[1094] = 0.0000000000000000e+00;
nmpcWorkspace.H[1095] = 0.0000000000000000e+00;
nmpcWorkspace.H[1096] = 0.0000000000000000e+00;
nmpcWorkspace.H[1097] = 0.0000000000000000e+00;
nmpcWorkspace.H[1098] = 0.0000000000000000e+00;
nmpcWorkspace.H[1099] = 0.0000000000000000e+00;
nmpcWorkspace.H[1100] = 0.0000000000000000e+00;
nmpcWorkspace.H[1101] = 0.0000000000000000e+00;
nmpcWorkspace.H[1102] = 0.0000000000000000e+00;
nmpcWorkspace.H[1103] = 0.0000000000000000e+00;
nmpcWorkspace.H[1224] = 0.0000000000000000e+00;
nmpcWorkspace.H[1225] = 0.0000000000000000e+00;
nmpcWorkspace.H[1226] = 0.0000000000000000e+00;
nmpcWorkspace.H[1227] = 0.0000000000000000e+00;
nmpcWorkspace.H[1228] = 0.0000000000000000e+00;
nmpcWorkspace.H[1229] = 0.0000000000000000e+00;
nmpcWorkspace.H[1230] = 0.0000000000000000e+00;
nmpcWorkspace.H[1231] = 0.0000000000000000e+00;
nmpcWorkspace.H[1232] = 0.0000000000000000e+00;
nmpcWorkspace.H[1233] = 0.0000000000000000e+00;
nmpcWorkspace.H[1234] = 0.0000000000000000e+00;
nmpcWorkspace.H[1235] = 0.0000000000000000e+00;
nmpcWorkspace.H[1236] = 0.0000000000000000e+00;
nmpcWorkspace.H[1237] = 0.0000000000000000e+00;
nmpcWorkspace.H[1238] = 0.0000000000000000e+00;
nmpcWorkspace.H[1239] = 0.0000000000000000e+00;
nmpcWorkspace.H[1360] = 0.0000000000000000e+00;
nmpcWorkspace.H[1361] = 0.0000000000000000e+00;
nmpcWorkspace.H[1362] = 0.0000000000000000e+00;
nmpcWorkspace.H[1363] = 0.0000000000000000e+00;
nmpcWorkspace.H[1364] = 0.0000000000000000e+00;
nmpcWorkspace.H[1365] = 0.0000000000000000e+00;
nmpcWorkspace.H[1366] = 0.0000000000000000e+00;
nmpcWorkspace.H[1367] = 0.0000000000000000e+00;
nmpcWorkspace.H[1368] = 0.0000000000000000e+00;
nmpcWorkspace.H[1369] = 0.0000000000000000e+00;
nmpcWorkspace.H[1370] = 0.0000000000000000e+00;
nmpcWorkspace.H[1371] = 0.0000000000000000e+00;
nmpcWorkspace.H[1372] = 0.0000000000000000e+00;
nmpcWorkspace.H[1373] = 0.0000000000000000e+00;
nmpcWorkspace.H[1374] = 0.0000000000000000e+00;
nmpcWorkspace.H[1375] = 0.0000000000000000e+00;
nmpcWorkspace.H[1496] = 0.0000000000000000e+00;
nmpcWorkspace.H[1497] = 0.0000000000000000e+00;
nmpcWorkspace.H[1498] = 0.0000000000000000e+00;
nmpcWorkspace.H[1499] = 0.0000000000000000e+00;
nmpcWorkspace.H[1500] = 0.0000000000000000e+00;
nmpcWorkspace.H[1501] = 0.0000000000000000e+00;
nmpcWorkspace.H[1502] = 0.0000000000000000e+00;
nmpcWorkspace.H[1503] = 0.0000000000000000e+00;
nmpcWorkspace.H[1504] = 0.0000000000000000e+00;
nmpcWorkspace.H[1505] = 0.0000000000000000e+00;
nmpcWorkspace.H[1506] = 0.0000000000000000e+00;
nmpcWorkspace.H[1507] = 0.0000000000000000e+00;
nmpcWorkspace.H[1508] = 0.0000000000000000e+00;
nmpcWorkspace.H[1509] = 0.0000000000000000e+00;
nmpcWorkspace.H[1510] = 0.0000000000000000e+00;
nmpcWorkspace.H[1511] = 0.0000000000000000e+00;
nmpcWorkspace.H[1632] = 0.0000000000000000e+00;
nmpcWorkspace.H[1633] = 0.0000000000000000e+00;
nmpcWorkspace.H[1634] = 0.0000000000000000e+00;
nmpcWorkspace.H[1635] = 0.0000000000000000e+00;
nmpcWorkspace.H[1636] = 0.0000000000000000e+00;
nmpcWorkspace.H[1637] = 0.0000000000000000e+00;
nmpcWorkspace.H[1638] = 0.0000000000000000e+00;
nmpcWorkspace.H[1639] = 0.0000000000000000e+00;
nmpcWorkspace.H[1640] = 0.0000000000000000e+00;
nmpcWorkspace.H[1641] = 0.0000000000000000e+00;
nmpcWorkspace.H[1642] = 0.0000000000000000e+00;
nmpcWorkspace.H[1643] = 0.0000000000000000e+00;
nmpcWorkspace.H[1644] = 0.0000000000000000e+00;
nmpcWorkspace.H[1645] = 0.0000000000000000e+00;
nmpcWorkspace.H[1646] = 0.0000000000000000e+00;
nmpcWorkspace.H[1647] = 0.0000000000000000e+00;
nmpcWorkspace.H[1768] = 0.0000000000000000e+00;
nmpcWorkspace.H[1769] = 0.0000000000000000e+00;
nmpcWorkspace.H[1770] = 0.0000000000000000e+00;
nmpcWorkspace.H[1771] = 0.0000000000000000e+00;
nmpcWorkspace.H[1772] = 0.0000000000000000e+00;
nmpcWorkspace.H[1773] = 0.0000000000000000e+00;
nmpcWorkspace.H[1774] = 0.0000000000000000e+00;
nmpcWorkspace.H[1775] = 0.0000000000000000e+00;
nmpcWorkspace.H[1776] = 0.0000000000000000e+00;
nmpcWorkspace.H[1777] = 0.0000000000000000e+00;
nmpcWorkspace.H[1778] = 0.0000000000000000e+00;
nmpcWorkspace.H[1779] = 0.0000000000000000e+00;
nmpcWorkspace.H[1780] = 0.0000000000000000e+00;
nmpcWorkspace.H[1781] = 0.0000000000000000e+00;
nmpcWorkspace.H[1782] = 0.0000000000000000e+00;
nmpcWorkspace.H[1783] = 0.0000000000000000e+00;
nmpcWorkspace.H[1904] = 0.0000000000000000e+00;
nmpcWorkspace.H[1905] = 0.0000000000000000e+00;
nmpcWorkspace.H[1906] = 0.0000000000000000e+00;
nmpcWorkspace.H[1907] = 0.0000000000000000e+00;
nmpcWorkspace.H[1908] = 0.0000000000000000e+00;
nmpcWorkspace.H[1909] = 0.0000000000000000e+00;
nmpcWorkspace.H[1910] = 0.0000000000000000e+00;
nmpcWorkspace.H[1911] = 0.0000000000000000e+00;
nmpcWorkspace.H[1912] = 0.0000000000000000e+00;
nmpcWorkspace.H[1913] = 0.0000000000000000e+00;
nmpcWorkspace.H[1914] = 0.0000000000000000e+00;
nmpcWorkspace.H[1915] = 0.0000000000000000e+00;
nmpcWorkspace.H[1916] = 0.0000000000000000e+00;
nmpcWorkspace.H[1917] = 0.0000000000000000e+00;
nmpcWorkspace.H[1918] = 0.0000000000000000e+00;
nmpcWorkspace.H[1919] = 0.0000000000000000e+00;
nmpcWorkspace.H[2040] = 0.0000000000000000e+00;
nmpcWorkspace.H[2041] = 0.0000000000000000e+00;
nmpcWorkspace.H[2042] = 0.0000000000000000e+00;
nmpcWorkspace.H[2043] = 0.0000000000000000e+00;
nmpcWorkspace.H[2044] = 0.0000000000000000e+00;
nmpcWorkspace.H[2045] = 0.0000000000000000e+00;
nmpcWorkspace.H[2046] = 0.0000000000000000e+00;
nmpcWorkspace.H[2047] = 0.0000000000000000e+00;
nmpcWorkspace.H[2048] = 0.0000000000000000e+00;
nmpcWorkspace.H[2049] = 0.0000000000000000e+00;
nmpcWorkspace.H[2050] = 0.0000000000000000e+00;
nmpcWorkspace.H[2051] = 0.0000000000000000e+00;
nmpcWorkspace.H[2052] = 0.0000000000000000e+00;
nmpcWorkspace.H[2053] = 0.0000000000000000e+00;
nmpcWorkspace.H[2054] = 0.0000000000000000e+00;
nmpcWorkspace.H[2055] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + Gx1[(lRun3 * 16) + (lRun1)]*Gx2[(lRun3 * 16) + (lRun2)];
}
nmpcWorkspace.H[(lRun1 * 136) + (lRun2)] += t;
}
}
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
g0[6] += 0.0;
;
g0[7] += 0.0;
;
g0[8] += 0.0;
;
g0[9] += 0.0;
;
g0[10] += 0.0;
;
g0[11] += 0.0;
;
g0[12] += 0.0;
;
g0[13] += 0.0;
;
g0[14] += 0.0;
;
g0[15] += 0.0;
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
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 256 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 16-16 ]), &(nmpcWorkspace.evGx[ lRun1 * 256 ]), &(nmpcWorkspace.d[ lRun1 * 16 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 256-256 ]), &(nmpcWorkspace.evGx[ lRun1 * 256 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 64 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6400 ]), &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6656 ]), &(nmpcWorkspace.evGx[ 6400 ]), &(nmpcWorkspace.QGx[ 6400 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6912 ]), &(nmpcWorkspace.evGx[ 6656 ]), &(nmpcWorkspace.QGx[ 6656 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 7168 ]), &(nmpcWorkspace.evGx[ 6912 ]), &(nmpcWorkspace.QGx[ 6912 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 7424 ]), &(nmpcWorkspace.evGx[ 7168 ]), &(nmpcWorkspace.QGx[ 7168 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 7424 ]), &(nmpcWorkspace.QGx[ 7424 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 256 + 256 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QE[ lRun3 * 64 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QE[ lRun3 * 64 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6400 ]), &(nmpcWorkspace.QGx[ 6400 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6656 ]), &(nmpcWorkspace.QGx[ 6656 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6912 ]), &(nmpcWorkspace.QGx[ 6912 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 7168 ]), &(nmpcWorkspace.QGx[ 7168 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 7424 ]), &(nmpcWorkspace.QGx[ 7424 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 64 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 64 ]), &(nmpcWorkspace.evGx[ lRun2 * 256 ]), &(nmpcWorkspace.H10[ lRun1 * 64 ]) );
}
}

for (lRun1 = 0;lRun1 < 16; ++lRun1)
for (lRun2 = 0;lRun2 < 120; ++lRun2)
nmpcWorkspace.H[(lRun1 * 136) + (lRun2 + 16)] = nmpcWorkspace.H10[(lRun2 * 16) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.QE[ lRun5 * 64 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.QE[ lRun5 * 64 ]) );
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
for (lRun2 = 0;lRun2 < 16; ++lRun2)
nmpcWorkspace.H[(lRun1 * 136 + 2176) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 16) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.d[ 112 ]), &(nmpcWorkspace.Qd[ 112 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.d[ 128 ]), &(nmpcWorkspace.Qd[ 128 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.d[ 160 ]), &(nmpcWorkspace.Qd[ 160 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.d[ 176 ]), &(nmpcWorkspace.Qd[ 176 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.d[ 192 ]), &(nmpcWorkspace.Qd[ 192 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.d[ 208 ]), &(nmpcWorkspace.Qd[ 208 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.d[ 224 ]), &(nmpcWorkspace.Qd[ 224 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.d[ 240 ]), &(nmpcWorkspace.Qd[ 240 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.d[ 256 ]), &(nmpcWorkspace.Qd[ 256 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.d[ 272 ]), &(nmpcWorkspace.Qd[ 272 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.d[ 288 ]), &(nmpcWorkspace.Qd[ 288 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.d[ 304 ]), &(nmpcWorkspace.Qd[ 304 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.d[ 320 ]), &(nmpcWorkspace.Qd[ 320 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.d[ 336 ]), &(nmpcWorkspace.Qd[ 336 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.d[ 352 ]), &(nmpcWorkspace.Qd[ 352 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.d[ 368 ]), &(nmpcWorkspace.Qd[ 368 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6400 ]), &(nmpcWorkspace.d[ 384 ]), &(nmpcWorkspace.Qd[ 384 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6656 ]), &(nmpcWorkspace.d[ 400 ]), &(nmpcWorkspace.Qd[ 400 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6912 ]), &(nmpcWorkspace.d[ 416 ]), &(nmpcWorkspace.Qd[ 416 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 7168 ]), &(nmpcWorkspace.d[ 432 ]), &(nmpcWorkspace.Qd[ 432 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 7424 ]), &(nmpcWorkspace.d[ 448 ]), &(nmpcWorkspace.Qd[ 448 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 464 ]), &(nmpcWorkspace.Qd[ 464 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 768 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1024 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1280 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1536 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1792 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2048 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2304 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2560 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2816 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3072 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3328 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3584 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3840 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4096 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4352 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4608 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4864 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5120 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5376 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5632 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5888 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6400 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6656 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6912 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 7168 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 7424 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 64 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 16 ]) );
}
}
nmpcWorkspace.lb[16] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[0];
nmpcWorkspace.lb[17] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[1];
nmpcWorkspace.lb[18] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[2];
nmpcWorkspace.lb[19] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[3];
nmpcWorkspace.lb[20] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[4];
nmpcWorkspace.lb[21] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[5];
nmpcWorkspace.lb[22] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[6];
nmpcWorkspace.lb[23] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[7];
nmpcWorkspace.lb[24] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[8];
nmpcWorkspace.lb[25] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[9];
nmpcWorkspace.lb[26] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[10];
nmpcWorkspace.lb[27] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[11];
nmpcWorkspace.lb[28] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[12];
nmpcWorkspace.lb[29] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[13];
nmpcWorkspace.lb[30] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[14];
nmpcWorkspace.lb[31] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[15];
nmpcWorkspace.lb[32] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[16];
nmpcWorkspace.lb[33] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[17];
nmpcWorkspace.lb[34] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[18];
nmpcWorkspace.lb[35] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[19];
nmpcWorkspace.lb[36] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[20];
nmpcWorkspace.lb[37] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[21];
nmpcWorkspace.lb[38] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[22];
nmpcWorkspace.lb[39] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[23];
nmpcWorkspace.lb[40] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[24];
nmpcWorkspace.lb[41] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[25];
nmpcWorkspace.lb[42] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[26];
nmpcWorkspace.lb[43] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[27];
nmpcWorkspace.lb[44] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[28];
nmpcWorkspace.lb[45] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[29];
nmpcWorkspace.lb[46] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[30];
nmpcWorkspace.lb[47] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[31];
nmpcWorkspace.lb[48] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[32];
nmpcWorkspace.lb[49] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[33];
nmpcWorkspace.lb[50] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[34];
nmpcWorkspace.lb[51] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[35];
nmpcWorkspace.lb[52] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[36];
nmpcWorkspace.lb[53] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[37];
nmpcWorkspace.lb[54] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[38];
nmpcWorkspace.lb[55] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[39];
nmpcWorkspace.lb[56] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[40];
nmpcWorkspace.lb[57] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[41];
nmpcWorkspace.lb[58] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[42];
nmpcWorkspace.lb[59] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[43];
nmpcWorkspace.lb[60] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[44];
nmpcWorkspace.lb[61] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[45];
nmpcWorkspace.lb[62] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[46];
nmpcWorkspace.lb[63] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[47];
nmpcWorkspace.lb[64] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[48];
nmpcWorkspace.lb[65] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[49];
nmpcWorkspace.lb[66] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[50];
nmpcWorkspace.lb[67] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[51];
nmpcWorkspace.lb[68] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[52];
nmpcWorkspace.lb[69] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[53];
nmpcWorkspace.lb[70] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[54];
nmpcWorkspace.lb[71] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[55];
nmpcWorkspace.lb[72] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[56];
nmpcWorkspace.lb[73] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[57];
nmpcWorkspace.lb[74] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[58];
nmpcWorkspace.lb[75] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[59];
nmpcWorkspace.lb[76] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[60];
nmpcWorkspace.lb[77] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[61];
nmpcWorkspace.lb[78] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[62];
nmpcWorkspace.lb[79] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[63];
nmpcWorkspace.lb[80] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[64];
nmpcWorkspace.lb[81] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[65];
nmpcWorkspace.lb[82] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[66];
nmpcWorkspace.lb[83] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[67];
nmpcWorkspace.lb[84] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[68];
nmpcWorkspace.lb[85] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[69];
nmpcWorkspace.lb[86] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[70];
nmpcWorkspace.lb[87] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[71];
nmpcWorkspace.lb[88] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[72];
nmpcWorkspace.lb[89] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[73];
nmpcWorkspace.lb[90] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[74];
nmpcWorkspace.lb[91] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[75];
nmpcWorkspace.lb[92] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[76];
nmpcWorkspace.lb[93] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[77];
nmpcWorkspace.lb[94] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[78];
nmpcWorkspace.lb[95] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[79];
nmpcWorkspace.lb[96] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[80];
nmpcWorkspace.lb[97] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[81];
nmpcWorkspace.lb[98] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[82];
nmpcWorkspace.lb[99] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[83];
nmpcWorkspace.lb[100] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[84];
nmpcWorkspace.lb[101] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[85];
nmpcWorkspace.lb[102] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[86];
nmpcWorkspace.lb[103] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[87];
nmpcWorkspace.lb[104] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[88];
nmpcWorkspace.lb[105] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[89];
nmpcWorkspace.lb[106] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[90];
nmpcWorkspace.lb[107] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[91];
nmpcWorkspace.lb[108] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[92];
nmpcWorkspace.lb[109] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[93];
nmpcWorkspace.lb[110] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[94];
nmpcWorkspace.lb[111] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[95];
nmpcWorkspace.lb[112] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[96];
nmpcWorkspace.lb[113] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[97];
nmpcWorkspace.lb[114] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[98];
nmpcWorkspace.lb[115] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[99];
nmpcWorkspace.lb[116] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[100];
nmpcWorkspace.lb[117] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[101];
nmpcWorkspace.lb[118] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[102];
nmpcWorkspace.lb[119] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[103];
nmpcWorkspace.lb[120] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[104];
nmpcWorkspace.lb[121] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[105];
nmpcWorkspace.lb[122] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[106];
nmpcWorkspace.lb[123] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[107];
nmpcWorkspace.lb[124] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[108];
nmpcWorkspace.lb[125] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[109];
nmpcWorkspace.lb[126] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[110];
nmpcWorkspace.lb[127] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[111];
nmpcWorkspace.lb[128] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[112];
nmpcWorkspace.lb[129] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[113];
nmpcWorkspace.lb[130] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[114];
nmpcWorkspace.lb[131] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[115];
nmpcWorkspace.lb[132] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[116];
nmpcWorkspace.lb[133] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[117];
nmpcWorkspace.lb[134] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[118];
nmpcWorkspace.lb[135] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[119];
nmpcWorkspace.ub[16] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[0];
nmpcWorkspace.ub[17] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[1];
nmpcWorkspace.ub[18] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[2];
nmpcWorkspace.ub[19] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[3];
nmpcWorkspace.ub[20] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[4];
nmpcWorkspace.ub[21] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[5];
nmpcWorkspace.ub[22] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[6];
nmpcWorkspace.ub[23] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[7];
nmpcWorkspace.ub[24] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[8];
nmpcWorkspace.ub[25] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[9];
nmpcWorkspace.ub[26] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[10];
nmpcWorkspace.ub[27] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[11];
nmpcWorkspace.ub[28] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[12];
nmpcWorkspace.ub[29] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[13];
nmpcWorkspace.ub[30] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[14];
nmpcWorkspace.ub[31] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[15];
nmpcWorkspace.ub[32] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[16];
nmpcWorkspace.ub[33] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[17];
nmpcWorkspace.ub[34] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[18];
nmpcWorkspace.ub[35] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[19];
nmpcWorkspace.ub[36] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[20];
nmpcWorkspace.ub[37] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[21];
nmpcWorkspace.ub[38] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[22];
nmpcWorkspace.ub[39] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[23];
nmpcWorkspace.ub[40] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[24];
nmpcWorkspace.ub[41] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[25];
nmpcWorkspace.ub[42] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[26];
nmpcWorkspace.ub[43] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[27];
nmpcWorkspace.ub[44] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[28];
nmpcWorkspace.ub[45] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[29];
nmpcWorkspace.ub[46] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[30];
nmpcWorkspace.ub[47] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[31];
nmpcWorkspace.ub[48] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[32];
nmpcWorkspace.ub[49] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[33];
nmpcWorkspace.ub[50] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[34];
nmpcWorkspace.ub[51] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[35];
nmpcWorkspace.ub[52] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[36];
nmpcWorkspace.ub[53] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[37];
nmpcWorkspace.ub[54] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[38];
nmpcWorkspace.ub[55] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[39];
nmpcWorkspace.ub[56] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[40];
nmpcWorkspace.ub[57] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[41];
nmpcWorkspace.ub[58] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[42];
nmpcWorkspace.ub[59] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[43];
nmpcWorkspace.ub[60] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[44];
nmpcWorkspace.ub[61] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[45];
nmpcWorkspace.ub[62] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[46];
nmpcWorkspace.ub[63] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[47];
nmpcWorkspace.ub[64] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[48];
nmpcWorkspace.ub[65] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[49];
nmpcWorkspace.ub[66] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[50];
nmpcWorkspace.ub[67] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[51];
nmpcWorkspace.ub[68] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[52];
nmpcWorkspace.ub[69] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[53];
nmpcWorkspace.ub[70] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[54];
nmpcWorkspace.ub[71] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[55];
nmpcWorkspace.ub[72] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[56];
nmpcWorkspace.ub[73] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[57];
nmpcWorkspace.ub[74] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[58];
nmpcWorkspace.ub[75] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[59];
nmpcWorkspace.ub[76] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[60];
nmpcWorkspace.ub[77] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[61];
nmpcWorkspace.ub[78] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[62];
nmpcWorkspace.ub[79] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[63];
nmpcWorkspace.ub[80] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[64];
nmpcWorkspace.ub[81] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[65];
nmpcWorkspace.ub[82] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[66];
nmpcWorkspace.ub[83] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[67];
nmpcWorkspace.ub[84] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[68];
nmpcWorkspace.ub[85] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[69];
nmpcWorkspace.ub[86] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[70];
nmpcWorkspace.ub[87] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[71];
nmpcWorkspace.ub[88] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[72];
nmpcWorkspace.ub[89] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[73];
nmpcWorkspace.ub[90] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[74];
nmpcWorkspace.ub[91] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[75];
nmpcWorkspace.ub[92] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[76];
nmpcWorkspace.ub[93] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[77];
nmpcWorkspace.ub[94] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[78];
nmpcWorkspace.ub[95] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[79];
nmpcWorkspace.ub[96] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[80];
nmpcWorkspace.ub[97] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[81];
nmpcWorkspace.ub[98] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[82];
nmpcWorkspace.ub[99] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[83];
nmpcWorkspace.ub[100] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[84];
nmpcWorkspace.ub[101] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[85];
nmpcWorkspace.ub[102] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[86];
nmpcWorkspace.ub[103] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[87];
nmpcWorkspace.ub[104] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[88];
nmpcWorkspace.ub[105] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[89];
nmpcWorkspace.ub[106] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[90];
nmpcWorkspace.ub[107] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[91];
nmpcWorkspace.ub[108] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[92];
nmpcWorkspace.ub[109] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[93];
nmpcWorkspace.ub[110] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[94];
nmpcWorkspace.ub[111] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[95];
nmpcWorkspace.ub[112] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[96];
nmpcWorkspace.ub[113] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[97];
nmpcWorkspace.ub[114] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[98];
nmpcWorkspace.ub[115] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[99];
nmpcWorkspace.ub[116] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[100];
nmpcWorkspace.ub[117] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[101];
nmpcWorkspace.ub[118] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[102];
nmpcWorkspace.ub[119] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[103];
nmpcWorkspace.ub[120] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[104];
nmpcWorkspace.ub[121] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[105];
nmpcWorkspace.ub[122] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[106];
nmpcWorkspace.ub[123] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[107];
nmpcWorkspace.ub[124] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[108];
nmpcWorkspace.ub[125] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[109];
nmpcWorkspace.ub[126] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[110];
nmpcWorkspace.ub[127] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[111];
nmpcWorkspace.ub[128] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[112];
nmpcWorkspace.ub[129] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[113];
nmpcWorkspace.ub[130] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[114];
nmpcWorkspace.ub[131] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[115];
nmpcWorkspace.ub[132] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[116];
nmpcWorkspace.ub[133] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[117];
nmpcWorkspace.ub[134] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[118];
nmpcWorkspace.ub[135] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];
nmpcWorkspace.Dx0[8] = nmpcVariables.x0[8] - nmpcVariables.x[8];
nmpcWorkspace.Dx0[9] = nmpcVariables.x0[9] - nmpcVariables.x[9];
nmpcWorkspace.Dx0[10] = nmpcVariables.x0[10] - nmpcVariables.x[10];
nmpcWorkspace.Dx0[11] = nmpcVariables.x0[11] - nmpcVariables.x[11];
nmpcWorkspace.Dx0[12] = nmpcVariables.x0[12] - nmpcVariables.x[12];
nmpcWorkspace.Dx0[13] = nmpcVariables.x0[13] - nmpcVariables.x[13];
nmpcWorkspace.Dx0[14] = nmpcVariables.x0[14] - nmpcVariables.x[14];
nmpcWorkspace.Dx0[15] = nmpcVariables.x0[15] - nmpcVariables.x[15];

for (lRun2 = 0; lRun2 < 540; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] -= nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] -= nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] -= nmpcVariables.yN[8];
nmpcWorkspace.DyN[9] -= nmpcVariables.yN[9];
nmpcWorkspace.DyN[10] -= nmpcVariables.yN[10];
nmpcWorkspace.DyN[11] -= nmpcVariables.yN[11];
nmpcWorkspace.DyN[12] -= nmpcVariables.yN[12];
nmpcWorkspace.DyN[13] -= nmpcVariables.yN[13];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 72 ]), &(nmpcWorkspace.Dy[ 18 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 144 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 216 ]), &(nmpcWorkspace.Dy[ 54 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 288 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 360 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 432 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 504 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 576 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 648 ]), &(nmpcWorkspace.Dy[ 162 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 720 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 792 ]), &(nmpcWorkspace.Dy[ 198 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 864 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 936 ]), &(nmpcWorkspace.Dy[ 234 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1008 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1080 ]), &(nmpcWorkspace.Dy[ 270 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1152 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1224 ]), &(nmpcWorkspace.Dy[ 306 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1296 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1368 ]), &(nmpcWorkspace.Dy[ 342 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1440 ]), &(nmpcWorkspace.Dy[ 360 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1512 ]), &(nmpcWorkspace.Dy[ 378 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1584 ]), &(nmpcWorkspace.Dy[ 396 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1656 ]), &(nmpcWorkspace.Dy[ 414 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1728 ]), &(nmpcWorkspace.Dy[ 432 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1800 ]), &(nmpcWorkspace.Dy[ 450 ]), &(nmpcWorkspace.g[ 116 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1872 ]), &(nmpcWorkspace.Dy[ 468 ]), &(nmpcWorkspace.g[ 120 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1944 ]), &(nmpcWorkspace.Dy[ 486 ]), &(nmpcWorkspace.g[ 124 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2016 ]), &(nmpcWorkspace.Dy[ 504 ]), &(nmpcWorkspace.g[ 128 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2088 ]), &(nmpcWorkspace.Dy[ 522 ]), &(nmpcWorkspace.g[ 132 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 288 ]), &(nmpcWorkspace.Dy[ 18 ]), &(nmpcWorkspace.QDy[ 16 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 576 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.QDy[ 32 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 864 ]), &(nmpcWorkspace.Dy[ 54 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1152 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.QDy[ 64 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1440 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1728 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2016 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.QDy[ 112 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2304 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.QDy[ 128 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2592 ]), &(nmpcWorkspace.Dy[ 162 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2880 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.QDy[ 160 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3168 ]), &(nmpcWorkspace.Dy[ 198 ]), &(nmpcWorkspace.QDy[ 176 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3456 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.QDy[ 192 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3744 ]), &(nmpcWorkspace.Dy[ 234 ]), &(nmpcWorkspace.QDy[ 208 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4032 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.QDy[ 224 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4320 ]), &(nmpcWorkspace.Dy[ 270 ]), &(nmpcWorkspace.QDy[ 240 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4608 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.QDy[ 256 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4896 ]), &(nmpcWorkspace.Dy[ 306 ]), &(nmpcWorkspace.QDy[ 272 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5184 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.QDy[ 288 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5472 ]), &(nmpcWorkspace.Dy[ 342 ]), &(nmpcWorkspace.QDy[ 304 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5760 ]), &(nmpcWorkspace.Dy[ 360 ]), &(nmpcWorkspace.QDy[ 320 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6048 ]), &(nmpcWorkspace.Dy[ 378 ]), &(nmpcWorkspace.QDy[ 336 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6336 ]), &(nmpcWorkspace.Dy[ 396 ]), &(nmpcWorkspace.QDy[ 352 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6624 ]), &(nmpcWorkspace.Dy[ 414 ]), &(nmpcWorkspace.QDy[ 368 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6912 ]), &(nmpcWorkspace.Dy[ 432 ]), &(nmpcWorkspace.QDy[ 384 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7200 ]), &(nmpcWorkspace.Dy[ 450 ]), &(nmpcWorkspace.QDy[ 400 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7488 ]), &(nmpcWorkspace.Dy[ 468 ]), &(nmpcWorkspace.QDy[ 416 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7776 ]), &(nmpcWorkspace.Dy[ 486 ]), &(nmpcWorkspace.QDy[ 432 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8064 ]), &(nmpcWorkspace.Dy[ 504 ]), &(nmpcWorkspace.QDy[ 448 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8352 ]), &(nmpcWorkspace.Dy[ 522 ]), &(nmpcWorkspace.QDy[ 464 ]) );

nmpcWorkspace.QDy[480] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[481] = + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[482] = + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[483] = + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[484] = + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[64]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[65]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[66]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[67]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[68]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[69]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[485] = + nmpcWorkspace.QN2[70]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[71]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[72]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[73]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[74]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[75]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[76]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[77]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[78]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[79]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[80]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[81]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[82]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[83]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[486] = + nmpcWorkspace.QN2[84]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[85]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[86]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[87]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[88]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[89]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[90]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[91]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[92]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[93]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[94]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[95]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[96]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[97]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[487] = + nmpcWorkspace.QN2[98]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[99]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[100]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[101]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[102]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[103]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[104]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[105]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[106]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[107]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[108]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[109]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[110]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[111]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[488] = + nmpcWorkspace.QN2[112]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[113]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[114]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[115]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[116]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[117]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[118]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[119]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[120]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[121]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[122]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[123]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[124]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[125]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[489] = + nmpcWorkspace.QN2[126]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[127]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[128]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[129]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[130]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[131]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[132]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[133]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[134]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[135]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[136]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[137]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[138]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[139]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[490] = + nmpcWorkspace.QN2[140]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[141]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[142]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[143]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[144]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[145]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[146]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[147]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[148]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[149]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[150]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[151]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[152]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[153]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[491] = + nmpcWorkspace.QN2[154]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[155]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[156]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[157]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[158]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[159]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[160]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[161]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[162]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[163]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[164]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[165]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[166]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[167]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[492] = + nmpcWorkspace.QN2[168]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[169]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[170]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[171]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[172]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[173]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[174]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[175]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[176]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[177]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[178]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[179]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[180]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[181]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[493] = + nmpcWorkspace.QN2[182]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[183]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[184]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[185]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[186]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[187]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[188]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[189]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[190]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[191]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[192]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[193]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[194]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[195]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[494] = + nmpcWorkspace.QN2[196]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[197]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[198]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[199]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[200]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[201]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[202]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[203]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[204]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[205]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[206]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[207]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[208]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[209]*nmpcWorkspace.DyN[13];
nmpcWorkspace.QDy[495] = + nmpcWorkspace.QN2[210]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[211]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[212]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[213]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[214]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[215]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[216]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[217]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[218]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[219]*nmpcWorkspace.DyN[9] + nmpcWorkspace.QN2[220]*nmpcWorkspace.DyN[10] + nmpcWorkspace.QN2[221]*nmpcWorkspace.DyN[11] + nmpcWorkspace.QN2[222]*nmpcWorkspace.DyN[12] + nmpcWorkspace.QN2[223]*nmpcWorkspace.DyN[13];

for (lRun2 = 0; lRun2 < 480; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 16] += nmpcWorkspace.Qd[lRun2];


for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
for (lRun4 = 0; lRun4 < 1; ++lRun4)
{
real_t t = 0.0;
for (lRun5 = 0; lRun5 < 480; ++lRun5)
{
t += + nmpcWorkspace.evGx[(lRun5 * 16) + (lRun2)]*nmpcWorkspace.QDy[(lRun5 + 16) + (lRun4)];
}
nmpcWorkspace.g[(lRun2) + (lRun4)] = t;
}
}


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QDy[ lRun2 * 16 + 16 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 16 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.lb[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.lb[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.lb[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.lb[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.lb[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.lb[11] = nmpcWorkspace.Dx0[11];
nmpcWorkspace.lb[12] = nmpcWorkspace.Dx0[12];
nmpcWorkspace.lb[13] = nmpcWorkspace.Dx0[13];
nmpcWorkspace.lb[14] = nmpcWorkspace.Dx0[14];
nmpcWorkspace.lb[15] = nmpcWorkspace.Dx0[15];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.ub[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.ub[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.ub[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.ub[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.ub[11] = nmpcWorkspace.Dx0[11];
nmpcWorkspace.ub[12] = nmpcWorkspace.Dx0[12];
nmpcWorkspace.ub[13] = nmpcWorkspace.Dx0[13];
nmpcWorkspace.ub[14] = nmpcWorkspace.Dx0[14];
nmpcWorkspace.ub[15] = nmpcWorkspace.Dx0[15];
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
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];
nmpcVariables.x[8] += nmpcWorkspace.x[8];
nmpcVariables.x[9] += nmpcWorkspace.x[9];
nmpcVariables.x[10] += nmpcWorkspace.x[10];
nmpcVariables.x[11] += nmpcWorkspace.x[11];
nmpcVariables.x[12] += nmpcWorkspace.x[12];
nmpcVariables.x[13] += nmpcWorkspace.x[13];
nmpcVariables.x[14] += nmpcWorkspace.x[14];
nmpcVariables.x[15] += nmpcWorkspace.x[15];

nmpcVariables.u[0] += nmpcWorkspace.x[16];
nmpcVariables.u[1] += nmpcWorkspace.x[17];
nmpcVariables.u[2] += nmpcWorkspace.x[18];
nmpcVariables.u[3] += nmpcWorkspace.x[19];
nmpcVariables.u[4] += nmpcWorkspace.x[20];
nmpcVariables.u[5] += nmpcWorkspace.x[21];
nmpcVariables.u[6] += nmpcWorkspace.x[22];
nmpcVariables.u[7] += nmpcWorkspace.x[23];
nmpcVariables.u[8] += nmpcWorkspace.x[24];
nmpcVariables.u[9] += nmpcWorkspace.x[25];
nmpcVariables.u[10] += nmpcWorkspace.x[26];
nmpcVariables.u[11] += nmpcWorkspace.x[27];
nmpcVariables.u[12] += nmpcWorkspace.x[28];
nmpcVariables.u[13] += nmpcWorkspace.x[29];
nmpcVariables.u[14] += nmpcWorkspace.x[30];
nmpcVariables.u[15] += nmpcWorkspace.x[31];
nmpcVariables.u[16] += nmpcWorkspace.x[32];
nmpcVariables.u[17] += nmpcWorkspace.x[33];
nmpcVariables.u[18] += nmpcWorkspace.x[34];
nmpcVariables.u[19] += nmpcWorkspace.x[35];
nmpcVariables.u[20] += nmpcWorkspace.x[36];
nmpcVariables.u[21] += nmpcWorkspace.x[37];
nmpcVariables.u[22] += nmpcWorkspace.x[38];
nmpcVariables.u[23] += nmpcWorkspace.x[39];
nmpcVariables.u[24] += nmpcWorkspace.x[40];
nmpcVariables.u[25] += nmpcWorkspace.x[41];
nmpcVariables.u[26] += nmpcWorkspace.x[42];
nmpcVariables.u[27] += nmpcWorkspace.x[43];
nmpcVariables.u[28] += nmpcWorkspace.x[44];
nmpcVariables.u[29] += nmpcWorkspace.x[45];
nmpcVariables.u[30] += nmpcWorkspace.x[46];
nmpcVariables.u[31] += nmpcWorkspace.x[47];
nmpcVariables.u[32] += nmpcWorkspace.x[48];
nmpcVariables.u[33] += nmpcWorkspace.x[49];
nmpcVariables.u[34] += nmpcWorkspace.x[50];
nmpcVariables.u[35] += nmpcWorkspace.x[51];
nmpcVariables.u[36] += nmpcWorkspace.x[52];
nmpcVariables.u[37] += nmpcWorkspace.x[53];
nmpcVariables.u[38] += nmpcWorkspace.x[54];
nmpcVariables.u[39] += nmpcWorkspace.x[55];
nmpcVariables.u[40] += nmpcWorkspace.x[56];
nmpcVariables.u[41] += nmpcWorkspace.x[57];
nmpcVariables.u[42] += nmpcWorkspace.x[58];
nmpcVariables.u[43] += nmpcWorkspace.x[59];
nmpcVariables.u[44] += nmpcWorkspace.x[60];
nmpcVariables.u[45] += nmpcWorkspace.x[61];
nmpcVariables.u[46] += nmpcWorkspace.x[62];
nmpcVariables.u[47] += nmpcWorkspace.x[63];
nmpcVariables.u[48] += nmpcWorkspace.x[64];
nmpcVariables.u[49] += nmpcWorkspace.x[65];
nmpcVariables.u[50] += nmpcWorkspace.x[66];
nmpcVariables.u[51] += nmpcWorkspace.x[67];
nmpcVariables.u[52] += nmpcWorkspace.x[68];
nmpcVariables.u[53] += nmpcWorkspace.x[69];
nmpcVariables.u[54] += nmpcWorkspace.x[70];
nmpcVariables.u[55] += nmpcWorkspace.x[71];
nmpcVariables.u[56] += nmpcWorkspace.x[72];
nmpcVariables.u[57] += nmpcWorkspace.x[73];
nmpcVariables.u[58] += nmpcWorkspace.x[74];
nmpcVariables.u[59] += nmpcWorkspace.x[75];
nmpcVariables.u[60] += nmpcWorkspace.x[76];
nmpcVariables.u[61] += nmpcWorkspace.x[77];
nmpcVariables.u[62] += nmpcWorkspace.x[78];
nmpcVariables.u[63] += nmpcWorkspace.x[79];
nmpcVariables.u[64] += nmpcWorkspace.x[80];
nmpcVariables.u[65] += nmpcWorkspace.x[81];
nmpcVariables.u[66] += nmpcWorkspace.x[82];
nmpcVariables.u[67] += nmpcWorkspace.x[83];
nmpcVariables.u[68] += nmpcWorkspace.x[84];
nmpcVariables.u[69] += nmpcWorkspace.x[85];
nmpcVariables.u[70] += nmpcWorkspace.x[86];
nmpcVariables.u[71] += nmpcWorkspace.x[87];
nmpcVariables.u[72] += nmpcWorkspace.x[88];
nmpcVariables.u[73] += nmpcWorkspace.x[89];
nmpcVariables.u[74] += nmpcWorkspace.x[90];
nmpcVariables.u[75] += nmpcWorkspace.x[91];
nmpcVariables.u[76] += nmpcWorkspace.x[92];
nmpcVariables.u[77] += nmpcWorkspace.x[93];
nmpcVariables.u[78] += nmpcWorkspace.x[94];
nmpcVariables.u[79] += nmpcWorkspace.x[95];
nmpcVariables.u[80] += nmpcWorkspace.x[96];
nmpcVariables.u[81] += nmpcWorkspace.x[97];
nmpcVariables.u[82] += nmpcWorkspace.x[98];
nmpcVariables.u[83] += nmpcWorkspace.x[99];
nmpcVariables.u[84] += nmpcWorkspace.x[100];
nmpcVariables.u[85] += nmpcWorkspace.x[101];
nmpcVariables.u[86] += nmpcWorkspace.x[102];
nmpcVariables.u[87] += nmpcWorkspace.x[103];
nmpcVariables.u[88] += nmpcWorkspace.x[104];
nmpcVariables.u[89] += nmpcWorkspace.x[105];
nmpcVariables.u[90] += nmpcWorkspace.x[106];
nmpcVariables.u[91] += nmpcWorkspace.x[107];
nmpcVariables.u[92] += nmpcWorkspace.x[108];
nmpcVariables.u[93] += nmpcWorkspace.x[109];
nmpcVariables.u[94] += nmpcWorkspace.x[110];
nmpcVariables.u[95] += nmpcWorkspace.x[111];
nmpcVariables.u[96] += nmpcWorkspace.x[112];
nmpcVariables.u[97] += nmpcWorkspace.x[113];
nmpcVariables.u[98] += nmpcWorkspace.x[114];
nmpcVariables.u[99] += nmpcWorkspace.x[115];
nmpcVariables.u[100] += nmpcWorkspace.x[116];
nmpcVariables.u[101] += nmpcWorkspace.x[117];
nmpcVariables.u[102] += nmpcWorkspace.x[118];
nmpcVariables.u[103] += nmpcWorkspace.x[119];
nmpcVariables.u[104] += nmpcWorkspace.x[120];
nmpcVariables.u[105] += nmpcWorkspace.x[121];
nmpcVariables.u[106] += nmpcWorkspace.x[122];
nmpcVariables.u[107] += nmpcWorkspace.x[123];
nmpcVariables.u[108] += nmpcWorkspace.x[124];
nmpcVariables.u[109] += nmpcWorkspace.x[125];
nmpcVariables.u[110] += nmpcWorkspace.x[126];
nmpcVariables.u[111] += nmpcWorkspace.x[127];
nmpcVariables.u[112] += nmpcWorkspace.x[128];
nmpcVariables.u[113] += nmpcWorkspace.x[129];
nmpcVariables.u[114] += nmpcWorkspace.x[130];
nmpcVariables.u[115] += nmpcWorkspace.x[131];
nmpcVariables.u[116] += nmpcWorkspace.x[132];
nmpcVariables.u[117] += nmpcWorkspace.x[133];
nmpcVariables.u[118] += nmpcWorkspace.x[134];
nmpcVariables.u[119] += nmpcWorkspace.x[135];

for (lRun1 = 0; lRun1 < 480; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + nmpcWorkspace.evGx[(lRun1 * 16) + (lRun3)]*nmpcWorkspace.x[(lRun3) + (lRun2)];
}
nmpcVariables.x[(lRun1 + 16) + (lRun2)] += t + nmpcWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 16 ]), &(nmpcVariables.x[ lRun1 * 16 + 16 ]) );
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
nmpcWorkspace.state[0] = nmpcVariables.x[index * 16];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 16 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 16 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 16 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 16 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 16 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 16 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 16 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[index * 16 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[index * 16 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[index * 16 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[index * 16 + 11];
nmpcWorkspace.state[12] = nmpcVariables.x[index * 16 + 12];
nmpcWorkspace.state[13] = nmpcVariables.x[index * 16 + 13];
nmpcWorkspace.state[14] = nmpcVariables.x[index * 16 + 14];
nmpcWorkspace.state[15] = nmpcVariables.x[index * 16 + 15];
nmpcWorkspace.state[336] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[337] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[338] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[339] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[340] = nmpcVariables.od[index * 9];
nmpcWorkspace.state[341] = nmpcVariables.od[index * 9 + 1];
nmpcWorkspace.state[342] = nmpcVariables.od[index * 9 + 2];
nmpcWorkspace.state[343] = nmpcVariables.od[index * 9 + 3];
nmpcWorkspace.state[344] = nmpcVariables.od[index * 9 + 4];
nmpcWorkspace.state[345] = nmpcVariables.od[index * 9 + 5];
nmpcWorkspace.state[346] = nmpcVariables.od[index * 9 + 6];
nmpcWorkspace.state[347] = nmpcVariables.od[index * 9 + 7];
nmpcWorkspace.state[348] = nmpcVariables.od[index * 9 + 8];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 16 + 16] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 16 + 17] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 16 + 18] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 16 + 19] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 16 + 20] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 16 + 21] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 16 + 22] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 16 + 23] = nmpcWorkspace.state[7];
nmpcVariables.x[index * 16 + 24] = nmpcWorkspace.state[8];
nmpcVariables.x[index * 16 + 25] = nmpcWorkspace.state[9];
nmpcVariables.x[index * 16 + 26] = nmpcWorkspace.state[10];
nmpcVariables.x[index * 16 + 27] = nmpcWorkspace.state[11];
nmpcVariables.x[index * 16 + 28] = nmpcWorkspace.state[12];
nmpcVariables.x[index * 16 + 29] = nmpcWorkspace.state[13];
nmpcVariables.x[index * 16 + 30] = nmpcWorkspace.state[14];
nmpcVariables.x[index * 16 + 31] = nmpcWorkspace.state[15];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 16] = nmpcVariables.x[index * 16 + 16];
nmpcVariables.x[index * 16 + 1] = nmpcVariables.x[index * 16 + 17];
nmpcVariables.x[index * 16 + 2] = nmpcVariables.x[index * 16 + 18];
nmpcVariables.x[index * 16 + 3] = nmpcVariables.x[index * 16 + 19];
nmpcVariables.x[index * 16 + 4] = nmpcVariables.x[index * 16 + 20];
nmpcVariables.x[index * 16 + 5] = nmpcVariables.x[index * 16 + 21];
nmpcVariables.x[index * 16 + 6] = nmpcVariables.x[index * 16 + 22];
nmpcVariables.x[index * 16 + 7] = nmpcVariables.x[index * 16 + 23];
nmpcVariables.x[index * 16 + 8] = nmpcVariables.x[index * 16 + 24];
nmpcVariables.x[index * 16 + 9] = nmpcVariables.x[index * 16 + 25];
nmpcVariables.x[index * 16 + 10] = nmpcVariables.x[index * 16 + 26];
nmpcVariables.x[index * 16 + 11] = nmpcVariables.x[index * 16 + 27];
nmpcVariables.x[index * 16 + 12] = nmpcVariables.x[index * 16 + 28];
nmpcVariables.x[index * 16 + 13] = nmpcVariables.x[index * 16 + 29];
nmpcVariables.x[index * 16 + 14] = nmpcVariables.x[index * 16 + 30];
nmpcVariables.x[index * 16 + 15] = nmpcVariables.x[index * 16 + 31];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[480] = xEnd[0];
nmpcVariables.x[481] = xEnd[1];
nmpcVariables.x[482] = xEnd[2];
nmpcVariables.x[483] = xEnd[3];
nmpcVariables.x[484] = xEnd[4];
nmpcVariables.x[485] = xEnd[5];
nmpcVariables.x[486] = xEnd[6];
nmpcVariables.x[487] = xEnd[7];
nmpcVariables.x[488] = xEnd[8];
nmpcVariables.x[489] = xEnd[9];
nmpcVariables.x[490] = xEnd[10];
nmpcVariables.x[491] = xEnd[11];
nmpcVariables.x[492] = xEnd[12];
nmpcVariables.x[493] = xEnd[13];
nmpcVariables.x[494] = xEnd[14];
nmpcVariables.x[495] = xEnd[15];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[480];
nmpcWorkspace.state[1] = nmpcVariables.x[481];
nmpcWorkspace.state[2] = nmpcVariables.x[482];
nmpcWorkspace.state[3] = nmpcVariables.x[483];
nmpcWorkspace.state[4] = nmpcVariables.x[484];
nmpcWorkspace.state[5] = nmpcVariables.x[485];
nmpcWorkspace.state[6] = nmpcVariables.x[486];
nmpcWorkspace.state[7] = nmpcVariables.x[487];
nmpcWorkspace.state[8] = nmpcVariables.x[488];
nmpcWorkspace.state[9] = nmpcVariables.x[489];
nmpcWorkspace.state[10] = nmpcVariables.x[490];
nmpcWorkspace.state[11] = nmpcVariables.x[491];
nmpcWorkspace.state[12] = nmpcVariables.x[492];
nmpcWorkspace.state[13] = nmpcVariables.x[493];
nmpcWorkspace.state[14] = nmpcVariables.x[494];
nmpcWorkspace.state[15] = nmpcVariables.x[495];
if (uEnd != 0)
{
nmpcWorkspace.state[336] = uEnd[0];
nmpcWorkspace.state[337] = uEnd[1];
nmpcWorkspace.state[338] = uEnd[2];
nmpcWorkspace.state[339] = uEnd[3];
}
else
{
nmpcWorkspace.state[336] = nmpcVariables.u[116];
nmpcWorkspace.state[337] = nmpcVariables.u[117];
nmpcWorkspace.state[338] = nmpcVariables.u[118];
nmpcWorkspace.state[339] = nmpcVariables.u[119];
}
nmpcWorkspace.state[340] = nmpcVariables.od[270];
nmpcWorkspace.state[341] = nmpcVariables.od[271];
nmpcWorkspace.state[342] = nmpcVariables.od[272];
nmpcWorkspace.state[343] = nmpcVariables.od[273];
nmpcWorkspace.state[344] = nmpcVariables.od[274];
nmpcWorkspace.state[345] = nmpcVariables.od[275];
nmpcWorkspace.state[346] = nmpcVariables.od[276];
nmpcWorkspace.state[347] = nmpcVariables.od[277];
nmpcWorkspace.state[348] = nmpcVariables.od[278];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[480] = nmpcWorkspace.state[0];
nmpcVariables.x[481] = nmpcWorkspace.state[1];
nmpcVariables.x[482] = nmpcWorkspace.state[2];
nmpcVariables.x[483] = nmpcWorkspace.state[3];
nmpcVariables.x[484] = nmpcWorkspace.state[4];
nmpcVariables.x[485] = nmpcWorkspace.state[5];
nmpcVariables.x[486] = nmpcWorkspace.state[6];
nmpcVariables.x[487] = nmpcWorkspace.state[7];
nmpcVariables.x[488] = nmpcWorkspace.state[8];
nmpcVariables.x[489] = nmpcWorkspace.state[9];
nmpcVariables.x[490] = nmpcWorkspace.state[10];
nmpcVariables.x[491] = nmpcWorkspace.state[11];
nmpcVariables.x[492] = nmpcWorkspace.state[12];
nmpcVariables.x[493] = nmpcWorkspace.state[13];
nmpcVariables.x[494] = nmpcWorkspace.state[14];
nmpcVariables.x[495] = nmpcWorkspace.state[15];
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

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125] + nmpcWorkspace.g[126]*nmpcWorkspace.x[126] + nmpcWorkspace.g[127]*nmpcWorkspace.x[127] + nmpcWorkspace.g[128]*nmpcWorkspace.x[128] + nmpcWorkspace.g[129]*nmpcWorkspace.x[129] + nmpcWorkspace.g[130]*nmpcWorkspace.x[130] + nmpcWorkspace.g[131]*nmpcWorkspace.x[131] + nmpcWorkspace.g[132]*nmpcWorkspace.x[132] + nmpcWorkspace.g[133]*nmpcWorkspace.x[133] + nmpcWorkspace.g[134]*nmpcWorkspace.x[134] + nmpcWorkspace.g[135]*nmpcWorkspace.x[135];
kkt = fabs( kkt );
for (index = 0; index < 136; ++index)
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
/** Row vector of size: 18 */
real_t tmpDy[ 18 ];

/** Row vector of size: 14 */
real_t tmpDyN[ 14 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 16];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 16 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 16 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 16 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 16 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 16 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 16 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 16 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[lRun1 * 16 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[lRun1 * 16 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[lRun1 * 16 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[lRun1 * 16 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[lRun1 * 16 + 12];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[lRun1 * 16 + 13];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[lRun1 * 16 + 14];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[lRun1 * 16 + 15];
nmpcWorkspace.objValueIn[16] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[17] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[lRun1 * 9];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[lRun1 * 9 + 1];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[lRun1 * 9 + 2];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[lRun1 * 9 + 3];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[lRun1 * 9 + 4];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[lRun1 * 9 + 5];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[lRun1 * 9 + 6];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[lRun1 * 9 + 7];
nmpcWorkspace.objValueIn[28] = nmpcVariables.od[lRun1 * 9 + 8];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 18] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 18];
nmpcWorkspace.Dy[lRun1 * 18 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 18 + 1];
nmpcWorkspace.Dy[lRun1 * 18 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 18 + 2];
nmpcWorkspace.Dy[lRun1 * 18 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 18 + 3];
nmpcWorkspace.Dy[lRun1 * 18 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 18 + 4];
nmpcWorkspace.Dy[lRun1 * 18 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 18 + 5];
nmpcWorkspace.Dy[lRun1 * 18 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 18 + 6];
nmpcWorkspace.Dy[lRun1 * 18 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 18 + 7];
nmpcWorkspace.Dy[lRun1 * 18 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 18 + 8];
nmpcWorkspace.Dy[lRun1 * 18 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 18 + 9];
nmpcWorkspace.Dy[lRun1 * 18 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 18 + 10];
nmpcWorkspace.Dy[lRun1 * 18 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 18 + 11];
nmpcWorkspace.Dy[lRun1 * 18 + 12] = nmpcWorkspace.objValueOut[12] - nmpcVariables.y[lRun1 * 18 + 12];
nmpcWorkspace.Dy[lRun1 * 18 + 13] = nmpcWorkspace.objValueOut[13] - nmpcVariables.y[lRun1 * 18 + 13];
nmpcWorkspace.Dy[lRun1 * 18 + 14] = nmpcWorkspace.objValueOut[14] - nmpcVariables.y[lRun1 * 18 + 14];
nmpcWorkspace.Dy[lRun1 * 18 + 15] = nmpcWorkspace.objValueOut[15] - nmpcVariables.y[lRun1 * 18 + 15];
nmpcWorkspace.Dy[lRun1 * 18 + 16] = nmpcWorkspace.objValueOut[16] - nmpcVariables.y[lRun1 * 18 + 16];
nmpcWorkspace.Dy[lRun1 * 18 + 17] = nmpcWorkspace.objValueOut[17] - nmpcVariables.y[lRun1 * 18 + 17];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[480];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[481];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[482];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[483];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[484];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[485];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[486];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[487];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[488];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[489];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[490];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[491];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[492];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[493];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[494];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[495];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[270];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[271];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[272];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[273];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[274];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[275];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[276];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[277];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.yN[8];
nmpcWorkspace.DyN[9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.yN[9];
nmpcWorkspace.DyN[10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.yN[10];
nmpcWorkspace.DyN[11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.yN[11];
nmpcWorkspace.DyN[12] = nmpcWorkspace.objValueOut[12] - nmpcVariables.yN[12];
nmpcWorkspace.DyN[13] = nmpcWorkspace.objValueOut[13] - nmpcVariables.yN[13];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 18]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 18 + 1]*nmpcVariables.W[19];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 18 + 2]*nmpcVariables.W[38];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 18 + 3]*nmpcVariables.W[57];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 18 + 4]*nmpcVariables.W[76];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 18 + 5]*nmpcVariables.W[95];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 18 + 6]*nmpcVariables.W[114];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 18 + 7]*nmpcVariables.W[133];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 18 + 8]*nmpcVariables.W[152];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 18 + 9]*nmpcVariables.W[171];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 18 + 10]*nmpcVariables.W[190];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 18 + 11]*nmpcVariables.W[209];
tmpDy[12] = + nmpcWorkspace.Dy[lRun1 * 18 + 12]*nmpcVariables.W[228];
tmpDy[13] = + nmpcWorkspace.Dy[lRun1 * 18 + 13]*nmpcVariables.W[247];
tmpDy[14] = + nmpcWorkspace.Dy[lRun1 * 18 + 14]*nmpcVariables.W[266];
tmpDy[15] = + nmpcWorkspace.Dy[lRun1 * 18 + 15]*nmpcVariables.W[285];
tmpDy[16] = + nmpcWorkspace.Dy[lRun1 * 18 + 16]*nmpcVariables.W[304];
tmpDy[17] = + nmpcWorkspace.Dy[lRun1 * 18 + 17]*nmpcVariables.W[323];
objVal += + nmpcWorkspace.Dy[lRun1 * 18]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 18 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 18 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 18 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 18 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 18 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 18 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 18 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 18 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 18 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 18 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 18 + 11]*tmpDy[11] + nmpcWorkspace.Dy[lRun1 * 18 + 12]*tmpDy[12] + nmpcWorkspace.Dy[lRun1 * 18 + 13]*tmpDy[13] + nmpcWorkspace.Dy[lRun1 * 18 + 14]*tmpDy[14] + nmpcWorkspace.Dy[lRun1 * 18 + 15]*tmpDy[15] + nmpcWorkspace.Dy[lRun1 * 18 + 16]*tmpDy[16] + nmpcWorkspace.Dy[lRun1 * 18 + 17]*tmpDy[17];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[15];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[30];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[45];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[60];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[75];
tmpDyN[6] = + nmpcWorkspace.DyN[6]*nmpcVariables.WN[90];
tmpDyN[7] = + nmpcWorkspace.DyN[7]*nmpcVariables.WN[105];
tmpDyN[8] = + nmpcWorkspace.DyN[8]*nmpcVariables.WN[120];
tmpDyN[9] = + nmpcWorkspace.DyN[9]*nmpcVariables.WN[135];
tmpDyN[10] = + nmpcWorkspace.DyN[10]*nmpcVariables.WN[150];
tmpDyN[11] = + nmpcWorkspace.DyN[11]*nmpcVariables.WN[165];
tmpDyN[12] = + nmpcWorkspace.DyN[12]*nmpcVariables.WN[180];
tmpDyN[13] = + nmpcWorkspace.DyN[13]*nmpcVariables.WN[195];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5] + nmpcWorkspace.DyN[6]*tmpDyN[6] + nmpcWorkspace.DyN[7]*tmpDyN[7] + nmpcWorkspace.DyN[8]*tmpDyN[8] + nmpcWorkspace.DyN[9]*tmpDyN[9] + nmpcWorkspace.DyN[10]*tmpDyN[10] + nmpcWorkspace.DyN[11]*tmpDyN[11] + nmpcWorkspace.DyN[12]*tmpDyN[12] + nmpcWorkspace.DyN[13]*tmpDyN[13];

objVal *= 0.5;
return objVal;
}

