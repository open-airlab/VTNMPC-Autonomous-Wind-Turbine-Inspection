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


void nmpc_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 13;
const real_t* od = in + 17;

/* Compute outputs: */
out[0] = xd[7];
out[1] = xd[8];
out[2] = xd[9];
out[3] = ((real_t)(5.0000000000000000e-01)*(((u[0]*xd[6])+(u[2]*xd[4]))-(u[1]*xd[5])));
out[4] = ((real_t)(5.0000000000000000e-01)*(((u[1]*xd[6])-(u[2]*xd[3]))+(u[0]*xd[5])));
out[5] = ((real_t)(5.0000000000000000e-01)*(((u[2]*xd[6])+(u[1]*xd[3]))-(u[0]*xd[4])));
out[6] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[0])*xd[3])-(u[1]*xd[4]))-(u[2]*xd[5])));
out[7] = ((((real_t)(2.0000000000000000e+00)*((xd[6]*xd[4])+(xd[3]*xd[5])))*u[3])*(real_t)(2.6315789473684209e-01));
out[8] = ((((real_t)(2.0000000000000000e+00)*((xd[4]*xd[5])-(xd[6]*xd[3])))*u[3])*(real_t)(2.6315789473684209e-01));
out[9] = ((((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[3])*xd[3]))-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))*u[3])*(real_t)(2.6315789473684209e-01))-(real_t)(9.8100000000000005e+00));
out[10] = od[3];
out[11] = od[4];
out[12] = od[5];
}



void nmpc_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 13;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
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
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
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
out[43] = (real_t)(1.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[56] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[57] = ((real_t)(5.0000000000000000e-01)*u[0]);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[65] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[66] = ((real_t)(5.0000000000000000e-01)*xd[4]);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = ((real_t)(5.0000000000000000e-01)*u[0]);
out[74] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = ((real_t)(5.0000000000000000e-01)*xd[5]);
out[82] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[83] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[3]));
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[89] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[0]));
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[4]));
out[99] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[100] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[0]));
out[106] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[107] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[3]));
out[116] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[4]));
out[117] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = ((((real_t)(2.0000000000000000e+00)*xd[5])*u[3])*(real_t)(2.6315789473684209e-01));
out[123] = ((((real_t)(2.0000000000000000e+00)*xd[6])*u[3])*(real_t)(2.6315789473684209e-01));
out[124] = ((((real_t)(2.0000000000000000e+00)*xd[3])*u[3])*(real_t)(2.6315789473684209e-01));
out[125] = ((((real_t)(2.0000000000000000e+00)*xd[4])*u[3])*(real_t)(2.6315789473684209e-01));
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (((real_t)(2.0000000000000000e+00)*((xd[6]*xd[4])+(xd[3]*xd[5])))*(real_t)(2.6315789473684209e-01));
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = ((((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[6]))*u[3])*(real_t)(2.6315789473684209e-01));
out[140] = ((((real_t)(2.0000000000000000e+00)*xd[5])*u[3])*(real_t)(2.6315789473684209e-01));
out[141] = ((((real_t)(2.0000000000000000e+00)*xd[4])*u[3])*(real_t)(2.6315789473684209e-01));
out[142] = ((((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[3]))*u[3])*(real_t)(2.6315789473684209e-01));
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (((real_t)(2.0000000000000000e+00)*((xd[4]*xd[5])-(xd[6]*xd[3])))*(real_t)(2.6315789473684209e-01));
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[3])+((real_t)(2.0000000000000000e+00)*xd[3])))*u[3])*(real_t)(2.6315789473684209e-01));
out[157] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])+((real_t)(2.0000000000000000e+00)*xd[4])))*u[3])*(real_t)(2.6315789473684209e-01));
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
out[169] = ((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[3])*xd[3]))-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))*(real_t)(2.6315789473684209e-01));
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(0.0000000000000000e+00);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = (real_t)(0.0000000000000000e+00);
out[210] = (real_t)(0.0000000000000000e+00);
out[211] = (real_t)(0.0000000000000000e+00);
out[212] = (real_t)(0.0000000000000000e+00);
out[213] = (real_t)(0.0000000000000000e+00);
out[214] = (real_t)(0.0000000000000000e+00);
out[215] = (real_t)(0.0000000000000000e+00);
out[216] = (real_t)(0.0000000000000000e+00);
out[217] = (real_t)(0.0000000000000000e+00);
out[218] = (real_t)(0.0000000000000000e+00);
out[219] = (real_t)(0.0000000000000000e+00);
out[220] = (real_t)(0.0000000000000000e+00);
}



void nmpc_solve_dim26_triangular( real_t* const A, real_t* const b )
{

b[25] = b[25]/A[675];
b[24] -= + A[649]*b[25];
b[24] = b[24]/A[648];
b[23] -= + A[623]*b[25];
b[23] -= + A[622]*b[24];
b[23] = b[23]/A[621];
b[22] -= + A[597]*b[25];
b[22] -= + A[596]*b[24];
b[22] -= + A[595]*b[23];
b[22] = b[22]/A[594];
b[21] -= + A[571]*b[25];
b[21] -= + A[570]*b[24];
b[21] -= + A[569]*b[23];
b[21] -= + A[568]*b[22];
b[21] = b[21]/A[567];
b[20] -= + A[545]*b[25];
b[20] -= + A[544]*b[24];
b[20] -= + A[543]*b[23];
b[20] -= + A[542]*b[22];
b[20] -= + A[541]*b[21];
b[20] = b[20]/A[540];
b[19] -= + A[519]*b[25];
b[19] -= + A[518]*b[24];
b[19] -= + A[517]*b[23];
b[19] -= + A[516]*b[22];
b[19] -= + A[515]*b[21];
b[19] -= + A[514]*b[20];
b[19] = b[19]/A[513];
b[18] -= + A[493]*b[25];
b[18] -= + A[492]*b[24];
b[18] -= + A[491]*b[23];
b[18] -= + A[490]*b[22];
b[18] -= + A[489]*b[21];
b[18] -= + A[488]*b[20];
b[18] -= + A[487]*b[19];
b[18] = b[18]/A[486];
b[17] -= + A[467]*b[25];
b[17] -= + A[466]*b[24];
b[17] -= + A[465]*b[23];
b[17] -= + A[464]*b[22];
b[17] -= + A[463]*b[21];
b[17] -= + A[462]*b[20];
b[17] -= + A[461]*b[19];
b[17] -= + A[460]*b[18];
b[17] = b[17]/A[459];
b[16] -= + A[441]*b[25];
b[16] -= + A[440]*b[24];
b[16] -= + A[439]*b[23];
b[16] -= + A[438]*b[22];
b[16] -= + A[437]*b[21];
b[16] -= + A[436]*b[20];
b[16] -= + A[435]*b[19];
b[16] -= + A[434]*b[18];
b[16] -= + A[433]*b[17];
b[16] = b[16]/A[432];
b[15] -= + A[415]*b[25];
b[15] -= + A[414]*b[24];
b[15] -= + A[413]*b[23];
b[15] -= + A[412]*b[22];
b[15] -= + A[411]*b[21];
b[15] -= + A[410]*b[20];
b[15] -= + A[409]*b[19];
b[15] -= + A[408]*b[18];
b[15] -= + A[407]*b[17];
b[15] -= + A[406]*b[16];
b[15] = b[15]/A[405];
b[14] -= + A[389]*b[25];
b[14] -= + A[388]*b[24];
b[14] -= + A[387]*b[23];
b[14] -= + A[386]*b[22];
b[14] -= + A[385]*b[21];
b[14] -= + A[384]*b[20];
b[14] -= + A[383]*b[19];
b[14] -= + A[382]*b[18];
b[14] -= + A[381]*b[17];
b[14] -= + A[380]*b[16];
b[14] -= + A[379]*b[15];
b[14] = b[14]/A[378];
b[13] -= + A[363]*b[25];
b[13] -= + A[362]*b[24];
b[13] -= + A[361]*b[23];
b[13] -= + A[360]*b[22];
b[13] -= + A[359]*b[21];
b[13] -= + A[358]*b[20];
b[13] -= + A[357]*b[19];
b[13] -= + A[356]*b[18];
b[13] -= + A[355]*b[17];
b[13] -= + A[354]*b[16];
b[13] -= + A[353]*b[15];
b[13] -= + A[352]*b[14];
b[13] = b[13]/A[351];
b[12] -= + A[337]*b[25];
b[12] -= + A[336]*b[24];
b[12] -= + A[335]*b[23];
b[12] -= + A[334]*b[22];
b[12] -= + A[333]*b[21];
b[12] -= + A[332]*b[20];
b[12] -= + A[331]*b[19];
b[12] -= + A[330]*b[18];
b[12] -= + A[329]*b[17];
b[12] -= + A[328]*b[16];
b[12] -= + A[327]*b[15];
b[12] -= + A[326]*b[14];
b[12] -= + A[325]*b[13];
b[12] = b[12]/A[324];
b[11] -= + A[311]*b[25];
b[11] -= + A[310]*b[24];
b[11] -= + A[309]*b[23];
b[11] -= + A[308]*b[22];
b[11] -= + A[307]*b[21];
b[11] -= + A[306]*b[20];
b[11] -= + A[305]*b[19];
b[11] -= + A[304]*b[18];
b[11] -= + A[303]*b[17];
b[11] -= + A[302]*b[16];
b[11] -= + A[301]*b[15];
b[11] -= + A[300]*b[14];
b[11] -= + A[299]*b[13];
b[11] -= + A[298]*b[12];
b[11] = b[11]/A[297];
b[10] -= + A[285]*b[25];
b[10] -= + A[284]*b[24];
b[10] -= + A[283]*b[23];
b[10] -= + A[282]*b[22];
b[10] -= + A[281]*b[21];
b[10] -= + A[280]*b[20];
b[10] -= + A[279]*b[19];
b[10] -= + A[278]*b[18];
b[10] -= + A[277]*b[17];
b[10] -= + A[276]*b[16];
b[10] -= + A[275]*b[15];
b[10] -= + A[274]*b[14];
b[10] -= + A[273]*b[13];
b[10] -= + A[272]*b[12];
b[10] -= + A[271]*b[11];
b[10] = b[10]/A[270];
b[9] -= + A[259]*b[25];
b[9] -= + A[258]*b[24];
b[9] -= + A[257]*b[23];
b[9] -= + A[256]*b[22];
b[9] -= + A[255]*b[21];
b[9] -= + A[254]*b[20];
b[9] -= + A[253]*b[19];
b[9] -= + A[252]*b[18];
b[9] -= + A[251]*b[17];
b[9] -= + A[250]*b[16];
b[9] -= + A[249]*b[15];
b[9] -= + A[248]*b[14];
b[9] -= + A[247]*b[13];
b[9] -= + A[246]*b[12];
b[9] -= + A[245]*b[11];
b[9] -= + A[244]*b[10];
b[9] = b[9]/A[243];
b[8] -= + A[233]*b[25];
b[8] -= + A[232]*b[24];
b[8] -= + A[231]*b[23];
b[8] -= + A[230]*b[22];
b[8] -= + A[229]*b[21];
b[8] -= + A[228]*b[20];
b[8] -= + A[227]*b[19];
b[8] -= + A[226]*b[18];
b[8] -= + A[225]*b[17];
b[8] -= + A[224]*b[16];
b[8] -= + A[223]*b[15];
b[8] -= + A[222]*b[14];
b[8] -= + A[221]*b[13];
b[8] -= + A[220]*b[12];
b[8] -= + A[219]*b[11];
b[8] -= + A[218]*b[10];
b[8] -= + A[217]*b[9];
b[8] = b[8]/A[216];
b[7] -= + A[207]*b[25];
b[7] -= + A[206]*b[24];
b[7] -= + A[205]*b[23];
b[7] -= + A[204]*b[22];
b[7] -= + A[203]*b[21];
b[7] -= + A[202]*b[20];
b[7] -= + A[201]*b[19];
b[7] -= + A[200]*b[18];
b[7] -= + A[199]*b[17];
b[7] -= + A[198]*b[16];
b[7] -= + A[197]*b[15];
b[7] -= + A[196]*b[14];
b[7] -= + A[195]*b[13];
b[7] -= + A[194]*b[12];
b[7] -= + A[193]*b[11];
b[7] -= + A[192]*b[10];
b[7] -= + A[191]*b[9];
b[7] -= + A[190]*b[8];
b[7] = b[7]/A[189];
b[6] -= + A[181]*b[25];
b[6] -= + A[180]*b[24];
b[6] -= + A[179]*b[23];
b[6] -= + A[178]*b[22];
b[6] -= + A[177]*b[21];
b[6] -= + A[176]*b[20];
b[6] -= + A[175]*b[19];
b[6] -= + A[174]*b[18];
b[6] -= + A[173]*b[17];
b[6] -= + A[172]*b[16];
b[6] -= + A[171]*b[15];
b[6] -= + A[170]*b[14];
b[6] -= + A[169]*b[13];
b[6] -= + A[168]*b[12];
b[6] -= + A[167]*b[11];
b[6] -= + A[166]*b[10];
b[6] -= + A[165]*b[9];
b[6] -= + A[164]*b[8];
b[6] -= + A[163]*b[7];
b[6] = b[6]/A[162];
b[5] -= + A[155]*b[25];
b[5] -= + A[154]*b[24];
b[5] -= + A[153]*b[23];
b[5] -= + A[152]*b[22];
b[5] -= + A[151]*b[21];
b[5] -= + A[150]*b[20];
b[5] -= + A[149]*b[19];
b[5] -= + A[148]*b[18];
b[5] -= + A[147]*b[17];
b[5] -= + A[146]*b[16];
b[5] -= + A[145]*b[15];
b[5] -= + A[144]*b[14];
b[5] -= + A[143]*b[13];
b[5] -= + A[142]*b[12];
b[5] -= + A[141]*b[11];
b[5] -= + A[140]*b[10];
b[5] -= + A[139]*b[9];
b[5] -= + A[138]*b[8];
b[5] -= + A[137]*b[7];
b[5] -= + A[136]*b[6];
b[5] = b[5]/A[135];
b[4] -= + A[129]*b[25];
b[4] -= + A[128]*b[24];
b[4] -= + A[127]*b[23];
b[4] -= + A[126]*b[22];
b[4] -= + A[125]*b[21];
b[4] -= + A[124]*b[20];
b[4] -= + A[123]*b[19];
b[4] -= + A[122]*b[18];
b[4] -= + A[121]*b[17];
b[4] -= + A[120]*b[16];
b[4] -= + A[119]*b[15];
b[4] -= + A[118]*b[14];
b[4] -= + A[117]*b[13];
b[4] -= + A[116]*b[12];
b[4] -= + A[115]*b[11];
b[4] -= + A[114]*b[10];
b[4] -= + A[113]*b[9];
b[4] -= + A[112]*b[8];
b[4] -= + A[111]*b[7];
b[4] -= + A[110]*b[6];
b[4] -= + A[109]*b[5];
b[4] = b[4]/A[108];
b[3] -= + A[103]*b[25];
b[3] -= + A[102]*b[24];
b[3] -= + A[101]*b[23];
b[3] -= + A[100]*b[22];
b[3] -= + A[99]*b[21];
b[3] -= + A[98]*b[20];
b[3] -= + A[97]*b[19];
b[3] -= + A[96]*b[18];
b[3] -= + A[95]*b[17];
b[3] -= + A[94]*b[16];
b[3] -= + A[93]*b[15];
b[3] -= + A[92]*b[14];
b[3] -= + A[91]*b[13];
b[3] -= + A[90]*b[12];
b[3] -= + A[89]*b[11];
b[3] -= + A[88]*b[10];
b[3] -= + A[87]*b[9];
b[3] -= + A[86]*b[8];
b[3] -= + A[85]*b[7];
b[3] -= + A[84]*b[6];
b[3] -= + A[83]*b[5];
b[3] -= + A[82]*b[4];
b[3] = b[3]/A[81];
b[2] -= + A[77]*b[25];
b[2] -= + A[76]*b[24];
b[2] -= + A[75]*b[23];
b[2] -= + A[74]*b[22];
b[2] -= + A[73]*b[21];
b[2] -= + A[72]*b[20];
b[2] -= + A[71]*b[19];
b[2] -= + A[70]*b[18];
b[2] -= + A[69]*b[17];
b[2] -= + A[68]*b[16];
b[2] -= + A[67]*b[15];
b[2] -= + A[66]*b[14];
b[2] -= + A[65]*b[13];
b[2] -= + A[64]*b[12];
b[2] -= + A[63]*b[11];
b[2] -= + A[62]*b[10];
b[2] -= + A[61]*b[9];
b[2] -= + A[60]*b[8];
b[2] -= + A[59]*b[7];
b[2] -= + A[58]*b[6];
b[2] -= + A[57]*b[5];
b[2] -= + A[56]*b[4];
b[2] -= + A[55]*b[3];
b[2] = b[2]/A[54];
b[1] -= + A[51]*b[25];
b[1] -= + A[50]*b[24];
b[1] -= + A[49]*b[23];
b[1] -= + A[48]*b[22];
b[1] -= + A[47]*b[21];
b[1] -= + A[46]*b[20];
b[1] -= + A[45]*b[19];
b[1] -= + A[44]*b[18];
b[1] -= + A[43]*b[17];
b[1] -= + A[42]*b[16];
b[1] -= + A[41]*b[15];
b[1] -= + A[40]*b[14];
b[1] -= + A[39]*b[13];
b[1] -= + A[38]*b[12];
b[1] -= + A[37]*b[11];
b[1] -= + A[36]*b[10];
b[1] -= + A[35]*b[9];
b[1] -= + A[34]*b[8];
b[1] -= + A[33]*b[7];
b[1] -= + A[32]*b[6];
b[1] -= + A[31]*b[5];
b[1] -= + A[30]*b[4];
b[1] -= + A[29]*b[3];
b[1] -= + A[28]*b[2];
b[1] = b[1]/A[27];
b[0] -= + A[25]*b[25];
b[0] -= + A[24]*b[24];
b[0] -= + A[23]*b[23];
b[0] -= + A[22]*b[22];
b[0] -= + A[21]*b[21];
b[0] -= + A[20]*b[20];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t nmpc_solve_dim26_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 26; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (25); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*26+i]);
	for( j=(i+1); j < 26; j++ ) {
		temp = fabs(A[j*26+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 26; ++k)
{
	nmpcWorkspace.rk_dim26_swap = A[i*26+k];
	A[i*26+k] = A[indexMax*26+k];
	A[indexMax*26+k] = nmpcWorkspace.rk_dim26_swap;
}
	nmpcWorkspace.rk_dim26_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = nmpcWorkspace.rk_dim26_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*26+i];
	for( j=i+1; j < 26; j++ ) {
		A[j*26+i] = -A[j*26+i]/A[i*26+i];
		for( k=i+1; k < 26; k++ ) {
			A[j*26+k] += A[j*26+i] * A[i*26+k];
		}
		b[j] += A[j*26+i] * b[i];
	}
}
det *= A[675];
det = fabs(det);
nmpc_solve_dim26_triangular( A, b );
return det;
}

void nmpc_solve_dim26_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

nmpcWorkspace.rk_dim26_bPerm[0] = b[rk_perm[0]];
nmpcWorkspace.rk_dim26_bPerm[1] = b[rk_perm[1]];
nmpcWorkspace.rk_dim26_bPerm[2] = b[rk_perm[2]];
nmpcWorkspace.rk_dim26_bPerm[3] = b[rk_perm[3]];
nmpcWorkspace.rk_dim26_bPerm[4] = b[rk_perm[4]];
nmpcWorkspace.rk_dim26_bPerm[5] = b[rk_perm[5]];
nmpcWorkspace.rk_dim26_bPerm[6] = b[rk_perm[6]];
nmpcWorkspace.rk_dim26_bPerm[7] = b[rk_perm[7]];
nmpcWorkspace.rk_dim26_bPerm[8] = b[rk_perm[8]];
nmpcWorkspace.rk_dim26_bPerm[9] = b[rk_perm[9]];
nmpcWorkspace.rk_dim26_bPerm[10] = b[rk_perm[10]];
nmpcWorkspace.rk_dim26_bPerm[11] = b[rk_perm[11]];
nmpcWorkspace.rk_dim26_bPerm[12] = b[rk_perm[12]];
nmpcWorkspace.rk_dim26_bPerm[13] = b[rk_perm[13]];
nmpcWorkspace.rk_dim26_bPerm[14] = b[rk_perm[14]];
nmpcWorkspace.rk_dim26_bPerm[15] = b[rk_perm[15]];
nmpcWorkspace.rk_dim26_bPerm[16] = b[rk_perm[16]];
nmpcWorkspace.rk_dim26_bPerm[17] = b[rk_perm[17]];
nmpcWorkspace.rk_dim26_bPerm[18] = b[rk_perm[18]];
nmpcWorkspace.rk_dim26_bPerm[19] = b[rk_perm[19]];
nmpcWorkspace.rk_dim26_bPerm[20] = b[rk_perm[20]];
nmpcWorkspace.rk_dim26_bPerm[21] = b[rk_perm[21]];
nmpcWorkspace.rk_dim26_bPerm[22] = b[rk_perm[22]];
nmpcWorkspace.rk_dim26_bPerm[23] = b[rk_perm[23]];
nmpcWorkspace.rk_dim26_bPerm[24] = b[rk_perm[24]];
nmpcWorkspace.rk_dim26_bPerm[25] = b[rk_perm[25]];
nmpcWorkspace.rk_dim26_bPerm[1] += A[26]*nmpcWorkspace.rk_dim26_bPerm[0];

nmpcWorkspace.rk_dim26_bPerm[2] += A[52]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[2] += A[53]*nmpcWorkspace.rk_dim26_bPerm[1];

nmpcWorkspace.rk_dim26_bPerm[3] += A[78]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[3] += A[79]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[3] += A[80]*nmpcWorkspace.rk_dim26_bPerm[2];

nmpcWorkspace.rk_dim26_bPerm[4] += A[104]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[4] += A[105]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[4] += A[106]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[4] += A[107]*nmpcWorkspace.rk_dim26_bPerm[3];

nmpcWorkspace.rk_dim26_bPerm[5] += A[130]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[5] += A[131]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[5] += A[132]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[5] += A[133]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[5] += A[134]*nmpcWorkspace.rk_dim26_bPerm[4];

nmpcWorkspace.rk_dim26_bPerm[6] += A[156]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[6] += A[157]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[6] += A[158]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[6] += A[159]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[6] += A[160]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[6] += A[161]*nmpcWorkspace.rk_dim26_bPerm[5];

nmpcWorkspace.rk_dim26_bPerm[7] += A[182]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[7] += A[183]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[7] += A[184]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[7] += A[185]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[7] += A[186]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[7] += A[187]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[7] += A[188]*nmpcWorkspace.rk_dim26_bPerm[6];

nmpcWorkspace.rk_dim26_bPerm[8] += A[208]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[8] += A[209]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[8] += A[210]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[8] += A[211]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[8] += A[212]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[8] += A[213]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[8] += A[214]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[8] += A[215]*nmpcWorkspace.rk_dim26_bPerm[7];

nmpcWorkspace.rk_dim26_bPerm[9] += A[234]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[9] += A[235]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[9] += A[236]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[9] += A[237]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[9] += A[238]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[9] += A[239]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[9] += A[240]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[9] += A[241]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[9] += A[242]*nmpcWorkspace.rk_dim26_bPerm[8];

nmpcWorkspace.rk_dim26_bPerm[10] += A[260]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[10] += A[261]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[10] += A[262]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[10] += A[263]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[10] += A[264]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[10] += A[265]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[10] += A[266]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[10] += A[267]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[10] += A[268]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[10] += A[269]*nmpcWorkspace.rk_dim26_bPerm[9];

nmpcWorkspace.rk_dim26_bPerm[11] += A[286]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[11] += A[287]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[11] += A[288]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[11] += A[289]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[11] += A[290]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[11] += A[291]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[11] += A[292]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[11] += A[293]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[11] += A[294]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[11] += A[295]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[11] += A[296]*nmpcWorkspace.rk_dim26_bPerm[10];

nmpcWorkspace.rk_dim26_bPerm[12] += A[312]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[12] += A[313]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[12] += A[314]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[12] += A[315]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[12] += A[316]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[12] += A[317]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[12] += A[318]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[12] += A[319]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[12] += A[320]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[12] += A[321]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[12] += A[322]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[12] += A[323]*nmpcWorkspace.rk_dim26_bPerm[11];

nmpcWorkspace.rk_dim26_bPerm[13] += A[338]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[13] += A[339]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[13] += A[340]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[13] += A[341]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[13] += A[342]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[13] += A[343]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[13] += A[344]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[13] += A[345]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[13] += A[346]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[13] += A[347]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[13] += A[348]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[13] += A[349]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[13] += A[350]*nmpcWorkspace.rk_dim26_bPerm[12];

nmpcWorkspace.rk_dim26_bPerm[14] += A[364]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[14] += A[365]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[14] += A[366]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[14] += A[367]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[14] += A[368]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[14] += A[369]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[14] += A[370]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[14] += A[371]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[14] += A[372]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[14] += A[373]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[14] += A[374]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[14] += A[375]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[14] += A[376]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[14] += A[377]*nmpcWorkspace.rk_dim26_bPerm[13];

nmpcWorkspace.rk_dim26_bPerm[15] += A[390]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[15] += A[391]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[15] += A[392]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[15] += A[393]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[15] += A[394]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[15] += A[395]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[15] += A[396]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[15] += A[397]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[15] += A[398]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[15] += A[399]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[15] += A[400]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[15] += A[401]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[15] += A[402]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[15] += A[403]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[15] += A[404]*nmpcWorkspace.rk_dim26_bPerm[14];

nmpcWorkspace.rk_dim26_bPerm[16] += A[416]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[16] += A[417]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[16] += A[418]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[16] += A[419]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[16] += A[420]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[16] += A[421]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[16] += A[422]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[16] += A[423]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[16] += A[424]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[16] += A[425]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[16] += A[426]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[16] += A[427]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[16] += A[428]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[16] += A[429]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[16] += A[430]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[16] += A[431]*nmpcWorkspace.rk_dim26_bPerm[15];

nmpcWorkspace.rk_dim26_bPerm[17] += A[442]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[17] += A[443]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[17] += A[444]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[17] += A[445]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[17] += A[446]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[17] += A[447]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[17] += A[448]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[17] += A[449]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[17] += A[450]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[17] += A[451]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[17] += A[452]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[17] += A[453]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[17] += A[454]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[17] += A[455]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[17] += A[456]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[17] += A[457]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[17] += A[458]*nmpcWorkspace.rk_dim26_bPerm[16];

nmpcWorkspace.rk_dim26_bPerm[18] += A[468]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[18] += A[469]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[18] += A[470]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[18] += A[471]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[18] += A[472]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[18] += A[473]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[18] += A[474]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[18] += A[475]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[18] += A[476]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[18] += A[477]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[18] += A[478]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[18] += A[479]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[18] += A[480]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[18] += A[481]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[18] += A[482]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[18] += A[483]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[18] += A[484]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[18] += A[485]*nmpcWorkspace.rk_dim26_bPerm[17];

nmpcWorkspace.rk_dim26_bPerm[19] += A[494]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[19] += A[495]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[19] += A[496]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[19] += A[497]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[19] += A[498]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[19] += A[499]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[19] += A[500]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[19] += A[501]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[19] += A[502]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[19] += A[503]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[19] += A[504]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[19] += A[505]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[19] += A[506]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[19] += A[507]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[19] += A[508]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[19] += A[509]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[19] += A[510]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[19] += A[511]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[19] += A[512]*nmpcWorkspace.rk_dim26_bPerm[18];

nmpcWorkspace.rk_dim26_bPerm[20] += A[520]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[20] += A[521]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[20] += A[522]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[20] += A[523]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[20] += A[524]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[20] += A[525]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[20] += A[526]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[20] += A[527]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[20] += A[528]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[20] += A[529]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[20] += A[530]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[20] += A[531]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[20] += A[532]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[20] += A[533]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[20] += A[534]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[20] += A[535]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[20] += A[536]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[20] += A[537]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[20] += A[538]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[20] += A[539]*nmpcWorkspace.rk_dim26_bPerm[19];

nmpcWorkspace.rk_dim26_bPerm[21] += A[546]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[21] += A[547]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[21] += A[548]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[21] += A[549]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[21] += A[550]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[21] += A[551]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[21] += A[552]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[21] += A[553]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[21] += A[554]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[21] += A[555]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[21] += A[556]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[21] += A[557]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[21] += A[558]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[21] += A[559]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[21] += A[560]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[21] += A[561]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[21] += A[562]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[21] += A[563]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[21] += A[564]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[21] += A[565]*nmpcWorkspace.rk_dim26_bPerm[19];
nmpcWorkspace.rk_dim26_bPerm[21] += A[566]*nmpcWorkspace.rk_dim26_bPerm[20];

nmpcWorkspace.rk_dim26_bPerm[22] += A[572]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[22] += A[573]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[22] += A[574]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[22] += A[575]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[22] += A[576]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[22] += A[577]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[22] += A[578]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[22] += A[579]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[22] += A[580]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[22] += A[581]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[22] += A[582]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[22] += A[583]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[22] += A[584]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[22] += A[585]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[22] += A[586]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[22] += A[587]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[22] += A[588]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[22] += A[589]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[22] += A[590]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[22] += A[591]*nmpcWorkspace.rk_dim26_bPerm[19];
nmpcWorkspace.rk_dim26_bPerm[22] += A[592]*nmpcWorkspace.rk_dim26_bPerm[20];
nmpcWorkspace.rk_dim26_bPerm[22] += A[593]*nmpcWorkspace.rk_dim26_bPerm[21];

nmpcWorkspace.rk_dim26_bPerm[23] += A[598]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[23] += A[599]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[23] += A[600]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[23] += A[601]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[23] += A[602]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[23] += A[603]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[23] += A[604]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[23] += A[605]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[23] += A[606]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[23] += A[607]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[23] += A[608]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[23] += A[609]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[23] += A[610]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[23] += A[611]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[23] += A[612]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[23] += A[613]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[23] += A[614]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[23] += A[615]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[23] += A[616]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[23] += A[617]*nmpcWorkspace.rk_dim26_bPerm[19];
nmpcWorkspace.rk_dim26_bPerm[23] += A[618]*nmpcWorkspace.rk_dim26_bPerm[20];
nmpcWorkspace.rk_dim26_bPerm[23] += A[619]*nmpcWorkspace.rk_dim26_bPerm[21];
nmpcWorkspace.rk_dim26_bPerm[23] += A[620]*nmpcWorkspace.rk_dim26_bPerm[22];

nmpcWorkspace.rk_dim26_bPerm[24] += A[624]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[24] += A[625]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[24] += A[626]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[24] += A[627]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[24] += A[628]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[24] += A[629]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[24] += A[630]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[24] += A[631]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[24] += A[632]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[24] += A[633]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[24] += A[634]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[24] += A[635]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[24] += A[636]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[24] += A[637]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[24] += A[638]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[24] += A[639]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[24] += A[640]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[24] += A[641]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[24] += A[642]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[24] += A[643]*nmpcWorkspace.rk_dim26_bPerm[19];
nmpcWorkspace.rk_dim26_bPerm[24] += A[644]*nmpcWorkspace.rk_dim26_bPerm[20];
nmpcWorkspace.rk_dim26_bPerm[24] += A[645]*nmpcWorkspace.rk_dim26_bPerm[21];
nmpcWorkspace.rk_dim26_bPerm[24] += A[646]*nmpcWorkspace.rk_dim26_bPerm[22];
nmpcWorkspace.rk_dim26_bPerm[24] += A[647]*nmpcWorkspace.rk_dim26_bPerm[23];

nmpcWorkspace.rk_dim26_bPerm[25] += A[650]*nmpcWorkspace.rk_dim26_bPerm[0];
nmpcWorkspace.rk_dim26_bPerm[25] += A[651]*nmpcWorkspace.rk_dim26_bPerm[1];
nmpcWorkspace.rk_dim26_bPerm[25] += A[652]*nmpcWorkspace.rk_dim26_bPerm[2];
nmpcWorkspace.rk_dim26_bPerm[25] += A[653]*nmpcWorkspace.rk_dim26_bPerm[3];
nmpcWorkspace.rk_dim26_bPerm[25] += A[654]*nmpcWorkspace.rk_dim26_bPerm[4];
nmpcWorkspace.rk_dim26_bPerm[25] += A[655]*nmpcWorkspace.rk_dim26_bPerm[5];
nmpcWorkspace.rk_dim26_bPerm[25] += A[656]*nmpcWorkspace.rk_dim26_bPerm[6];
nmpcWorkspace.rk_dim26_bPerm[25] += A[657]*nmpcWorkspace.rk_dim26_bPerm[7];
nmpcWorkspace.rk_dim26_bPerm[25] += A[658]*nmpcWorkspace.rk_dim26_bPerm[8];
nmpcWorkspace.rk_dim26_bPerm[25] += A[659]*nmpcWorkspace.rk_dim26_bPerm[9];
nmpcWorkspace.rk_dim26_bPerm[25] += A[660]*nmpcWorkspace.rk_dim26_bPerm[10];
nmpcWorkspace.rk_dim26_bPerm[25] += A[661]*nmpcWorkspace.rk_dim26_bPerm[11];
nmpcWorkspace.rk_dim26_bPerm[25] += A[662]*nmpcWorkspace.rk_dim26_bPerm[12];
nmpcWorkspace.rk_dim26_bPerm[25] += A[663]*nmpcWorkspace.rk_dim26_bPerm[13];
nmpcWorkspace.rk_dim26_bPerm[25] += A[664]*nmpcWorkspace.rk_dim26_bPerm[14];
nmpcWorkspace.rk_dim26_bPerm[25] += A[665]*nmpcWorkspace.rk_dim26_bPerm[15];
nmpcWorkspace.rk_dim26_bPerm[25] += A[666]*nmpcWorkspace.rk_dim26_bPerm[16];
nmpcWorkspace.rk_dim26_bPerm[25] += A[667]*nmpcWorkspace.rk_dim26_bPerm[17];
nmpcWorkspace.rk_dim26_bPerm[25] += A[668]*nmpcWorkspace.rk_dim26_bPerm[18];
nmpcWorkspace.rk_dim26_bPerm[25] += A[669]*nmpcWorkspace.rk_dim26_bPerm[19];
nmpcWorkspace.rk_dim26_bPerm[25] += A[670]*nmpcWorkspace.rk_dim26_bPerm[20];
nmpcWorkspace.rk_dim26_bPerm[25] += A[671]*nmpcWorkspace.rk_dim26_bPerm[21];
nmpcWorkspace.rk_dim26_bPerm[25] += A[672]*nmpcWorkspace.rk_dim26_bPerm[22];
nmpcWorkspace.rk_dim26_bPerm[25] += A[673]*nmpcWorkspace.rk_dim26_bPerm[23];
nmpcWorkspace.rk_dim26_bPerm[25] += A[674]*nmpcWorkspace.rk_dim26_bPerm[24];


nmpc_solve_dim26_triangular( A, nmpcWorkspace.rk_dim26_bPerm );
b[0] = nmpcWorkspace.rk_dim26_bPerm[0];
b[1] = nmpcWorkspace.rk_dim26_bPerm[1];
b[2] = nmpcWorkspace.rk_dim26_bPerm[2];
b[3] = nmpcWorkspace.rk_dim26_bPerm[3];
b[4] = nmpcWorkspace.rk_dim26_bPerm[4];
b[5] = nmpcWorkspace.rk_dim26_bPerm[5];
b[6] = nmpcWorkspace.rk_dim26_bPerm[6];
b[7] = nmpcWorkspace.rk_dim26_bPerm[7];
b[8] = nmpcWorkspace.rk_dim26_bPerm[8];
b[9] = nmpcWorkspace.rk_dim26_bPerm[9];
b[10] = nmpcWorkspace.rk_dim26_bPerm[10];
b[11] = nmpcWorkspace.rk_dim26_bPerm[11];
b[12] = nmpcWorkspace.rk_dim26_bPerm[12];
b[13] = nmpcWorkspace.rk_dim26_bPerm[13];
b[14] = nmpcWorkspace.rk_dim26_bPerm[14];
b[15] = nmpcWorkspace.rk_dim26_bPerm[15];
b[16] = nmpcWorkspace.rk_dim26_bPerm[16];
b[17] = nmpcWorkspace.rk_dim26_bPerm[17];
b[18] = nmpcWorkspace.rk_dim26_bPerm[18];
b[19] = nmpcWorkspace.rk_dim26_bPerm[19];
b[20] = nmpcWorkspace.rk_dim26_bPerm[20];
b[21] = nmpcWorkspace.rk_dim26_bPerm[21];
b[22] = nmpcWorkspace.rk_dim26_bPerm[22];
b[23] = nmpcWorkspace.rk_dim26_bPerm[23];
b[24] = nmpcWorkspace.rk_dim26_bPerm[24];
b[25] = nmpcWorkspace.rk_dim26_bPerm[25];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t nmpc_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
int nmpc_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

nmpcWorkspace.rk_ttt = 0.0000000000000000e+00;
nmpcWorkspace.rk_xxx[13] = rk_eta[234];
nmpcWorkspace.rk_xxx[14] = rk_eta[235];
nmpcWorkspace.rk_xxx[15] = rk_eta[236];
nmpcWorkspace.rk_xxx[16] = rk_eta[237];
nmpcWorkspace.rk_xxx[17] = rk_eta[238];
nmpcWorkspace.rk_xxx[18] = rk_eta[239];
nmpcWorkspace.rk_xxx[19] = rk_eta[240];
nmpcWorkspace.rk_xxx[20] = rk_eta[241];
nmpcWorkspace.rk_xxx[21] = rk_eta[242];
nmpcWorkspace.rk_xxx[22] = rk_eta[243];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 13; ++j)
{
nmpcWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_kkk[tmp_index1 * 2];
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
nmpc_diffs( nmpcWorkspace.rk_xxx, &(nmpcWorkspace.rk_diffsTemp2[ run1 * 221 ]) );
for (j = 0; j < 13; ++j)
{
tmp_index1 = (run1 * 13) + (j);
nmpcWorkspace.rk_A[tmp_index1 * 26] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 1] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 1)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 2] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 2)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 3] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 3)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 4] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 4)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 5] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 5)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 6] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 6)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 7] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 7)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 8] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 8)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 9] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 9)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 10] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 10)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 11] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 11)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 12] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 12)];
if( 0 == run1 ) nmpcWorkspace.rk_A[(tmp_index1 * 26) + (j)] -= 1.0000000000000000e+00;
nmpcWorkspace.rk_A[tmp_index1 * 26 + 13] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 14] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 1)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 15] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 2)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 16] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 3)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 17] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 4)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 18] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 5)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 19] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 6)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 20] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 7)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 21] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 8)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 22] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 9)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 23] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 10)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 24] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 11)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 25] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 12)];
if( 1 == run1 ) nmpcWorkspace.rk_A[(tmp_index1 * 26) + (j + 13)] -= 1.0000000000000000e+00;
}
nmpc_rhs( nmpcWorkspace.rk_xxx, nmpcWorkspace.rk_rhsTemp );
nmpcWorkspace.rk_b[run1 * 13] = nmpcWorkspace.rk_kkk[run1] - nmpcWorkspace.rk_rhsTemp[0];
nmpcWorkspace.rk_b[run1 * 13 + 1] = nmpcWorkspace.rk_kkk[run1 + 2] - nmpcWorkspace.rk_rhsTemp[1];
nmpcWorkspace.rk_b[run1 * 13 + 2] = nmpcWorkspace.rk_kkk[run1 + 4] - nmpcWorkspace.rk_rhsTemp[2];
nmpcWorkspace.rk_b[run1 * 13 + 3] = nmpcWorkspace.rk_kkk[run1 + 6] - nmpcWorkspace.rk_rhsTemp[3];
nmpcWorkspace.rk_b[run1 * 13 + 4] = nmpcWorkspace.rk_kkk[run1 + 8] - nmpcWorkspace.rk_rhsTemp[4];
nmpcWorkspace.rk_b[run1 * 13 + 5] = nmpcWorkspace.rk_kkk[run1 + 10] - nmpcWorkspace.rk_rhsTemp[5];
nmpcWorkspace.rk_b[run1 * 13 + 6] = nmpcWorkspace.rk_kkk[run1 + 12] - nmpcWorkspace.rk_rhsTemp[6];
nmpcWorkspace.rk_b[run1 * 13 + 7] = nmpcWorkspace.rk_kkk[run1 + 14] - nmpcWorkspace.rk_rhsTemp[7];
nmpcWorkspace.rk_b[run1 * 13 + 8] = nmpcWorkspace.rk_kkk[run1 + 16] - nmpcWorkspace.rk_rhsTemp[8];
nmpcWorkspace.rk_b[run1 * 13 + 9] = nmpcWorkspace.rk_kkk[run1 + 18] - nmpcWorkspace.rk_rhsTemp[9];
nmpcWorkspace.rk_b[run1 * 13 + 10] = nmpcWorkspace.rk_kkk[run1 + 20] - nmpcWorkspace.rk_rhsTemp[10];
nmpcWorkspace.rk_b[run1 * 13 + 11] = nmpcWorkspace.rk_kkk[run1 + 22] - nmpcWorkspace.rk_rhsTemp[11];
nmpcWorkspace.rk_b[run1 * 13 + 12] = nmpcWorkspace.rk_kkk[run1 + 24] - nmpcWorkspace.rk_rhsTemp[12];
}
det = nmpc_solve_dim26_system( nmpcWorkspace.rk_A, nmpcWorkspace.rk_b, nmpcWorkspace.rk_dim26_perm );
for (j = 0; j < 2; ++j)
{
nmpcWorkspace.rk_kkk[j] += nmpcWorkspace.rk_b[j * 13];
nmpcWorkspace.rk_kkk[j + 2] += nmpcWorkspace.rk_b[j * 13 + 1];
nmpcWorkspace.rk_kkk[j + 4] += nmpcWorkspace.rk_b[j * 13 + 2];
nmpcWorkspace.rk_kkk[j + 6] += nmpcWorkspace.rk_b[j * 13 + 3];
nmpcWorkspace.rk_kkk[j + 8] += nmpcWorkspace.rk_b[j * 13 + 4];
nmpcWorkspace.rk_kkk[j + 10] += nmpcWorkspace.rk_b[j * 13 + 5];
nmpcWorkspace.rk_kkk[j + 12] += nmpcWorkspace.rk_b[j * 13 + 6];
nmpcWorkspace.rk_kkk[j + 14] += nmpcWorkspace.rk_b[j * 13 + 7];
nmpcWorkspace.rk_kkk[j + 16] += nmpcWorkspace.rk_b[j * 13 + 8];
nmpcWorkspace.rk_kkk[j + 18] += nmpcWorkspace.rk_b[j * 13 + 9];
nmpcWorkspace.rk_kkk[j + 20] += nmpcWorkspace.rk_b[j * 13 + 10];
nmpcWorkspace.rk_kkk[j + 22] += nmpcWorkspace.rk_b[j * 13 + 11];
nmpcWorkspace.rk_kkk[j + 24] += nmpcWorkspace.rk_b[j * 13 + 12];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 13; ++j)
{
nmpcWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_kkk[tmp_index1 * 2];
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
nmpc_rhs( nmpcWorkspace.rk_xxx, nmpcWorkspace.rk_rhsTemp );
nmpcWorkspace.rk_b[run1 * 13] = nmpcWorkspace.rk_kkk[run1] - nmpcWorkspace.rk_rhsTemp[0];
nmpcWorkspace.rk_b[run1 * 13 + 1] = nmpcWorkspace.rk_kkk[run1 + 2] - nmpcWorkspace.rk_rhsTemp[1];
nmpcWorkspace.rk_b[run1 * 13 + 2] = nmpcWorkspace.rk_kkk[run1 + 4] - nmpcWorkspace.rk_rhsTemp[2];
nmpcWorkspace.rk_b[run1 * 13 + 3] = nmpcWorkspace.rk_kkk[run1 + 6] - nmpcWorkspace.rk_rhsTemp[3];
nmpcWorkspace.rk_b[run1 * 13 + 4] = nmpcWorkspace.rk_kkk[run1 + 8] - nmpcWorkspace.rk_rhsTemp[4];
nmpcWorkspace.rk_b[run1 * 13 + 5] = nmpcWorkspace.rk_kkk[run1 + 10] - nmpcWorkspace.rk_rhsTemp[5];
nmpcWorkspace.rk_b[run1 * 13 + 6] = nmpcWorkspace.rk_kkk[run1 + 12] - nmpcWorkspace.rk_rhsTemp[6];
nmpcWorkspace.rk_b[run1 * 13 + 7] = nmpcWorkspace.rk_kkk[run1 + 14] - nmpcWorkspace.rk_rhsTemp[7];
nmpcWorkspace.rk_b[run1 * 13 + 8] = nmpcWorkspace.rk_kkk[run1 + 16] - nmpcWorkspace.rk_rhsTemp[8];
nmpcWorkspace.rk_b[run1 * 13 + 9] = nmpcWorkspace.rk_kkk[run1 + 18] - nmpcWorkspace.rk_rhsTemp[9];
nmpcWorkspace.rk_b[run1 * 13 + 10] = nmpcWorkspace.rk_kkk[run1 + 20] - nmpcWorkspace.rk_rhsTemp[10];
nmpcWorkspace.rk_b[run1 * 13 + 11] = nmpcWorkspace.rk_kkk[run1 + 22] - nmpcWorkspace.rk_rhsTemp[11];
nmpcWorkspace.rk_b[run1 * 13 + 12] = nmpcWorkspace.rk_kkk[run1 + 24] - nmpcWorkspace.rk_rhsTemp[12];
}
nmpc_solve_dim26_system_reuse( nmpcWorkspace.rk_A, nmpcWorkspace.rk_b, nmpcWorkspace.rk_dim26_perm );
for (j = 0; j < 2; ++j)
{
nmpcWorkspace.rk_kkk[j] += nmpcWorkspace.rk_b[j * 13];
nmpcWorkspace.rk_kkk[j + 2] += nmpcWorkspace.rk_b[j * 13 + 1];
nmpcWorkspace.rk_kkk[j + 4] += nmpcWorkspace.rk_b[j * 13 + 2];
nmpcWorkspace.rk_kkk[j + 6] += nmpcWorkspace.rk_b[j * 13 + 3];
nmpcWorkspace.rk_kkk[j + 8] += nmpcWorkspace.rk_b[j * 13 + 4];
nmpcWorkspace.rk_kkk[j + 10] += nmpcWorkspace.rk_b[j * 13 + 5];
nmpcWorkspace.rk_kkk[j + 12] += nmpcWorkspace.rk_b[j * 13 + 6];
nmpcWorkspace.rk_kkk[j + 14] += nmpcWorkspace.rk_b[j * 13 + 7];
nmpcWorkspace.rk_kkk[j + 16] += nmpcWorkspace.rk_b[j * 13 + 8];
nmpcWorkspace.rk_kkk[j + 18] += nmpcWorkspace.rk_b[j * 13 + 9];
nmpcWorkspace.rk_kkk[j + 20] += nmpcWorkspace.rk_b[j * 13 + 10];
nmpcWorkspace.rk_kkk[j + 22] += nmpcWorkspace.rk_b[j * 13 + 11];
nmpcWorkspace.rk_kkk[j + 24] += nmpcWorkspace.rk_b[j * 13 + 12];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 13; ++j)
{
nmpcWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_kkk[tmp_index1 * 2];
nmpcWorkspace.rk_xxx[j] += + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
nmpc_diffs( nmpcWorkspace.rk_xxx, &(nmpcWorkspace.rk_diffsTemp2[ run1 * 221 ]) );
for (j = 0; j < 13; ++j)
{
tmp_index1 = (run1 * 13) + (j);
nmpcWorkspace.rk_A[tmp_index1 * 26] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 1] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 1)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 2] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 2)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 3] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 3)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 4] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 4)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 5] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 5)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 6] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 6)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 7] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 7)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 8] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 8)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 9] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 9)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 10] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 10)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 11] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 11)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 12] = + nmpc_Ah_mat[run1 * 2]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 12)];
if( 0 == run1 ) nmpcWorkspace.rk_A[(tmp_index1 * 26) + (j)] -= 1.0000000000000000e+00;
nmpcWorkspace.rk_A[tmp_index1 * 26 + 13] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 14] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 1)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 15] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 2)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 16] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 3)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 17] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 4)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 18] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 5)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 19] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 6)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 20] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 7)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 21] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 8)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 22] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 9)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 23] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 10)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 24] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 11)];
nmpcWorkspace.rk_A[tmp_index1 * 26 + 25] = + nmpc_Ah_mat[run1 * 2 + 1]*nmpcWorkspace.rk_diffsTemp2[(run1 * 221) + (j * 17 + 12)];
if( 1 == run1 ) nmpcWorkspace.rk_A[(tmp_index1 * 26) + (j + 13)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 13; ++run1)
{
for (i = 0; i < 2; ++i)
{
nmpcWorkspace.rk_b[i * 13] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1)];
nmpcWorkspace.rk_b[i * 13 + 1] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 17)];
nmpcWorkspace.rk_b[i * 13 + 2] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 34)];
nmpcWorkspace.rk_b[i * 13 + 3] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 51)];
nmpcWorkspace.rk_b[i * 13 + 4] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 68)];
nmpcWorkspace.rk_b[i * 13 + 5] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 85)];
nmpcWorkspace.rk_b[i * 13 + 6] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 102)];
nmpcWorkspace.rk_b[i * 13 + 7] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 119)];
nmpcWorkspace.rk_b[i * 13 + 8] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 136)];
nmpcWorkspace.rk_b[i * 13 + 9] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 153)];
nmpcWorkspace.rk_b[i * 13 + 10] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 170)];
nmpcWorkspace.rk_b[i * 13 + 11] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 187)];
nmpcWorkspace.rk_b[i * 13 + 12] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (run1 + 204)];
}
if( 0 == run1 ) {
det = nmpc_solve_dim26_system( nmpcWorkspace.rk_A, nmpcWorkspace.rk_b, nmpcWorkspace.rk_dim26_perm );
}
 else {
nmpc_solve_dim26_system_reuse( nmpcWorkspace.rk_A, nmpcWorkspace.rk_b, nmpcWorkspace.rk_dim26_perm );
}
for (i = 0; i < 2; ++i)
{
nmpcWorkspace.rk_diffK[i] = nmpcWorkspace.rk_b[i * 13];
nmpcWorkspace.rk_diffK[i + 2] = nmpcWorkspace.rk_b[i * 13 + 1];
nmpcWorkspace.rk_diffK[i + 4] = nmpcWorkspace.rk_b[i * 13 + 2];
nmpcWorkspace.rk_diffK[i + 6] = nmpcWorkspace.rk_b[i * 13 + 3];
nmpcWorkspace.rk_diffK[i + 8] = nmpcWorkspace.rk_b[i * 13 + 4];
nmpcWorkspace.rk_diffK[i + 10] = nmpcWorkspace.rk_b[i * 13 + 5];
nmpcWorkspace.rk_diffK[i + 12] = nmpcWorkspace.rk_b[i * 13 + 6];
nmpcWorkspace.rk_diffK[i + 14] = nmpcWorkspace.rk_b[i * 13 + 7];
nmpcWorkspace.rk_diffK[i + 16] = nmpcWorkspace.rk_b[i * 13 + 8];
nmpcWorkspace.rk_diffK[i + 18] = nmpcWorkspace.rk_b[i * 13 + 9];
nmpcWorkspace.rk_diffK[i + 20] = nmpcWorkspace.rk_b[i * 13 + 10];
nmpcWorkspace.rk_diffK[i + 22] = nmpcWorkspace.rk_b[i * 13 + 11];
nmpcWorkspace.rk_diffK[i + 24] = nmpcWorkspace.rk_b[i * 13 + 12];
}
for (i = 0; i < 13; ++i)
{
nmpcWorkspace.rk_diffsNew2[(i * 17) + (run1)] = (i == run1-0);
nmpcWorkspace.rk_diffsNew2[(i * 17) + (run1)] += + nmpcWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 13; ++j)
{
tmp_index1 = (i * 13) + (j);
tmp_index2 = (run1) + (j * 17);
nmpcWorkspace.rk_b[tmp_index1] = - nmpcWorkspace.rk_diffsTemp2[(i * 221) + (tmp_index2 + 13)];
}
}
nmpc_solve_dim26_system_reuse( nmpcWorkspace.rk_A, nmpcWorkspace.rk_b, nmpcWorkspace.rk_dim26_perm );
for (i = 0; i < 2; ++i)
{
nmpcWorkspace.rk_diffK[i] = nmpcWorkspace.rk_b[i * 13];
nmpcWorkspace.rk_diffK[i + 2] = nmpcWorkspace.rk_b[i * 13 + 1];
nmpcWorkspace.rk_diffK[i + 4] = nmpcWorkspace.rk_b[i * 13 + 2];
nmpcWorkspace.rk_diffK[i + 6] = nmpcWorkspace.rk_b[i * 13 + 3];
nmpcWorkspace.rk_diffK[i + 8] = nmpcWorkspace.rk_b[i * 13 + 4];
nmpcWorkspace.rk_diffK[i + 10] = nmpcWorkspace.rk_b[i * 13 + 5];
nmpcWorkspace.rk_diffK[i + 12] = nmpcWorkspace.rk_b[i * 13 + 6];
nmpcWorkspace.rk_diffK[i + 14] = nmpcWorkspace.rk_b[i * 13 + 7];
nmpcWorkspace.rk_diffK[i + 16] = nmpcWorkspace.rk_b[i * 13 + 8];
nmpcWorkspace.rk_diffK[i + 18] = nmpcWorkspace.rk_b[i * 13 + 9];
nmpcWorkspace.rk_diffK[i + 20] = nmpcWorkspace.rk_b[i * 13 + 10];
nmpcWorkspace.rk_diffK[i + 22] = nmpcWorkspace.rk_b[i * 13 + 11];
nmpcWorkspace.rk_diffK[i + 24] = nmpcWorkspace.rk_b[i * 13 + 12];
}
for (i = 0; i < 13; ++i)
{
nmpcWorkspace.rk_diffsNew2[(i * 17) + (run1 + 13)] = + nmpcWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + nmpcWorkspace.rk_kkk[0]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + nmpcWorkspace.rk_kkk[2]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + nmpcWorkspace.rk_kkk[4]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + nmpcWorkspace.rk_kkk[6]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + nmpcWorkspace.rk_kkk[8]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[9]*(real_t)5.0000000000000003e-02;
rk_eta[5] += + nmpcWorkspace.rk_kkk[10]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[11]*(real_t)5.0000000000000003e-02;
rk_eta[6] += + nmpcWorkspace.rk_kkk[12]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[13]*(real_t)5.0000000000000003e-02;
rk_eta[7] += + nmpcWorkspace.rk_kkk[14]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[15]*(real_t)5.0000000000000003e-02;
rk_eta[8] += + nmpcWorkspace.rk_kkk[16]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[17]*(real_t)5.0000000000000003e-02;
rk_eta[9] += + nmpcWorkspace.rk_kkk[18]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[19]*(real_t)5.0000000000000003e-02;
rk_eta[10] += + nmpcWorkspace.rk_kkk[20]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[21]*(real_t)5.0000000000000003e-02;
rk_eta[11] += + nmpcWorkspace.rk_kkk[22]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[23]*(real_t)5.0000000000000003e-02;
rk_eta[12] += + nmpcWorkspace.rk_kkk[24]*(real_t)5.0000000000000003e-02 + nmpcWorkspace.rk_kkk[25]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 13; ++i)
{
for (j = 0; j < 13; ++j)
{
tmp_index2 = (j) + (i * 13);
rk_eta[tmp_index2 + 13] = nmpcWorkspace.rk_diffsNew2[(i * 17) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 182] = nmpcWorkspace.rk_diffsNew2[(i * 17) + (j + 13)];
}
}
resetIntegrator = 0;
nmpcWorkspace.rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 13; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



