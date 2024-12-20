#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void end_effector_kinematics_velocity_sparse_jacobian(double const *const * in,
                                                      double*const * out,
                                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[93];

   v[0] = cos(x[5]);
   v[1] = sin(x[1]);
   v[2] = 0 - v[1];
   v[3] = cos(x[1]);
   v[4] = 0.377127346689844 * v[2] + -0.926161413775755 * v[3];
   v[5] = v[4] * x[6];
   v[6] = sin(x[2]);
   v[7] = 0 - v[6];
   v[8] = cos(x[2]);
   v[9] = -0.92612369641659 * v[7] + 0.377219960945431 * v[8];
   v[10] = 0.377127346689844 * v[3] + -0.926161413775755 * v[1];
   v[11] = v[10] * x[6];
   v[7] = -0.377219960945431 * v[7] + -0.92612369641659 * v[8];
   v[12] = v[9] * v[11] + v[7] * v[5];
   v[13] = 0 - 0.27009 * v[5] - 0.30854 * v[12];
   v[14] = x[7] + 3.40198083137455e-06 * x[6];
   v[15] = x[8] + v[14];
   v[14] = 0.27009 * v[14];
   v[16] = 0.30854 * v[15] + v[7] * v[14];
   v[17] = -0.377219960945431 * v[8] + -0.92612369641659 * v[6];
   v[18] = v[17] * v[14];
   v[19] = 3.67320510330504e-06 * v[13] + 7.34641020664359e-06 * v[16] + 0.999999999966269 * v[18];
   v[20] = sin(x[3]);
   v[21] = 0 - v[20];
   v[22] = cos(x[3]);
   v[23] = 0.999999999993254 * v[21] + 2.6984871461853e-11 * v[22];
   v[24] = -4.2351647362715e-22 * v[21] + -0.999999999973015 * v[22];
   v[21] = -3.67320510341607e-06 * v[21] + 7.34641020659403e-06 * v[22];
   v[25] = v[23] * v[13] + v[24] * v[16] + v[21] * v[18];
   v[26] = sin(x[4]);
   v[27] = 0.999999999993254 * v[26];
   v[8] = -0.92612369641659 * v[8] + 0.377219960945431 * v[6];
   v[6] = v[8] * v[11] + v[17] * v[5];
   v[28] = 3.67320510330504e-06 * v[15] + 7.34641020664359e-06 * v[12] + 0.999999999966269 * v[6] + x[9];
   v[29] = -3.67320510363811e-06 * v[26];
   v[30] = v[23] * v[15] + v[24] * v[12] + v[21] * v[6];
   v[31] = cos(x[4]);
   v[32] = 0.999999999993254 * v[22] + 2.6984871461853e-11 * v[20];
   v[33] = -4.2351647362715e-22 * v[22] + -0.999999999973015 * v[20];
   v[22] = -3.67320510341607e-06 * v[22] + 7.34641020659403e-06 * v[20];
   v[20] = v[32] * v[15] + v[33] * v[12] + v[22] * v[6];
   v[34] = -3.67320510363811e-06 * v[19] + -0.999999999993254 * v[25] + 0.085 * (v[27] * v[28] + v[29] * v[30] + v[31] * v[20]);
   v[35] = sin(x[5]);
   v[36] = 0 - v[35];
   v[37] = -3.67320510363811e-06 * v[36];
   v[38] = 0.999999999993254 * v[31];
   v[39] = -3.67320510363811e-06 * v[31];
   v[26] = 0 - v[26];
   v[40] = v[32] * v[13] + v[33] * v[16] + v[22] * v[18];
   v[41] = v[38] * v[19] + v[39] * v[25] + v[26] * v[40];
   v[36] = -0.999999999993254 * v[36];
   v[42] = v[27] * v[19] + v[29] * v[25] + v[31] * v[40] - 0.085 * (-3.67320510363811e-06 * v[28] + -0.999999999993254 * v[30] + x[10]);
   v[43] = v[0] * v[34] + v[37] * v[41] + v[36] * v[42];
   v[44] = -3.67320510363811e-06 * v[0];
   v[45] = -0.999999999993254 * v[0];
   v[46] = v[35] * v[34] + v[44] * v[41] + v[45] * v[42];
   v[47] = v[43] * v[0] + v[46] * v[35];
   v[48] = -3.67320510363811e-06 * v[42] + 0.999999999993254 * v[41];
   v[49] = v[43] * v[37] + v[46] * v[44] + v[48] * 0.999999999993254;
   v[50] = v[43] * v[36] + v[46] * v[45] + v[48] * -3.67320510363811e-06;
   v[51] = v[47] * -3.67320510363811e-06 + v[49] * v[38] + v[50] * v[27];
   v[47] = v[47] * -0.999999999993254 + v[49] * v[39] + v[50] * v[29];
   v[52] = v[49] * v[26] + v[50] * v[31];
   v[53] = v[51] * 3.67320510330504e-06 + v[47] * v[23] + v[52] * v[32];
   v[54] = v[51] * 7.34641020664359e-06 + v[47] * v[24] + v[52] * v[33];
   v[51] = v[51] * 0.999999999966269 + v[47] * v[21] + v[52] * v[22];
   v[55] = v[54] * v[7] + v[51] * v[17];
   v[2] = -0.926161413782003 * v[2] + -0.3771273466873 * v[3];
   v[56] = v[54] * v[9] + v[51] * v[8];
   v[57] = -0.926161413782003 * v[3] + -0.3771273466873 * v[1];
   v[58] = sin(x[0]);
   v[3] = -3.67320510363811e-06 * v[3];
   v[1] = -3.67320510363811e-06 * v[1];
   v[59] = cos(x[0]);
   jac[0] = 0 - (v[53] * 1.38526609444467e-06 + v[55] * v[2] + v[56] * v[57]) * v[58] + (0 - (v[53] * -0.999999999993254 + v[55] * v[3] + v[56] * v[1])) * v[59];
   v[53] = cos(x[0]);
   v[60] = sin(x[0]);
   v[61] = 0 - v[60];
   v[62] = 1.38526609444467e-06 * v[53] + -0.999999999993254 * v[61];
   v[63] = v[53] * v[57] + v[61] * v[1];
   v[64] = v[53] * v[2] + v[61] * v[3];
   v[65] = v[63] * v[9] + v[64] * v[7];
   v[66] = v[63] * v[8] + v[64] * v[17];
   v[67] = v[62] * v[23] + v[65] * v[24] + v[66] * v[21];
   v[68] = 3.67320510330504e-06 * v[62] + 7.34641020664359e-06 * v[65] + 0.999999999966269 * v[66];
   v[69] = -0.999999999993254 * v[67] + -3.67320510363811e-06 * v[68];
   v[70] = v[62] * v[32] + v[65] * v[33] + v[66] * v[22];
   v[71] = v[68] * v[38] + v[67] * v[39] + v[70] * v[26];
   v[72] = v[68] * v[27] + v[67] * v[29] + v[70] * v[31];
   v[73] = v[69] * v[0] + v[71] * v[37] + v[72] * v[36];
   v[74] = v[69] * v[35] + v[71] * v[44] + v[72] * v[45];
   v[75] = -3.67320510363811e-06 * v[72] + 0.999999999993254 * v[71];
   v[76] = v[73] * v[36] + v[74] * v[45] + v[75] * -3.67320510363811e-06;
   jac[10] = (0 - v[76]) * 0.085;
   v[77] = v[73] * v[0] + v[74] * v[35];
   v[78] = v[77] * 0.085;
   jac[9] = jac[10] * -3.67320510363811e-06 + v[78] * v[27];
   v[79] = jac[10] * -0.999999999993254 + v[78] * v[29];
   v[80] = v[78] * v[31];
   v[75] = v[73] * v[37] + v[74] * v[44] + v[75] * 0.999999999993254;
   v[81] = v[77] * -3.67320510363811e-06 + v[75] * v[38] + v[76] * v[27];
   v[77] = v[77] * -0.999999999993254 + v[75] * v[39] + v[76] * v[29];
   v[82] = v[75] * v[26] + v[76] * v[31];
   v[83] = v[81] * 3.67320510330504e-06 + v[77] * v[23] + v[82] * v[32];
   v[84] = jac[9] * 7.34641020664359e-06 + v[79] * v[24] + v[80] * v[33] + (0 - v[83]) * 0.30854;
   v[85] = jac[9] * 0.999999999966269 + v[79] * v[21] + v[80] * v[22];
   v[83] = v[84] * v[7] + v[85] * v[17] + (0 - v[83]) * 0.27009;
   v[86] = v[83] * x[6];
   v[87] = v[55] * v[53];
   v[88] = v[84] * v[9] + v[85] * v[8];
   v[89] = v[88] * x[6];
   v[90] = v[56] * v[53];
   v[91] = sin(x[1]);
   v[92] = cos(x[1]);
   jac[1] = 0 - (v[86] * -0.926161413775755 + v[55] * v[61] * -3.67320510363811e-06 + v[87] * -0.3771273466873 + v[89] * 0.377127346689844 + v[90] * -0.926161413782003) * v[91] + (v[89] * -0.926161413775755 + v[56] * v[61] * -3.67320510363811e-06 + v[90] * -0.3771273466873 - (v[86] * 0.377127346689844 + v[87] * -0.926161413782003)) * v[92];
   v[90] = v[81] * 7.34641020664359e-06 + v[77] * v[24] + v[82] * v[33];
   v[89] = v[84] * v[5] + v[90] * v[14] + v[54] * v[64];
   v[84] = v[84] * v[11] + v[54] * v[63];
   v[81] = v[81] * 0.999999999966269 + v[77] * v[21] + v[82] * v[22];
   v[64] = v[85] * v[5] + v[81] * v[14] + v[51] * v[64];
   v[85] = v[85] * v[11] + v[51] * v[63];
   v[63] = sin(x[2]);
   v[51] = cos(x[2]);
   jac[2] = 0 - (v[89] * -0.92612369641659 + v[84] * 0.377219960945431 + v[64] * -0.377219960945431 + v[85] * -0.92612369641659) * v[63] + (v[64] * -0.92612369641659 + v[85] * 0.377219960945431 - (v[89] * -0.377219960945431 + v[84] * -0.92612369641659)) * v[51];
   v[85] = v[79] * v[15] + v[77] * v[13] + v[47] * v[62];
   v[64] = v[79] * v[12] + v[77] * v[16] + v[47] * v[65];
   v[77] = v[79] * v[6] + v[77] * v[18] + v[47] * v[66];
   v[62] = v[80] * v[15] + v[82] * v[13] + v[52] * v[62];
   v[65] = v[80] * v[12] + v[82] * v[16] + v[52] * v[65];
   v[82] = v[80] * v[6] + v[82] * v[18] + v[52] * v[66];
   v[66] = sin(x[3]);
   v[52] = cos(x[3]);
   jac[3] = 0 - (v[85] * 2.6984871461853e-11 + v[64] * -0.999999999973015 + v[77] * 7.34641020659403e-06 + v[62] * 0.999999999993254 + v[65] * -4.2351647362715e-22 + v[82] * -3.67320510341607e-06) * v[66] + (v[62] * 2.6984871461853e-11 + v[65] * -0.999999999973015 + v[82] * 7.34641020659403e-06 - (v[85] * 0.999999999993254 + v[64] * -4.2351647362715e-22 + v[77] * -3.67320510341607e-06)) * v[52];
   v[82] = sin(x[4]);
   v[65] = cos(x[4]);
   jac[4] = 0 - (v[78] * v[20] + v[76] * v[40] + v[50] * v[70] + (v[75] * v[19] + v[49] * v[68]) * 0.999999999993254 + (v[75] * v[25] + v[49] * v[67]) * -3.67320510363811e-06) * v[82] + ((v[78] * v[28] + v[76] * v[19] + v[50] * v[68]) * 0.999999999993254 + (v[78] * v[30] + v[76] * v[25] + v[50] * v[67]) * -3.67320510363811e-06 - (v[75] * v[40] + v[49] * v[70])) * v[65];
   v[75] = sin(x[5]);
   v[78] = cos(x[5]);
   jac[5] = 0 - (v[73] * v[34] + v[43] * v[69] + (v[74] * v[41] + v[46] * v[71]) * -3.67320510363811e-06 + (v[74] * v[42] + v[46] * v[72]) * -0.999999999993254) * v[75] + (v[74] * v[34] + v[46] * v[69] - ((v[73] * v[41] + v[43] * v[71]) * -3.67320510363811e-06 + (v[73] * v[42] + v[43] * v[72]) * -0.999999999993254)) * v[78];
   jac[8] = jac[9] * 3.67320510330504e-06 + v[79] * v[23] + v[80] * v[32] + v[90] * 0.30854;
   jac[7] = jac[8] + (v[90] * v[7] + v[81] * v[17]) * 0.27009;
   jac[6] = jac[7] * 3.40198083137455e-06 + v[83] * v[4] + v[88] * v[10];
   v[88] = v[43] * v[0] + v[46] * v[35];
   v[83] = v[43] * v[37] + v[46] * v[44] + v[48] * 0.999999999993254;
   v[81] = v[43] * v[36] + v[46] * v[45] + v[48] * -3.67320510363811e-06;
   v[90] = v[88] * -3.67320510363811e-06 + v[83] * v[38] + v[81] * v[27];
   v[88] = v[88] * -0.999999999993254 + v[83] * v[39] + v[81] * v[29];
   v[80] = v[83] * v[26] + v[81] * v[31];
   v[79] = v[90] * 3.67320510330504e-06 + v[88] * v[23] + v[80] * v[32];
   v[74] = v[90] * 7.34641020664359e-06 + v[88] * v[24] + v[80] * v[33];
   v[90] = v[90] * 0.999999999966269 + v[88] * v[21] + v[80] * v[22];
   v[73] = v[74] * v[7] + v[90] * v[17];
   v[72] = v[74] * v[9] + v[90] * v[8];
   jac[11] = 0 - (v[79] * -0.999999999993254 + v[73] * v[3] + v[72] * v[1]) * v[58] + (v[79] * 1.38526609444467e-06 + v[73] * v[2] + v[72] * v[57]) * v[59];
   v[79] = 1.38526609444467e-06 * v[60] + -0.999999999993254 * v[53];
   v[1] = v[60] * v[57] + v[53] * v[1];
   v[3] = v[60] * v[2] + v[53] * v[3];
   v[2] = v[1] * v[9] + v[3] * v[7];
   v[57] = v[1] * v[8] + v[3] * v[17];
   v[59] = v[79] * v[23] + v[2] * v[24] + v[57] * v[21];
   v[58] = 3.67320510330504e-06 * v[79] + 7.34641020664359e-06 * v[2] + 0.999999999966269 * v[57];
   v[71] = -0.999999999993254 * v[59] + -3.67320510363811e-06 * v[58];
   v[69] = v[79] * v[32] + v[2] * v[33] + v[57] * v[22];
   v[76] = v[58] * v[38] + v[59] * v[39] + v[69] * v[26];
   v[70] = v[58] * v[27] + v[59] * v[29] + v[69] * v[31];
   v[68] = v[71] * v[0] + v[76] * v[37] + v[70] * v[36];
   v[67] = v[71] * v[35] + v[76] * v[44] + v[70] * v[45];
   v[50] = -3.67320510363811e-06 * v[70] + 0.999999999993254 * v[76];
   v[49] = v[68] * v[36] + v[67] * v[45] + v[50] * -3.67320510363811e-06;
   jac[21] = (0 - v[49]) * 0.085;
   v[62] = v[68] * v[0] + v[67] * v[35];
   v[77] = v[62] * 0.085;
   jac[20] = jac[21] * -3.67320510363811e-06 + v[77] * v[27];
   v[64] = jac[21] * -0.999999999993254 + v[77] * v[29];
   v[85] = v[77] * v[31];
   v[50] = v[68] * v[37] + v[67] * v[44] + v[50] * 0.999999999993254;
   v[47] = v[62] * -3.67320510363811e-06 + v[50] * v[38] + v[49] * v[27];
   v[62] = v[62] * -0.999999999993254 + v[50] * v[39] + v[49] * v[29];
   v[84] = v[50] * v[26] + v[49] * v[31];
   v[89] = v[47] * 3.67320510330504e-06 + v[62] * v[23] + v[84] * v[32];
   v[54] = jac[20] * 7.34641020664359e-06 + v[64] * v[24] + v[85] * v[33] + (0 - v[89]) * 0.30854;
   v[87] = jac[20] * 0.999999999966269 + v[64] * v[21] + v[85] * v[22];
   v[89] = v[54] * v[7] + v[87] * v[17] + (0 - v[89]) * 0.27009;
   v[86] = v[89] * x[6];
   v[61] = v[73] * v[60];
   v[56] = v[54] * v[9] + v[87] * v[8];
   v[55] = v[56] * x[6];
   v[60] = v[72] * v[60];
   jac[12] = 0 - (v[86] * -0.926161413775755 + v[73] * v[53] * -3.67320510363811e-06 + v[61] * -0.3771273466873 + v[55] * 0.377127346689844 + v[60] * -0.926161413782003) * v[91] + (v[55] * -0.926161413775755 + v[72] * v[53] * -3.67320510363811e-06 + v[60] * -0.3771273466873 - (v[86] * 0.377127346689844 + v[61] * -0.926161413782003)) * v[92];
   v[60] = v[47] * 7.34641020664359e-06 + v[62] * v[24] + v[84] * v[33];
   v[55] = v[54] * v[5] + v[60] * v[14] + v[74] * v[3];
   v[54] = v[54] * v[11] + v[74] * v[1];
   v[47] = v[47] * 0.999999999966269 + v[62] * v[21] + v[84] * v[22];
   v[3] = v[87] * v[5] + v[47] * v[14] + v[90] * v[3];
   v[87] = v[87] * v[11] + v[90] * v[1];
   jac[13] = 0 - (v[55] * -0.92612369641659 + v[54] * 0.377219960945431 + v[3] * -0.377219960945431 + v[87] * -0.92612369641659) * v[63] + (v[3] * -0.92612369641659 + v[87] * 0.377219960945431 - (v[55] * -0.377219960945431 + v[54] * -0.92612369641659)) * v[51];
   v[87] = v[64] * v[15] + v[62] * v[13] + v[88] * v[79];
   v[3] = v[64] * v[12] + v[62] * v[16] + v[88] * v[2];
   v[62] = v[64] * v[6] + v[62] * v[18] + v[88] * v[57];
   v[79] = v[85] * v[15] + v[84] * v[13] + v[80] * v[79];
   v[2] = v[85] * v[12] + v[84] * v[16] + v[80] * v[2];
   v[84] = v[85] * v[6] + v[84] * v[18] + v[80] * v[57];
   jac[14] = 0 - (v[87] * 2.6984871461853e-11 + v[3] * -0.999999999973015 + v[62] * 7.34641020659403e-06 + v[79] * 0.999999999993254 + v[2] * -4.2351647362715e-22 + v[84] * -3.67320510341607e-06) * v[66] + (v[79] * 2.6984871461853e-11 + v[2] * -0.999999999973015 + v[84] * 7.34641020659403e-06 - (v[87] * 0.999999999993254 + v[3] * -4.2351647362715e-22 + v[62] * -3.67320510341607e-06)) * v[52];
   jac[15] = 0 - (v[77] * v[20] + v[49] * v[40] + v[81] * v[69] + (v[50] * v[19] + v[83] * v[58]) * 0.999999999993254 + (v[50] * v[25] + v[83] * v[59]) * -3.67320510363811e-06) * v[82] + ((v[77] * v[28] + v[49] * v[19] + v[81] * v[58]) * 0.999999999993254 + (v[77] * v[30] + v[49] * v[25] + v[81] * v[59]) * -3.67320510363811e-06 - (v[50] * v[40] + v[83] * v[69])) * v[65];
   jac[16] = 0 - (v[68] * v[34] + v[43] * v[71] + (v[67] * v[41] + v[46] * v[76]) * -3.67320510363811e-06 + (v[67] * v[42] + v[46] * v[70]) * -0.999999999993254) * v[75] + (v[67] * v[34] + v[46] * v[71] - ((v[68] * v[41] + v[43] * v[76]) * -3.67320510363811e-06 + (v[68] * v[42] + v[43] * v[70]) * -0.999999999993254)) * v[78];
   jac[19] = jac[20] * 3.67320510330504e-06 + v[64] * v[23] + v[85] * v[32] + v[60] * 0.30854;
   jac[18] = jac[19] + (v[60] * v[7] + v[47] * v[17]) * 0.27009;
   jac[17] = jac[18] * 3.40198083137455e-06 + v[89] * v[4] + v[56] * v[10];
   v[56] = v[43] * v[0] + v[46] * v[35];
   v[89] = v[43] * v[37] + v[46] * v[44] + v[48] * 0.999999999993254;
   v[48] = v[43] * v[36] + v[46] * v[45] + v[48] * -3.67320510363811e-06;
   v[47] = v[56] * -3.67320510363811e-06 + v[89] * v[38] + v[48] * v[27];
   v[56] = v[56] * -0.999999999993254 + v[89] * v[39] + v[48] * v[29];
   v[60] = v[89] * v[26] + v[48] * v[31];
   v[85] = v[47] * 7.34641020664359e-06 + v[56] * v[24] + v[60] * v[33];
   v[47] = v[47] * 0.999999999966269 + v[56] * v[21] + v[60] * v[22];
   v[64] = v[10] * v[9] + v[4] * v[7];
   v[67] = v[10] * v[8] + v[4] * v[17];
   v[68] = 3.40198083137455e-06 * v[23] + v[64] * v[24] + v[67] * v[21];
   v[70] = 1.24961733511509e-11 + 7.34641020664359e-06 * v[64] + 0.999999999966269 * v[67];
   v[76] = -0.999999999993254 * v[68] + -3.67320510363811e-06 * v[70];
   v[71] = 3.40198083137455e-06 * v[32] + v[64] * v[33] + v[67] * v[22];
   v[50] = v[70] * v[38] + v[68] * v[39] + v[71] * v[26];
   v[77] = v[70] * v[27] + v[68] * v[29] + v[71] * v[31];
   v[49] = v[76] * v[0] + v[50] * v[37] + v[77] * v[36];
   v[69] = v[76] * v[35] + v[50] * v[44] + v[77] * v[45];
   v[58] = -3.67320510363811e-06 * v[77] + 0.999999999993254 * v[50];
   v[45] = v[49] * v[36] + v[69] * v[45] + v[58] * -3.67320510363811e-06;
   jac[31] = (0 - v[45]) * 0.085;
   v[35] = v[49] * v[0] + v[69] * v[35];
   v[0] = v[35] * 0.085;
   jac[30] = jac[31] * -3.67320510363811e-06 + v[0] * v[27];
   v[36] = jac[31] * -0.999999999993254 + v[0] * v[29];
   v[59] = v[0] * v[31];
   v[58] = v[49] * v[37] + v[69] * v[44] + v[58] * 0.999999999993254;
   v[38] = v[35] * -3.67320510363811e-06 + v[58] * v[38] + v[45] * v[27];
   v[35] = v[35] * -0.999999999993254 + v[58] * v[39] + v[45] * v[29];
   v[26] = v[58] * v[26] + v[45] * v[31];
   v[31] = v[38] * 3.67320510330504e-06 + v[35] * v[23] + v[26] * v[32];
   v[39] = jac[30] * 7.34641020664359e-06 + v[36] * v[24] + v[59] * v[33] + (0 - v[31]) * 0.30854;
   v[29] = jac[30] * 0.999999999966269 + v[36] * v[21] + v[59] * v[22];
   v[31] = v[39] * v[7] + v[29] * v[17] + (0 - v[31]) * 0.27009;
   v[27] = v[85] * v[7] + v[47] * v[17] + v[31] * x[6];
   v[44] = v[39] * v[9] + v[29] * v[8];
   v[8] = v[85] * v[9] + v[47] * v[8] + v[44] * x[6];
   jac[22] = 0 - (v[27] * -0.926161413775755 + v[8] * 0.377127346689844) * v[91] + (v[8] * -0.926161413775755 - v[27] * 0.377127346689844) * v[92];
   v[33] = v[38] * 7.34641020664359e-06 + v[35] * v[24] + v[26] * v[33];
   v[24] = v[39] * v[5] + v[33] * v[14] + v[85] * v[4];
   v[39] = v[39] * v[11] + v[85] * v[10];
   v[38] = v[38] * 0.999999999966269 + v[35] * v[21] + v[26] * v[22];
   v[14] = v[29] * v[5] + v[38] * v[14] + v[47] * v[4];
   v[29] = v[29] * v[11] + v[47] * v[10];
   jac[23] = 0 - (v[24] * -0.92612369641659 + v[39] * 0.377219960945431 + v[14] * -0.377219960945431 + v[29] * -0.92612369641659) * v[63] + (v[14] * -0.92612369641659 + v[29] * 0.377219960945431 - (v[24] * -0.377219960945431 + v[39] * -0.92612369641659)) * v[51];
   v[29] = v[36] * v[15] + v[35] * v[13] + v[56] * 3.40198083137455e-06;
   v[14] = v[36] * v[12] + v[35] * v[16] + v[56] * v[64];
   v[35] = v[36] * v[6] + v[35] * v[18] + v[56] * v[67];
   v[15] = v[59] * v[15] + v[26] * v[13] + v[60] * 3.40198083137455e-06;
   v[64] = v[59] * v[12] + v[26] * v[16] + v[60] * v[64];
   v[26] = v[59] * v[6] + v[26] * v[18] + v[60] * v[67];
   jac[24] = 0 - (v[29] * 2.6984871461853e-11 + v[14] * -0.999999999973015 + v[35] * 7.34641020659403e-06 + v[15] * 0.999999999993254 + v[64] * -4.2351647362715e-22 + v[26] * -3.67320510341607e-06) * v[66] + (v[15] * 2.6984871461853e-11 + v[64] * -0.999999999973015 + v[26] * 7.34641020659403e-06 - (v[29] * 0.999999999993254 + v[14] * -4.2351647362715e-22 + v[35] * -3.67320510341607e-06)) * v[52];
   jac[25] = 0 - (v[0] * v[20] + v[45] * v[40] + v[48] * v[71] + (v[58] * v[19] + v[89] * v[70]) * 0.999999999993254 + (v[58] * v[25] + v[89] * v[68]) * -3.67320510363811e-06) * v[82] + ((v[0] * v[28] + v[45] * v[19] + v[48] * v[70]) * 0.999999999993254 + (v[0] * v[30] + v[45] * v[25] + v[48] * v[68]) * -3.67320510363811e-06 - (v[58] * v[40] + v[89] * v[71])) * v[65];
   jac[26] = 0 - (v[49] * v[34] + v[43] * v[76] + (v[69] * v[41] + v[46] * v[50]) * -3.67320510363811e-06 + (v[69] * v[42] + v[46] * v[77]) * -0.999999999993254) * v[75] + (v[69] * v[34] + v[46] * v[76] - ((v[49] * v[41] + v[43] * v[50]) * -3.67320510363811e-06 + (v[49] * v[42] + v[43] * v[77]) * -0.999999999993254)) * v[78];
   jac[29] = jac[30] * 3.67320510330504e-06 + v[36] * v[23] + v[59] * v[32] + v[33] * 0.30854;
   jac[28] = jac[29] + (v[33] * v[7] + v[38] * v[17]) * 0.27009;
   jac[27] = jac[28] * 3.40198083137455e-06 + v[31] * v[4] + v[44] * v[10];
}

