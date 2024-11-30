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

void end_effector_kinematics_velocity_forward_zero(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[41];

   v[0] = cos(x[0]);
   v[1] = sin(x[0]);
   v[2] = 0 - v[1];
   v[3] = 1.38526609444467e-06 * v[0] + -0.999999999993254 * v[2];
   v[4] = sin(x[3]);
   v[5] = 0 - v[4];
   v[6] = cos(x[3]);
   v[7] = 0.999999999993254 * v[5] + 2.6984871461853e-11 * v[6];
   v[8] = cos(x[1]);
   v[9] = sin(x[1]);
   v[10] = -0.926161413782003 * v[8] + -0.3771273466873 * v[9];
   v[11] = -3.67320510363811e-06 * v[9];
   v[12] = v[0] * v[10] + v[2] * v[11];
   v[13] = sin(x[2]);
   v[14] = 0 - v[13];
   v[15] = cos(x[2]);
   v[16] = -0.92612369641659 * v[14] + 0.377219960945431 * v[15];
   v[17] = 0 - v[9];
   v[18] = -0.926161413782003 * v[17] + -0.3771273466873 * v[8];
   v[19] = -3.67320510363811e-06 * v[8];
   v[2] = v[0] * v[18] + v[2] * v[19];
   v[14] = -0.377219960945431 * v[14] + -0.92612369641659 * v[15];
   v[20] = v[12] * v[16] + v[2] * v[14];
   v[21] = -4.2351647362715e-22 * v[5] + -0.999999999973015 * v[6];
   v[22] = -0.92612369641659 * v[15] + 0.377219960945431 * v[13];
   v[15] = -0.377219960945431 * v[15] + -0.92612369641659 * v[13];
   v[2] = v[12] * v[22] + v[2] * v[15];
   v[5] = -3.67320510341607e-06 * v[5] + 7.34641020659403e-06 * v[6];
   v[12] = v[3] * v[7] + v[20] * v[21] + v[2] * v[5];
   v[13] = 3.67320510330504e-06 * v[3] + 7.34641020664359e-06 * v[20] + 0.999999999966269 * v[2];
   v[23] = -0.999999999993254 * v[12] + -3.67320510363811e-06 * v[13];
   v[24] = cos(x[5]);
   v[25] = cos(x[4]);
   v[26] = 0.999999999993254 * v[25];
   v[27] = -3.67320510363811e-06 * v[25];
   v[28] = 0.999999999993254 * v[6] + 2.6984871461853e-11 * v[4];
   v[29] = -4.2351647362715e-22 * v[6] + -0.999999999973015 * v[4];
   v[6] = -3.67320510341607e-06 * v[6] + 7.34641020659403e-06 * v[4];
   v[2] = v[3] * v[28] + v[20] * v[29] + v[2] * v[6];
   v[20] = sin(x[4]);
   v[3] = 0 - v[20];
   v[4] = v[13] * v[26] + v[12] * v[27] + v[2] * v[3];
   v[30] = sin(x[5]);
   v[31] = 0 - v[30];
   v[32] = -3.67320510363811e-06 * v[31];
   v[33] = 0.999999999993254 * v[20];
   v[20] = -3.67320510363811e-06 * v[20];
   v[2] = v[13] * v[33] + v[12] * v[20] + v[2] * v[25];
   v[31] = -0.999999999993254 * v[31];
   v[17] = 0.377127346689844 * v[17] + -0.926161413775755 * v[8];
   v[13] = v[17] * x[6];
   v[9] = 0.377127346689844 * v[8] + -0.926161413775755 * v[9];
   v[8] = v[9] * x[6];
   v[12] = v[16] * v[8] + v[14] * v[13];
   v[34] = 0 - 0.27009 * v[13] - 0.30854 * v[12];
   v[35] = x[7] + 3.40198083137455e-06 * x[6];
   v[36] = x[8] + v[35];
   v[35] = 0.27009 * v[35];
   v[37] = 0.30854 * v[36] + v[14] * v[35];
   v[35] = v[15] * v[35];
   v[38] = 3.67320510330504e-06 * v[34] + 7.34641020664359e-06 * v[37] + 0.999999999966269 * v[35];
   v[39] = v[7] * v[34] + v[21] * v[37] + v[5] * v[35];
   v[8] = v[22] * v[8] + v[15] * v[13];
   v[13] = 3.67320510330504e-06 * v[36] + 7.34641020664359e-06 * v[12] + 0.999999999966269 * v[8] + x[9];
   v[40] = v[7] * v[36] + v[21] * v[12] + v[5] * v[8];
   v[8] = -3.67320510363811e-06 * v[38] + -0.999999999993254 * v[39] + 0.085 * (v[33] * v[13] + v[20] * v[40] + v[25] * (v[28] * v[36] + v[29] * v[12] + v[6] * v[8]));
   v[35] = v[28] * v[34] + v[29] * v[37] + v[6] * v[35];
   v[37] = v[26] * v[38] + v[27] * v[39] + v[3] * v[35];
   v[35] = v[33] * v[38] + v[20] * v[39] + v[25] * v[35] - 0.085 * (-3.67320510363811e-06 * v[13] + -0.999999999993254 * v[40] + x[10]);
   v[40] = v[24] * v[8] + v[32] * v[37] + v[31] * v[35];
   v[13] = -3.67320510363811e-06 * v[24];
   v[39] = -0.999999999993254 * v[24];
   v[8] = v[30] * v[8] + v[13] * v[37] + v[39] * v[35];
   v[35] = -3.67320510363811e-06 * v[35] + 0.999999999993254 * v[37];
   y[0] = (v[23] * v[24] + v[4] * v[32] + v[2] * v[31]) * v[40] + (v[23] * v[30] + v[4] * v[13] + v[2] * v[39]) * v[8] + (-3.67320510363811e-06 * v[2] + 0.999999999993254 * v[4]) * v[35];
   v[2] = 1.38526609444467e-06 * v[1] + -0.999999999993254 * v[0];
   v[11] = v[1] * v[10] + v[0] * v[11];
   v[19] = v[1] * v[18] + v[0] * v[19];
   v[18] = v[11] * v[16] + v[19] * v[14];
   v[19] = v[11] * v[22] + v[19] * v[15];
   v[11] = v[2] * v[7] + v[18] * v[21] + v[19] * v[5];
   v[1] = 3.67320510330504e-06 * v[2] + 7.34641020664359e-06 * v[18] + 0.999999999966269 * v[19];
   v[0] = -0.999999999993254 * v[11] + -3.67320510363811e-06 * v[1];
   v[19] = v[2] * v[28] + v[18] * v[29] + v[19] * v[6];
   v[18] = v[1] * v[26] + v[11] * v[27] + v[19] * v[3];
   v[19] = v[1] * v[33] + v[11] * v[20] + v[19] * v[25];
   y[1] = (v[0] * v[24] + v[18] * v[32] + v[19] * v[31]) * v[40] + (v[0] * v[30] + v[18] * v[13] + v[19] * v[39]) * v[8] + (-3.67320510363811e-06 * v[19] + 0.999999999993254 * v[18]) * v[35];
   v[14] = v[9] * v[16] + v[17] * v[14];
   v[9] = v[9] * v[22] + v[17] * v[15];
   v[5] = 3.40198083137455e-06 * v[7] + v[14] * v[21] + v[9] * v[5];
   v[21] = 1.24961733511509e-11 + 7.34641020664359e-06 * v[14] + 0.999999999966269 * v[9];
   v[7] = -0.999999999993254 * v[5] + -3.67320510363811e-06 * v[21];
   v[9] = 3.40198083137455e-06 * v[28] + v[14] * v[29] + v[9] * v[6];
   v[3] = v[21] * v[26] + v[5] * v[27] + v[9] * v[3];
   v[9] = v[21] * v[33] + v[5] * v[20] + v[9] * v[25];
   y[2] = (v[7] * v[24] + v[3] * v[32] + v[9] * v[31]) * v[40] + (v[7] * v[30] + v[3] * v[13] + v[9] * v[39]) * v[8] + (-3.67320510363811e-06 * v[9] + 0.999999999993254 * v[3]) * v[35];
}
