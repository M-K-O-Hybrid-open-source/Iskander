#define one_root3 .5773503
#define two_root3 1.154700
#include "sine.h"
#include "math.h"

void clarke_trans(float Ia,float Ib,float *Ialpha_ptr, float *Ibeta_ptr);
void park_trans(float Ialpha,float Ibeta,float theta, float *Iq_ptr, float *Id_ptr);
void park_inv_trans(float Iq,float Id, float theta, float *Iaplha_ptr, float *Ibeta_ptr);
void clarke_inv_trans(float Ialpha,float Ibeta,float *Ia, float *Ib, float *Ic);

