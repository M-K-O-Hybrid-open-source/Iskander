#include "park_clarke.h"


/* input is abc, out is alpha and beta */
void clarke_trans(float Ia,float Ib,float *Ialpha_ptr, float *Ibeta_ptr){
	*Ialpha_ptr = Ia;
	*Ibeta_ptr = one_root3*(Ia+(2*Ib));
}


/* input is alpha nad beta and theta, output is dq */
void park_trans(float Ialpha,float Ibeta,float theta, float *Iq_ptr, float *Id_ptr){
	*Id_ptr = (Ialpha*cos(theta)+Ibeta*sin(theta));
	*Iq_ptr = (-Ialpha*sin(theta)+Ibeta*cos(theta));
}


/* input is dq and theta, output is alpha and beta */ 
void park_inv_trans(float Iq,float Id, float theta, float *Iaplha_ptr, float *Ibeta_ptr){
	*Iaplha_ptr = (Id*cos(theta)-Iq*sin(theta));
	*Ibeta_ptr  = (Id*sin(theta)+Iq*cos(theta));
}

/* input is alpha and beta ,output is abc */ 
void clarke_inv_trans(float Ialpha,float Ibeta,float *Ia, float *Ib, float *Ic){
	*Ia = Ialpha;
	*Ib = (-.5*Ialpha) + .86602*Ibeta;
	*Ic = (-.5*Ialpha) - .86602*Ibeta;
}









