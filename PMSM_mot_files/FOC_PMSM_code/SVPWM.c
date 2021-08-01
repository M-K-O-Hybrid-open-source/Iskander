/* inputs are the three voltages
	 1- get N
	 2- get sctor
	 3- get T1,T2,T0
	 4- get the three unique values
	 5- get S1,S2,S3 duty cycles scaled from 0 to 1
*/
#include "stm32f10x.h"


// inputs are the three phase refrance signals, and three adresses to get this values 
// this is how to use it in code 
// SVPWM(Ia_out,Ib_out,Ic_out,&S1,&S2,&S3);
// here the inputs are the Ia,b,c, adress of double S1,2,3
// and after excuting this function you will get the values of the factor to be multiplyed by the 
// timer register

// dont forget to normalizee the inputs values by the DC bus value, (VA*sqrt(3))/Vdc
// to get a value from -1 and 1
// like the three refrances in the simulation


void SVPWM(double I,double J,double K, double* S1, double* S2, double* S3){
			
	uint8_t F1,F2,F3,N,sector;
	double T1,T2,T0half;
	if(I>=0){F1=1;}else{F1=0;}
	if(J>=0){F2=1;}else{F2=0;}
	if(K>=0){F3=1;}else{F3=0;}
	
	N=F1+2*F2+4*F3;

	// N condition
	switch(N){
		case 1:
			sector=6;
			break;
		case 2:
			sector=2;
			break;				
		case 3:
			sector=1;
			break;		
		case 4:
			sector=4;
			break;		
		case 5:
			sector=5;
			break;
		case 6:
			sector=3;
			break;				
		default:
			sector=0;
	}
	
	// sector condition 
	switch(sector){
		case 1:
			T1=I;
			T2=J;
			break;
		case 2:
			T1=-K;
			T2=-I;
			break;
		case 3:
			T1=J;
			T2=K;
			break;
		case 4:
			T1=-I;
			T2=-J;
			break;	
		case 5:
			T1=K;
			T2=I;
			break;
		case 6:
			T1=-J;
			T2=-K;
			break;	
	}
	T0half=.5*(1-T1-T2);
	
	
	// Sfactors
	switch(sector){
		case 1:
			I=T1+T2+T0half;
			J=T2+T0half;
			K=T0half;
			break;
		case 2:
			I=T1+T0half;
			J=T1+T2+T0half;
			K=T0half;
			break;
		case 3:
			I=T0half;
			J=T1+T2+T0half;
			K=T2+T0half;
			break;
		case 4:
			I=T0half;
			J=T1+T0half;
			K=T1+T2+T0half;
		  break;
		case 5:
			I=T2+T0half;
			J=T0half;
			K=T1+T2+T0half;
			break;
		case 6:
			I=T1+T2+T0half;
			J=T0half;
			K=T1+T0half;
		  break;	
		default:
			I=J=K=0;	
	}

	*S1=I;
	*S2=J;
	*S3=K;
	
}






