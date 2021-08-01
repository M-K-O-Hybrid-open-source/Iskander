// motor c/ch
#define Rs 18.7	
#define Ls .02682
#define Phmig .1717


#define Jm 2.26e-5
#define Bv 1.1e-4
#define Zp 2

#define Vdc 300
#define alpha .9

#define zeta .707


// velocity controller
#define alphav .8
#define a Bv/Jm
#define b 1.5*Zp*Zp*Phmig*alphav/Jm
#define zetav .707
#define BWv 16.7
#define Kcv (2*zetav*BWv-a)/b
#define tauiv (2*zetav*BWv-a)/(BWv*BWv)


// Id controller
#define BWd  (1/(1-alpha))*Rs/Ls
#define Kcd  2/zeta*BWd*Ls-Rs
#define tauid  Kcq/(Ls*BWd*BWd)

// Iq controller
#define BWq  (1/(1-alpha))*Rs/Ls
#define Kcq  2/zeta*BWq*Ls-Rs
#define tauiq  Kcq/(Ls*BWq*BWq)

