/* kalman2d.c - Quality-aware 2D Kalman (port of Python KalmanFilter2D) */
#include "kalman2d.h"
#include <math.h>
#include <string.h>
#include <time.h>
static double ms(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static void mz(double m[4][4]){memset(m,0,sizeof(double)*16);}
static void me(double m[4][4]){mz(m);m[0][0]=m[1][1]=m[2][2]=m[3][3]=1.0;}
static void mm(const double A[4][4],const double B[4][4],double C[4][4]){
    double t[4][4];int i,j,k;for(i=0;i<4;i++)for(j=0;j<4;j++){t[i][j]=0;for(k=0;k<4;k++)t[i][j]+=A[i][k]*B[k][j];}memcpy(C,t,sizeof(double)*16);}
static void mt(const double A[4][4],double T[4][4]){int i,j;for(i=0;i<4;i++)for(j=0;j<4;j++)T[j][i]=A[i][j];}
static void ma(double A[4][4],const double B[4][4]){int i,j;for(i=0;i<4;i++)for(j=0;j<4;j++)A[i][j]+=B[i][j];}

void kf2d_init(KalmanFilter2D *k, double pn, double mn){
    memset(k->x,0,sizeof(k->x));me(k->P);
    k->P[0][0]=k->P[1][1]=k->P[2][2]=k->P[3][3]=1000.0;
    k->Q_base=pn;k->R_base=mn;k->last_time=-1;k->initialized=false;
}
void kf2d_reset(KalmanFilter2D *k){kf2d_init(k,k->Q_base,k->R_base);}

void kf2d_process(KalmanFilter2D *k, double mx, double my, int q, double *ox, double *oy){
    double now=ms();
    if(!k->initialized){k->x[0]=mx;k->x[1]=my;k->x[2]=0;k->x[3]=0;k->last_time=now;k->initialized=true;*ox=mx;*oy=my;return;}
    double dt=now-k->last_time; if(dt<=0)dt=0.01;
    if(dt>1.0){k->x[0]=mx;k->x[1]=my;k->x[2]=0;k->x[3]=0;k->last_time=now;*ox=mx;*oy=my;return;}
    k->last_time=now;
    /* PREDICT */
    double F[4][4];me(F);F[0][2]=dt;F[1][3]=dt;
    double xp[4]={k->x[0]+k->x[2]*dt,k->x[1]+k->x[3]*dt,k->x[2],k->x[3]};
    double FT[4][4],t1[4][4],Q[4][4];mt(F,FT);mm(F,k->P,t1);mm(t1,FT,k->P);
    double d2=dt*dt,d3=d2*dt;double qb=k->Q_base;mz(Q);
    Q[0][0]=qb*d3/3;Q[0][2]=qb*d2/2;Q[1][1]=qb*d3/3;Q[1][3]=qb*d2/2;
    Q[2][0]=qb*d2/2;Q[2][2]=qb*dt;Q[3][1]=qb*d2/2;Q[3][3]=qb*dt;
    ma(k->P,Q);memcpy(k->x,xp,sizeof(xp));
    /* UPDATE - quality-aware noise */
    double ns;
    if(q<=0)ns=k->R_base*100.0;else if(q>=100)ns=k->R_base*0.5;else ns=k->R_base*(100.0/(double)q);
    double y0=mx-k->x[0],y1=my-k->x[1];
    double S00=k->P[0][0]+ns,S01=k->P[0][1],S10=k->P[1][0],S11=k->P[1][1]+ns;
    double det=S00*S11-S01*S10;if(fabs(det)<1e-12){*ox=k->x[0];*oy=k->x[1];return;}
    double id=1.0/det,i00=S11*id,i01=-S01*id,i10=-S10*id,i11=S00*id;
    double K[4][2];int i;
    for(i=0;i<4;i++){double p0=k->P[i][0],p1=k->P[i][1];K[i][0]=p0*i00+p1*i10;K[i][1]=p0*i01+p1*i11;}
    for(i=0;i<4;i++)k->x[i]+=K[i][0]*y0+K[i][1]*y1;
    /* Joseph form P=(I-KH)P(I-KH)'+KRK' */
    double IKH[4][4];me(IKH);for(i=0;i<4;i++){IKH[i][0]-=K[i][0];IKH[i][1]-=K[i][1];}
    double IKHT[4][4],t2[4][4],Pn[4][4];mt(IKH,IKHT);mm(IKH,k->P,t1);mm(t1,IKHT,Pn);
    int j;for(i=0;i<4;i++)for(j=0;j<4;j++)Pn[i][j]+=ns*(K[i][0]*K[j][0]+K[i][1]*K[j][1]);
    memcpy(k->P,Pn,sizeof(Pn));*ox=k->x[0];*oy=k->x[1];
}
void kf2d_get_velocity(const KalmanFilter2D *k, double *vx, double *vy){*vx=k->x[2];*vy=k->x[3];}
double kf2d_uncertainty(const KalmanFilter2D *k){return sqrt(k->P[0][0]+k->P[1][1]);}
