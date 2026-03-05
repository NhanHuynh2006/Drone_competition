/* pid.c - 1:1 port of Python pid.py */
#include "pid.h"
#include <math.h>
#include <time.h>
static double mono_s(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}

void pid_init(PIDController *p, double kp, double ki, double kd, double mo, double dz, double al){
    p->kp=kp;p->ki=ki;p->kd=kd;p->max_output=mo;p->deadzone=dz;p->d_filter_alpha=al;
    p->integral=0;p->prev_error=0;p->prev_derivative=0;p->prev_time=-1;
}
void pid_reset(PIDController *p){p->integral=0;p->prev_error=0;p->prev_derivative=0;p->prev_time=-1;}

double pid_compute(PIDController *p, double error){
    double now=mono_s();
    if(fabs(error)<p->deadzone) error=0;
    if(p->prev_time<0){p->prev_time=now;p->prev_error=error;return p->kp*error;}
    double dt=now-p->prev_time; if(dt<=0||dt>1.0)dt=0.02; p->prev_time=now;
    double pt=p->kp*error;
    p->integral+=error*dt;
    double mi=p->max_output/fmax(p->ki,1e-6);
    if(p->integral>mi)p->integral=mi; if(p->integral<-mi)p->integral=-mi;
    double it=p->ki*p->integral;
    double rd=(error-p->prev_error)/dt;
    double df=p->d_filter_alpha*rd+(1.0-p->d_filter_alpha)*p->prev_derivative;
    p->prev_derivative=df; double dt2=p->kd*df; p->prev_error=error;
    double out=pt+it+dt2;
    if(out>p->max_output)out=p->max_output; if(out<-p->max_output)out=-p->max_output;
    return out;
}
void pid_set_gains(PIDController *p, double kp, double ki, double kd){
    if(kp>=0)p->kp=kp;if(ki>=0)p->ki=ki;if(kd>=0)p->kd=kd;
}
