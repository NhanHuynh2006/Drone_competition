/* visual_servo.c - 1:1 port of Python visual_servo.py */
#include "visual_servo.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

void vs_init(VisualServo *v,int fw,int fh,double kp,double ki,double kd,double mo,double dz){
    v->fw=fw;v->fh=fh;v->ccx=fw/2;v->ccy=fh/2;
    pid_init(&v->px,kp,ki,kd,mo,dz,0.3);
    pid_init(&v->py,kp,ki,kd,mo,dz,0.3);
    v->mode=VS_IDLE;v->stab_ctr=0;v->target_cls=-1;
}
void vs_mode(VisualServo *v,VSMode m,int cls){v->mode=m;v->target_cls=cls;v->stab_ctr=0;pid_reset(&v->px);pid_reset(&v->py);}
void vs_reset(VisualServo *v){v->mode=VS_IDLE;v->stab_ctr=0;pid_reset(&v->px);pid_reset(&v->py);}

/* Find best detection matching target class */
static const Det* find_target(const Det *d,int nd,int cls){
    const Det *best=NULL;float ba=-1;
    for(int i=0;i<nd;i++){
        if(cls>=0 && d[i].cls!=cls) continue;
        if(d[i].rel_area>ba){ba=d[i].rel_area;best=&d[i];}
    }
    return best;
}

VSResult vs_compute(VisualServo *v,const Det *d,int nd,float alt){
    VSResult r;memset(&r,0,sizeof(r));r.status="idle";
    if(v->mode==VS_IDLE) return r;

    const Det *t=find_target(d,nd,v->target_cls);
    if(!t){r.status="no_target";return r;}

    /* Error in pixels, normalized to [-1,1] (same as Python) */
    float ex=(float)(t->cx-v->ccx)/(float)(v->fw/2);
    float ey=(float)(t->cy-v->ccy)/(float)(v->fh/2);
    r.ex=ex;r.ey=ey;r.area=t->rel_area;
    r.dist=sqrt(ex*ex+ey*ey);

    /* PID compute -> velocity (same as Python) */
    float vx_cmd=-(float)pid_compute(&v->py,ey); /* forward = -ey (ArduPilot NED) */
    float vy_cmd= (float)pid_compute(&v->px,ex); /* right   =  ex */

    /* Stability check (same as Python: error<0.1 for N frames) */
    if(r.dist<0.1) v->stab_ctr++; else v->stab_ctr=0;
    r.stable=v->stab_ctr>10;
    r.stab_cnt=v->stab_ctr;

    switch(v->mode){
    case VS_CENTER:
        r.vx=vx_cmd;r.vy=vy_cmd;r.vz=0;r.status="centering";break;
    case VS_FLY_THROUGH:
        r.vx=vx_cmd;r.vy=vy_cmd;
        /* Forward speed based on area (same as Python) */
        if(r.stable && r.area>0.01){r.fwd=0.4;r.vx+=0.4;r.status="flying_through";}
        else{r.status="aligning";}
        if(r.area>0.3){r.passed=true;r.status="passed_through";}
        break;
    case VS_DESCEND:
        r.vx=vx_cmd*0.5;r.vy=vy_cmd*0.5;
        r.vz=0.15; /* descend slowly */
        if(alt<0.15){r.landed=true;r.status="landed";}
        else r.status="descending";
        break;
    case VS_DROP:
        r.vx=vx_cmd;r.vy=vy_cmd;r.vz=0;
        r.status=r.stable?"drop_ready":"aligning_drop";
        break;
    default:break;
    }
    return r;
}
