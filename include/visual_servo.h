/* visual_servo.h - port of Python visual_servo.py */
#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H
#include "pid.h"
#include "detector.h"
#include <stdbool.h>
typedef enum { VS_IDLE=0, VS_CENTER, VS_FLY_THROUGH, VS_DESCEND, VS_DROP } VSMode;
typedef struct { float vx,vy,vz,ex,ey,dist,area,fwd; bool stable,passed,landed; int stab_cnt; const char *status; } VSResult;
typedef struct { int fw,fh,ccx,ccy; PIDController px,py; VSMode mode; int stab_ctr,target_cls; } VisualServo;
void vs_init(VisualServo *v,int fw,int fh,double kp,double ki,double kd,double mo,double dz);
void vs_mode(VisualServo *v,VSMode m,int cls);
VSResult vs_compute(VisualServo *v,const Det *d,int nd,float alt);
void vs_reset(VisualServo *v);
#endif
