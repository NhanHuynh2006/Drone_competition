/* dwm1001.h - DWM1001 UWB driver (port of Python decawave_1001 + uwb_nav) */
#ifndef DWM1001_H
#define DWM1001_H
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "kalman2d.h"
typedef struct { int32_t x,y,z; uint8_t quality; } dwm_pos_t;
typedef struct { uint16_t addr; int32_t dist_mm; uint8_t quality; int32_t ax,ay,az; double last_seen; } dwm_anchor_t;
#define DWM_MAX_ANCHORS 8
typedef struct {
    int fd; pthread_mutex_t smtx;
    KalmanFilter2D kalman;
    dwm_pos_t raw; double fx,fy; int alt_mm,qual;
    dwm_anchor_t anchors[DWM_MAX_ANCHORS]; int n_anch;
    int cons_err,tot_rd,tot_err; bool loc_rdy,net_join; double last_good;
    pthread_mutex_t dmtx; pthread_t thr; bool running; int upd_cnt; double hz;
} UWBNav;
int  uwb_init(UWBNav *n,const char *port,double pn,double mn,double hz);
int  uwb_start(UWBNav *n);
void uwb_stop(UWBNav *n);
void uwb_raw(UWBNav *n,int32_t *x,int32_t *y,int32_t *z);
void uwb_filtered(UWBNav *n,double *x,double *y);
void uwb_vel(UWBNav *n,double *vx,double *vy);
int  uwb_qual(UWBNav *n);
bool uwb_healthy(UWBNav *n);
double uwb_dist(UWBNav *n,double tx,double ty);
double uwb_bear(UWBNav *n,double tx,double ty);
void uwb_nav_to(UWBNav *n,double tx,double ty,double ms,double ar,double *vx,double *vy,bool *arr);
#endif
