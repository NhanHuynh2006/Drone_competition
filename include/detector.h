/* detector.h - YOLO NCNN detector (port of Python detector.py) */
#ifndef DETECTOR_H
#define DETECTOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <pthread.h>
#define MAX_DET 20
typedef struct {
    int cls; float conf; int x1,y1,x2,y2,cx,cy,w,h; float rel_area;
    const char *name; /* "hoop_red","hoop_yellow","hoop_blue","landing_pad","white_sq","hoop_color" */
} Det;
/* NCNN detector */
typedef struct NCNNDetImpl NCNNDetImpl;
typedef struct { NCNNDetImpl *impl; int sz; float ct,it; float fps,avg_ms; } NCNNDet;
NCNNDet* ncnn_det_create(const char *param, const char *bin, int sz, float ct, float it, int nth);
int  ncnn_det_run(NCNNDet *d, const unsigned char *bgr, int w, int h, Det *out, int mx);
void ncnn_det_destroy(NCNNDet *d);
/* Async wrapper */
typedef struct {
    NCNNDet *det; Det latest[MAX_DET]; int n; unsigned char *buf; int fw,fh;
    bool ready; pthread_mutex_t lk; pthread_cond_t cv; pthread_t thr; bool run; int cnt;
} AsyncDet;
AsyncDet* adet_create(NCNNDet *d, int fw, int fh);
void adet_submit(AsyncDet *a, const unsigned char *bgr);
int  adet_get(AsyncDet *a, Det *out, int mx);
void adet_destroy(AsyncDet *a);
#ifdef __cplusplus
}
#endif
#endif
