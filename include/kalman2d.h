#ifndef KALMAN2D_H
#define KALMAN2D_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    double x[4]; double P[4][4]; double Q_base, R_base, last_time; bool initialized;
} KalmanFilter2D;
void   kf2d_init(KalmanFilter2D *k, double pn, double mn);
void   kf2d_reset(KalmanFilter2D *k);
void   kf2d_process(KalmanFilter2D *k, double mx, double my, int q, double *ox, double *oy);
void   kf2d_get_velocity(const KalmanFilter2D *k, double *vx, double *vy);
double kf2d_uncertainty(const KalmanFilter2D *k);
#endif
