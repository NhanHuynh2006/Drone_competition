/* flight_controller.h - MAVLink over UDP (port of Python flight_controller.py) */
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <netinet/in.h>
#include "pid.h"
/* Callback types for sensor fusion integration */
typedef void (*fc_imu_cb_t)(float ax, float ay, float az, float gx, float gy, float gz, void *ctx);
typedef void (*fc_att_cb_t)(float roll, float pitch, float yaw, float rs, float ps, float ys, void *ctx);
typedef void (*fc_range_cb_t)(float distance, void *ctx);
typedef void (*fc_baro_cb_t)(float alt, void *ctx);

typedef struct {
    int sock; struct sockaddr_in dest; socklen_t dlen;
    uint8_t tsys,tcomp;
    float vcmd[3]; pthread_mutex_t vmtx;
    PIDController apid; float talt; bool alt_hold;
    float cur_alt,lat,lon,hdg; bool armed; char mode[16];
    /* IMU/Attitude/Rangefinder from Pixhawk6C */
    float imu_ax,imu_ay,imu_az;     /* m/s^2, SCALED_IMU2 */
    float imu_gx,imu_gy,imu_gz;     /* rad/s */
    float att_roll,att_pitch,att_yaw; /* rad, ATTITUDE */
    float att_rs,att_ps,att_ys;       /* rad/s */
    float range_dist;                 /* m, RANGEFINDER */
    /* Sensor fusion callbacks - called from recv thread */
    fc_imu_cb_t   imu_cb;   void *imu_ctx;
    fc_att_cb_t   att_cb;   void *att_ctx;
    fc_range_cb_t range_cb; void *range_ctx;
    fc_baro_cb_t  baro_cb;  void *baro_ctx;
    pthread_mutex_t tmtx; pthread_t sthr,rthr; bool running;
} FC;
int  fc_connect(FC *f,const char *ip,uint16_t port);
void fc_vel(FC *f,float vx,float vy,float vz);
void fc_hover(FC *f);
void fc_set_alt(FC *f,float a);
int  fc_arm_takeoff(FC *f,float a);
void fc_land(FC *f);
float fc_alt(FC *f);
bool fc_armed(FC *f);
void fc_shutdown(FC *f);
/* Register sensor fusion callbacks */
void fc_set_imu_callback(FC *f, fc_imu_cb_t cb, void *ctx);
void fc_set_att_callback(FC *f, fc_att_cb_t cb, void *ctx);
void fc_set_range_callback(FC *f, fc_range_cb_t cb, void *ctx);
void fc_set_baro_callback(FC *f, fc_baro_cb_t cb, void *ctx);
#endif
