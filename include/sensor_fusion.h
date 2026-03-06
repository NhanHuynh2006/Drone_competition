/* sensor_fusion.h - Multi-sensor EKF fusion
   Fuses: UWB position + Optical Flow velocity + Pixhawk6C IMU/Baro/Rangefinder
   State: [x, y, z, vx, vy, vz, ax_bias, ay_bias]
   
   Data sources:
   - UWB DWM1001:    position (x,y) @ 10-30Hz, quality-aware noise
   - Optical Flow:   velocity (vx,vy) from camera @ 30Hz via calcOpticalFlowPyrLK
   - Pixhawk6C IMU:  acceleration (ax,ay,az) via SCALED_IMU2 @ 50Hz
   - Pixhawk6C Baro: altitude via GLOBAL_POSITION_INT.relative_alt
   - Rangefinder:    altitude via RANGEFINDER @ 10-50Hz (downward lidar/sonar)
   - Attitude:       roll,pitch,yaw via ATTITUDE @ 50Hz (for frame rotation)
*/
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#define SF_STATE_DIM 8  /* x,y,z,vx,vy,vz,ax_bias,ay_bias */

/* Optical flow result from camera */
typedef struct {
    float vx, vy;           /* velocity estimate m/s (body frame) */
    float quality;          /* 0-1, based on number of good tracked points */
    double timestamp;
} OptFlowData;

/* IMU data from Pixhawk6C (SCALED_IMU2 message) */
typedef struct {
    float ax, ay, az;       /* acceleration m/s^2 (body frame, gravity removed) */
    float gx, gy, gz;       /* gyro rad/s */
    double timestamp;
} IMUData;

/* Attitude from Pixhawk6C (ATTITUDE message) */
typedef struct {
    float roll, pitch, yaw;  /* radians */
    float rollspeed, pitchspeed, yawspeed;
    double timestamp;
} AttitudeData;

/* Rangefinder from Pixhawk6C (RANGEFINDER message) */
typedef struct {
    float distance;         /* meters, downward */
    double timestamp;
} RangeData;

/* Baro altitude from GLOBAL_POSITION_INT */
typedef struct {
    float alt;              /* relative altitude meters */
    double timestamp;
} BaroData;

/* Fused output */
typedef struct {
    double x, y, z;         /* position meters (world/NED frame) */
    double vx, vy, vz;      /* velocity m/s */
    double ax_bias, ay_bias; /* estimated accelerometer bias */
    double uncertainty_xy;   /* position uncertainty sqrt(Pxx+Pyy) */
    double uncertainty_z;    /* altitude uncertainty */
    int    uwb_updates;
    int    flow_updates;
    int    imu_updates;
    int    range_count;
    double last_update;
} FusedState;

/* Main sensor fusion struct */
typedef struct {
    /* EKF state */
    double x[SF_STATE_DIM];
    double P[SF_STATE_DIM][SF_STATE_DIM];
    double last_predict_time;
    bool   initialized;

    /* Noise parameters (tunable) */
    double q_pos;           /* process noise position */
    double q_vel;           /* process noise velocity */
    double q_bias;          /* process noise accel bias */
    double r_uwb_base;      /* UWB measurement noise base */
    double r_flow;          /* optical flow noise */
    double r_range;         /* rangefinder noise */
    double r_baro;          /* barometer noise */
    double r_imu;           /* IMU accel noise */

    /* Latest sensor data (for interpolation/staleness check) */
    AttitudeData attitude;
    IMUData      last_imu;
    double       last_uwb_time;
    double       last_flow_time;
    double       last_range_time;

    /* Thread safety */
    pthread_mutex_t mtx;

    /* Stats */
    int uwb_count, flow_count, imu_count, range_count, baro_count;
} SensorFusion;

/* Init/reset */
void sf_init(SensorFusion *sf);
void sf_set_noise(SensorFusion *sf,
                  double q_pos, double q_vel, double q_bias,
                  double r_uwb, double r_flow, double r_range, double r_baro);

/* Prediction step - call at IMU rate (~50Hz) */
void sf_predict_imu(SensorFusion *sf, const IMUData *imu, const AttitudeData *att);

/* Update steps - call when each sensor has new data */
void sf_update_uwb(SensorFusion *sf, double ux, double uy, int quality);
void sf_update_optflow(SensorFusion *sf, const OptFlowData *flow, const AttitudeData *att, float altitude);
void sf_update_rangefinder(SensorFusion *sf, const RangeData *range);
void sf_update_baro(SensorFusion *sf, const BaroData *baro);
void sf_update_attitude(SensorFusion *sf, const AttitudeData *att);

/* Get fused result (thread-safe) */
void sf_get_state(SensorFusion *sf, FusedState *out);

/* Get position only (convenience) */
void sf_get_position(SensorFusion *sf, double *x, double *y, double *z);
void sf_get_velocity(SensorFusion *sf, double *vx, double *vy, double *vz);

/* Optical flow computation from camera frames */
typedef struct {
    /* Previous frame grayscale + points */
    unsigned char *prev_gray;
    float prev_pts[200][2];  /* tracked points */
    int   n_pts;
    int   frame_w, frame_h;
    bool  has_prev;
    int   frame_count;
    int   detect_interval;   /* re-detect features every N frames */
    double last_time;
} OptFlowTracker;

void oft_init(OptFlowTracker *t, int w, int h, int detect_interval);
/* Process new BGR frame, output flow velocity in body frame
   altitude needed to convert pixel motion to real velocity */
OptFlowData oft_process(OptFlowTracker *t, const unsigned char *bgr, int w, int h, float altitude);
void oft_reset(OptFlowTracker *t);
void oft_destroy(OptFlowTracker *t);

#ifdef __cplusplus
}
#endif

#endif
