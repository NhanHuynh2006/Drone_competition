/* sensor_fusion.cpp - Multi-sensor EKF fusion
   UWB + Optical Flow + Pixhawk6C IMU/Baro/Rangefinder
   
   EKF State [8]: x, y, z, vx, vy, vz, ax_bias, ay_bias
   
   Predict: IMU accel (body->world via attitude) drives state forward
   Update sources:
     - UWB: measures (x,y) with quality-dependent noise
     - Optical Flow: measures (vx,vy) body frame, rotate to world
     - Rangefinder: measures z (altitude)
     - Baro: measures z (altitude, noisier than rangefinder)
   
   Coordinate frame: NED (North-East-Down), z positive downward
   But for indoor we use: x=forward, y=right, z=altitude(up positive)
*/
#include "sensor_fusion.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <time.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

/* ====== Matrix helpers for 8x8 EKF ====== */
#define N SF_STATE_DIM

static double mono_s(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static void mat_zero(double m[N][N]) { memset(m, 0, sizeof(double)*N*N); }
static void mat_eye(double m[N][N]) { mat_zero(m); for(int i=0;i<N;i++) m[i][i]=1.0; }
static void mat_copy(double dst[N][N], const double src[N][N]) { memcpy(dst,src,sizeof(double)*N*N); }

static void mat_mul(const double A[N][N], const double B[N][N], double C[N][N]) {
    double tmp[N][N];
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        tmp[i][j]=0; for(int k=0;k<N;k++) tmp[i][j]+=A[i][k]*B[k][j];
    }
    memcpy(C,tmp,sizeof(double)*N*N);
}
static void mat_trans(const double A[N][N], double AT[N][N]) {
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) AT[j][i]=A[i][j];
}
static void mat_add(double A[N][N], const double B[N][N]) {
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) A[i][j]+=B[i][j];
}

/* Body to world rotation using roll, pitch, yaw */
static void body_to_world(float roll, float pitch, float yaw,
                          float bx, float by, float bz,
                          double *wx, double *wy, double *wz) {
    float cr=cosf(roll), sr=sinf(roll);
    float cp=cosf(pitch), sp=sinf(pitch);
    float cy=cosf(yaw), sy=sinf(yaw);
    /* Rotation matrix R = Rz(yaw)*Ry(pitch)*Rx(roll) */
    *wx = (cy*cp)*bx + (cy*sp*sr - sy*cr)*by + (cy*sp*cr + sy*sr)*bz;
    *wy = (sy*cp)*bx + (sy*sp*sr + cy*cr)*by + (sy*sp*cr - cy*sr)*bz;
    *wz = (-sp)*bx   + (cp*sr)*by             + (cp*cr)*bz;
}

extern "C" {

void sf_init(SensorFusion *sf) {
    memset(sf, 0, sizeof(*sf));
    mat_eye(sf->P);
    for(int i=0;i<N;i++) sf->P[i][i] = 100.0; /* large initial uncertainty */
    sf->initialized = false;
    sf->last_predict_time = -1;

    /* Default noise params */
    sf->q_pos  = 0.01;
    sf->q_vel  = 0.5;
    sf->q_bias = 0.001;
    sf->r_uwb_base = 0.05;  /* 50mm base noise */
    sf->r_flow = 0.3;
    sf->r_range = 0.02;     /* 20mm rangefinder */
    sf->r_baro = 0.5;       /* 500mm baro */
    sf->r_imu  = 0.1;

    pthread_mutex_init(&sf->mtx, NULL);
    printf("[SF] Sensor Fusion EKF initialized (state dim=%d)\n", N);
}

void sf_set_noise(SensorFusion *sf,
                  double q_pos, double q_vel, double q_bias,
                  double r_uwb, double r_flow, double r_range, double r_baro) {
    sf->q_pos=q_pos; sf->q_vel=q_vel; sf->q_bias=q_bias;
    sf->r_uwb_base=r_uwb; sf->r_flow=r_flow; sf->r_range=r_range; sf->r_baro=r_baro;
}

/* ====== EKF PREDICT with IMU ====== */
void sf_predict_imu(SensorFusion *sf, const IMUData *imu, const AttitudeData *att) {
    pthread_mutex_lock(&sf->mtx);
    double now = imu->timestamp;

    if (!sf->initialized || sf->last_predict_time < 0) {
        sf->last_predict_time = now;
        sf->last_imu = *imu;
        if (att) sf->attitude = *att;
        pthread_mutex_unlock(&sf->mtx);
        return;
    }

    double dt = now - sf->last_predict_time;
    if (dt <= 0 || dt > 1.0) { sf->last_predict_time = now; pthread_mutex_unlock(&sf->mtx); return; }
    sf->last_predict_time = now;
    sf->last_imu = *imu;
    sf->imu_count++;

    /* Remove bias from accel */
    float ax_cor = imu->ax - (float)sf->x[6];
    float ay_cor = imu->ay - (float)sf->x[7];
    float az_cor = imu->az;

    /* Rotate body accel to world frame */
    double awx, awy, awz;
    float r = att ? att->roll : sf->attitude.roll;
    float p = att ? att->pitch : sf->attitude.pitch;
    float y = att ? att->yaw : sf->attitude.yaw;
    body_to_world(r, p, y, ax_cor, ay_cor, az_cor, &awx, &awy, &awz);

    /* State prediction: constant acceleration model
       x  += vx*dt + 0.5*ax*dt^2
       vx += ax*dt
       bias stays constant */
    double dt2 = dt*dt;
    sf->x[0] += sf->x[3]*dt + 0.5*awx*dt2;  /* x */
    sf->x[1] += sf->x[4]*dt + 0.5*awy*dt2;  /* y */
    sf->x[2] += sf->x[5]*dt + 0.5*awz*dt2;  /* z */
    sf->x[3] += awx*dt;                       /* vx */
    sf->x[4] += awy*dt;                       /* vy */
    sf->x[5] += awz*dt;                       /* vz */
    /* x[6], x[7] bias unchanged */

    /* F = Jacobian of state transition */
    double F[N][N]; mat_eye(F);
    F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;
    /* dx/d_bias_ax: through rotation */
    F[0][6] = -0.5*dt2; /* simplified: bias mainly affects own axis */
    F[1][7] = -0.5*dt2;
    F[3][6] = -dt;
    F[4][7] = -dt;

    /* P = F*P*F' + Q */
    double FT[N][N], tmp[N][N];
    mat_trans(F, FT);
    mat_mul(F, sf->P, tmp);
    mat_mul(tmp, FT, sf->P);

    /* Process noise Q */
    double Q[N][N]; mat_zero(Q);
    double dt3 = dt2*dt, dt4=dt3*dt;
    /* Position process noise driven by acceleration uncertainty */
    Q[0][0] = sf->q_pos + sf->q_vel*dt3/3;
    Q[1][1] = sf->q_pos + sf->q_vel*dt3/3;
    Q[2][2] = sf->q_pos + sf->q_vel*dt3/3;
    Q[3][3] = sf->q_vel*dt;
    Q[4][4] = sf->q_vel*dt;
    Q[5][5] = sf->q_vel*dt;
    Q[0][3] = Q[3][0] = sf->q_vel*dt2/2;
    Q[1][4] = Q[4][1] = sf->q_vel*dt2/2;
    Q[2][5] = Q[5][2] = sf->q_vel*dt2/2;
    Q[6][6] = sf->q_bias*dt;
    Q[7][7] = sf->q_bias*dt;
    mat_add(sf->P, Q);

    pthread_mutex_unlock(&sf->mtx);
}

/* ====== Generic scalar update helper ====== */
/* Updates state x and covariance P given:
   H[N] = measurement Jacobian row, z = measurement, h = predicted measurement, R = noise */
static void scalar_update(double x[N], double P[N][N],
                          const double H[N], double z, double h, double R) {
    /* Innovation */
    double y = z - h;

    /* S = H*P*H' + R (scalar) */
    double S = R;
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) S += H[i]*P[i][j]*H[j];
    if (fabs(S) < 1e-15) return;

    /* K = P*H'/S */
    double K[N];
    for(int i=0;i<N;i++) {
        K[i] = 0;
        for(int j=0;j<N;j++) K[i] += P[i][j]*H[j];
        K[i] /= S;
    }

    /* x = x + K*y */
    for(int i=0;i<N;i++) x[i] += K[i]*y;

    /* P = (I-K*H)*P (Joseph form for stability) */
    double IKH[N][N]; mat_eye(IKH);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) IKH[i][j] -= K[i]*H[j];
    double IKHT[N][N], tmp[N][N], Pnew[N][N];
    mat_trans(IKH, IKHT);
    mat_mul(IKH, P, tmp);
    mat_mul(tmp, IKHT, Pnew);
    /* + K*R*K' */
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) Pnew[i][j] += R*K[i]*K[j];
    mat_copy(P, Pnew);
}

/* ====== UWB UPDATE ====== */
void sf_update_uwb(SensorFusion *sf, double ux, double uy, int quality) {
    pthread_mutex_lock(&sf->mtx);

    if (!sf->initialized) {
        sf->x[0] = ux; sf->x[1] = uy;
        sf->initialized = true;
        sf->last_predict_time = mono_s();
        printf("[SF] Initialized at UWB (%.0f, %.0f)\n", ux, uy);
    }

    /* Quality-aware noise: same logic as original kalman2d */
    double noise;
    if (quality <= 0) noise = sf->r_uwb_base * 100.0;
    else if (quality >= 100) noise = sf->r_uwb_base * 0.5;
    else noise = sf->r_uwb_base * (100.0 / (double)quality);

    /* Update X: H = [1,0,0,0,0,0,0,0] */
    double Hx[N] = {0}; Hx[0] = 1.0;
    scalar_update(sf->x, sf->P, Hx, ux, sf->x[0], noise);

    /* Update Y: H = [0,1,0,0,0,0,0,0] */
    double Hy[N] = {0}; Hy[1] = 1.0;
    scalar_update(sf->x, sf->P, Hy, uy, sf->x[1], noise);

    sf->uwb_count++;
    sf->last_uwb_time = mono_s();
    pthread_mutex_unlock(&sf->mtx);
}

/* ====== OPTICAL FLOW UPDATE ====== */
void sf_update_optflow(SensorFusion *sf, const OptFlowData *flow,
                       const AttitudeData *att, float altitude) {
    pthread_mutex_lock(&sf->mtx);
    if (!sf->initialized) { pthread_mutex_unlock(&sf->mtx); return; }

    /* Rotate flow velocity from body to world frame */
    double wvx, wvy, wvz;
    float r = att ? att->roll : sf->attitude.roll;
    float p = att ? att->pitch : sf->attitude.pitch;
    float y = att ? att->yaw : sf->attitude.yaw;
    body_to_world(r, p, y, flow->vx, flow->vy, 0, &wvx, &wvy, &wvz);

    /* Noise based on flow quality and altitude */
    double noise = sf->r_flow;
    if (flow->quality < 0.3) noise *= 5.0;   /* low quality = high noise */
    else if (flow->quality < 0.6) noise *= 2.0;
    if (altitude > 2.0) noise *= (altitude / 2.0); /* higher = less accurate */

    /* Update Vx: H = [0,0,0,1,0,0,0,0] */
    double Hvx[N] = {0}; Hvx[3] = 1.0;
    scalar_update(sf->x, sf->P, Hvx, wvx, sf->x[3], noise);

    /* Update Vy: H = [0,0,0,0,1,0,0,0] */
    double Hvy[N] = {0}; Hvy[4] = 1.0;
    scalar_update(sf->x, sf->P, Hvy, wvy, sf->x[4], noise);

    sf->flow_count++;
    sf->last_flow_time = mono_s();
    pthread_mutex_unlock(&sf->mtx);
}

/* ====== RANGEFINDER UPDATE ====== */
void sf_update_rangefinder(SensorFusion *sf, const RangeData *range) {
    pthread_mutex_lock(&sf->mtx);
    if (!sf->initialized) { pthread_mutex_unlock(&sf->mtx); return; }

    /* Rangefinder measures altitude (z) directly */
    if (range->distance <= 0 || range->distance > 10.0) {
        pthread_mutex_unlock(&sf->mtx); return; /* invalid */
    }

    double Hz[N] = {0}; Hz[2] = 1.0;
    scalar_update(sf->x, sf->P, Hz, (double)range->distance, sf->x[2], sf->r_range);

    sf->range_count++;
    sf->last_range_time = mono_s();
    pthread_mutex_unlock(&sf->mtx);
}

/* ====== BARO UPDATE ====== */
void sf_update_baro(SensorFusion *sf, const BaroData *baro) {
    pthread_mutex_lock(&sf->mtx);
    if (!sf->initialized) { pthread_mutex_unlock(&sf->mtx); return; }

    double Hz[N] = {0}; Hz[2] = 1.0;
    scalar_update(sf->x, sf->P, Hz, (double)baro->alt, sf->x[2], sf->r_baro);

    sf->baro_count++;
    pthread_mutex_unlock(&sf->mtx);
}

/* ====== ATTITUDE UPDATE (just store, no EKF update) ====== */
void sf_update_attitude(SensorFusion *sf, const AttitudeData *att) {
    pthread_mutex_lock(&sf->mtx);
    sf->attitude = *att;
    pthread_mutex_unlock(&sf->mtx);
}

/* ====== GET STATE ====== */
void sf_get_state(SensorFusion *sf, FusedState *out) {
    pthread_mutex_lock(&sf->mtx);
    out->x = sf->x[0]; out->y = sf->x[1]; out->z = sf->x[2];
    out->vx = sf->x[3]; out->vy = sf->x[4]; out->vz = sf->x[5];
    out->ax_bias = sf->x[6]; out->ay_bias = sf->x[7];
    out->uncertainty_xy = sqrt(sf->P[0][0] + sf->P[1][1]);
    out->uncertainty_z = sqrt(sf->P[2][2]);
    out->uwb_updates = sf->uwb_count;
    out->flow_updates = sf->flow_count;
    out->imu_updates = sf->imu_count;
    out->last_update = sf->last_predict_time;
    pthread_mutex_unlock(&sf->mtx);
}

void sf_get_position(SensorFusion *sf, double *x, double *y, double *z) {
    pthread_mutex_lock(&sf->mtx);
    *x = sf->x[0]; *y = sf->x[1]; *z = sf->x[2];
    pthread_mutex_unlock(&sf->mtx);
}

void sf_get_velocity(SensorFusion *sf, double *vx, double *vy, double *vz) {
    pthread_mutex_lock(&sf->mtx);
    *vx = sf->x[3]; *vy = sf->x[4]; *vz = sf->x[5];
    pthread_mutex_unlock(&sf->mtx);
}

/* ================================================================
   OPTICAL FLOW TRACKER - OpenCV Lucas-Kanade
   Converts pixel motion to body-frame velocity using altitude
   ================================================================ */

void oft_init(OptFlowTracker *t, int w, int h, int detect_interval) {
    memset(t, 0, sizeof(*t));
    t->frame_w = w; t->frame_h = h;
    t->detect_interval = detect_interval;
    t->prev_gray = (unsigned char*)calloc(w * h, 1);
    t->has_prev = false;
    t->n_pts = 0;
    t->last_time = -1;
    printf("[OF] Tracker init %dx%d, detect every %d frames\n", w, h, detect_interval);
}

OptFlowData oft_process(OptFlowTracker *t, const unsigned char *bgr, int w, int h, float altitude) {
    OptFlowData result = {0, 0, 0, mono_s()};

    /* Convert to grayscale */
    cv::Mat color(h, w, CV_8UC3, (void*)bgr);
    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

    t->frame_count++;

    if (!t->has_prev || t->n_pts < 10 || (t->frame_count % t->detect_interval) == 0) {
        /* Detect new features using goodFeaturesToTrack (Shi-Tomasi corners) */
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(gray, corners, 200, 0.01, 10);
        t->n_pts = (int)corners.size();
        if (t->n_pts > 200) t->n_pts = 200;
        for (int i = 0; i < t->n_pts; i++) {
            t->prev_pts[i][0] = corners[i].x;
            t->prev_pts[i][1] = corners[i].y;
        }
        memcpy(t->prev_gray, gray.data, w * h);
        t->has_prev = true;
        t->last_time = result.timestamp;
        return result; /* no flow on first frame / re-detect */
    }

    double dt = result.timestamp - t->last_time;
    if (dt <= 0 || dt > 0.5) {
        memcpy(t->prev_gray, gray.data, w * h);
        t->last_time = result.timestamp;
        return result;
    }

    /* Lucas-Kanade optical flow */
    cv::Mat prev_mat(h, w, CV_8UC1, t->prev_gray);
    std::vector<cv::Point2f> prev_points(t->n_pts), next_points;
    for (int i = 0; i < t->n_pts; i++)
        prev_points[i] = cv::Point2f(t->prev_pts[i][0], t->prev_pts[i][1]);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_mat, gray, prev_points, next_points,
                             status, err, cv::Size(21, 21), 3);

    /* Compute average pixel displacement of good points */
    float sum_dx = 0, sum_dy = 0;
    int good = 0;
    for (int i = 0; i < t->n_pts; i++) {
        if (!status[i] || err[i] > 12.0f) continue;
        sum_dx += next_points[i].x - prev_points[i].x;
        sum_dy += next_points[i].y - prev_points[i].y;
        good++;
    }

    if (good >= 5) {
        float avg_dx = sum_dx / good;
        float avg_dy = sum_dy / good;

        /* Convert pixel motion to real velocity:
           v = (pixel_displacement * altitude) / (focal_length * dt)
           Approximate focal length from FOV: f = w / (2 * tan(FOV/2))
           For typical 62deg H-FOV at 640px: f ~ 540 pixels */
        float focal = (float)w * 0.84f; /* ~62 deg HFOV approx */
        if (altitude < 0.1f) altitude = 0.1f;

        /* Body frame: x=forward(up in image), y=right(right in image)
           Camera typically points down, so:
           flow_x (horizontal) -> vy (rightward)
           flow_y (vertical)   -> -vx (forward, image y is down) */
        result.vy =  (avg_dx * altitude) / (focal * (float)dt);
        result.vx = -(avg_dy * altitude) / (focal * (float)dt);
        result.quality = (float)good / (float)t->n_pts;

        /* Clamp unreasonable velocities */
        if (fabsf(result.vx) > 5.0f) result.vx = 0;
        if (fabsf(result.vy) > 5.0f) result.vy = 0;
    }

    /* Update tracked points for next frame */
    int j = 0;
    for (int i = 0; i < t->n_pts; i++) {
        if (status[i] && err[i] <= 12.0f) {
            t->prev_pts[j][0] = next_points[i].x;
            t->prev_pts[j][1] = next_points[i].y;
            j++;
        }
    }
    t->n_pts = j;

    memcpy(t->prev_gray, gray.data, w * h);
    t->last_time = result.timestamp;
    return result;
}

void oft_reset(OptFlowTracker *t) {
    t->has_prev = false; t->n_pts = 0; t->frame_count = 0;
}

void oft_destroy(OptFlowTracker *t) {
    if (t->prev_gray) { free(t->prev_gray); t->prev_gray = NULL; }
}

} /* extern "C" */
