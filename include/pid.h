#ifndef PID_H
#define PID_H
typedef struct {
    double kp, ki, kd, max_output, deadzone, d_filter_alpha;
    double integral, prev_error, prev_derivative, prev_time;
} PIDController;
void   pid_init(PIDController *p, double kp, double ki, double kd, double mo, double dz, double al);
void   pid_reset(PIDController *p);
double pid_compute(PIDController *p, double error);
void   pid_set_gains(PIDController *p, double kp, double ki, double kd);
#endif
