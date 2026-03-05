/* ball_dropper.h - port of Python ball_dropper.py (RPi5 hardware PWM) */
#ifndef BALL_DROPPER_H
#define BALL_DROPPER_H
#include <stdbool.h>
typedef struct { int pwm_chip; int pwm_channel; int close_us; int open_us; bool is_open; bool initialized; } BallDropper;
int  bd_init(BallDropper *b, int chip, int channel, int close_us, int open_us);
void bd_drop(BallDropper *b);
void bd_close(BallDropper *b);
void bd_drop_and_close(BallDropper *b, int delay_ms);
void bd_cleanup(BallDropper *b);
#endif
