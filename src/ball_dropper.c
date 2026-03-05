/* ball_dropper.c - RPi5 PWM servo via sysfs (port of Python ball_dropper.py) */
#include "ball_dropper.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define PWM_PERIOD_NS 20000000  /* 20ms = 50Hz servo */

static int pwm_write(int chip, int ch, const char *attr, const char *val){
    char path[128];
    snprintf(path,sizeof(path),"/sys/class/pwm/pwmchip%d/pwm%d/%s",chip,ch,attr);
    int fd=open(path,O_WRONLY);if(fd<0)return -1;
    write(fd,val,strlen(val));close(fd);return 0;
}
static int pwm_export(int chip, int ch){
    char path[128],val[8];
    snprintf(path,sizeof(path),"/sys/class/pwm/pwmchip%d/export",chip);
    int fd=open(path,O_WRONLY);if(fd<0)return -1;
    snprintf(val,sizeof(val),"%d",ch);write(fd,val,strlen(val));close(fd);
    usleep(100000);return 0;
}
static void set_duty(BallDropper *b, int us){
    char val[16];long ns=(long)us*1000L;snprintf(val,sizeof(val),"%ld",ns);
    pwm_write(b->pwm_chip,b->pwm_channel,"duty_cycle",val);
}
int bd_init(BallDropper *b, int chip, int channel, int close_us, int open_us){
    b->pwm_chip=chip;b->pwm_channel=channel;b->close_us=close_us;b->open_us=open_us;
    b->is_open=false;b->initialized=false;
    pwm_export(chip,channel);
    char per[16];snprintf(per,sizeof(per),"%d",PWM_PERIOD_NS);
    pwm_write(chip,channel,"period",per);
    pwm_write(chip,channel,"enable","1");
    set_duty(b,close_us);b->initialized=true;
    printf("[DROP]Init chip%d/ch%d close=%d open=%d\n",chip,channel,close_us,open_us);
    return 0;
}
void bd_drop(BallDropper *b){set_duty(b,b->open_us);b->is_open=true;printf("[DROP]Open\n");}
void bd_close(BallDropper *b){set_duty(b,b->close_us);b->is_open=false;printf("[DROP]Close\n");}
void bd_drop_and_close(BallDropper *b, int delay_ms){
    bd_drop(b);usleep(delay_ms*1000);bd_close(b);
}
void bd_cleanup(BallDropper *b){if(b->initialized){bd_close(b);pwm_write(b->pwm_chip,b->pwm_channel,"enable","0");}}
