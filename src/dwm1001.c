/* dwm1001.c - 1:1 port of Python decawave_1001.py + uwb_nav.py */
#include "dwm1001.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <errno.h>

static double ms(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}

static int ser_open(const char *p){
    int fd=open(p,O_RDWR|O_NOCTTY|O_SYNC);if(fd<0){perror("[UWB]open");return -1;}
    struct termios t;tcgetattr(fd,&t);cfmakeraw(&t);
    cfsetispeed(&t,B115200);cfsetospeed(&t,B115200);
    t.c_cflag|=(CLOCAL|CREAD);t.c_cflag&=~CRTSCTS;t.c_cc[VMIN]=0;t.c_cc[VTIME]=1;
    tcflush(fd,TCIOFLUSH);tcsetattr(fd,TCSANOW,&t);return fd;
}
static int ser_rd(int fd,uint8_t *b,int n,int tms){
    int tot=0;double dl=ms()+tms/1000.0;
    while(tot<n&&ms()<dl){int r=read(fd,b+tot,n-tot);if(r>0)tot+=r;else usleep(1000);}return tot;
}
/* dwm_pos_get [0x02,0x00]->18B: same as Python get_pos()->DwmPositionResponse */
static int dpos(int fd,pthread_mutex_t *m,dwm_pos_t *p){
    uint8_t c[2]={0x02,0x00},r[18];
    pthread_mutex_lock(m);write(fd,c,2);int n=ser_rd(fd,r,18,200);pthread_mutex_unlock(m);
    if(n<18||r[0]!=0x40||r[2]!=0x00||r[3]!=0x41)return -1;
    uint8_t *d=&r[5];
    p->x=(int32_t)(d[0]|(d[1]<<8)|(d[2]<<16)|(d[3]<<24));
    p->y=(int32_t)(d[4]|(d[5]<<8)|(d[6]<<16)|(d[7]<<24));
    p->z=(int32_t)(d[8]|(d[9]<<8)|(d[10]<<16)|(d[11]<<24));
    p->quality=d[12];return 0;
}
/* dwm_status_get [0x32,0x00] */
static int dstat(int fd,pthread_mutex_t *m,bool *loc,bool *net){
    uint8_t c[2]={0x32,0x00},r[8];
    pthread_mutex_lock(m);write(fd,c,2);int n=ser_rd(fd,r,8,200);pthread_mutex_unlock(m);
    if(n<6||r[0]!=0x40||r[2]!=0x00)return -1;
    *loc=(r[5]&0x01)!=0;*net=(r[5]&0x02)!=0;return 0;
}
/* dwm_cfg_get [0x08,0x00] */
static int dcfg(int fd,pthread_mutex_t *m){
    uint8_t c[2]={0x08,0x00},r[8];
    pthread_mutex_lock(m);write(fd,c,2);int n=ser_rd(fd,r,8,200);pthread_mutex_unlock(m);
    if(n<7||r[0]!=0x40||r[2]!=0x00)return -1;
    printf("[UWB] Mode:%s LocEng:%d\n",!(r[6]&0x20)?"Tag":"Anchor",(r[5]&0x40)!=0);return 0;
}
/* dwm_loc_get [0x0C,0x00]: anchors */
static int dloc(int fd,pthread_mutex_t *m,dwm_anchor_t *a,int *na){
    uint8_t c[2]={0x0C,0x00},r[256];
    pthread_mutex_lock(m);write(fd,c,2);
    int n=ser_rd(fd,r,3,200);if(n<3||r[0]!=0x40||r[2]!=0x00){pthread_mutex_unlock(m);return -1;}
    n=3+ser_rd(fd,r+3,sizeof(r)-3,300);pthread_mutex_unlock(m);
    *na=0;if(n<21)return -2;
    if(n>20&&r[18]==0x49){
        int cnt=r[20];if(cnt>DWM_MAX_ANCHORS)cnt=DWM_MAX_ANCHORS;*na=cnt;
        for(int i=0;i<cnt&&(21+i*20+20)<=n;i++){
            uint8_t *d=&r[21+i*20];
            a[i].addr=(uint16_t)(d[0]|(d[1]<<8));
            a[i].dist_mm=(int32_t)(d[2]|(d[3]<<8)|(d[4]<<16)|(d[5]<<24));
            a[i].quality=d[6];
            a[i].ax=(int32_t)(d[7]|(d[8]<<8)|(d[9]<<16)|(d[10]<<24));
            a[i].ay=(int32_t)(d[11]|(d[12]<<8)|(d[13]<<16)|(d[14]<<24));
            a[i].az=(int32_t)(d[15]|(d[16]<<8)|(d[17]<<16)|(d[18]<<24));
            a[i].last_seen=ms();
        }
    }
    return 0;
}
static void herr(UWBNav *n){
    n->tot_err++;n->cons_err++;
    if(n->cons_err<=3)usleep(10000);
    else if(n->cons_err<=10){printf("[UWB]err x%d reset\n",n->cons_err);uint8_t b[3]={0xFF,0xFF,0xFF};write(n->fd,b,3);usleep(100000);}
    else{if(n->cons_err==11)printf("[UWB]too many err\n");usleep(500000);}
}
static void *uwb_poll(void *arg){
    UWBNav *n=(UWBNav*)arg;double iv=1.0/n->hz;int lc=0;
    while(n->running){
        double t0=ms();n->tot_rd++;lc++;
        dwm_pos_t p;
        if(dpos(n->fd,&n->smtx,&p)==0){
            double fx2,fy2;kf2d_process(&n->kalman,(double)p.x,(double)p.y,p.quality,&fx2,&fy2);
            pthread_mutex_lock(&n->dmtx);
            n->raw=p;n->fx=fx2;n->fy=fy2;n->alt_mm=p.z;n->qual=p.quality;
            n->upd_cnt++;n->cons_err=0;n->last_good=ms();
            pthread_mutex_unlock(&n->dmtx);
            if(lc>=10){lc=0;dwm_anchor_t a[DWM_MAX_ANCHORS];int na=0;
                if(dloc(n->fd,&n->smtx,a,&na)==0){pthread_mutex_lock(&n->dmtx);n->n_anch=na;memcpy(n->anchors,a,sizeof(dwm_anchor_t)*na);pthread_mutex_unlock(&n->dmtx);}
            }
        }else herr(n);
        double sl=iv-(ms()-t0);if(sl>0)usleep((useconds_t)(sl*1e6));
    }
    return NULL;
}
int uwb_init(UWBNav *n,const char *port,double pn,double mn,double hz){
    memset(n,0,sizeof(*n));pthread_mutex_init(&n->smtx,NULL);pthread_mutex_init(&n->dmtx,NULL);
    kf2d_init(&n->kalman,pn,mn);n->hz=hz;n->running=false;
    n->fd=ser_open(port);if(n->fd<0)return -1;
    printf("[UWB]Connected %s\n",port);dcfg(n->fd,&n->smtx);
    printf("[UWB]Waiting net...\n");double dl=ms()+10.0;
    while(ms()<dl){bool l,ne;if(dstat(n->fd,&n->smtx,&l,&ne)==0){n->loc_rdy=l;n->net_join=ne;if(l&&ne){printf("[UWB]Ready!\n");break;}}usleep(500000);}
    return 0;
}
int uwb_start(UWBNav *n){n->running=true;pthread_create(&n->thr,NULL,uwb_poll,n);printf("[UWB]Poll %.0fHz\n",n->hz);return 0;}
void uwb_stop(UWBNav *n){n->running=false;pthread_join(n->thr,NULL);if(n->fd>=0)close(n->fd);printf("[UWB]Stop\n");}
void uwb_raw(UWBNav *n,int32_t *x,int32_t *y,int32_t *z){pthread_mutex_lock(&n->dmtx);*x=n->raw.x;*y=n->raw.y;*z=n->raw.z;pthread_mutex_unlock(&n->dmtx);}
void uwb_filtered(UWBNav *n,double *x,double *y){pthread_mutex_lock(&n->dmtx);*x=n->fx;*y=n->fy;pthread_mutex_unlock(&n->dmtx);}
void uwb_vel(UWBNav *n,double *vx,double *vy){kf2d_get_velocity(&n->kalman,vx,vy);}
int uwb_qual(UWBNav *n){pthread_mutex_lock(&n->dmtx);int q=n->qual;pthread_mutex_unlock(&n->dmtx);return q;}
bool uwb_healthy(UWBNav *n){pthread_mutex_lock(&n->dmtx);bool ok=(n->fd>=0)&&(n->cons_err<=5);if(n->last_good>0&&(ms()-n->last_good)>2.0)ok=false;pthread_mutex_unlock(&n->dmtx);return ok;}
double uwb_dist(UWBNav *n,double tx,double ty){double x,y;uwb_filtered(n,&x,&y);return sqrt((x-tx)*(x-tx)+(y-ty)*(y-ty));}
double uwb_bear(UWBNav *n,double tx,double ty){double x,y;uwb_filtered(n,&x,&y);return atan2(ty-y,tx-x);}
void uwb_nav_to(UWBNav *n,double tx,double ty,double msp,double ar,double *vx,double *vy,bool *arr){
    double d=uwb_dist(n,tx,ty);if(d<ar){*vx=0;*vy=0;*arr=true;return;}*arr=false;
    double b=uwb_bear(n,tx,ty),sp=msp;if(d<1000)sp=msp*(d/1000.0);if(sp<0.05)sp=0.05;
    int q=uwb_qual(n);if(q<20)sp*=0.3;else if(q<50)sp*=0.7;
    *vx=sp*cos(b);*vy=sp*sin(b);
}
