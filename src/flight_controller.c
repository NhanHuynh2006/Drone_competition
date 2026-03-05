/* flight_controller.c - MAVLink over UDP (port of Python flight_controller.py)
   Requires: deps/c_library_v2 (git clone mavlink/c_library_v2) */
#include "flight_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
/* MAVLink C headers */
#include <mavlink/ardupilotmega/mavlink.h>
#define GCS_S 255
#define GCS_C 190
#define BL 2048
static double ms(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static void smsg(FC *f,mavlink_message_t *m){uint8_t b[BL];uint16_t n=mavlink_msg_to_send_buffer(b,m);sendto(f->sock,b,n,0,(struct sockaddr*)&f->dest,f->dlen);}

static void svel(FC *f,float vx,float vy,float vz){
    mavlink_message_t m;
    mavlink_msg_set_position_target_local_ned_pack(GCS_S,GCS_C,&m,0,f->tsys,f->tcomp,
        MAV_FRAME_LOCAL_NED,0x0DC7,0,0,0,vx,vy,vz,0,0,0,0,0);
    smsg(f,&m);
}
/* Send thread 10Hz (same as Python _send_loop) */
static void *fc_sloop(void *a){
    FC *f=(FC*)a;struct timespec nx;clock_gettime(CLOCK_MONOTONIC,&nx);
    while(f->running){
        nx.tv_nsec+=100000000L;if(nx.tv_nsec>=1000000000L){nx.tv_sec++;nx.tv_nsec-=1000000000L;}
        float vx,vy,vz;
        pthread_mutex_lock(&f->vmtx);vx=f->vcmd[0];vy=f->vcmd[1];vz=f->vcmd[2];pthread_mutex_unlock(&f->vmtx);
        if(f->alt_hold && f->cur_alt>0){
            float ae=f->talt-f->cur_alt;
            float va=-(float)pid_compute(&f->apid,ae);
            vz+=va;
        }
        if(vz>1)vz=1;if(vz<-1)vz=-1;
        svel(f,vx,vy,vz);
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&nx,NULL);
    }
    return NULL;
}
/* Recv thread (same as Python _telemetry_loop) */
static void *fc_rloop(void *a){
    FC *f=(FC*)a;uint8_t buf[BL];mavlink_message_t msg;mavlink_status_t st;
    while(f->running){
        int n=recvfrom(f->sock,buf,BL,0,NULL,NULL);if(n<=0)continue;
        for(int i=0;i<n;i++){
            if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)){
                pthread_mutex_lock(&f->tmtx);
                switch(msg.msgid){
                case MAVLINK_MSG_ID_HEARTBEAT:{
                    mavlink_heartbeat_t hb;mavlink_msg_heartbeat_decode(&msg,&hb);
                    f->armed=(hb.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
                    switch(hb.custom_mode){
                        case 0:strncpy(f->mode,"STABILIZE",15);break;case 4:strncpy(f->mode,"GUIDED",15);break;
                        case 9:strncpy(f->mode,"LAND",15);break;default:snprintf(f->mode,15,"M%d",(int)hb.custom_mode);break;
                    }
                    if(!f->tsys)f->tsys=msg.sysid;break;}
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{
                    mavlink_global_position_int_t g;mavlink_msg_global_position_int_decode(&msg,&g);
                    f->cur_alt=g.relative_alt/1000.0f;f->lat=g.lat/1e7f;f->lon=g.lon/1e7f;f->hdg=g.hdg/100.0f;break;}
                case MAVLINK_MSG_ID_RANGEFINDER:{
                    mavlink_rangefinder_t r;mavlink_msg_rangefinder_decode(&msg,&r);
                    if(r.distance>0&&r.distance<10){f->cur_alt=r.distance;f->range_dist=r.distance;}
                    break;}
                case MAVLINK_MSG_ID_SCALED_IMU2:{
                    mavlink_scaled_imu2_t si;mavlink_msg_scaled_imu2_decode(&msg,&si);
                    f->imu_ax=si.xacc/1000.0f*9.81f; /* mG -> m/s^2 */
                    f->imu_ay=si.yacc/1000.0f*9.81f;
                    f->imu_az=si.zacc/1000.0f*9.81f;
                    f->imu_gx=si.xgyro/1000.0f; /* mrad/s -> rad/s */
                    f->imu_gy=si.ygyro/1000.0f;
                    f->imu_gz=si.zgyro/1000.0f;
                    break;}
                case MAVLINK_MSG_ID_ATTITUDE:{
                    mavlink_attitude_t at;mavlink_msg_attitude_decode(&msg,&at);
                    f->att_roll=at.roll;f->att_pitch=at.pitch;f->att_yaw=at.yaw;
                    f->att_rs=at.rollspeed;f->att_ps=at.pitchspeed;f->att_ys=at.yawspeed;
                    break;}
                default:break;
                }
                pthread_mutex_unlock(&f->tmtx);
                /* Fire callbacks outside lock for sensor fusion */
                if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU2 && f->imu_cb)
                    f->imu_cb(f->imu_ax,f->imu_ay,f->imu_az,f->imu_gx,f->imu_gy,f->imu_gz,f->imu_ctx);
                if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE && f->att_cb)
                    f->att_cb(f->att_roll,f->att_pitch,f->att_yaw,f->att_rs,f->att_ps,f->att_ys,f->att_ctx);
                if(msg.msgid==MAVLINK_MSG_ID_RANGEFINDER && f->range_cb && f->range_dist>0)
                    f->range_cb(f->range_dist,f->range_ctx);
                if(msg.msgid==MAVLINK_MSG_ID_GLOBAL_POSITION_INT && f->baro_cb)
                    f->baro_cb(f->cur_alt,f->baro_ctx);
            }
        }
    }
    return NULL;
}

int fc_connect(FC *f,const char *ip,uint16_t port){
    memset(f,0,sizeof(*f));pthread_mutex_init(&f->vmtx,NULL);pthread_mutex_init(&f->tmtx,NULL);
    strncpy(f->mode,"UNKNOWN",15);pid_init(&f->apid,0.5,0.05,0.15,0.5,0.05,0.3);
    f->talt=1.5;f->alt_hold=true;
    f->sock=socket(PF_INET,SOCK_DGRAM,0);if(f->sock<0)return -1;
    struct sockaddr_in la={0};la.sin_family=AF_INET;la.sin_addr.s_addr=INADDR_ANY;la.sin_port=htons(port);
    bind(f->sock,(struct sockaddr*)&la,sizeof(la));
    struct timeval tv={0,500000};setsockopt(f->sock,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
    f->dest.sin_family=AF_INET;f->dest.sin_addr.s_addr=inet_addr(ip);f->dest.sin_port=htons(port);f->dlen=sizeof(f->dest);
    printf("[FC]Waiting heartbeat %s:%d\n",ip,port);
    uint8_t buf[BL];mavlink_message_t msg;mavlink_status_t st;double dl=ms()+30;
    while(ms()<dl){int n=recvfrom(f->sock,buf,BL,0,NULL,NULL);if(n<=0)continue;
        for(int i=0;i<n;i++){if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)){
            if(msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){f->tsys=msg.sysid;f->tcomp=msg.compid;printf("[FC]Connected sys%d\n",f->tsys);goto ok;}}}
    }
    printf("[FC]Timeout\n");return -1;
ok:
    {mavlink_message_t m;mavlink_msg_request_data_stream_pack(GCS_S,GCS_C,&m,f->tsys,f->tcomp,MAV_DATA_STREAM_ALL,10,1);smsg(f,&m);}
    f->running=true;pthread_create(&f->sthr,NULL,fc_sloop,f);pthread_create(&f->rthr,NULL,fc_rloop,f);return 0;
}
void fc_vel(FC *f,float vx,float vy,float vz){
    pthread_mutex_lock(&f->vmtx);
    f->vcmd[0]=fmaxf(-1.5,fminf(1.5,vx));f->vcmd[1]=fmaxf(-1.5,fminf(1.5,vy));f->vcmd[2]=vz;
    pthread_mutex_unlock(&f->vmtx);
}
void fc_hover(FC *f){fc_vel(f,0,0,0);}
void fc_set_alt(FC *f,float a){f->talt=a;f->alt_hold=true;}
int fc_arm_takeoff(FC *f,float a){
    f->talt=a;double dl=ms()+15;
    while(ms()<dl){if(strcmp(f->mode,"GUIDED")==0)break;printf("[FC]Wait GUIDED, now:%s\n",f->mode);sleep(1);}
    if(strcmp(f->mode,"GUIDED")!=0){printf("[FC]Not GUIDED!\n");return -1;}
    mavlink_message_t m;
    mavlink_msg_command_long_pack(GCS_S,GCS_C,&m,f->tsys,f->tcomp,MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0);smsg(f,&m);
    printf("[FC]Arm sent\n");sleep(1);
    mavlink_msg_command_long_pack(GCS_S,GCS_C,&m,f->tsys,f->tcomp,MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,a);smsg(f,&m);
    printf("[FC]Takeoff %.1fm\n",a);
    dl=ms()+15;while(f->cur_alt<a-0.3&&ms()<dl){printf("[FC]Climb %.1f/%.1f\n",f->cur_alt,a);usleep(500000);}
    printf("[FC]Takeoff done %.1fm\n",f->cur_alt);return 0;
}
void fc_land(FC *f){
    f->alt_hold=false;mavlink_message_t m;
    mavlink_msg_command_long_pack(GCS_S,GCS_C,&m,f->tsys,f->tcomp,MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0);smsg(f,&m);
}
float fc_alt(FC *f){pthread_mutex_lock(&f->tmtx);float a=f->cur_alt;pthread_mutex_unlock(&f->tmtx);return a;}
bool fc_armed(FC *f){pthread_mutex_lock(&f->tmtx);bool a=f->armed;pthread_mutex_unlock(&f->tmtx);return a;}
void fc_shutdown(FC *f){fc_hover(f);f->running=false;pthread_join(f->sthr,NULL);pthread_join(f->rthr,NULL);close(f->sock);}
void fc_set_imu_callback(FC *f,fc_imu_cb_t cb,void *ctx){f->imu_cb=cb;f->imu_ctx=ctx;}
void fc_set_att_callback(FC *f,fc_att_cb_t cb,void *ctx){f->att_cb=cb;f->att_ctx=ctx;}
void fc_set_range_callback(FC *f,fc_range_cb_t cb,void *ctx){f->range_cb=cb;f->range_ctx=ctx;}
void fc_set_baro_callback(FC *f,fc_baro_cb_t cb,void *ctx){f->baro_cb=cb;f->baro_ctx=ctx;}
