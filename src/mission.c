/* mission.c - 1:1 port of Python mission.py state machine */
#include "mission.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

static double mss(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}

static void trans(Mission *m, MissionState s){
    printf("[MISSION] %d -> %d\n",m->state,s);
    m->state=s;m->state_start=mss();
}

void mission_init(Mission *m, const Waypoint *wps, int n){
    memset(m,0,sizeof(*m));m->state=MS_INIT;
    m->n_wp=n>MAX_WAYPOINTS?MAX_WAYPOINTS:n;
    memcpy(m->wps,wps,sizeof(Waypoint)*m->n_wp);
    m->wp_idx=0;m->timeout=30.0;
}
MissionState mission_state(const Mission *m){return m->state;}
const Waypoint* mission_wp(const Mission *m){
    if(m->wp_idx>=m->n_wp)return NULL;return &m->wps[m->wp_idx];
}
void mission_next_wp(Mission *m){m->wp_idx++;if(m->wp_idx<m->n_wp)trans(m,MS_NAV_WP);else trans(m,MS_DONE);}

MissionCmd mission_update(Mission *m, const double uwb[2], float alt, bool vs_stab, bool vs_pass, bool vs_land){
    MissionCmd c;memset(&c,0,sizeof(c));c.alt=1.5;c.action="none";c.target_cls=-1;
    const Waypoint *wp=mission_wp(m);
    double elapsed=mss()-m->state_start;

    switch(m->state){
    case MS_INIT: trans(m,MS_TAKEOFF); break;
    case MS_TAKEOFF: c.alt=1.5;c.action="takeoff";
        if(alt>1.2){trans(m,MS_NAV_WP);}break;
    case MS_NAV_WP:
        if(!wp){trans(m,MS_DONE);break;}
        {double dx=wp->x-uwb[0],dy=wp->y-uwb[1],dist=sqrt(dx*dx+dy*dy);
        if(dist<400){
            if(strcmp(wp->action,"fly_through_hoop")==0) trans(m,MS_SEARCH);
            else if(strcmp(wp->action,"drop_ball")==0) trans(m,MS_NAV_DROP);
            else if(strcmp(wp->action,"precision_land")==0) trans(m,MS_NAV_LAND);
            else mission_next_wp(m);
            break;
        }
        double b=atan2(dy,dx),sp=dist>2000?0.8:dist>1000?0.5:0.3;
        c.vx=sp*cos(b);c.vy=sp*sin(b);c.alt=(float)(wp->z/1000.0);}break;
    case MS_SEARCH: c.action="search";c.target_cls=wp?0:-1;
        if(elapsed>10){trans(m,MS_ALIGN_HOOP);}break;
    case MS_ALIGN_HOOP: c.action="align_hoop";c.target_cls=wp?0:-1;
        if(vs_stab){trans(m,MS_FLY_HOOP);}
        if(elapsed>m->timeout){mission_next_wp(m);}break;
    case MS_FLY_HOOP: c.action="fly_hoop";c.target_cls=wp?0:-1;
        if(vs_pass){m->hoops_passed++;mission_next_wp(m);}
        if(elapsed>15){mission_next_wp(m);}break;
    case MS_NAV_DROP:
        if(!wp){trans(m,MS_DONE);break;}
        {double dx=wp->x-uwb[0],dy=wp->y-uwb[1],dist=sqrt(dx*dx+dy*dy);
        if(dist<300) trans(m,MS_ALIGN_DROP);
        else{double b=atan2(dy,dx);c.vx=0.3*cos(b);c.vy=0.3*sin(b);}}break;
    case MS_ALIGN_DROP: c.action="align_drop";c.target_cls=4;
        if(vs_stab){trans(m,MS_DROP_BALL);}
        if(elapsed>m->timeout){trans(m,MS_DROP_BALL);}break;
    case MS_DROP_BALL: c.do_drop=true;c.action="drop";m->ball_dropped=true;
        mission_next_wp(m);break;
    case MS_NAV_LAND:
        if(!wp){trans(m,MS_PRECISION_LAND);break;}
        {double dx=wp->x-uwb[0],dy=wp->y-uwb[1],dist=sqrt(dx*dx+dy*dy);
        if(dist<300)trans(m,MS_PRECISION_LAND);
        else{double b=atan2(dy,dx);c.vx=0.3*cos(b);c.vy=0.3*sin(b);}}break;
    case MS_PRECISION_LAND: c.action="land";
        if(vs_land||alt<0.15)trans(m,MS_LANDED);break;
    case MS_LANDED: c.action="landed";trans(m,MS_DONE);break;
    case MS_DONE: c.action="done";break;
    }
    return c;
}
