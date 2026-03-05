/* mission.h - Mission state machine (port of Python mission.py) */
#ifndef MISSION_H
#define MISSION_H
#include <stdbool.h>
#include <stdint.h>
typedef enum {
    MS_INIT=0,MS_TAKEOFF,MS_NAV_WP,MS_SEARCH,MS_ALIGN_HOOP,MS_FLY_HOOP,
    MS_NAV_DROP,MS_ALIGN_DROP,MS_DROP_BALL,MS_NAV_LAND,MS_PRECISION_LAND,
    MS_LANDED,MS_DONE
} MissionState;
typedef struct { double x,y,z; const char *action; const char *color; } Waypoint;
#define MAX_WAYPOINTS 20
typedef struct {
    MissionState state; Waypoint wps[MAX_WAYPOINTS]; int n_wp,wp_idx;
    int hoops_passed; bool ball_dropped; double state_start; double timeout;
} Mission;
void mission_init(Mission *m, const Waypoint *wps, int n);
MissionState mission_state(const Mission *m);
const Waypoint* mission_wp(const Mission *m);
void mission_next_wp(Mission *m);
/* Returns velocity commands: vx,vy,vz,target_alt,action */
typedef struct { float vx,vy,vz,alt; const char *action; int target_cls; bool do_drop; } MissionCmd;
MissionCmd mission_update(Mission *m, const double uwb_xy[2], float alt, bool vs_stable, bool vs_passed, bool vs_landed);
#endif
