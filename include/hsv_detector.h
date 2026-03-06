/* hsv_detector.h - HSV color detection (port of Python process.py)
   Detects red/yellow/blue hoops + white squares using HSV thresholds */
#ifndef HSV_DETECTOR_H
#define HSV_DETECTOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include "detector.h"  /* shares Det struct */

typedef struct {
    int min_area;
    /* HSV ranges (per color) */
    int red_h_lo, red_h_hi, red_s_lo, red_v_lo;
    int yellow_h_lo, yellow_h_hi, yellow_s_lo, yellow_v_lo;
    int blue_h_lo, blue_h_hi, blue_s_lo, blue_v_lo;
    int white_s_hi, white_v_lo;
    /* Temporal smoothing state: index 0=red 1=yellow 2=blue 3=white */
    float smooth_cx[4], smooth_cy[4]; /* EMA of centroid positions  */
    int   lost_frames[4];             /* consecutive frames with no detect */
    int   max_lost;                   /* frames to keep last detection alive */
} HSVConfig;

void hsv_config_default(HSVConfig *c);

/* Process one BGR frame, fill Det array, return count.
   Also outputs safe_radius and velocity suggestion for each detection.
   draw_frame: if not NULL, draws contours/circles/text on it. */
/* cfg is non-const: temporal smoothing state is updated each call */
int hsv_detect(HSVConfig *cfg,
               const unsigned char *bgr, int w, int h,
               Det *out, int max_det,
               unsigned char *draw_frame);

#ifdef __cplusplus
}
#endif
#endif
