/* hsv_detector.cpp - Robust HSV color detection
   Key improvements:
   - CLAHE preprocessing: normalizes lighting so HSV is stable across frames
   - Red uses H:0-15 AND H:160-180 (red wraps in HSV wheel)
   - Morphological CLOSE (large kernel) fills holes → solid blobs
   - Convex hull: clean outline instead of jagged contour
   - Circularity + convexity score: rejects non-circular blobs
   - Temporal EMA smoothing: centroid position averaged across frames,
     last known position kept for max_lost frames → no more flickering */
#include "hsv_detector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <cmath>
#include <cstdio>

extern "C" {

void hsv_config_default(HSVConfig *c) {
    c->min_area = 600;
    /* Red: salmon/pink landing pad has lower saturation → s_lo=40
       Two hue ranges: 0-15 (warm red) and 160-180 (cool red wrap) */
    c->red_h_lo=0;     c->red_h_hi=15;   c->red_s_lo=40;  c->red_v_lo=60;
    /* Yellow hoop: wider range for different lighting */
    c->yellow_h_lo=15; c->yellow_h_hi=40; c->yellow_s_lo=60; c->yellow_v_lo=60;
    /* Blue hoop */
    c->blue_h_lo=90;   c->blue_h_hi=140;  c->blue_s_lo=60;  c->blue_v_lo=40;
    /* White square: very strict - computed from original HSV (not CLAHE)
       Real white fabric: S<35, V>200. Concrete floor won't pass. */
    c->white_s_hi=30;  c->white_v_lo=210;
    /* Temporal smoothing */
    c->max_lost = 15;  /* keep last detect alive for 15 frames (~0.5s at 30fps) */
    for (int i = 0; i < 4; i++) {
        c->smooth_cx[i] = -1.f;
        c->smooth_cy[i] = -1.f;
        c->lost_frames[i] = 999;
    }
}

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Build a clean binary mask:
   1. inRange (with optional 2nd hue range for red wrap)
   2. CLOSE (large kernel) → fills internal holes
   3. OPEN  (small kernel) → removes stray specks                           */
static cv::Mat make_mask(const cv::Mat &hsv,
                         int h_lo, int h_hi, int s_lo, int v_lo,
                         int h_lo2=-1, int h_hi2=-1)
{
    cv::Mat m;
    cv::inRange(hsv, cv::Scalar(h_lo, s_lo, v_lo),
                     cv::Scalar(h_hi, 255, 255), m);
    if (h_lo2 >= 0) {
        cv::Mat m2;
        cv::inRange(hsv, cv::Scalar(h_lo2, s_lo, v_lo),
                         cv::Scalar(h_hi2, 255, 255), m2);
        m |= m2;
    }
    cv::Mat kc = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19,19));
    cv::morphologyEx(m, m, cv::MORPH_CLOSE, kc);
    cv::Mat ko = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::morphologyEx(m, m, cv::MORPH_OPEN, ko);
    return m;
}

/* Circularity score: 1.0 = perfect circle */
static float circularity(const std::vector<cv::Point> &c) {
    double area = cv::contourArea(c);
    double peri = cv::arcLength(c, true);
    if (peri < 1.0) return 0.f;
    return (float)(4.0 * M_PI * area / (peri * peri));
}

/* Convexity score: convexHull_area / contour_area — near 1.0 = convex shape */
static float convexity(const std::vector<cv::Point> &c) {
    std::vector<cv::Point> hull;
    cv::convexHull(c, hull);
    double ha = cv::contourArea(hull);
    double ca = cv::contourArea(c);
    if (ha < 1.0) return 0.f;
    return (float)(ca / ha);
}

/* EMA update: alpha=0.25 new, 0.75 history → smoother, less jitter */
static float ema(float prev, float cur) {
    return (prev < 0.f) ? cur : 0.25f * cur + 0.75f * prev;
}

/* ── main detect ─────────────────────────────────────────────────────────── */

int hsv_detect(HSVConfig *cfg,          /* non-const: updates temporal state */
               const unsigned char *bgr, int w, int h,
               Det *out, int max_det,
               unsigned char *draw_frame)
{
    cv::Mat frame(h, w, CV_8UC3, (void*)bgr);

    /* ── Step 1: CLAHE on L channel of LAB → normalize lighting ── */
    cv::Mat lab;
    cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_ch(3);
    cv::split(lab, lab_ch);
    static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
    clahe->apply(lab_ch[0], lab_ch[0]);
    cv::merge(lab_ch, lab);
    cv::Mat norm_bgr;
    cv::cvtColor(lab, norm_bgr, cv::COLOR_Lab2BGR);

    /* ── Step 2: Blur + HSV ── */
    cv::Mat blurred;
    cv::GaussianBlur(norm_bgr, blurred, cv::Size(7,7), 0);
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    /* ── Step 3: Color masks ── */
    /* Red wraps: H 0-15 + H 160-180 */
    cv::Mat mask_red    = make_mask(hsv, cfg->red_h_lo, cfg->red_h_hi,
                                    cfg->red_s_lo, cfg->red_v_lo, 160, 180);
    cv::Mat mask_yellow = make_mask(hsv, cfg->yellow_h_lo, cfg->yellow_h_hi,
                                    cfg->yellow_s_lo, cfg->yellow_v_lo);
    cv::Mat mask_blue   = make_mask(hsv, cfg->blue_h_lo,   cfg->blue_h_hi,
                                    cfg->blue_s_lo,   cfg->blue_v_lo);

    /* White mask: computed from ORIGINAL HSV (before CLAHE) to avoid
       CLAHE boosting concrete floor into "white" range.
       Strict thresholds: S<35, V>200 — real white fabric only.        */
    cv::Mat hsv_orig;
    cv::cvtColor(frame, hsv_orig, cv::COLOR_BGR2HSV);
    cv::Mat mask_white;
    cv::inRange(hsv_orig, cv::Scalar(0,   0,   210),
                          cv::Scalar(180, 30, 255), mask_white);
    /* OPEN first (15x15): break apart floor tiles so they don't merge */
    cv::Mat wo = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15));
    cv::morphologyEx(mask_white, mask_white, cv::MORPH_OPEN, wo);
    /* Then CLOSE (25x25): fill small gaps within actual white pad */
    cv::Mat wk = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25));
    cv::morphologyEx(mask_white, mask_white, cv::MORPH_CLOSE, wk);

    int count = 0;
    cv::Mat draw;
    if (draw_frame) draw = cv::Mat(h, w, CV_8UC3, draw_frame);

    /* ── Step 4a: Find white_sq FIRST, then blank its region from color masks ──
       The landing pad has a colored circle inside the white square.
       By zeroing that region in color masks before hoop detection,
       the landing pad marker can never be mistaken for a real hoop.          */
    cv::Rect pad_rect(0,0,0,0);
    {
        std::vector<std::vector<cv::Point>> cont_w;
        cv::findContours(mask_white.clone(), cont_w, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        /* Only mask if white_sq is between 3%-30% of frame.
           Too small = noise, too large = floor tiles merged together */
        double min_pad_area = (double)(w * h) * 0.03;
        double max_pad_area = (double)(w * h) * 0.30;
        int best=-1; double best_a=min_pad_area;
        for (int i=0;i<(int)cont_w.size();i++){
            double a=cv::contourArea(cont_w[i]);
            if(a>best_a && a<max_pad_area){best_a=a;best=i;}
        }
        if(best>=0){
            cv::Rect br=cv::boundingRect(cont_w[best]);
            float ratio=(float)br.width/(float)(br.height+1);
            if(ratio>=0.3f && ratio<=3.0f){
                /* Expand slightly to fully cover the inner colored circle */
                int pad=(int)((br.width+br.height)*0.05f);
                pad_rect=cv::Rect(
                    std::max(0, br.x-pad),
                    std::max(0, br.y-pad),
                    std::min(w, br.x+br.width +pad*2)-std::max(0,br.x-pad),
                    std::min(h, br.y+br.height+pad*2)-std::max(0,br.y-pad));
                /* Zero out landing pad region in all color masks */
                mask_red   (pad_rect).setTo(0);
                mask_yellow(pad_rect).setTo(0);
                mask_blue  (pad_rect).setTo(0);
            }
        }
    }

    /* ── Step 4: Per-color detection with temporal smoothing ── */
    struct ColorDef {
        cv::Mat  *mask;
        const char *name;
        int       cls;       /* index into smooth_cx/cy/lost_frames */
        cv::Scalar color;
        float     min_circ;  /* circularity threshold */
    };
    /* min_circ=0.38: filters rectangular planks (circ~0.30) but accepts
       hoops viewed at angle (ellipse circ~0.40-0.60) and face-on (circ~0.70+) */
    ColorDef cdefs[] = {
        { &mask_red,    "hoop_red",    0, cv::Scalar(0,60,255),  0.38f },
        { &mask_yellow, "hoop_yellow", 1, cv::Scalar(0,200,255), 0.38f },
        { &mask_blue,   "hoop_blue",   2, cv::Scalar(255,80,0),  0.38f },
    };

    for (auto &cd : cdefs) {
        if (count >= max_det) break;
        int idx = cd.cls;

        /* Find contours, pick the largest that passes ALL shape filters.
           This prevents large environmental noise from shadowing a real hoop. */
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(*cd.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = (double)(w * h) * 0.50; /* hoop can't be >50% of frame */
        int best = -1; double best_area = (double)cfg->min_area;
        for (int i = 0; i < (int)contours.size(); i++) {
            double a = cv::contourArea(contours[i]);
            if (a <= best_area || a > max_area) continue;
            float ci = circularity(contours[i]);
            float co = convexity(contours[i]);
            if (ci >= cd.min_circ && co >= 0.60f) {
                best_area = a; best = i;
            }
        }

        bool detected = false;
        if (best >= 0) {
            float circ = circularity(contours[best]);
            float conv = convexity(contours[best]);
            {
                std::vector<cv::Point> hull;
                cv::convexHull(contours[best], hull);

                cv::Moments M = cv::moments(hull);
                if (M.m00 > 0) {
                    float cx = (float)(M.m10 / M.m00);
                    float cy = (float)(M.m01 / M.m00);

                    /* EMA smoothing */
                    cfg->smooth_cx[idx] = ema(cfg->smooth_cx[idx], cx);
                    cfg->smooth_cy[idx] = ema(cfg->smooth_cy[idx], cy);
                    cfg->lost_frames[idx] = 0;
                    detected = true;

                    /* Fill Det */
                    cv::Rect br = cv::boundingRect(hull);
                    Det *d = &out[count++];
                    d->cls = cd.cls; d->conf = circ; d->name = cd.name;
                    d->cx = (int)cfg->smooth_cx[idx];
                    d->cy = (int)cfg->smooth_cy[idx];
                    d->x1 = br.x; d->y1 = br.y;
                    d->x2 = br.x+br.width; d->y2 = br.y+br.height;
                    d->w  = br.width; d->h = br.height;
                    d->rel_area = (float)best_area / (float)(w*h);

                    if (draw_frame) {
                        std::vector<std::vector<cv::Point>> tmp = {hull};
                        cv::drawContours(draw, tmp, 0, cd.color, 3);

                        cv::Point2f center; float radius;
                        cv::minEnclosingCircle(hull, center, radius);
                        cv::circle(draw, cv::Point((int)center.x,(int)center.y),
                                   (int)radius, cd.color, 2);
                        cv::circle(draw, cv::Point((int)center.x,(int)center.y),
                                   (int)(radius*2/3), cv::Scalar(0,255,0), 1);

                        int icx = d->cx, icy = d->cy;
                        cv::circle(draw, cv::Point(icx,icy), 7, cv::Scalar(0,0,255), -1);
                        cv::line(draw, cv::Point(w/2,h/2), cv::Point(icx,icy),
                                 cv::Scalar(0,255,255), 2);

                        char txt[80];
                        snprintf(txt,sizeof(txt),"%s c:%.2f v:%.2f", cd.name, circ, conv);
                        cv::putText(draw, txt, cv::Point(br.x, br.y-8),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cd.color, 2);
                    }
                }
            }
        }

        /* Temporal persistence: reuse last known position if lost recently */
        if (!detected) {
            cfg->lost_frames[idx]++;
            if (cfg->lost_frames[idx] <= cfg->max_lost &&
                cfg->smooth_cx[idx] >= 0.f && count < max_det) {
                int icx = (int)cfg->smooth_cx[idx];
                int icy = (int)cfg->smooth_cy[idx];
                Det *d = &out[count++];
                d->cls = cd.cls; d->conf = 0.0f; d->name = cd.name;
                d->cx = icx; d->cy = icy;
                d->x1 = icx-30; d->y1 = icy-30;
                d->x2 = icx+30; d->y2 = icy+30;
                d->w = 60; d->h = 60; d->rel_area = 0.f;

                if (draw_frame) {
                    /* Dashed circle to show "remembered" position */
                    cv::circle(draw, cv::Point(icx,icy), 20, cd.color, 1);
                    char txt[64];
                    snprintf(txt,sizeof(txt),"%s (lost %d)", cd.name, cfg->lost_frames[idx]);
                    cv::putText(draw, txt, cv::Point(icx-30, icy-25),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cd.color, 1);
                }
            }
        }
    }

    /* ── Step 5: White square (landing pad) ── */
    if (count < max_det) {
        int idx = 3;
        std::vector<std::vector<cv::Point>> cont_w;
        cv::findContours(mask_white, cont_w, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int best = -1; double best_area = (double)cfg->min_area;
        for (int i = 0; i < (int)cont_w.size(); i++) {
            double a = cv::contourArea(cont_w[i]);
            if (a > best_area) { best_area = a; best = i; }
        }

        bool detected = false;
        if (best >= 0) {
            cv::Rect br = cv::boundingRect(cont_w[best]);
            float ratio = (float)br.width / (float)(br.height + 1);
            if (ratio >= 0.3f && ratio <= 3.0f) {   /* loose square check */
                cv::Moments M = cv::moments(cont_w[best]);
                if (M.m00 > 0) {
                    float cx = (float)(M.m10 / M.m00);
                    float cy = (float)(M.m01 / M.m00);
                    cfg->smooth_cx[idx] = ema(cfg->smooth_cx[idx], cx);
                    cfg->smooth_cy[idx] = ema(cfg->smooth_cy[idx], cy);
                    cfg->lost_frames[idx] = 0;
                    detected = true;

                    Det *d = &out[count++];
                    d->cls = 4; d->conf = 1.0f; d->name = "white_sq";
                    d->cx = (int)cfg->smooth_cx[idx];
                    d->cy = (int)cfg->smooth_cy[idx];
                    d->x1 = br.x; d->y1 = br.y;
                    d->x2 = br.x+br.width; d->y2 = br.y+br.height;
                    d->w = br.width; d->h = br.height;
                    d->rel_area = (float)best_area / (float)(w*h);

                    if (draw_frame) {
                        cv::rectangle(draw, br, cv::Scalar(255,255,255), 3);
                        cv::putText(draw, "WHITE SQ", cv::Point(br.x, br.y-8),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
                        cv::circle(draw, cv::Point(d->cx, d->cy), 7,
                                   cv::Scalar(200,200,200), -1);
                        cv::line(draw, cv::Point(w/2,h/2),
                                 cv::Point(d->cx, d->cy), cv::Scalar(200,200,200), 2);
                    }
                }
            }
        }

        if (!detected) {
            cfg->lost_frames[idx]++;
            if (cfg->lost_frames[idx] <= cfg->max_lost &&
                cfg->smooth_cx[idx] >= 0.f && count < max_det) {
                int icx = (int)cfg->smooth_cx[idx];
                int icy = (int)cfg->smooth_cy[idx];
                Det *d = &out[count++];
                d->cls = 4; d->conf = 0.f; d->name = "white_sq";
                d->cx = icx; d->cy = icy;
                d->x1 = icx-30; d->y1 = icy-30;
                d->x2 = icx+30; d->y2 = icy+30;
                d->w = 60; d->h = 60; d->rel_area = 0.f;
            }
        }
    }

    /* ── Step 6: Suppress hoop detections whose center falls inside white_sq ──
       The landing pad has a colored circle (yellow/red) in the center.
       If a hoop center is inside the white_sq bounding box → it is a landing
       pad marker, NOT a real hoop → remove it from the output.              */
    {
        /* Find white_sq bbox (if any) */
        int wx1=-1, wy1=-1, wx2=-1, wy2=-1;
        for (int i = 0; i < count; i++) {
            if (out[i].cls == 4) {
                wx1 = out[i].x1; wy1 = out[i].y1;
                wx2 = out[i].x2; wy2 = out[i].y2;
                break;
            }
        }
        if (wx1 >= 0) {
            /* Expand bbox by 10% to catch edge cases */
            int pad = (int)((wx2-wx1) * 0.10f);
            wx1 -= pad; wy1 -= pad; wx2 += pad; wy2 += pad;

            int new_count = 0;
            for (int i = 0; i < count; i++) {
                bool is_hoop = (out[i].cls < 4);
                bool inside  = (out[i].cx > wx1 && out[i].cx < wx2 &&
                                out[i].cy > wy1 && out[i].cy < wy2);
                if (is_hoop && inside && out[i].conf > 0.f) {
                    /* This colored blob is the landing pad center marker.
                       Re-label it as white_sq (cls=4) so mission code knows
                       it's the landing pad, not a hoop to fly through.      */
                    out[i].cls  = 4;
                    out[i].name = "white_sq";
                    /* Keep cx/cy as the precise center of the colored marker */
                }
                out[new_count++] = out[i];
            }
            count = new_count;
        }
    }

    /* Frame center marker */
    if (draw_frame)
        cv::circle(draw, cv::Point(w/2, h/2), 8, cv::Scalar(0,255,0), -1);

    return count;
}

} /* extern "C" */
