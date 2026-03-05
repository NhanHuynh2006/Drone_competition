/* hsv_detector.cpp - 1:1 port of Python process.py
   HSV color detection for hoops (red/yellow/blue) + white squares
   Uses OpenCV C++ API, same logic: inRange -> findContours -> moments */
#include "hsv_detector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <cmath>
#include <cstdio>

extern "C" {

void hsv_config_default(HSVConfig *c) {
    c->min_area = 500;
    /* Same HSV ranges as Python process.py */
    c->red_h_lo=0;    c->red_h_hi=10;   c->red_s_lo=100; c->red_v_lo=100;
    c->yellow_h_lo=20; c->yellow_h_hi=30; c->yellow_s_lo=100; c->yellow_v_lo=100;
    c->blue_h_lo=100;  c->blue_h_hi=130;  c->blue_s_lo=100; c->blue_v_lo=100;
    c->white_s_hi=40;  c->white_v_lo=200;
}

/* Detect colored hoops + white squares, same as Python process.py */
int hsv_detect(const HSVConfig *cfg,
               const unsigned char *bgr, int w, int h,
               Det *out, int max_det,
               unsigned char *draw_frame)
{
    /* Wrap raw pointer as cv::Mat (no copy) */
    cv::Mat frame(h, w, CV_8UC3, (void*)bgr);
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    /* Same masks as Python */
    cv::Mat mask_red, mask_yellow, mask_blue, mask_white;
    cv::inRange(hsv, cv::Scalar(cfg->red_h_lo, cfg->red_s_lo, cfg->red_v_lo),
                     cv::Scalar(cfg->red_h_hi, 255, 255), mask_red);
    cv::inRange(hsv, cv::Scalar(cfg->yellow_h_lo, cfg->yellow_s_lo, cfg->yellow_v_lo),
                     cv::Scalar(cfg->yellow_h_hi, 255, 255), mask_yellow);
    cv::inRange(hsv, cv::Scalar(cfg->blue_h_lo, cfg->blue_s_lo, cfg->blue_v_lo),
                     cv::Scalar(cfg->blue_h_hi, 255, 255), mask_blue);
    cv::inRange(hsv, cv::Scalar(0, 0, cfg->white_v_lo),
                     cv::Scalar(180, cfg->white_s_hi, 255), mask_white);

    /* Combined color mask (same as Python: mask = mask_red | mask_yellow | mask_blue) */
    cv::Mat mask_color = mask_red | mask_yellow | mask_blue;

    int count = 0;
    cv::Mat draw;
    if (draw_frame) draw = cv::Mat(h, w, CV_8UC3, draw_frame);

    /* === Color hoops (same loop as Python) === */
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size() && count < max_det; i++) {
        double area = cv::contourArea(contours[i]);
        if (area < cfg->min_area) continue;

        /* Determine color by checking which mask the centroid falls in */
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 == 0) continue;
        int cx = (int)(M.m10 / M.m00);
        int cy = (int)(M.m01 / M.m00);

        /* Bounding rect */
        cv::Rect br = cv::boundingRect(contours[i]);

        /* Determine hoop color (same logic: check which mask has the centroid pixel) */
        const char *name = "hoop_color";
        int cls = 0;
        if (cx >= 0 && cx < w && cy >= 0 && cy < h) {
            if (mask_red.at<uint8_t>(cy, cx) > 0) { name = "hoop_red"; cls = 0; }
            else if (mask_yellow.at<uint8_t>(cy, cx) > 0) { name = "hoop_yellow"; cls = 1; }
            else if (mask_blue.at<uint8_t>(cy, cx) > 0) { name = "hoop_blue"; cls = 2; }
        }

        Det *d = &out[count++];
        d->cls = cls; d->conf = 1.0f; d->name = name;
        d->x1 = br.x; d->y1 = br.y;
        d->x2 = br.x + br.width; d->y2 = br.y + br.height;
        d->cx = cx; d->cy = cy;
        d->w = br.width; d->h = br.height;
        d->rel_area = (float)(br.width * br.height) / (float)(w * h);

        /* Draw on frame if requested (same as Python) */
        if (draw_frame) {
            cv::drawContours(draw, contours, (int)i, cv::Scalar(0,255,0), 2);
            /* Safe zone 2/3 radius (same as Python) */
            cv::Point2f center; float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            int safe_r = (int)(radius * 2 / 3);
            cv::circle(draw, cv::Point((int)center.x,(int)center.y), safe_r, cv::Scalar(0,255,0), 1);
            cv::circle(draw, cv::Point(cx,cy), 5, cv::Scalar(0,0,225), -1);
            /* Velocity vector (same as Python: vx=-dy, vy=dx) */
            int dx = cx - w/2, dy = cy - h/2;
            int vx = -dy, vy = dx;
            cv::line(draw, cv::Point(w/2,h/2), cv::Point(cx,cy), cv::Scalar(255,255,0), 1);
            char txt[64]; snprintf(txt, sizeof(txt), "vx:%d vy:%d", vx, vy);
            cv::putText(draw, txt, cv::Point(cx+8,cy), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
        }
    }

    /* === White squares (same loop as Python) === */
    std::vector<std::vector<cv::Point>> cont_w;
    cv::findContours(mask_white, cont_w, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < cont_w.size() && count < max_det; i++) {
        double area = cv::contourArea(cont_w[i]);
        if (area < cfg->min_area) continue;

        double peri = cv::arcLength(cont_w[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cont_w[i], approx, 0.04 * peri, true);

        /* Must be 4 vertices (quadrilateral) */
        if (approx.size() != 4) continue;

        cv::Rect br = cv::boundingRect(approx);
        float ratio = (float)br.width / (float)br.height;

        /* Check near-square ratio 0.8-1.2 (same as Python) */
        if (ratio < 0.8f || ratio > 1.2f) continue;

        cv::Moments M = cv::moments(cont_w[i]);
        if (M.m00 == 0) continue;
        int cx = (int)(M.m10 / M.m00);
        int cy = (int)(M.m01 / M.m00);

        Det *d = &out[count++];
        d->cls = 4; d->conf = 1.0f; d->name = "white_sq";
        d->x1 = br.x; d->y1 = br.y;
        d->x2 = br.x + br.width; d->y2 = br.y + br.height;
        d->cx = cx; d->cy = cy;
        d->w = br.width; d->h = br.height;
        d->rel_area = (float)(br.width * br.height) / (float)(w * h);

        if (draw_frame) {
            std::vector<std::vector<cv::Point>> tmp = {approx};
            cv::drawContours(draw, tmp, 0, cv::Scalar(255,255,255), 2);
            cv::putText(draw, "WHITE SQ", cv::Point(br.x, br.y-8),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
            cv::circle(draw, cv::Point(cx,cy), 5, cv::Scalar(200,200,200), -1);
            int dx = cx - w/2, dy = cy - h/2;
            cv::line(draw, cv::Point(w/2,h/2), cv::Point(cx,cy), cv::Scalar(200,200,200), 1);
        }
    }

    /* Draw center point (same as Python) */
    if (draw_frame)
        cv::circle(draw, cv::Point(w/2, h/2), 6, cv::Scalar(0,255,0), -1);

    return count;
}

} /* extern "C" */
