/* main.cpp - Main control loop (port of Python main.py)
   Threads: camera, AI detect, HSV detect, UWB poll, MAVLink send, control loop */
#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <time.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include "pid.h"
#include "kalman2d.h"
#include "dwm1001.h"
#include "detector.h"
#include "hsv_detector.h"
#include "visual_servo.h"
#include "mission.h"
#include "flight_controller.h"
#include "ball_dropper.h"
#include "sensor_fusion.h"
}

static volatile bool g_running = true;
static void sighand(int s) { g_running = false; printf("\n[MAIN] Ctrl+C -> stopping\n"); }

/* Global sensor fusion for callbacks */
static SensorFusion g_sf;
static double mono_s(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}

/* MAVLink -> Sensor Fusion callbacks */
static void on_imu(float ax,float ay,float az,float gx,float gy,float gz,void *ctx) {
    IMUData d={ax,ay,az,gx,gy,gz,mono_s()};
    sf_predict_imu(&g_sf, &d, NULL); /* attitude already updated */
}
static void on_att(float r,float p,float y,float rs,float ps,float ys,void *ctx) {
    AttitudeData d={r,p,y,rs,ps,ys,mono_s()};
    sf_update_attitude(&g_sf, &d);
}
static void on_range(float dist,void *ctx) {
    RangeData d={dist,mono_s()};
    sf_update_rangefinder(&g_sf, &d);
}
static void on_baro(float alt,void *ctx) {
    BaroData d={alt,mono_s()};
    sf_update_baro(&g_sf, &d);
}

/* ====== CONFIG (same as Python config.yaml) ====== */
struct Config {
    int cam_idx=0, cam_w=640, cam_h=480, cam_fps=30;
    char model_param[128]="models/yolov8n.ncnn.param";
    char model_bin[128]="models/yolov8n.ncnn.bin";
    int det_size=320; float det_conf=0.45, det_iou=0.45; int det_threads=4;
    char uwb_port[64]="/dev/ttyACM0"; double uwb_pn=0.01, uwb_mn=0.05, uwb_hz=30;
    char fc_ip[32]="127.0.0.1"; int fc_port=14550;
    int bd_chip=2, bd_chan=0, bd_close=1000, bd_open=2000;
    float takeoff_alt=1.5;
    double vs_kp=0.5, vs_ki=0.05, vs_kd=0.15, vs_mo=0.5, vs_dz=0.02;
    bool use_ai=true;  /* true=NCNN YOLO, false=HSV color */
    bool use_fusion=true; /* true=EKF fusion, false=UWB only */
    int  flow_detect_interval=5; /* re-detect features every N frames */
};

int main(int argc, char **argv) {
    signal(SIGINT, sighand);
    Config cfg;

    /* Parse args */
    bool test_cam=false, test_uwb=false, test_hsv=false, sim=false;
    for(int i=1;i<argc;i++){
        if(strcmp(argv[i],"--test-camera")==0) test_cam=true;
        else if(strcmp(argv[i],"--test-uwb")==0) test_uwb=true;
        else if(strcmp(argv[i],"--test-hsv")==0) test_hsv=true;
        else if(strcmp(argv[i],"--sim")==0) sim=true;
        else if(strcmp(argv[i],"--hsv")==0) cfg.use_ai=false;
        else if(strcmp(argv[i],"--ai")==0) cfg.use_ai=true;
        else if(strcmp(argv[i],"--fusion")==0) cfg.use_fusion=true;
        else if(strcmp(argv[i],"--no-fusion")==0) cfg.use_fusion=false;
    }
    printf("[MAIN] Detection: %s\n", cfg.use_ai ? "AI (NCNN YOLO)" : "HSV Color");

    /* === Camera === */
    cv::VideoCapture cap(cfg.cam_idx, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.cam_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.cam_h);
    cap.set(cv::CAP_PROP_FPS, cfg.cam_fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if(!cap.isOpened()){printf("[CAM]Failed open %d\n",cfg.cam_idx);return 1;}
    printf("[CAM]Opened %dx%d@%d\n",cfg.cam_w,cfg.cam_h,cfg.cam_fps);

    /* === Test modes === */
    if(test_cam){
        printf("[TEST]Camera mode\n");cv::Mat f;
        while(g_running){cap.read(f);if(f.empty())continue;
            printf("Frame %dx%d\n",f.cols,f.rows);usleep(100000);}
        return 0;
    }
    if(test_hsv){
        printf("[TEST]HSV detection mode (port of process.py)\n");
        HSVConfig hcfg; hsv_config_default(&hcfg);
        cv::Mat f;
        while(g_running){
            cap.read(f);if(f.empty())continue;
            Det dets[MAX_DET];
            int n=hsv_detect(&hcfg, f.data, f.cols, f.rows, dets, MAX_DET, f.data);
            printf("HSV: %d detections\n",n);
            for(int i=0;i<n;i++)
                printf("  [%s] cx=%d cy=%d area=%.3f\n",dets[i].name,dets[i].cx,dets[i].cy,dets[i].rel_area);
            usleep(33000);
        }
        return 0;
    }
    if(test_uwb){
        printf("[TEST]UWB mode\n");
        UWBNav uwb; uwb_init(&uwb,cfg.uwb_port,cfg.uwb_pn,cfg.uwb_mn,cfg.uwb_hz);
        uwb_start(&uwb);sleep(1);
        while(g_running){
            int32_t x,y,z;uwb_raw(&uwb,&x,&y,&z);
            double fx,fy;uwb_filtered(&uwb,&fx,&fy);
            printf("Raw:%d,%d,%d Filt:%.0f,%.0f Q:%d\n",x,y,z,fx,fy,uwb_qual(&uwb));
            usleep(200000);
        }
        uwb_stop(&uwb);return 0;
    }

    /* === Full system === */
    /* Sensor Fusion EKF */
    sf_init(&g_sf);
    printf("[MAIN] Sensor fusion: %s\n", cfg.use_fusion ? "EKF (UWB+Flow+IMU+Range)" : "UWB only");

    /* UWB */
    UWBNav uwb;
    uwb_init(&uwb, cfg.uwb_port, cfg.uwb_pn, cfg.uwb_mn, cfg.uwb_hz);
    uwb_start(&uwb);

    /* Optical Flow Tracker */
    OptFlowTracker flow_tracker;
    if (cfg.use_fusion)
        oft_init(&flow_tracker, cfg.cam_w, cfg.cam_h, cfg.flow_detect_interval);

    /* Detector (AI or HSV) */
    NCNNDet *ai_det = NULL;
    AsyncDet *async_det = NULL;
    HSVConfig hsv_cfg;
    if(cfg.use_ai){
        ai_det = ncnn_det_create(cfg.model_param, cfg.model_bin, cfg.det_size,
                                  cfg.det_conf, cfg.det_iou, cfg.det_threads);
        if(ai_det) async_det = adet_create(ai_det, cfg.cam_w, cfg.cam_h);
    } else {
        hsv_config_default(&hsv_cfg);
    }

    /* Visual servo */
    VisualServo vs;
    vs_init(&vs, cfg.cam_w, cfg.cam_h, cfg.vs_kp, cfg.vs_ki, cfg.vs_kd, cfg.vs_mo, cfg.vs_dz);

    /* Ball dropper */
    BallDropper bd;
    bd_init(&bd, cfg.bd_chip, cfg.bd_chan, cfg.bd_close, cfg.bd_open);

    /* Flight controller */
    FC fc;
    if(!sim){
        if(fc_connect(&fc, cfg.fc_ip, cfg.fc_port)!=0){
            printf("[MAIN]FC connect failed\n"); g_running=false;
        } else {
            /* Register sensor fusion callbacks from Pixhawk6C */
            if (cfg.use_fusion) {
                fc_set_imu_callback(&fc, on_imu, NULL);
                fc_set_att_callback(&fc, on_att, NULL);
                fc_set_range_callback(&fc, on_range, NULL);
                fc_set_baro_callback(&fc, on_baro, NULL);
                printf("[MAIN] Pixhawk6C sensor callbacks registered\n");
            }
            fc_arm_takeoff(&fc, cfg.takeoff_alt);
        }
    }

    /* Mission */
    Waypoint wps[] = {
        {2000,1000,1500,"fly_through_hoop","red"},
        {4000,2000,1500,"fly_through_hoop","yellow"},
        {3000,3000,1500,"drop_ball",""},
        {1000,1000,500,"precision_land",""},
    };
    Mission mission;
    mission_init(&mission, wps, 4);

    /* === Main loop (same as Python run()) === */
    printf("[MAIN]Starting main loop\n");
    cv::Mat frame;
    int frame_cnt = 0;

    while(g_running){
        double t0 = ms();
        cap.read(frame);
        if(frame.empty()) continue;
        frame_cnt++;

        /* Detection */
        Det dets[MAX_DET]; int ndet = 0;
        if(cfg.use_ai && async_det){
            adet_submit(async_det, frame.data);
            ndet = adet_get(async_det, dets, MAX_DET);
        } else {
            ndet = hsv_detect(&hsv_cfg, frame.data, frame.cols, frame.rows, dets, MAX_DET, NULL);
        }

        /* Altitude */
        float alt = sim ? 1.5f : fc_alt(&fc);

        /* === Sensor Fusion updates === */
        /* Feed UWB into fusion */
        double uwb_raw_x, uwb_raw_y;
        uwb_filtered(&uwb, &uwb_raw_x, &uwb_raw_y);
        int uwb_q = uwb_qual(&uwb);
        if (cfg.use_fusion) {
            sf_update_uwb(&g_sf, uwb_raw_x, uwb_raw_y, uwb_q);
        }

        /* Compute optical flow from camera and feed into fusion */
        if (cfg.use_fusion) {
            OptFlowData flow = oft_process(&flow_tracker, frame.data, frame.cols, frame.rows, alt);
            if (flow.quality > 0.1f) {
                sf_update_optflow(&g_sf, &flow, NULL, alt);
            }
        }

        /* Get fused or raw position for mission */
        double pos_xy[2];
        if (cfg.use_fusion) {
            double pz;
            sf_get_position(&g_sf, &pos_xy[0], &pos_xy[1], &pz);
            /* Use fused altitude if rangefinder is feeding */
            FusedState fs; sf_get_state(&g_sf, &fs);
            if (fs.range_count > 0) alt = (float)fs.z;
        } else {
            pos_xy[0] = uwb_raw_x;
            pos_xy[1] = uwb_raw_y;
        }

        /* Visual servo */
        VSResult vsr = vs_compute(&vs, dets, ndet, alt);

        /* Mission update - uses fused position */
        MissionCmd mc = mission_update(&mission, pos_xy, alt, vsr.stable, vsr.passed, vsr.landed);

        /* Combine commands (same as Python) */
        float vx=0, vy=0, vz=0;
        if(mc.target_cls >= 0){
            vs_mode(&vs, VS_CENTER, mc.target_cls);
            vx = mc.vx + vsr.vx;
            vy = mc.vy + vsr.vy;
        } else {
            vx = mc.vx; vy = mc.vy;
        }
        vz = mc.vz + vsr.vz;

        /* Ball drop */
        if(mc.do_drop) bd_drop_and_close(&bd, 500);

        /* Send to FC */
        if(!sim){
            fc_vel(&fc, vx, vy, vz);
            if(mc.alt > 0) fc_set_alt(&fc, mc.alt);
        }

        /* Status print every 30 frames */
        if(frame_cnt % 30 == 0){
            double fps = 1.0 / fmax(ms()-t0, 1e-6);
            printf("\n--- Frame %d ---\n", frame_cnt);
            printf("  FPS:%.0f Det:%d State:%d WP:%d\n", fps, ndet, mission.state, mission.wp_idx);
            printf("  Pos:(%.0f,%.0f) Q:%d Alt:%.2f\n", pos_xy[0], pos_xy[1], uwb_q, alt);
            if (cfg.use_fusion) {
                FusedState fs; sf_get_state(&g_sf, &fs);
                printf("  Fusion: xy_unc=%.1f z_unc=%.2f UWB:%d Flow:%d IMU:%d Range:%d\n",
                       fs.uncertainty_xy, fs.uncertainty_z,
                       fs.uwb_updates, fs.flow_updates, fs.imu_updates, g_sf.range_count);
                printf("  Vel:(%.2f,%.2f,%.2f) Bias:(%.3f,%.3f)\n",
                       fs.vx, fs.vy, fs.vz, fs.ax_bias, fs.ay_bias);
            }
            printf("  Vel:%.2f,%.2f,%.2f\n", vx, vy, vz);
            for(int i=0;i<ndet;i++)
                printf("  [%s] c=%.2f cx=%d cy=%d\n", dets[i].name, dets[i].conf, dets[i].cx, dets[i].cy);
        }

        /* Done? */
        if(mission.state == MS_DONE){
            printf("[MAIN]Mission complete!\n");
            if(!sim) fc_land(&fc);
            break;
        }
    }

    /* Cleanup */
    printf("[MAIN]Shutting down\n");
    if(!sim) fc_shutdown(&fc);
    uwb_stop(&uwb);
    bd_cleanup(&bd);
    if (cfg.use_fusion) oft_destroy(&flow_tracker);
    if(async_det) adet_destroy(async_det);
    if(ai_det) ncnn_det_destroy(ai_det);
    return 0;
}
