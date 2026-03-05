/* detector.cpp - NCNN YOLOv8 inference (port of Python detector.py) */
#include "detector.h"
#include <ncnn/net.h>
#include <ncnn/mat.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>
#include <cstdio>
#include <cstdlib>

static const char *CLASS_NAMES[] = {"hoop_red","hoop_yellow","hoop_blue","landing_pad"};

struct NCNNDetImpl { ncnn::Net net; };

extern "C" {

NCNNDet* ncnn_det_create(const char *param, const char *bin, int sz, float ct, float it, int nth){
    NCNNDet *d=new NCNNDet(); d->impl=new NCNNDetImpl();
    d->sz=sz; d->ct=ct; d->it=it; d->fps=0; d->avg_ms=0;
    d->impl->net.opt.num_threads=nth;
    d->impl->net.opt.use_fp16_packed=true;
    d->impl->net.opt.use_fp16_storage=true;
    d->impl->net.opt.use_fp16_arithmetic=true;
    d->impl->net.opt.lightmode=true;
    if(d->impl->net.load_param(param)!=0){printf("[Det]ERR param:%s\n",param);delete d->impl;delete d;return nullptr;}
    if(d->impl->net.load_model(bin)!=0){printf("[Det]ERR bin:%s\n",bin);delete d->impl;delete d;return nullptr;}
    printf("[Det]NCNN loaded %s %dx%d\n",param,sz,sz);
    return d;
}

int ncnn_det_run(NCNNDet *d, const unsigned char *bgr, int w, int h, Det *out, int mx){
    auto t0=std::chrono::steady_clock::now();
    int sz=d->sz;
    float sc=std::min((float)sz/w,(float)sz/h);
    int nw=(int)(w*sc),nh=(int)(h*sc);
    ncnn::Mat in=ncnn::Mat::from_pixels_resize(bgr,ncnn::Mat::PIXEL_BGR2RGB,w,h,nw,nh);
    int wp=sz-nw,hp=sz-nh;
    ncnn::Mat inp; ncnn::copy_make_border(in,inp,hp/2,hp-hp/2,wp/2,wp-wp/2,ncnn::BORDER_CONSTANT,114.f);
    const float norm[3]={1/255.f,1/255.f,1/255.f}; inp.substract_mean_normalize(0,norm);
    ncnn::Extractor ex=d->impl->net.create_extractor();
    ex.input("in0",inp); ncnn::Mat no; ex.extract("out0",no);
    int np2=no.h, nc=no.w-4; if(nc<=0)nc=4;
    struct R{float x1,y1,x2,y2;int c;float s;};
    std::vector<R> raw;
    for(int i=0;i<np2;i++){
        const float *p=no.row(i);
        float bx=p[0],by=p[1],bw=p[2],bh=p[3];
        int bc=0;float bs=-1e9f;
        for(int c2=0;c2<nc;c2++){if(p[4+c2]>bs){bs=p[4+c2];bc=c2;}}
        if(bs<d->ct)continue;
        float x1=(bx-bw/2-wp/2)/sc,y1=(by-bh/2-hp/2)/sc;
        float x2=(bx+bw/2-wp/2)/sc,y2=(by+bh/2-hp/2)/sc;
        x1=std::max(0.f,std::min((float)w,x1));y1=std::max(0.f,std::min((float)h,y1));
        x2=std::max(0.f,std::min((float)w,x2));y2=std::max(0.f,std::min((float)h,y2));
        raw.push_back({x1,y1,x2,y2,bc,bs});
    }
    std::sort(raw.begin(),raw.end(),[](const R&a,const R&b){return a.s>b.s;});
    std::vector<bool> sup(raw.size(),false);
    int cnt=0;
    for(size_t i=0;i<raw.size()&&cnt<mx;i++){
        if(sup[i])continue; R&r=raw[i];
        Det *det=&out[cnt++];
        det->cls=r.c; det->conf=r.s;
        det->name=(r.c<4)?CLASS_NAMES[r.c]:"unknown";
        det->x1=(int)r.x1;det->y1=(int)r.y1;det->x2=(int)r.x2;det->y2=(int)r.y2;
        det->cx=(det->x1+det->x2)/2;det->cy=(det->y1+det->y2)/2;
        det->w=det->x2-det->x1;det->h=det->y2-det->y1;
        det->rel_area=(float)(det->w*det->h)/(float)(w*h);
        for(size_t j=i+1;j<raw.size();j++){
            if(sup[j])continue;
            float ix1=std::max(r.x1,raw[j].x1),iy1=std::max(r.y1,raw[j].y1);
            float ix2=std::min(r.x2,raw[j].x2),iy2=std::min(r.y2,raw[j].y2);
            float inter=std::max(0.f,ix2-ix1)*std::max(0.f,iy2-iy1);
            float a1=(r.x2-r.x1)*(r.y2-r.y1),a2=(raw[j].x2-raw[j].x1)*(raw[j].y2-raw[j].y1);
            if(inter/(a1+a2-inter)>d->it)sup[j]=true;
        }
    }
    auto t1=std::chrono::steady_clock::now();
    double msec=std::chrono::duration<double,std::milli>(t1-t0).count();
    d->avg_ms=d->avg_ms*0.9+msec*0.1;
    if(msec>0)d->fps=d->fps*0.9+(1000.0/msec)*0.1;
    return cnt;
}

void ncnn_det_destroy(NCNNDet *d){if(d){if(d->impl)delete d->impl;delete d;}}

/* Async detector thread */
static void *adet_loop(void *arg){
    AsyncDet *a=(AsyncDet*)arg;
    while(a->run){
        pthread_mutex_lock(&a->lk);
        while(!a->ready&&a->run) pthread_cond_wait(&a->cv,&a->lk);
        if(!a->run){pthread_mutex_unlock(&a->lk);break;}
        a->ready=false;
        size_t sz=a->fw*a->fh*3;
        unsigned char *loc=(unsigned char*)malloc(sz);
        memcpy(loc,a->buf,sz);
        int fw2=a->fw,fh2=a->fh;
        pthread_mutex_unlock(&a->lk);
        Det dets[MAX_DET];
        int n=ncnn_det_run(a->det,loc,fw2,fh2,dets,MAX_DET);
        pthread_mutex_lock(&a->lk);
        a->n=n;memcpy(a->latest,dets,sizeof(Det)*n);a->cnt++;
        pthread_mutex_unlock(&a->lk);
        free(loc);
    }
    return nullptr;
}
AsyncDet* adet_create(NCNNDet *d,int fw,int fh){
    AsyncDet *a=new AsyncDet();a->det=d;a->fw=fw;a->fh=fh;
    a->buf=(unsigned char*)malloc(fw*fh*3);a->ready=false;a->n=0;a->cnt=0;
    pthread_mutex_init(&a->lk,NULL);pthread_cond_init(&a->cv,NULL);
    a->run=true;pthread_create(&a->thr,NULL,adet_loop,a);return a;
}
void adet_submit(AsyncDet *a,const unsigned char *bgr){
    pthread_mutex_lock(&a->lk);memcpy(a->buf,bgr,a->fw*a->fh*3);a->ready=true;
    pthread_cond_signal(&a->cv);pthread_mutex_unlock(&a->lk);
}
int adet_get(AsyncDet *a,Det *out,int mx){
    pthread_mutex_lock(&a->lk);int n=a->n;if(n>mx)n=mx;
    memcpy(out,a->latest,sizeof(Det)*n);pthread_mutex_unlock(&a->lk);return n;
}
void adet_destroy(AsyncDet *a){
    if(!a)return;a->run=false;pthread_cond_signal(&a->cv);
    pthread_join(a->thr,NULL);free(a->buf);delete a;
}

} /* extern "C" */
