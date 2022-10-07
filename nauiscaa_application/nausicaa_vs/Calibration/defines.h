#pragma once

#define MJPEG_WRITE
#define RECTIFY_FIRST


extern bool  SCENE_REPLAY;
extern bool  SAVE_PC;
extern bool SAVE_IMG;

extern unsigned long long start_time;
extern unsigned long long restart_time;
extern unsigned int partial_time;
extern unsigned long long end_time;
extern unsigned int  virtual_time;
extern bool time_running;


//#ifndef SAVE_IMG
//#define SAVE_IMG 1
//#endif


#ifndef VIDEO_STREAM
#define VIDEO_STREAM 0
#endif