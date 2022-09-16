#pragma once
#define SCENE_REPLAY
#define MJPEG_WRITE
#define RECTIFY_FIRST


#ifdef SCENE_REPLAY
extern unsigned long long start_time;
extern unsigned long long restart_time;
extern unsigned int partial_time;
extern unsigned long long end_time;
extern unsigned int  virtual_time;
extern bool time_running;
#endif

#ifndef SAVE_IMG
#define SAVE_IMG 0
#endif

#ifndef SAVE_PC
#define SAVE_PC 0
#endif

#ifndef VIDEO_STREAM
#define VIDEO_STREAM 0
#endif