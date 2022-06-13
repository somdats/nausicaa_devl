#pragma once
#define SCENE_REPLAY


#ifdef SCENE_REPLAY
extern unsigned long long start_time;
extern unsigned long long restart_time;
extern unsigned int partial_time;
extern unsigned long long end_time;
extern unsigned int  virtual_time;
extern bool time_running;
#endif