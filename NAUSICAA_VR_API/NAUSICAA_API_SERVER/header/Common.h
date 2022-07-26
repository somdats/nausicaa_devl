#pragma once

#include<algorithm>
#include<array>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vcg/math/shot.h>
#include "server.h"


extern std::map<unsigned int, vcg::Shotf> virtualCameras;
extern unsigned int activeCamera;
extern bool streamON;
extern bool lidarOn[2];
extern bool camerasOn[6];

extern std::mutex m;
extern std::condition_variable condv;
extern bool picked;
extern bool pick_point;
extern int pick_x, pick_y;
extern float picked_point[3];

void sampleGeometry(float xi, float yi, float& x, float& y, float& z);

extern Server serverComm;
extern Server serverStream;
