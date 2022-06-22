#pragma once

#include<algorithm>
#include<array>

#include "server.h"


extern std::map<unsigned int, vcg::Shotf> virtualCameras;
extern unsigned int activeCamera;
extern bool streamON;
extern bool lidarOn[2];
extern bool camerasOn[6];

void sampleGeometry(float xi, float yi, float& x, float& y, float& z);

extern Server serverComm;
extern Server serverStream;
