#pragma once

#include<algorithm>
#include<array>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vcg/math/shot.h>
#include "../../../nauiscaa_application/nausicaa_vs/Calibration/marker.h"
#include "server.h"


extern std::map<unsigned int, vcg::Shotf> virtualCameras;
extern std::map<unsigned int, Marker> markers;
extern unsigned int activeCamera;
extern bool streamON;
extern bool lidarOn[2];
extern bool camerasOn[6];
extern bool showBackground;
extern float bottom_sel, top_sel, inner_sel, outer_sel;

extern std::mutex m,activeCamera_mutex,m_sel;
extern std::condition_variable condv,cond_sel;
extern bool picked;
extern bool pick_point;
extern bool sel_points,selected_points;
extern int pick_x, pick_y;
extern float picked_point[6];
extern std::vector<vcg::Point3f> points_to_send;

void sampleGeometry(float xi, float yi, float& x, float& y, float& z);

extern Server serverComm;
extern Server serverStream;
