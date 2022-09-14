

#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>


#include "..\header\deserialize.h"
#include "..\header\server.h"
#include "..\header\nausicaa_api_server.h"
#include "..\header\Common.h"


std::map<unsigned int, vcg::Shotf> virtualCameras;
std::map<unsigned int, Marker> markers;
unsigned int activeCamera;
bool streamON;
bool lidarOn[2];
bool camerasOn[6];

std::mutex m;
std::condition_variable condv;
bool picked = false;


extern float  LatDecimalDegrees, LonDecimalDegrees, ElevationMeters;
extern float  pitchDegrees, rollDegrees;
extern float  bowDirectionDegrees;
extern int NUMCAM;


Server serverComm, serverStream;


void call_API_function(std::string message) {
	std::string fname = func_name(message);
	if (fname == std::string("startStreaming"))
	{
		streamON = true;
	}
	else
	if (fname == std::string("stopStreaming"))
	{
		streamON = false;
	}
	else
	if (fname == std::string("updatePositionWGS84"))
	{
		LatDecimalDegrees = deserialize_float(message);
		LonDecimalDegrees = deserialize_float(message);
		ElevationMeters = deserialize_float(message);
	}
	else
	if (fname == std::string("updatePitchRoll"))
	{

		pitchDegrees = deserialize_float(message);
		rollDegrees = deserialize_float(message);

	}
	else
	if (fname == std::string("updateBowDirection"))
	{
		bowDirectionDegrees = deserialize_float(message);
	}
	else
	if (fname == std::string("addVirtualCamera"))
	{
		std::cout << "addVirtualCamera" << std::endl;
		unsigned int newId = virtualCameras.size();
		virtualCameras[newId] = vcg::Shotf();
		serverComm.send(std::to_string(newId));
	}
	else
	if (fname == std::string("getVirtualCamera"))
	{
		int id = deserialize_int(message);
	}
	else
	if (fname == std::string("getVirtualCameraList"))
	{
		int  none = -1;
		std::vector<int> cId;
		for (std::map<unsigned int, vcg::Shotf>::iterator ic = virtualCameras.begin(); ic != virtualCameras.end(); ++ic)
			cId.push_back((*ic).first);
		if(!cId.empty())
			serverComm.send((char*) & *cId.begin(), cId.size()*sizeof(int));
		else
			serverComm.send((char*) & none, 4);

	}
	if (fname == std::string("renderFromCamera"))
	{
		int id = deserialize_int(message);
		activeCamera = id ;
		std::cout << "rendering from camera:" << std::endl;
	}
	else
	if (fname == std::string("selectQuality"))
	{
		float q = deserialize_float(message);
		// set quality
	}
	else
	if (fname == std::string("sampleGeometry"))
	{
		pick_x =	deserialize_float(message);
		pick_y =	deserialize_float(message);

		
		std::unique_lock lk(m);
		
		pick_point = true;
		condv.wait(lk, [] {return picked;});
		picked = false;
		serverComm.send((char*)  & picked_point[0], 6 * sizeof(float));

		lk.unlock();

	}
	else
	if (fname == std::string("enableLidar"))
	{
		int id = deserialize_int(message);
		lidarOn[id] = true;
	}
	else
	if (fname == std::string("enableAllLidars"))
	{
		lidarOn[0] = lidarOn[1] = true;
	}
	else
	if (fname == std::string("disableAllLidars"))
	{
		lidarOn[0] = lidarOn[1] = false;
	}
	else
	if (fname == std::string("disableLidar"))
	{
		int id = deserialize_int(message);
		lidarOn[id] = false;
	}
	else
	if (fname == std::string("enableCamera"))
	{
		int id = deserialize_int(message);
		camerasOn[id] = true;
	}
	else
	if (fname == std::string("disableCamera"))
	{
		int id = deserialize_int(message);
		camerasOn[id] = false;
	}
	else
	if (fname == std::string("enableAllCameras"))
	{
		for(int i= 0; i < NUMCAM; ++i) 
			camerasOn[i] = true;
	}
	else
	if (fname == std::string("disableAllCameras"))
	{
		for (int i = 0; i < NUMCAM; ++i)
			camerasOn[i] = false;
	}
	else
	if (fname == std::string("addMarker"))
	{
		int newID = markers.size();
		markers[newID].label  = deserialize_string(message);
		//markers[newID].pos[0] = deserialize_float(message);
		//markers[newID].pos[1] = deserialize_float(message);
		//markers[newID].pos[2] = deserialize_float(message);
		markers[newID].png_data = new unsigned char[serverComm.blob_bin_length];
		memcpy_s(markers[newID].png_data, serverComm.blob_bin_length, serverComm.blob_bin, serverComm.blob_bin_length);
		markers[newID].png_data_length = serverComm.blob_bin_length;
	}
	if (fname == std::string("setPerspective"))
	{
		int id			= deserialize_int(message);
		float angle		= deserialize_float(message);
		float ratio		= deserialize_float(message);
		float focal		= deserialize_float(message);
		int viewportX	= deserialize_int(message);
		int viewportY	= deserialize_int(message);

		virtualCameras[id].Intrinsics.SetPerspective(angle, ratio, focal, vcg::Point2i(viewportX, viewportY));
	}
	else
	if (fname == std::string("setViewport"))
	{
		int id = deserialize_int(message);
		int viewportX = deserialize_int(message);
		int viewportY = deserialize_int(message);

		virtualCameras[id].Intrinsics.ViewportPx = vcg::Point2i(viewportX, viewportY);
	}
	else
	if (fname == std::string("setFocalLength"))
	{
		int id = deserialize_int(message);
		float focal = deserialize_float(message);

		virtualCameras[id].Intrinsics.FocalMm = focal;
	}
	else
	if (fname == std::string("setAngle"))
	{
		int id = deserialize_int(message);
		float angle = deserialize_float(message);

		vcg::Camera<float>& c = virtualCameras[id].Intrinsics;
		float l, r, t, b,focal;
		c.GetFrustum(l, r, b, t, focal);
		c.SetPerspective(angle,  (r-l)/(t-b), focal, c.ViewportPx);
	}
	else
	if (fname == std::string("setAspectratio"))
	{
		int id = deserialize_int(message);
		// not implemented
	}
	else
	if (fname == std::string("lookTowards"))
	{
		int id = deserialize_int(message);
		float eyeX = deserialize_float(message);
		float eyeY = deserialize_float(message);
		float eyeZ = deserialize_float(message);
		float dirX = deserialize_float(message);
		float dirY = deserialize_float(message);
		float dirZ = deserialize_float(message);
		float upX  = deserialize_float(message);
		float upY  = deserialize_float(message);
		float upZ  = deserialize_float(message);
		virtualCameras[id].LookAt(eyeX, eyeY,eyeZ, dirX, dirY, dirZ, upX, upY, upZ);
	}
	else
	if (fname == std::string("setPosition"))
	{
		int id = deserialize_int(message);
		float eyeX = deserialize_float(message);
		float eyeY = deserialize_float(message);
		float eyeZ = deserialize_float(message);

		virtualCameras[id].SetViewPoint(vcg::Point3f(eyeX, eyeY, eyeZ));
		std::cout << "set position done!" << std::endl;
	}
	if (fname == std::string("setViewDirection"))
	{
		int id = deserialize_int(message);
		float dirX = deserialize_float(message);
		float dirY = deserialize_float(message);
		float dirZ = deserialize_float(message);

		virtualCameras[id].LookTowards(vcg::Point3f(dirX, dirY, dirZ), vcg::Point3f(0.0, 1.0, 0.0));
		std::cout << "view-direction set:" << std::endl;
	}
	else
	/*if (fname == std::string("setPosition"))
	{
		int id = deserialize_int(message);
		float dirX = deserialize_float(message);
		float dirY = deserialize_float(message);
		float dirZ = deserialize_float(message);

		virtualCameras[id].LookTowards(vcg::Point3f(dirX, dirY, dirZ),vcg::Point3f(0.0,1.0,0.0));
	}
	else*/
	if (fname == std::string("move"))
	{
		int id = deserialize_int(message);
		int dirid = deserialize_int(message);
		int amount = deserialize_int(message);

		vcg::Point3f p = virtualCameras[id].GetViewPoint();
		p[dirid] += amount;
		virtualCameras[id].SetViewPoint(p);
	}
	else
	if (fname == std::string("rotate"))
	{
		int id = deserialize_int(message);
		int dirid = deserialize_int(message);
		int amount = deserialize_int(message);

		vcg::Matrix44f M = virtualCameras[id].Extrinsics.Rot();
		vcg::Matrix44f R;
		R.SetRotateDeg(-amount, vcg::Point3f(dirid == 1, dirid == 0, dirid == 2));
		M = M * R;
		virtualCameras[id].Extrinsics.SetRot(M);
	}
	else
	if (fname == std::string("setCameraFrustum"))
	{
		int id = deserialize_int(message);
		float sx = deserialize_float(message);
		float dx= deserialize_float(message);
		float bt = deserialize_float(message);
		float tp = deserialize_float(message);;
		float focal = deserialize_float(message);
		int vx = deserialize_int(message);
		int vy = deserialize_int(message);

		virtualCameras[id].Intrinsics.SetFrustum(sx, dx, bt, tp, focal, vcg::Point2i(vx, vy));
		
		std::cout << " camera frustrum set" << std::endl;
	}
	else
	if (fname == std::string("getFrustum"))
	{
		int id = deserialize_int(message);
		float frustum[5];
		virtualCameras[id].Intrinsics.GetFrustum(frustum[0], frustum[1], frustum[2], frustum[3], frustum[4]);
		serverComm.send((char*)&frustum[0], 5 * sizeof(float));
	}
	else
		if (fname == std::string("getPositionAndDirection"))
		{
			int id = deserialize_int(message);
			float posdir[9];
			*(vcg::Point3f*)&posdir[0] = virtualCameras[id].GetViewPoint();
			vcg::Matrix44f rot = virtualCameras[id].Extrinsics.Rot();
			*(vcg::Point3f*)&posdir[3] = virtualCameras[id].Extrinsics.Rot().GetColumn3(2);
			posdir[3] *= -1.0;
			posdir[4] *= -1.0;
			posdir[5] *= -1.0;

			*(vcg::Point3f*)&posdir[6] = virtualCameras[id].Extrinsics.Rot().GetColumn3(1);
			serverComm.send((char*)&posdir[0], 9 * sizeof(float));
		}
}



