
#include <map>

#include "..\header\nausicaa_api_server.h"
#include "..\header\deserialize.h"
#include "..\header\server.h"

std::map<unsigned int, vcg::Shotf> virtualCameras;
unsigned int activeCamera;


void call_API_function(std::string message) {
	std::string fname = func_name(message);
	if (fname == std::string("startStreaming"))
	{
		// start the streaming
	}
	else
	if (fname == std::string("stopStreaming"))
	{
		// stop the streaming
	}
	else
	if (fname == std::string("addVirtualCamera"))
	{
		std::cout << "addVritualCamera" << std::endl;
		unsigned int newId = virtualCameras.size();
		virtualCameras[newId] = vcg::Shotf();
		send(std::to_string(newId));
	}
	else
	if (fname == std::string("getVirtualCamera"))
	{
		int id = deserialize_int(message);
	}
	else
	if (fname == std::string("getVirtualCameraList"))
	{
		// return camera list
	}
	if (fname == std::string("renderFromCamera"))
	{
		int id = deserialize_int(message);
		activeCamera = id ;
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
		float xi =  deserialize_float(message);
		float yi = deserialize_float(message);
		float longitude = deserialize_float(message);
		float latitude = deserialize_float(message);
		// sampleGeometry(xi,yi,longitude,latitude)
	}
	else
	if (fname == std::string("sampleGeometry"))
	{
		float xi = deserialize_float(message);
		float yi = deserialize_float(message);
		float longitude = deserialize_float(message);
		float latitude = deserialize_float(message);
		// sampleGeometry(xi,yi,longitude,latitude)
	}
	else
	if (fname == std::string("enableLidar"))
	{
		int id = deserialize_int(message);
		// enable lidar
	}
	else
	if (fname == std::string("disableLidar"))
	{
		int id = deserialize_int(message);
		// disable lidar
	}
	else
	if (fname == std::string("enableCamera"))
	{
	int id = deserialize_int(message);
	// enable camera
	}
	else
	if (fname == std::string("disableCamera"))
	{
		int id = deserialize_int(message);
		// disable camera
	}
	else
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
	}
	else
	if (fname == std::string("setViewDirection"))
	{
		int id = deserialize_int(message);
		float dirX = deserialize_float(message);
		float dirY = deserialize_float(message);
		float dirZ = deserialize_float(message);

		virtualCameras[id].LookTowards(vcg::Point3f(dirX, dirY, dirZ),vcg::Point3f(0.0,1.0,0.0));
	}
	else
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
	if (fname == std::string("setCameraFrustrum"))
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
	}
	else
	if (fname == std::string("getFrustum"))
	{
		int id = deserialize_int(message);
		float sx, dx, bt, tp, focal;
		virtualCameras[id].Intrinsics.GetFrustum(sx, dx, bt, tp, focal);
	}
}


