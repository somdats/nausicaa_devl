

#include<winsock2.h>
#include <ws2tcpip.h>

#include"..\header\nausicaa_api.h"
#include "..\header\client.h"
#include "..\header\serialize.h"


#define NI cbF("not implemented yet\n");


void (*cbF)(const char*) {};


void VRSubsystem::setCallback(void (*ft)(const char*)){
	cbF = ft;
};

void VRSubsystem::readConfigFile(const char*) { NI }

int  VRSubsystem::connectToVRServer(const char * ip_addr) {
	int err_comm = connect(ip_addr);
	int err_stream = connect_stream(ip_addr);
	if(err_comm !=0)
		std::cout <<"error err_comm " << err_comm << std::endl;
	if (err_stream != 0)
		std::cout << "error err_comm " << err_stream << std::endl;
	return (err_comm == 0 && err_stream == 0);
}

void VRSubsystem::disconnectToVRServer() {
	send_message(message()[std::string("disconnectToVRServer")].msg);
	std::cout << "disconnect To VRServer" << std::endl;
}

void VRSubsystem::startStreaming() {
	start_stream();
//	send_message(message()[std::string("startStreaming")].msg);
	std::cout << " streaming start" << std::endl;
}

image_buffer VRSubsystem::readFrame() {
	return receive_image();
}

void VRSubsystem::stopStreaming() {
	send_message(message()[std::string("stopStreaming")].msg);
}

VirtualCameraID VRSubsystem::addVirtualCamera() {
	send_message(message()[std::string("addVirtualCamera")].msg);
	return receive_int();;
}

void VRSubsystem::getVirtualCamera(VirtualCameraID id) {
	std::cout << "removed " << id <<  std::endl;
}

void VRSubsystem::getVirtualCameraList() {
	send_message(message()[std::string("getVirtualCameraList")].msg);
	std::cout << "get VirtualCameraList" <<  std::endl;
}
	

void VRSubsystem::renderFromCamera(VirtualCameraID id) {
	send_message(message()[std::string("renderFromCamera")][id].msg);
	std::cout << "render camera with:" << id << std::endl;
}

VirtualCameraID VRSubsystem::renderCamera() {
	VirtualCameraID id (0);
	std::cout << "removed" << std::endl;
	return id;
}

void VRSubsystem::selectQuality(float val) {
	std::cout << "Quality selection:" << val << std::endl;
}

void VRSubsystem::sampleGeometry(float xi, float yi, float& longitude, float& latitude)
{ 
	std::cout << "sample-Geometry" << std::endl;
}

void VRSubsystem::enableLidar(lidarID id) { 
	send_message(message()[std::string("enableLidar")][id].msg);
	std::cout << "enable Lidar" << id << std::endl;
}

void VRSubsystem::disableLidar(lidarID id) { 
	send_message(message()[std::string("disableLidar")][id].msg);
	std::cout << "disableLidar " << id << std::endl;
}

void VRSubsystem::enableCamera(cameraID id) { 
	send_message(message()[std::string("enableCamera")][id].msg);

	std::cout << "enable Camera " << id << std::endl;
}

void VRSubsystem::disableCamera(cameraID id) { 
	send_message(message()[std::string("disableCamera")][id].msg);

	std::cout << "disable Camera " << id << std::endl;
}

gliphID VRSubsystem::place3DGliph(gliphTypeID, float x, float y, float d) {
	gliphID id(-1);
	std::cout << "Place 3D gliph" << std::endl;
	return id;
}

void VRSubsystem::remove3DGliph(gliphID) { 

	std::cout << " remove 3d gliph" << std::endl;
}

gliphTypeID VRSubsystem::typeOf(gliphID) {
	return 0;
}


void VRSubsystem::enableGliphs() { 
	std::cout << " enable gliph" << std::endl;
}

void VRSubsystem::disableGliphs() {
	std::cout << " disable  gliph" << std::endl;

}


void VirtualCamera::setViewport(VirtualCameraID cId, int new_viewportX, int new_viewportY) {
	send_message(message()[std::string("setViewport")][cId][new_viewportX][new_viewportY].msg);
	std::cout << " set view-port" << std::endl;
}

void VirtualCamera::setFocalLength(VirtualCameraID cId, float newFocalLength) {
	send_message(message()[std::string("setFocalLength")][cId][newFocalLength].msg);
	std::cout << " setFocalLength" << std::endl;
	
}

void VirtualCamera::setAngle(VirtualCameraID cId, float newAngle) {
	send_message(message()[std::string("setAngle")][cId][newAngle].msg);

	std::cout << " setAngle" << std::endl;
	
}

void VirtualCamera::setAspectratio(VirtualCameraID cId, float newAspectRatio) {
	send_message(message()[std::string("setAspectratio")][cId][newAspectRatio].msg);

	std::cout << "setAspectratio" << std::endl;
}

void VirtualCamera::lookTowards(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ, float dirX,
	float dirY, float dirZ, float upX, float upY, float upZ) {
	send_message(message()[std::string("lookTowards")][cId][eyeX][eyeY][eyeZ][dirX][dirY][dirZ][upX][upY][upZ].msg);

	std::cout << "lookTowards" << std::endl;
	
}

void VirtualCamera::setPosition(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ) {
	send_message(message()[std::string("setPosition")][cId][eyeX][eyeY][eyeZ].msg);

	std::cout << "setPosition" << std::endl;
}

void VirtualCamera::setViewDirection(VirtualCameraID cId, float dirX, float dirY, float dirZ) {
	send_message(message()[std::string("setViewDirection")][cId][dirX][dirY][dirZ].msg);
	// what is viewDirection?
}
void VirtualCamera::setCameraFrustrum(VirtualCameraID cId, float sx, float dx, float bt, float tp, float focalLength,
	int viewPortX, int viewPortY) {
	send_message(message()[std::string("setCameraFrustrum")][cId][sx][dx][bt][tp][focalLength][viewPortX][viewPortY].msg);
	std::cout << "Set CameraFrustrum" << std::endl;
	
}


void VirtualCamera::getFrustum(VirtualCameraID cId, int& sx, int& dx, int& bt, int& tp, float& nr) {
	send_message(message()[std::string("getFrustum")][cId][sx][dx][bt][tp][nr].msg);
	std::cout << "get CameraFrustrum" << std::endl;
}


void VirtualCamera::setPerspective(VirtualCameraID cId, float AngleDeg, float AspectRatio, float Focal, int viewportX, int viewportY) {
	send_message(message()[std::string("setPerspective")][cId][AngleDeg][AspectRatio][Focal][viewportX][viewportY].msg);
	std::cout << "set perspective:" << std::endl;
}
void VirtualCamera::move(VirtualCameraID cId, changeDir xyz, float amount) {
	send_message(message()[std::string("move")][cId][xyz][amount].msg);

	std::cout << "Move" << std::endl;
}

void VirtualCamera::rotate(VirtualCameraID cId, angleDir yaw_pitch_roll, float amountDeg) {

	send_message(message()[std::string("rotate")][cId][yaw_pitch_roll][amountDeg].msg);
	std::cout << "rotate" << std::endl;

}



