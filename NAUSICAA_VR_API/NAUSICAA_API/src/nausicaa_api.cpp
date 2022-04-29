#include "..\header\nausicaa_api.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include "..\header\client.h"
#include "..\header\serialize.h"


#define NI cbF("not implemented yet\n");


void (*cbF)(const char*) {};

Client clientComm, clientStream;

void VRSubsystem::setCallback(void (*ft)(const char*)) {
	cbF = ft;
};

void VRSubsystem::readConfigFile(const char*) { NI }

int  VRSubsystem::connectToVRServer(const char* ip_addr) {
	int err_comm = clientComm.connect(ip_addr,81);
	if (err_comm != 0) {
		std::cout << "error err_comm " << err_comm << std::endl;
		return err_comm;
	}
	int err_stream = clientStream.connect(ip_addr,82);
	if (err_stream != 0) {
		std::cout << "error err_comm " << err_stream << std::endl;
		return  -err_stream;
	}
	return 0;
}

void VRSubsystem::disconnectToVRServer() {
	clientComm.send_message(message()[std::string("disconnectToVRServer")].msg);
	std::cout << "disconnect To VRServer" << std::endl;
}

void VRSubsystem::startStreaming() {
	clientComm.send_message(message()[std::string("startStreaming")].msg);
	std::cout << " streaming start" << std::endl;
}

image_buffer VRSubsystem::readFrame(int* byteCount) {
	return clientStream.receive_image(byteCount);
}

void VRSubsystem::stopStreaming() {
	clientComm.send_message(message()[std::string("stopStreaming")].msg);
}

VirtualCameraID VRSubsystem::addVirtualCamera() {
	clientComm.send_message(message()[std::string("addVirtualCamera")].msg);
	return clientComm.receive_int();
}

void VRSubsystem::getVirtualCamera(VirtualCameraID id) {
	std::cout << "removed " << id << std::endl;
}

int *  VRSubsystem::getVirtualCameraList(int * n_cameras) {
	int byteCount;
	clientComm.send_message(message()[std::string("getVirtualCameraList")].msg);
	int * res  =  (int*) clientComm.receive_image(&byteCount);
	*n_cameras = byteCount / 4;
	return res;
}


void VRSubsystem::renderFromCamera(VirtualCameraID id) {
	clientComm.send_message(message()[std::string("renderFromCamera")][id].msg);
	std::cout << "render camera with:" << id << std::endl;
}

VirtualCameraID VRSubsystem::renderCamera() {
	VirtualCameraID id(0);
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
	clientComm.send_message(message()[std::string("enableLidar")][id].msg);
	std::cout << "enable Lidar" << id << std::endl;
}

void VRSubsystem::disableLidar(lidarID id) {
	clientComm.send_message(message()[std::string("disableLidar")][id].msg);
	std::cout << "disableLidar " << id << std::endl;
}

void VRSubsystem::enableCamera(cameraID id) {
	clientComm.send_message(message()[std::string("enableCamera")][id].msg);

	std::cout << "enable Camera " << id << std::endl;
}

void VRSubsystem::disableCamera(cameraID id) {
	clientComm.send_message(message()[std::string("disableCamera")][id].msg);

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
	clientComm.send_message(message()[std::string("setViewport")][cId][new_viewportX][new_viewportY].msg);
	std::cout << " set view-port" << std::endl;
}

void VirtualCamera::setFocalLength(VirtualCameraID cId, float newFocalLength) {
	clientComm.send_message(message()[std::string("setFocalLength")][cId][newFocalLength].msg);
	std::cout << " setFocalLength" << std::endl;

}

void VirtualCamera::setAngle(VirtualCameraID cId, float newAngle) {
	clientComm.send_message(message()[std::string("setAngle")][cId][newAngle].msg);

	std::cout << " setAngle" << std::endl;

}

void VirtualCamera::setAspectratio(VirtualCameraID cId, float newAspectRatio) {
	clientComm.send_message(message()[std::string("setAspectratio")][cId][newAspectRatio].msg);

	std::cout << "setAspectratio" << std::endl;
}

void VirtualCamera::lookTowards(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ, float dirX,
	float dirY, float dirZ, float upX, float upY, float upZ) {
	clientComm.send_message(message()[std::string("lookTowards")][cId][eyeX][eyeY][eyeZ][dirX][dirY][dirZ][upX][upY][upZ].msg);

	std::cout << "lookTowards" << std::endl;

}

void VirtualCamera::setPosition(VirtualCameraID cId, float eyeX, float eyeY, float eyeZ) {
	clientComm.send_message(message()[std::string("setPosition")][cId][eyeX][eyeY][eyeZ].msg);

	std::cout << "setPosition" << std::endl;
}

void VirtualCamera::setViewDirection(VirtualCameraID cId, float dirX, float dirY, float dirZ) {
	clientComm.send_message(message()[std::string("setViewDirection")][cId][dirX][dirY][dirZ].msg);
	// what is viewDirection?
}
void VirtualCamera::setCameraFrustrum(VirtualCameraID cId, float sx, float dx, float bt, float tp, float focalLength,
	int viewPortX, int viewPortY) {
	clientComm.send_message(message()[std::string("setCameraFrustrum")][cId][sx][dx][bt][tp][focalLength][viewPortX][viewPortY].msg);
	std::cout << "Set CameraFrustrum" << std::endl;

}


void VirtualCamera::getFrustum(VirtualCameraID cId, int& sx, int& dx, int& bt, int& tp, float& nr) {
	clientComm.send_message(message()[std::string("getFrustum")][cId][sx][dx][bt][tp][nr].msg);
	std::cout << "get CameraFrustrum" << std::endl;
}


void VirtualCamera::setPerspective(VirtualCameraID cId, float AngleDeg, float AspectRatio, float Focal, int viewportX, int viewportY) {
	clientComm.send_message(message()[std::string("setPerspective")][cId][AngleDeg][AspectRatio][Focal][viewportX][viewportY].msg);
	std::cout << "set perspective:" << std::endl;
}
void VirtualCamera::move(VirtualCameraID cId, changeDir xyz, float amount) {
	clientComm.send_message(message()[std::string("move")][cId][xyz][amount].msg);

	std::cout << "Move" << std::endl;
}

void VirtualCamera::rotate(VirtualCameraID cId, angleDir yaw_pitch_roll, float amountDeg) {

	clientComm.send_message(message()[std::string("rotate")][cId][yaw_pitch_roll][amountDeg].msg);
	std::cout << "rotate" << std::endl;

}



