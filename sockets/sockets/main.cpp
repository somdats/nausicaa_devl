

#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\client.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\serialize.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\nausicaa_api.h"

int value;
void main() {


	int err = VRSubsystem::connectToVRServer("146.48.84.241");
	if ( err !=0)
	{
		std::cout << "connection failed with err \n" <<err << std::endl;
		exit(0);
	};

	{
		int* cameras, n;
		cameras = VRSubsystem::getVirtualCameraList(&n);
		printf("number of cameras: %d\n", n);
		for (int i = 0;i < n; ++i)
			printf("camera id: %d\n", cameras[i]);
	}

	int newCamera = VRSubsystem::addVirtualCamera();
	VirtualCamera::setCameraFrustrum(newCamera, -0.2, 0.2, -0.2, 0.2, 0.2,
		640, 480);
	VirtualCamera::setPosition(newCamera,2.0, -1.0, 2.0);
	VirtualCamera::setViewDirection(newCamera,0.0, 0.0, -1.0);
	VRSubsystem::renderFromCamera(newCamera);

	VRSubsystem::startStreaming();

	VRSubsystem::disableLidar(0);
	VRSubsystem::enableLidar(0);
	VRSubsystem::disableCamera(0);
	VRSubsystem::enableCamera(0);

	char * frame;
	int size;
	while(true) {
		frame = VRSubsystem::readFrame(&size);
		if (frame) {
			FILE* fo = fopen("frame.jpg", "wb");
			fwrite(frame, size, 1, fo);
			fclose(fo);
		}
		}
}



