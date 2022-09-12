
#include <opencv2/opencv.hpp>
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\client.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\serialize.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\nausicaa_api.h"

using namespace cv;

int value;
float X, Y, Z, Xg, Yg, Zg;
cv::Mat im;

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN) {
		VRSubsystem::sampleGeometry(x, im.rows-y, &X, &Y, &Z, &Xg, &Yg, &Zg);
		printf("%d %d \n", x, im.rows - y);
		printf("local: %f %f %f \n Geo: %f %f %f  ", X, Y, Z, Xg, Yg, Zg);
	}
}


void main() {
	cv::namedWindow("monitor", 1);
	cv::setMouseCallback("monitor", CallBackFunc,0);
	
	FILE * iconF  = fopen("boat_icon.png","rb");
	fseek(iconF, 0,SEEK_END);
	int length = ftell(iconF);
	char* data = new char[length];
	fread(data, 1, length, iconF);
	

	int err = VRSubsystem::connectToVRServer("127.0.0.1");
	if ( err !=0)
	{
		std::cout << "connection failed with err \n" <<err << std::endl;
		exit(0);
	};
	{
		VRSubsystem::addMarker(data, length, 1.0, "prova", 5);
	}
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
	VirtualCamera::setPosition(newCamera,0.0, 0.0, 5.0);
	VirtualCamera::setViewDirection(newCamera,0.0, 0.0, -1.0);

	float  sx, dx, bt, tp, nr;
	VirtualCamera::getFrustum(newCamera, &sx, &dx, &bt, &tp, &nr);
	printf("cam %d, frustum:  %f %f %f %f %f \n", newCamera,sx, dx, bt, tp, nr);

	float eX, eY, eZ, dX, dY, dZ, uX, uY, uZ;
	VirtualCamera::getPositionAndDirection(newCamera, &eX, &eY,&eZ, &dX, &dY, &dZ, &uX,&uY,&uZ);
	printf("cam %d, posdir: %f %f %f %f %f %f %f %f %f \n", newCamera, eX, eY, eZ, dX, dY, dZ, uX, uY, uZ);


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
			im = cv::imread("frame.jpg");
			cv::imshow("monitor", im);
			if (cv::waitKey(1) == 'b') {
				break;
			}
		}
		}
}



