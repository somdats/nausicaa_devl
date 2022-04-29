

#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\client.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\serialize.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API\header\nausicaa_api.h"

int value;
void main() {
/*
	int err = connect("127.0.0.1");
	if (err != 0) {
		printf("err: %d \n", err);
		exit(0);
	}
	send_message("hi");
	value = receive_int();

	printf("mes: %d \n", value);

	exit(0);
	*/

	int err = VRSubsystem::connectToVRServer("146.48.84.241");
	if ( err !=0)
	{
		std::cout << "connection failed with err \n" <<err << std::endl;
		exit(0);
	};
	int newCamera = VRSubsystem::addVirtualCamera();
	VirtualCamera::setCameraFrustrum(newCamera, -0.2, 0.2, -0.2, 0.2, 0.2,
		640, 480);
	VirtualCamera::setPosition(newCamera,2.0, -1.0, 2.0);
	VirtualCamera::setViewDirection(newCamera,0.0, 0.0, -1.0);
	VRSubsystem::renderFromCamera(newCamera);

	VRSubsystem::startStreaming();

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





//
///*
//Create a TCP socket
//*/
//
//#include<stdio.h>
//#include<winsock2.h>
//
//#pragma comment(lib,"ws2_32.lib") //Winsock Library
//
//int main(int argc, char *argv[])
//{
//	WSADATA wsa;
//	SOCKET s;
//	struct sockaddr_in server;
//	char *message, server_reply[2000];
//	int recv_size;
//
//	printf("\nInitialising Winsock...");
//	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
//	{
//		printf("Failed. Error Code : %d", WSAGetLastError());
//		return 1;
//	}
//
//	printf("Initialised.\n");
//
//	//Create a socket
//	if ((s = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
//	{
//		printf("Could not create socket : %d", WSAGetLastError());
//	}
//
//	printf("Socket created.\n");
//
//
//	server.sin_addr.s_addr = inet_addr("127.0.0.1");
//	server.sin_family = AF_INET;
//	server.sin_port = htons(81);
//
//	//Connect to remote server
//	if (connect(s, (struct sockaddr *)&server, sizeof(server)) < 0)
//	{
//		printf("socket failed with error: %ld\n", WSAGetLastError());
//		return 1;
//	}
//
//	puts("Connected");
//
//	//Send some data
//	message = "GET / HTTP/1.1\r\n\r\n";
//	if (send(s, message, strlen(message), 0) < 0)
//	{
//		puts("Send failed");
//		return 1;
//	}
//	puts("Data Send\n");
//
//	//Receive a reply from the server
//	if ((recv_size = recv(s, server_reply, 2000, 0)) == SOCKET_ERROR)
//	{
//		puts("recv failed");
//	}
//
//	puts("Reply received\n");
//
//	//Add a NULL terminating character to make it a proper string before printing
//	server_reply[recv_size] = '\0';
//	puts(server_reply);
//
//	return 0;
//}