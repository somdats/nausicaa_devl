#include<io.h>
#include<stdio.h>
#include<winsock2.h>
#include "..\header\server.h"

#pragma comment(lib,"ws2_32.lib") //Winsock Library

WSADATA wsa;
SOCKET s, new_socket;
struct sockaddr_in server, client;
int c;
char *message;


int start_server() {
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		return 1;
	}

	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}

	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(81);

	//Bind
	if (bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	puts("Bind done");

	//Listen to incoming connections
	listen(s, 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");

	c = sizeof(struct sockaddr_in);

//	if ((new_socket = accept(s, (struct sockaddr *)&client, &c)) != INVALID_SOCKET)
//		puts("Connection accepted");
}

int accepting_connections() {
	if ((new_socket = accept(s, (struct sockaddr *)&client, &c)) != INVALID_SOCKET)
	{
		puts("Connection accepted");
		return 1;
	}
	else
		return 0;
}

int incoming_message(std::string &message) {
	char recvbuf[512];
	int recvbuflen = 512;
	int iResult = recv(new_socket, recvbuf, recvbuflen, 0);
	if (iResult > 0) {
		printf("Bytes received: %d\n", iResult);
		recvbuf[iResult] = '\0';
		message = std::string(recvbuf);
//		send(new_socket, "received", strlen("received"), 0);
		return 1;
	}
	else {
		message = std::string();
		return 0;
	}

}

int send(std::string message) {
	send(new_socket, message.c_str(), message.size(), 0);
	return 0;
}


int stop_server() {
	closesocket(s);
	WSACleanup();
	return 0;
}


// -----------------------------------------

WSADATA wsaS;
SOCKET sS, new_socketS;
struct sockaddr_in serverS, clientS;
int cS;
char* messageS;


int start_server_stream() {
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsaS) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		return 1;
	}

	printf("Initialised.\n");

	//Create a socket
	if ((sS = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}

	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	serverS.sin_family = AF_INET;
	serverS.sin_addr.s_addr = INADDR_ANY;
	serverS.sin_port = htons(82);

	//Bind
	if (bind(sS, (struct sockaddr*)&serverS, sizeof(serverS)) == SOCKET_ERROR)
	{
		printf("Bind streamer failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	puts("Bind streamer done");

	//Listen to incoming connections
	listen(sS, 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");

	cS = sizeof(struct sockaddr_in);

//	if ((new_socketS = accept(sS, (struct sockaddr*)&clientS, &cS)) != INVALID_SOCKET)
//		puts("Connection accepted");
}

int accepting_connections_stream() {
	if ((new_socketS = accept(sS, (struct sockaddr *)&clientS, &cS)) != INVALID_SOCKET)
	{
		puts("Connection accepted");
		return 1;
	}
	else
		return 0;
}

int wait_for_start(std::string& message) {
	char recvbuf[512];
	int recvbuflen = 512;
	int iResult = recv(new_socketS, recvbuf, recvbuflen, 0);
	if (iResult > 0) {
		printf("Bytes received: %d\n", iResult);
		recvbuf[iResult] = '\0';
		message = std::string(recvbuf);
		//		send(new_socketS, "received", strlen("received"), 0);
		return 1;
	}
	else
		return 0;
}

int send(char *  image_buffer, int size) {
	send(new_socketS, image_buffer, size, 0);
	return 1;
}


int stop_server_stream() {
	closesocket(sS);
	WSACleanup();
	return 0;
}
