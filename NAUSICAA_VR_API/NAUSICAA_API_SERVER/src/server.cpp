#include<io.h>
#include<stdio.h>
#include<winsock2.h>
#include "..\header\server.h"

#pragma comment(lib,"ws2_32.lib") //Winsock Library


int Server::start_server(int _port) {
	port = _port;
	printf("\nInitialising Winsock...");
	int err;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		err = WSAGetLastError();
		printf("Failed. Error Code : %d", err);
		return err;
	}

	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
	{
		err = WSAGetLastError();
		printf("Could not create socket : %d", err);
		return err;
	}

	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(port);

	//Bind
	if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		err = WSAGetLastError();
		printf("Bind failed with error code : %d",err);
		return err;
	}

	puts("Bind done");

	//Listen to incoming connections
	listen(s, 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");

	c = sizeof(struct sockaddr_in);

	//	if ((new_socket = accept(s, (struct sockaddr *)&client, &c)) != INVALID_SOCKET)
	//		puts("Connection accepted");
	return 0;
}


int Server::accepting_connections() {
if ((new_socket = accept(s, (struct sockaddr*)&client, &c)) != INVALID_SOCKET)
{
	
	printf("Connection accepted from %s on port %d\n", inet_ntoa(client.sin_addr), port);
	return 1;
}
else
	return 0;
}


int Server::incoming_message(std::string& message) {
	char recvbuf[512];
	int recvbuflen = 512;
	int iResult = recv(new_socket, recvbuf, recvbuflen, 0);
	if (iResult > 0) {
		printf("Bytes received: %d\n", iResult);
		recvbuf[iResult] = '\0';
		message = std::string(recvbuf);
		//		send(new_socket, "received", strlen("received"), 0);
		return 0;
	}
	else {
		message = std::string();
		return 1;
	}

}

int Server::send(std::string message) {
	::send(new_socket, message.c_str(), message.size(), 0);
	return 0;
}

int Server::send(char *data, int length) {
	::send(new_socket, data, length, 0);
	return 0;
}

int Server::stop_server() {
	closesocket(s);
	WSACleanup();
	return 0;
}
 
