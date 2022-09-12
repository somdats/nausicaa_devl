#include "..\header\client.h"
/*
Create a TCP socket
*/

#include<stdio.h>
#include<winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library



int Client::connect(std::string addr, int _port) {
	port = _port;
	int err = 0;

	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		err = WSAGetLastError();
		printf("WSAStartup Failed. Error Code : %d",err);
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


	server.sin_addr.s_addr = inet_addr(addr.c_str());
	server.sin_family = AF_INET;
	server.sin_port = htons(port);

	//Connect to remote server
	if (::connect(s, (struct sockaddr*)&server, sizeof(server)) < 0)
	{
		err = WSAGetLastError();
		printf("socket failed with error: %ld\n",err);
		return err;
	}

	puts("Connected");
	return 0;

}


int   Client::send_message(std::string msg, char * blob, int blob_size  ) {

	*((int*)message) = msg.length();
	memcpy_s(message+4, msg.length(), msg.c_str(), msg.length());
	memcpy_s(message+4+ msg.length(), blob_size, blob, blob_size);

	if (send(s, message ,4+ msg.length()+ blob_size, 0) < 0)
	{
		puts("Send failed");
		return 1;
	}
	puts("Data Send\n");
}

int  Client::receive_int() {
	if ((recv_size = recv(s, server_reply, 2000, 0)) == SOCKET_ERROR)
	{
		puts("recv failed");
	}
	server_reply[recv_size] = '\0';
	return std::stoi(std::string(server_reply));
}



char* Client::receive_image(int* byteCount) {
	if ((recv_size = recv(s, server_reply, 3000000, 0)) == SOCKET_ERROR)
	{
		puts("recv failed");
		return NULL;
	}
	server_reply[recv_size] = '\0';
	*byteCount = recv_size;
	return server_reply;
}



