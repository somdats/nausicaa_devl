#include "client.h"
/*
Create a TCP socket
*/

#include<stdio.h>
#include<winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

WSADATA wsa;
SOCKET s;
struct sockaddr_in server;
char* message, server_reply[2000];
int recv_size;

int connect(std::string addr) {

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


	server.sin_addr.s_addr = inet_addr(addr.c_str());
	server.sin_family = AF_INET;
	server.sin_port = htons(81);

	//Connect to remote server
	if (connect(s, (struct sockaddr*)&server, sizeof(server)) < 0)
	{
		printf("socket failed with error: %ld\n", WSAGetLastError());
		return 1;
	}

	puts("Connected");

}

int  send_message(std::string message) {

	//Send some data
	//message = "GET / HTTP/1.1\r\n\r\n";
	if (send(s, message.c_str(), message.length(), 0) < 0)
	{
		puts("Send failed");
		return 1;
	}
	puts("Data Send\n");

	//Receive a reply from the server
//	if ((recv_size = recv(s, server_reply, 2000, 0)) == SOCKET_ERROR)
//	{
//		puts("recv failed");
//	}

//	puts("Reply received\n");

	//Add a NULL terminating character to make it a proper string before printing
//	server_reply[recv_size] = '\0';
//	puts(server_reply);
}
int receive_int() {
	if ((recv_size = recv(s, server_reply, 2000, 0)) == SOCKET_ERROR)
	{
		puts("recv failed");
	}
	server_reply[recv_size] = '\0';
	return std::stoi(std::string(server_reply));
}

int disconnect();
int receive(std::string& message);

//----------------------------------------------------

WSADATA wsaS;
SOCKET sS;
struct sockaddr_in serverS;
char* messageS, server_replyS[3000000];
int recv_sizeS;

int connect_stream(std::string addr) {

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

	int portS = 82;
	serverS.sin_addr.s_addr = inet_addr(addr.c_str());
	serverS.sin_family = AF_INET;
	serverS.sin_port = htons(portS);

	//Connect to remote server
	if (connect(sS, (struct sockaddr*)&serverS, sizeof(serverS)) < 0)
	{
		printf("socket failed with error: %ld\n", WSAGetLastError());
		return 1;
	}

	puts("Connected");

}

int  start_stream() {

	//Send some data
	//message = "GET / HTTP/1.1\r\n\r\n";
	if (send(s, "start", 5, 0) < 0)
	{
		puts("Send failed");
		return 1;
	}
	puts("Data Send\n");

	//Receive a reply from the server
//	if ((recv_size = recv(s, server_reply, 2000, 0)) == SOCKET_ERROR)
//	{
//		puts("recv failed");
//	}

//	puts("Reply received\n");

	//Add a NULL terminating character to make it a proper string before printing
//	server_reply[recv_size] = '\0';
//	puts(server_reply);
}

char* receive_image(int* byteCount) {
	if ((recv_sizeS = recv(sS, server_replyS, 3000000, 0)) == SOCKET_ERROR)
	{
		puts("recv failed");
	}
	server_replyS[recv_sizeS] = '\0';
	*byteCount = recv_sizeS;
	printf("size of image byte: %d", *byteCount);
	return server_replyS;
}



