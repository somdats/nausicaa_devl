#pragma once
#include <string>

#include<io.h>
#include<stdio.h>
#include<winsock2.h>
#include "..\header\server.h"

#pragma comment(lib,"ws2_32.lib") //Winsock Library


struct Server {
	WSADATA wsa;
	SOCKET s, new_socket;
	struct sockaddr_in server, client;
	int c;
	char* message;
	int port;

	int start_server(int );

	int accepting_connections();

	int incoming_message(std::string& message);

	int send(std::string message);

	int send(char* data, int length) {

	int stop_server();
};
