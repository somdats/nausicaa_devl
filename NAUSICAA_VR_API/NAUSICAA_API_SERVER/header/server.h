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
	char * message;
	char   blob_bin[10485760];
	int blob_bin_length;
	int port;
	bool stop_signal;


	int start_server(int );

	void stop_server();

	void close_socket();

	int accepting_connections();

	int receive(char * buf, int n);

	int incoming_message(std::string& message);

	int send(std::string message);

	int send(char* data, int length);

	int close();
};
