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

	stop_signal = false;

	return 0;
}

void Server::stop_server() {
	stop_signal = true;
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
	/* a message has the form
	*  NUMBER@MESSAGE[BLOB] where:
	*  NUMBER  = string encoding the length of the text part of the message
	*  MESSAGE 
	*  BLOB    = binary data attached to the message
	*/
	char recvbuf[512];
	int recvbuflen = 512;
	int iResult = recv(new_socket, recvbuf, recvbuflen, 0);
	if (iResult > 0) {
		std::string header(recvbuf,10);
		int  size_header = header.find_first_of('@')+1;
		std::string res = header.substr(0, size_header-1);
		int size_string_part = std::stoi(res);
		message = std::string(recvbuf+ size_header, size_string_part);
		memcpy_s(this->blob_bin, iResult - size_header - size_string_part, recvbuf + size_header + size_string_part, iResult - size_header - size_string_part);
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

int Server::close() {
	closesocket(s);
	WSACleanup();
	return 0;
}