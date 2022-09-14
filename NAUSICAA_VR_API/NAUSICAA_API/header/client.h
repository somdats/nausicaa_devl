#pragma once
#include <string>
#include<winsock2.h>

struct Client{
WSADATA wsa;
SOCKET s;
struct sockaddr_in server;
char  message [10485760], server_reply[3000000];
int recv_size;
int port;

int connect(std::string addr,int port);

int send_message(std::string message, char * blob = NULL,  int blob_size = 0);

int receive_int();

int disconnect();
int receive(std::string& message);

char*  receive_image(int* byteCount);

};