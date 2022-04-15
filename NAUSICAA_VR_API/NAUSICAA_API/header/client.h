#pragma once
#include <string>

int connect(std::string addr);
int  disconnect();
int  send_message(std::string message);
int  receive(std::string & message);
int receive_int();

int connect_stream(std::string addr);
int  start_stream();
char *   receive_image();

