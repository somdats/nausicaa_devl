#pragma once
#include <string>

int start_server();
int accepting_connections();
int accepting_connections_stream();
int stop_server();
int start_server_stream();
int stop_server_stream();
int wait_for_start(std::string& message);

int incoming_message(std::string &message);
int send(std::string message);
int send(char* image_buffer, int size);