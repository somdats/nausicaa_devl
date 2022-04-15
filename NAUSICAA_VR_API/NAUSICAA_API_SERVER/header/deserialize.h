#pragma once
#include <string>

std::string func_name(std::string &message);
int deserialize_int(std::string& remain);
float deserialize_float(std::string& remain);
std::string deserialize_string(std::string& remain);
void call_API_function(std::string  message);