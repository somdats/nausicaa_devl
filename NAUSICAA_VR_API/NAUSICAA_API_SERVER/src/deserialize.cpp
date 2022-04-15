

#include "..\header\deserialize.h"

std::string func_name(std::string &  message) {
	int  sz = message.find_first_of('@');
	std::string res =   message.substr(0, sz);
	message.erase(0, sz + 1);
	return res;
}


int deserialize_int(std::string & remain) {
	int  sz = remain.find_first_of('@');
	std::string res = remain.substr(0, sz);
	remain.erase(0, sz + 1);
	return std::stoi(res);
}


float deserialize_float(std::string& remain) {
	int  sz = remain.find_first_of('@');
	std::string res = remain.substr(0, sz);
	remain.erase(0, sz + 1);
	return std::stof(res);
}

std::string deserialize_string(std::string& remain) {
	int  sz = remain.find_first_of('@');
	std::string res = remain.substr(0, sz);
	remain.erase(0, sz + 1);
	return res;
}