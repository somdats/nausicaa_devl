

#include <cstdarg>
#include <vector>

#include "..\header\serialize.h"

std::string serialize(float v) {
	return std::to_string(v);
}
std::string serialize(int v) {
	return std::to_string(v);
}
std::string serialize(std::string v) {
	return v;
}

