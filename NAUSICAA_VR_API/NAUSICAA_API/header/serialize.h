#pragma once

#include <string>
#include <cstdarg>

std::string serialize(float v);
std::string serialize(int v);
std::string serialize(std::string v);

struct message {

	message &operator [](float v) { msg += serialize(v) + "@"; return *this; }
	message &operator [](int v) { msg += serialize(v) + "@"; return *this; }
	message &operator [](std::string v) { msg += serialize(v) + "@"; return *this; }

	std::string msg;
};
