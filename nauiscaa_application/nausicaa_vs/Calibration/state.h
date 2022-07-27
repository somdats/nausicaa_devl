#pragma once


#include <string>
struct State {
	static std::string & filename() { static std::string fn; return fn; };

	static void set_filename(std::string);
	static void save_state();
	static void load_state();
};