#pragma once


#include <string>
struct State {
	std::string filename;

	void set_filename(std::string);
	void save_state();
	void load_state();
};