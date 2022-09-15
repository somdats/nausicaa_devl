#pragma once

#include <string>
#include <GL/glew.h>
#include <vcg/space/point3.h>

struct Marker {
	std::string label;
	vcg::Point3f pos;	// position in 3D space (WGS84)
	int tc[2];			// texture coordinates (in texels)
	unsigned char* png_data; //
	int png_data_length;
	bool visible;	
	float width;
};
