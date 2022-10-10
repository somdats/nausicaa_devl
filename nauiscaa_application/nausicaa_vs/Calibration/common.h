#pragma once

#include "camera_reader.h"
#include <GL/glew.h>

extern std::vector<::Camera>  cameras;
struct LidarRender;
extern LidarRender lidars[2];

struct GLERR {
    GLERR() {
        int err = glGetError();
        if (err) {
            printf((const char*)gluErrorString(err));
            exit(0);
        }
    }
};
