#pragma once

#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include <vector>
#include <utility> 
#include <GL/glew.h>

 

typedef  std::pair< vcg::Point3f, vcg::Point2f> Correspondence3D2D;
typedef std::pair< vcg::Point3f, vcg::Point3f> Correspondence3D3D;

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
