#pragma once

#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include <vector>
#include <utility> 
#include <GL/glew.h>

 

struct  Correspondence3D2D : public   std::pair< vcg::Point3f, vcg::Point2f> {
    Correspondence3D2D( ) {  }
    Correspondence3D2D(const Correspondence3D2D& c) { (*this) = c; }
    Correspondence3D2D(std::pair < vcg::Point3f  , vcg::Point2f > p32 , int il) {
        (*this).first  = p32.first;
        (*this).second = p32.second;
     iLidar = il;
    }
    int iLidar;
};

typedef std::pair< vcg::Point3f, vcg::Point3f> Correspondence3D3D;

struct LidarRender;
extern LidarRender lidars[2];

struct GLERR {
    GLERR(int line, const char * file) {
        int err = glGetError();
        if (err) {
            printf("%s at line %d in file %s\n",(const char*)gluErrorString(err),line,file);
            exit(0);
        }
    }
    GLERR() {
        GLERR(-1, ".");
    }
};
