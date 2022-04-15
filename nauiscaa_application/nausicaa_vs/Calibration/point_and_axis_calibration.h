#ifndef POINT_AND_AXIS_CALIBRATION_H
#define POINT_AND_AXIS_CALIBRATION_H

#include <vcg/math/matrix44.h>
#include <vcg/math/shot.h>

struct CameraMatch{
    vcg::Point3f o;     // projection vector of the origin
    vcg::Point2f o_im;  // o in image space
    vcg::Point2f axis_im[3];  // o in image space
    vcg::Point2f xax;  // 1,0,0, in image space

    vcg::Camera<float> camera;
};

vcg::Shotf find_extrinsics(CameraMatch _cm);

#endif // POINT_AND_AXIS_CALIBRATION_H
