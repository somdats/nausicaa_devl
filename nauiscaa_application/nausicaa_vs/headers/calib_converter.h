#pragma once
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <opencv2/core.hpp>



namespace calib_converter
{

double convertOcam2Mei(const std::vector<double>& poly_inv,
    const Eigen::Vector2d& principal_point,
    const Eigen::Vector2d & img_size,double c, double d, double e,
    Eigen::Matrix3f & K_out,
    std::array<float, 5> & D_out,
    float& xi_out);

Eigen::Vector2d WorldToPlane(const Eigen::Vector3d& P, const std::vector<double>& inv_poly,
    const Eigen::Vector2d& pp, double c, double d, double e);
}

