/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntianli91
 * @Date: 2019-07-07 09:02:07
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 16:25:29
 */
#ifndef CAMERA_H_
#define CAMERA_H_
#include "vo/common_headers.h"
#include "vo/config.h"

using namespace Eigen;
using namespace Sophus;
namespace vo{

class Camera{
    public:
    // in case of unaligned pointer
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;
    // camera intrinsics
    float f_x_, f_y_, c_x_, c_y_;
    float depth_scale_;
    // constructor
    Camera(){
        f_x_ = Config::getParam<float>("camera.fx");
        f_y_ = Config::getParam<float>("camera.fy");
        c_x_ = Config::getParam<float>("camera.cx");
        c_y_ = Config::getParam<float>("camera.cy");
        depth_scale_ = Config::getParam<float>("camera.depth_scale");
    };
    Camera(float f_x, float f_y, float c_x, float c_y, float depth_scale):
        f_x_(f_x), f_y_(f_y), c_x_(c_x), c_y_(c_y), depth_scale_(depth_scale){}
    // Transformation matrix

    Vector3d world2camera(const Vector3d& p_w, const SE3<double>& T_c_w);
    Vector3d camera2world(const Vector3d& p_c, const SE3<double>& T_c_w);

    Vector2d camera2pixel(const Vector3d& p_c);
    Vector3d pixel2camera(const Vector2d& p_p, double depth=1);

    Vector2d world2pixel(const Vector3d& p_w, const SE3<double>& T_c_w);
    Vector3d pixel2world(const Vector2d& p_p, const SE3<double>& T_C_W, double depth=1);
};
}
#endif 