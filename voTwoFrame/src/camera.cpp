/*
 * @Description: camera class
 * @Author: Yuntian Li
 * @Github: https://github.com/OBKoro1
 * @Date: 2019-07-07 10:31:27
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-07 10:48:14
 */
#include "vo/camera.h"

namespace vo{
    
    Vector3d Camera::world2camera(const Vector3d& p_w, const SE3<double>& T_c_w){
        return T_c_w * p_w;
    }
 
    Vector3d Camera::camera2world(const Vector3d& p_c, const SE3<double>& T_c_w){
        return T_c_w.inverse() * p_c;
    }

    Vector2d Camera::camera2pixel(const Vector3d& p_c){
        return Vector2d(
            f_x_ * p_c(0, 0) / p_c(2, 0) + c_x_,
            f_y_ * p_c(1, 0) / p_c(2, 0) + c_y_
        );
    }

    Vector3d Camera::pixel2camera(const Vector2d& p_p, double depth){
        return Vector3d(
            (p_p(0, 0) - c_x_) * depth / f_x_,
            (p_p(1, 0) - c_y_) * depth / f_y_,
            depth
        );
    }

    Vector2d Camera::world2pixel(const Vector3d& p_w, const SE3<double>& T_c_w){
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Vector3d Camera::pixel2world(const Vector2d& p_p, const SE3<double>& T_c_w, double depth){
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }

}
