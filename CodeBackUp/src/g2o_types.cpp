/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-11 15:14:03
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-11 15:27:49
 */
#include "vo/g2o_types.h"

namespace vo
{
    void EdgeProjectXYZ2UBPoseOnly::computeError(){
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        _error = _measurement - camera_->camera2pixel ( pose->estimate().map(pt_) );
    }   

    void EdgeProjectXYZ2UBPoseOnly::linearizeOplus(){
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Vector3d xyz_trans = T.map(pt_);
       double x = xyz_trans[0];
       double y = xyz_trans[1];
       double z = xyz_trans[2];
       double z_2 = z * z;

        _jacobianOplusXi ( 0,0 ) = x * y / z_2 * camera_->f_x_;
        _jacobianOplusXi ( 0,1 ) = -(1 + (x * x / z_2)) * camera_->f_x_;
        _jacobianOplusXi ( 0,2 ) = y/z * camera_->f_x_;
        _jacobianOplusXi ( 0,3 ) = -1./z * camera_->f_x_;
        _jacobianOplusXi ( 0,4 ) = 0;
        _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->f_x_;

        _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->f_y_;
        _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->f_y_;
        _jacobianOplusXi ( 1,2 ) = -x/z *camera_->f_y_;
        _jacobianOplusXi ( 1,3 ) = 0;
        _jacobianOplusXi ( 1,4 ) = -1./z *camera_->f_y_;
        _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->f_y_; 

    } 
}