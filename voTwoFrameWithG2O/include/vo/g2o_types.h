/*
 * @Description: custom g2o type
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-14 15:43:53
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-14 15:47:50
 */
#include "vo/common_headers.h"
#ifndef G2O_TYPES_H
#define G2O_TYPES_H
#include "vo/common_headers.h"
#include "vo/camera.h"
using namespace Eigen;


namespace vo{
    class EdgeProjectXYZ2UBPoseOnly:public g2o::BaseUnaryEdge<2, Vector2d, g2o::VertexSE3Expmap> 
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read(std::istream& in){}
        virtual bool write(std::ostream& out) const{}

        Vector3d pt_;
        Camera::Ptr camera_;

    };
}

#endif
