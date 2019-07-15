/*
 * @Description: Traj Point
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-14 13:54:46
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-14 15:23:20
 */
#include "vo/common_headers.h"
using namespace Eigen; 


namespace vo
{
    class TrajPoint{
        public:
        typedef std::shared_ptr<TrajPoint> Ptr;
        double time_stamp_;
        Matrix3d rotationMatrix_;
        Quaterniond quaternion_;
        Vector3d translation_;        

        static unsigned long fractory_id_;


        TrajPoint(){}
        protected:
    };
}