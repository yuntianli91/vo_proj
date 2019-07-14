/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-14 15:28:56
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-14 15:31:16
 */
#include "vo/common_headers.h"
using namespace Eigen;

namespace vo
{
    class Traj
    {
        public:
        struct TrajPoint
        {
            double time_stamp_;
            Vector3d translation_;
            Quaterniond quaternion_;
        };

        vector<TrajPoint> traj_pts;
        
    };
}