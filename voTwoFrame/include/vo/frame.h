/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-12 14:09:38
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 14:10:29
 */
#include "vo/camera.h"
#ifndef FRAME_H_
#define FRAME_H_
#include "vo/common_headers.h"
#include "vo/camera.h"

using namespace Eigen;
using namespace Sophus;

namespace vo{
    class Frame{
                // public function members
        public:
        // public data members
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_; // frame id
        bool kf_tag_; //tag of keyframe
        double time_stamp_; // frame time stamp
        Camera::Ptr camera_;
        SE3<double> T_c_w_; //frame pose matrix
        cv::Mat color_, depth_; // color and depth image
        
        public:
        Frame();
        Frame(unsigned long id, bool ky_tag=false, double time_stamp=0, SE3<double> T_c_w=SE3<double>(), 
            Camera::Ptr camera=nullptr, cv::Mat color=cv::Mat(), cv::Mat depth=cv::Mat());        
        // factory function
        static Frame::Ptr createFrame();
        // find depth
        double findDepth(const cv::KeyPoint& kp);
        // use const tag to declare read-only function
        Vector3d getCenter() const;
        bool isInFrame(const Vector3d& pt_w);
 
    };
}


#endif