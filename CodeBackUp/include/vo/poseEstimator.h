/*
 * @Description: pose estimator
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-09 16:31:59
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 10:14:24
 */
#include "vo/common_headers.h"
#include "vo/frame.h"
#include "vo/g2o_types.h"
#include "vo/mappoint.h"

#ifndef POSE_ESTIMATOR_H_
#define POSE_ESTIMATOR_H_

using namespace Eigen;

namespace vo
{
    class PoseEstimator
    {
        public:
        typedef shared_ptr<PoseEstimator> Ptr;
        Frame::Ptr curr_;
        Sophus::SE3<double> T_c_w_estimated_;
        cv::Mat K_; 
        vector<cv::Point3d> pts_3d_;
        vector<cv::Point2d> pts_2d_;
        cv::Mat inliers_;
        int num_inliers_;

        vector<MapPoint::Ptr> matched_pts_3d_; //matched 3d points in map
        vector<int> matched_index_; // matched points index;
        vector<cv::KeyPoint> keypoints_curr_;
        
        PoseEstimator(){};
        // PoseEstimator()

        void poseEstimatorFeature();
        void poseEstimatorPhotometric(); 

        protected:
           // PnP variables
        cv::Mat rvec0_, R0_, tvec0_;
        
        

        void initiatePNP();
    };
}

#endif