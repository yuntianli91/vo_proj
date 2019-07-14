/*
 * @Description: Visual Odometry
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 11:18:42
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 09:30:31
 */
#include "vo/common_headers.h"
#include "vo/camera.h"
#include "vo/frame.h"
#include "vo/mappoint.h"
#include "vo/map.h"
#include "vo/config.h"
#include "vo/poseEstimator.h"

using namespace Eigen;

namespace vo
{
    // ==================================================================================================================
    // marrco flags
    // ==================================================================================================================
    // class declaration 
    class VisualOdometry
    {
        public:
            // --------------------------------------------- marco----------------------------------------------------- //
            typedef shared_ptr<VisualOdometry> Ptr;
            enum VOState{
                INITIALIZING = -1,
                OK = 0,
                LOST
            };
            
            // --------------------------------------------- variable members ------------------------------------------ //
            // internel variables
            VOState state_;
            Map::Ptr map_;
            Frame::Ptr ref_;
            Frame::Ptr curr_;
            PoseEstimator::Ptr poseEstimator_;
            
            vector<cv::KeyPoint> keypoints_curr_; 
            cv::Mat descriptor_curr_; 
            cv::BFMatcher matcher_bf_;
            cv::FlannBasedMatcher matcher_flann_;
            vector<MapPoint::Ptr> matched_pts_3d_; //matched 3d points in map
            vector<int> matched_index_; // matched points index;

            vector<string> rgb_files_, depth_files_;
            vector<double> rgb_times_, depth_times_;
            // parameters
            string dataset_dir_;
            double fx_, fy_, cx_, cy_, depth_scale_;
            Eigen::Matrix3d K_;
            double scale_factor_, match_ratio_, keyframe_rotation_, keyframe_translation_, map_point_erase_ratio_; 
            int number_of_features_, level_pyramid_, max_num_lost_, min_inliers_;

            int num_lost_, num_inliers_; // actual lost number and inlier number
            double keyframe_rot_, keyframe_trans_; // actuall keyframe rotation and translation


            // ORB feature detector
            cv::Ptr<cv::ORB> orb_ ;
            Sophus::SE3<double> T_c_w_estimated_;
            // --------------------------------------------- function members ------------------------------------------- // 
            // constructor and destructor
            VisualOdometry();
            ~VisualOdometry();
            // add frame to VO
            bool addFrame(Frame::Ptr frame);
            // "capture" frame from dataset or camera
            bool loadDataset();
            bool captureFrame();
            bool captureFrame(const string& path);
            bool captureFrame(const int& camera_index);
            //  pose estimation between frames
            void poseEstimation(const cv::Mat& img1, const cv::Mat& img2, Sophus::SE3<double>& pose);
        
        protected:
            // feature operations 
            void detectFeature();
            void computeDescriptor();
            void matchFeature();
            // photometric operations
            // map maintenance
            void addKeyFrame();
            void addMapPoint();
            void optimizeMap();
            // status check
            bool checkEstimatedPose();
            bool checkKeyFrame();
            double getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point );

};
}