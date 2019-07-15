/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-12 14:22:38
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 14:27:55
 */
#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_
#include "vo/common_headers.h"
#include "vo/map.h"
#include "vo/frame.h"
using namespace Eigen;

namespace vo
{
class VisualOdometry
{
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };
        
        VOState     state_;     // current VO status
        Frame::Ptr  ref_;       // reference frame 
        Frame::Ptr  curr_;      // current frame 
        Map::Ptr    map_;       // map with all frames and map points
        
        cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
        vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame 
        vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
        cv::Mat                 descriptors_curr_;  // descriptor in current frame 
        cv::Mat                 descriptors_ref_;   // descriptor in reference frame 
        vector<cv::DMatch>      feature_matches_;
        
        Sophus::SE3<double> T_c_w_estimated_;  // the estimated pose of current frame 
        int num_inliers_;        // number of inlier features in icp
        int num_lost_;           // number of lost times
        
        // parameters 
        int num_of_features_;   // number of features
        double scale_factor_;   // scale in image pyramid
        int level_pyramid_;     // number of pyramid levels
        float match_ratio_;      // ratio for selecting  good matches
        int max_num_lost_;      // max number of continuous lost times
        int min_inliers_;       // minimum inliers
        double map_point_erase_ratio_; //ratio for removing mappoint
        
        double key_frame_min_rot;   // minimal rotation of two key-frames
        double key_frame_min_trans; // minimal translation of two key-frames

        vector<cv::Point3d> pts_3d_; //3d ref points for pnp
        vector<cv::Point2d> pts_2d_; //2d keypoints for pnp
        vector<MapPoint::Ptr> mpts_matched_; //matched mappoints of current frame
        vector<int> kps_index_; //matched keypoints index of current frame

        cv::Mat rvec_, tvec_, inliers_; // solvePNP results;

        VisualOdometry();
        ~VisualOdometry();
    
        bool addFrame( Frame::Ptr frame );      // add a new frame 

         

    protected:
        void featureDetector();
        void computeDescriptor();
        void featureMatch();
        void poseEstimation();
        void poseOptimization();
        
        void addKeyFrame();
        void addMapPoints();
        bool checkEstimatedPose(); 
        bool checkKeyFrame();

        void optimizeMap();
        double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

};

}

#endif