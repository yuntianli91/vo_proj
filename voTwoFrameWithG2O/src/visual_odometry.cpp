/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-12 14:29:08
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-14 19:07:38
 */
#include "vo/config.h"
#include "vo/g2o_types.h"
#include "vo/visual_odometry.h"

namespace vo
{
    VisualOdometry::VisualOdometry():
    state_(INITIALIZING), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_inliers_ ( 0 ), num_lost_( 0 )
    {
        
        num_of_features_    = Config::getParam<int> ( "number_of_features" );
        scale_factor_       = Config::getParam<double> ( "scale_factor" );
        level_pyramid_      = Config::getParam<int> ( "level_pyramid" );
        match_ratio_        = Config::getParam<float> ( "match_ratio" );
        max_num_lost_       = Config::getParam<float> ( "max_num_lost" );
        min_inliers_        = Config::getParam<int> ( "min_inliers" );
        key_frame_min_rot   = Config::getParam<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::getParam<double> ( "keyframe_translation" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry(){};

    bool VisualOdometry::addFrame(Frame::Ptr frame){
        switch (state_)
        {
            case INITIALIZING:
                // reference frame and current frame
                ref_ = curr_ = frame;
                ref_->T_c_w_ = Sophus::SE3d(
                    Eigen::Quaterniond(-0.3909, 0.8851, 0.2362, -0.0898),
                    Eigen::Vector3d(1.3112, 0.8507, 1.5186)
                );
                cout << "Initial translation is: " << ref_->T_c_w_.translation() << endl;

                map_->insertKeyFrame(frame);
                featureDetector();
                computeDescriptor();
                setRef3DPoints();
                // addKeyFrame();
                state_ = OK;
                break;
            case OK:
                // update current frame
                // reference frame unchanged
                curr_ = frame;
                featureDetector();
                computeDescriptor();
                featureMatch();
                poseEstimation();
                // TODO: Check pose
                if ( checkEstimatedPose() == true ) // a good estimation
                {
                    curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
                    ref_ = curr_;
                    setRef3DPoints();
                    num_lost_ = 0;
                    if ( checkKeyFrame() == true ) // is a key-frame
                    {
                        addKeyFrame();
                    }
                }
                else // bad estimation due to various reasons
                {
                    num_lost_++;
                    if ( num_lost_ > max_num_lost_ )
                    {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            case LOST:
                cout << "VO has lost track" << endl;
                break;
        }
        
    }

    void VisualOdometry::featureDetector(){
        orb_->detect(curr_->color_, keypoints_curr_);
    }

    void VisualOdometry::computeDescriptor(){
        orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
    }

    void VisualOdometry::featureMatch(){
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(descriptors_ref_,descriptors_curr_, matches);
        // find minimum Hamming distance
        float min_dist = std::min_element(matches.begin(), matches.end(),
                                    [](const cv::DMatch& m1, const cv::DMatch& m2){
                                        return m1.distance < m2.distance;
                                    })->distance;
        // select matchea
        feature_matches_.clear();
        for (cv::DMatch m:matches){
            if (m.distance < max<float>(match_ratio_ * min_dist, 30.0)){
                feature_matches_.push_back(m);
            }
        }

        cout << "good matches:" << feature_matches_.size() << endl;
    }

    void VisualOdometry::poseEstimation(){
        pts_3d_.clear();pts_2d_.clear();
        // construct 3d reference points and 2d feature points
        for (cv::DMatch m:feature_matches_){
            pts_3d_.push_back(pts_3d_ref_[m.queryIdx]);
            pts_2d_.push_back(keypoints_curr_[m.trainIdx].pt);
        }
        // construct camera instrict
        cv::Mat K = (cv::Mat_<double>(3, 3)<<
            ref_->camera_->f_x_, 0.0, ref_->camera_->c_x_,
            0.0, ref_->camera_->f_y_, ref_->camera_->c_y_,
            0.0, 0.0, 1.0 );
        
        cv::solvePnPRansac(pts_3d_, pts_2d_, K, cv::Mat(), rvec_, tvec_, false, 100, 4.0, 0.99, inliers_);
        cv::Mat R0;
        cv::Rodrigues(rvec_, R0);

        Eigen::Matrix3d Rotation_Matrix;
        Rotation_Matrix << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2),
                R0.at<double>(1, 0), R0.at<double>(1, 1), R0.at<double>(1, 2),
                R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);
        
        num_inliers_ = inliers_.rows;
        cout << "pnp inliers: " << num_inliers_ << endl;
        T_c_r_estimated_ = SE3<double>(
            SO3<double>(Rotation_Matrix), 
            Vector3d( tvec_.at<double>(0,0), tvec_.at<double>(1,0), tvec_.at<double>(2,0))
         );
         cout << "SE3 by PNPRansic is: " << endl << T_c_r_estimated_.matrix() << endl;

         poseOptimization();

         
    }

    void VisualOdometry::poseOptimization(){
         typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
        Block::LinearSolverType* solver_type = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(solver_type));
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(g2o::SE3Quat(
            T_c_r_estimated_.rotationMatrix(), T_c_r_estimated_.translation()
        ));
        optimizer.addVertex(pose);

        for (int  i=0; i < inliers_.rows; i++){
            int index = inliers_.at<int>(i, 0);

            vo::EdgeProjectXYZ2UBPoseOnly* edge = new vo::EdgeProjectXYZ2UBPoseOnly();
            edge->setId(i); //TODO: i or i+1 ?
            edge->setVertex(0, pose);
            edge->camera_ = curr_->camera_;
            edge->pt_ = Vector3d(pts_3d_[index].x, pts_3d_[index].y, pts_3d_[index].z);
            edge->setMeasurement(Vector2d(pts_2d_[index].x, pts_2d_[index].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        T_c_r_estimated_ = SE3<double>(
            pose->estimate().rotation(),
            pose->estimate().translation()
        );
        cout << "T_c_w_estimated after optimization is: " << endl << T_c_r_estimated_.matrix() << endl;
    }
    
    void VisualOdometry::setRef3DPoints(){
        // clear all 3d point of last reference frame
        pts_3d_ref_.clear();
        descriptors_ref_ = cv::Mat();
        for (int i = 0; i < descriptors_curr_.rows; i++){
            double d = ref_->findDepth(keypoints_curr_[i]);
            // only use points with positive depth ad refereence points 
            if (d > 0){
                Vector2d pt_uv(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y);
                // Vector3d pt_cam(ref_->camera_->pixel2camera(
                    // Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y)), d);
                Vector3d pt_cam;
                pt_cam = ref_->camera_->pixel2camera(pt_uv, d);
                pts_3d_ref_.push_back(cv::Point3f(pt_cam(0, 0), pt_cam(1, 0), pt_cam(2, 0)));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }
        
    void VisualOdometry::addKeyFrame(){
        cout<<"adding a key-frame"<<endl;
        map_->insertKeyFrame ( curr_ );
    }
  
    bool VisualOdometry::checkEstimatedPose(){
        // check if the estimated pose is good
        if ( num_inliers_ < min_inliers_ )
        {
            cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    } 
    
    
    bool VisualOdometry::checkKeyFrame(){
        Sophus::Vector6d d = T_c_r_estimated_.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

}