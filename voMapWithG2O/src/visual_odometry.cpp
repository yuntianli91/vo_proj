/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-12 14:29:08
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 15:08:54
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
        map_point_erase_ratio_ = Config::getParam<double> ("map_point_erase_ratio");

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

                featureDetector();
                computeDescriptor();
                addKeyFrame();
                state_ = OK;
                break;
            case OK:
                // update current frame
                // reference frame unchanged
                curr_ = frame;
                curr_->T_c_w_ = ref_->T_c_w_;
                featureDetector();
                computeDescriptor();
                featureMatch();
                poseEstimation();
                // TODO: Check pose
                if ( checkEstimatedPose() == true ) // a good estimation
                {
                    curr_->T_c_w_ = T_c_w_estimated_;
                    num_lost_ = 0;
                    optimizeMap();
                    if ( checkKeyFrame() == true ) // is a key-frame
                    {
                        addKeyFrame();
                    }
                }
                else // bad estimation due to various reasons
                {
                    addMapPoints();
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
        return true;
    }

    void VisualOdometry::featureDetector(){
        orb_->detect(curr_->color_, keypoints_curr_);
    }

    void VisualOdometry::computeDescriptor(){
        orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
    }

    void VisualOdometry::featureMatch(){
        // selected mappoint
        vector<MapPoint::Ptr> mpts_visible; //visible mappoints
        cv::Mat mpts_descripors; // descriptors of visible mappoints

        for (auto& mpt_pair:map_->map_points_){
            MapPoint::Ptr mpt = mpt_pair.second; 
            // check whether mappoint is visible
            if(curr_->isInFrame(mpt->pos_)){
                mpt->visible_times_++;
                mpts_visible.push_back(mpt);
                mpts_descripors.push_back(mpt->descriptor_);
            }
        }

        vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(mpts_descripors, descriptors_curr_, matches);
        // find minimum Hamming distance
        float min_dist = std::min_element(matches.begin(), matches.end(),
                                    [](const cv::DMatch& m1, const cv::DMatch& m2){
                                        return m1.distance < m2.distance;
                                    })->distance;
        // select good matched mappoint and ketpoints
        mpts_matched_.clear();
        kps_index_.clear();
        for (cv::DMatch m:matches){
            if (m.distance < max<float>(match_ratio_ * min_dist, 30.0)){
                mpts_matched_.push_back(mpts_visible[m.queryIdx]);
                kps_index_.push_back(m.trainIdx);
            }
        }

        cout << "good matches:" << mpts_matched_.size() << endl;
    }

    void VisualOdometry::poseEstimation(){
        pts_3d_.clear();pts_2d_.clear();
        // construct 3d reference points and 2d feature points
        for (MapPoint::Ptr mpt:mpts_matched_){
            pts_3d_.push_back(mpt->getPoseCV());
        }
        // TODO: 2d constructor
        for (int index:kps_index_){
            pts_2d_.push_back(keypoints_curr_[index].pt);
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
        T_c_w_estimated_ = SE3<double>(
            SO3<double>(Rotation_Matrix), 
            Vector3d( tvec_.at<double>(0,0), tvec_.at<double>(1,0), tvec_.at<double>(2,0))
         );
         cout << "SE3 by PNPRansic is: " << endl << T_c_w_estimated_.matrix() << endl;

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
            T_c_w_estimated_.rotationMatrix(), T_c_w_estimated_.translation()
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
            mpts_matched_[index]->matched_times_++;
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        T_c_w_estimated_ = SE3<double>(
            pose->estimate().rotation(),
            pose->estimate().translation()
        );
        cout << "T_c_w_estimated after optimization is: " << endl << T_c_w_estimated_.matrix() << endl;
    }
        
    void VisualOdometry::addKeyFrame(){
        if(map_->key_frames_.empty()){
            cout << "Adding all points of first frame to the map." << endl;
            for (size_t i=0; i< keypoints_curr_.size(); i++){
                double d = curr_->findDepth(keypoints_curr_[i]);
                if (d < 0){continue;}
                Vector3d pt_w = curr_->camera_->pixel2world(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                                                        curr_->T_c_w_, d);
                // direction vector of mappoint                
                Vector3d n = pt_w - ref_->getCenter();
                n.normalize();

                MapPoint::Ptr mpt = MapPoint::createMapPoint(pt_w, n, curr_, descriptors_curr_.row(i).clone());
                map_->insertMapPoint(mpt);
            }   
        }
        cout<<"adding a key-frame"<<endl;
        
        map_->insertKeyFrame ( curr_ );
        ref_ = curr_;
    }

    void VisualOdometry::addMapPoints(){
        // generate flags for mpts to indicate whether it's already in the map
        vector<bool> mpts_added(keypoints_curr_.size(), false);
        for (int index:kps_index_){
            mpts_added[index] = true;
        }

        for (size_t i=0; i < keypoints_curr_.size(); i++){
            if (mpts_added[i] == true){
                continue;
            }

            double d = curr_->findDepth(keypoints_curr_[i]);// ref_ or curr_ ?
            if (d < 0){continue;}
            Vector3d pt_w = curr_->camera_->pixel2world(
                        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                        curr_->T_c_w_, d);
            
            Vector3d n = pt_w - ref_->getCenter();
            n.normalize();

            MapPoint::Ptr mpt = MapPoint::createMapPoint(pt_w, n, curr_, descriptors_curr_.row(i).clone());
            map_->insertMapPoint(mpt);
        }

    }

    void VisualOdometry::optimizeMap(){
        for (auto iter = map_->map_points_.begin();iter != map_->map_points_.end();){
            // remove mappoint out of sight
            if (!curr_->isInFrame(iter->second->pos_)){
                iter = map_->map_points_.erase(iter); // erase element in unordered map and return next iterator
                continue;
            }
            // remove mappoint with bad match
            float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
            if(match_ratio < map_point_erase_ratio_){
                iter = map_->map_points_.erase(iter);
                continue;
            }
            // remove mappoint out of FOV
            double angle = getViewAngle( curr_, iter->second );
            if ( angle > M_PI/6. )
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            iter++;
        }

        // add mappoints if too little
        if ( mpts_matched_.size()<100 )
            addMapPoints();
        // remove mappoints if too large
        if ( map_->map_points_.size() > 1000 )  
        {
            // TODO map is too large, remove some one 
            map_point_erase_ratio_ += 0.05;
        }
        else 
        map_point_erase_ratio_ = 0.1;
        
        cout<<"current map points: "<<map_->map_points_.size()<<endl;

    }
  
    double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
    {
        Vector3d n = point->pos_ - frame->getCenter();
        n.normalize();
        return acos( n.transpose()*point->norm_ );
    }

    bool VisualOdometry::checkEstimatedPose(){
        // check if the estimated pose is good
        if ( num_inliers_ < min_inliers_ )
        {
            cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
        // calculate T between curr_ and ref_
        Sophus::SE3d T_c_r_estimated = ref_->T_c_w_ * T_c_w_estimated_.inverse();
        Sophus::Vector6d d = T_c_r_estimated.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    } 
    
    
    bool VisualOdometry::checkKeyFrame(){
        // TODO CHECK 
        Sophus::SE3d T_c_r_estimated = ref_->T_c_w_ * T_c_w_estimated_.inverse();
        Sophus::Vector6d d = T_c_r_estimated.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

}