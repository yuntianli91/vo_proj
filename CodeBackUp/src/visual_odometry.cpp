/*
 * @Description: vo
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 16:10:22
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 10:17:39
 */
#include "vo/visual_odometry.h"
namespace vo
{
// constructor
VisualOdometry::VisualOdometry():state_(INITIALIZING), map_(new Map), ref_(nullptr), curr_(nullptr), poseEstimator_(new PoseEstimator), num_lost_(0), num_inliers_(0){
    // !! all pointer must be initialized, other wise there
    // will be Segment fault when you call that member.
    // ========================= get parameters from YAML ============================ //
    // ------------------------- dataset directory ----------------------------------- //
    dataset_dir_ = Config::getParam<string>("dataset_dir");
    // ------------------------- camera instrinsics ---------------------------------- //
    fx_ = Config::getParam<double>("camera.fx");
    fy_ = Config::getParam<double>("camera.fy");
    cx_ = Config::getParam<double>("camera.cx");
    cy_ = Config::getParam<double>("camera.cy");
    K_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1; 
    depth_scale_ = Config::getParam<double>("camera.depth_scale");
    // ------------------------- vo parameters --------------------------------------- //
    number_of_features_ = Config::getParam<int>("number_of_features");
    scale_factor_ = Config::getParam<double>("scale_factor");
    level_pyramid_ = Config::getParam<int>("level_pyramid");
    match_ratio_ = Config::getParam<double>("match_ratio");
    max_num_lost_= Config::getParam<int>("max_num_lost");
    min_inliers_= Config::getParam<int>("min_inliers");
    keyframe_rotation_= Config::getParam<double>("keyframe_rotation");
    keyframe_translation_= Config::getParam<double>("keyframe_translation");
    map_point_erase_ratio_= Config::getParam<double>("map_point_erase_ratio");

    orb_ = cv::ORB::create(number_of_features_, scale_factor_, level_pyramid_);
    matcher_bf_ = cv::BFMatcher(cv::NORM_HAMMING);
    matcher_flann_ = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(5, 10, 2));

    // -------------------------- output parameters ---------------------------------- //
    // cout.precision(2);
    cout << "===== Dataset Ditectory =====" << endl << dataset_dir_ << endl
        << "VO initialized with following parameters:" << endl
        << "===== Camera Instrinsics =====" << endl
        << K_ << endl << "depth_scale: " << depth_scale_ << endl
        << "===== VO Parameters =====" << endl << "number_of_features: " << number_of_features_ << endl
        << "scale_factor: " << scale_factor_ << endl << "level_pyramid: " << level_pyramid_ << endl
        << "match_ratio: " << match_ratio_ << endl << "max_num_lost: " << max_num_lost_ << endl
        << "min_inliers: " << min_inliers_ << endl << "keyframe_rotation: " << keyframe_rotation_ << endl
        << "keyframe_translation: " << keyframe_translation_ << endl << "map_point_erase_ratio " << map_point_erase_ratio_ << endl;
}

// destructor
VisualOdometry::~VisualOdometry(){

}

// "capture" Frame from dataset or real camera
bool VisualOdometry::loadDataset(){
    // path can either be a dataset dir or gstream pipeline
    // current only dataset

    // open associate files
    ifstream fin (dataset_dir_ + "/associate.txt");
    // check open satus
    if(!fin){
        cerr << "Please geneate the associate file called associate.txt !" << endl;
        return false;
    }
    
    while ( fin.peek()!=EOF ){
        string rgb_file, rgb_time, depth_file, depth_time;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        rgb_times_.push_back(atof(rgb_time.c_str()));
        rgb_files_.push_back(dataset_dir_+ "/" + rgb_file);
        depth_times_.push_back(atof(depth_time.c_str()));
        depth_files_.push_back(dataset_dir_ + "/" + depth_file); 

        if(fin.good()==false || rgb_time == "0"){break;}

    }

    return true;
}
// add frame to vo
bool VisualOdometry::addFrame(Frame::Ptr frame){
    switch (state_){
        case INITIALIZING:
        {
            // initiate first frame ad keyframe
            curr_ = ref_ = frame;
            // extract features from keyframe
            detectFeature();
            computeDescriptor();
            addKeyFrame();
            state_ = OK;
            break;
        }
        case OK:
        {
            curr_ = frame;
            curr_->T_c_w_ = ref_->T_c_w_;
            detectFeature();
            computeDescriptor();
            matchFeature();
            // TODO: POSE ESTIMATION
            // current frame
            poseEstimator_->curr_ = curr_;
            // camera matrix
            cv::Mat K_cv = (cv::Mat_<double>(3, 3)<<
                K_(0, 0), K_(0, 1), K_(0, 2),
                K_(1, 0), K_(1, 1), K_(1, 2),
                K_(2, 0), K_(2, 1), K_(2, 2)
            );
            poseEstimator_->K_ = K_cv;
            poseEstimator_->keypoints_curr_ = keypoints_curr_;
            poseEstimator_->matched_index_ = matched_index_;
            poseEstimator_->matched_pts_3d_ = matched_pts_3d_;
            // pose estimation with feature
            poseEstimator_->poseEstimatorFeature();
            num_inliers_ = poseEstimator_->num_inliers_;
            matched_pts_3d_ = poseEstimator_->matched_pts_3d_;
            // TODO: CHECK POSE AND KEYFRAME
            if (checkEstimatedPose() == true){
                curr_->T_c_w_ = poseEstimator_->T_c_w_estimated_;
                optimizeMap();
                num_lost_ = 0;
                if (checkKeyFrame() == true){
                    addKeyFrame();
                }
            } 
            break; 
        }
        case LOST:
        {
            cout << "vo has lost track !" << endl;
            break;
        }
    }
    
    return true;
}
// detect orb features
void VisualOdometry::detectFeature(){
    boost::timer timer;
    orb_->detect(curr_->color_, keypoints_curr_);
    cout << "feature detectors cost: " << timer.elapsed() << "s, " 
        << keypoints_curr_.size() << " keypoints detected." << endl;
}
// deature compute and match
void VisualOdometry::computeDescriptor(){
    boost::timer timer;
    orb_->compute(curr_->color_, keypoints_curr_, descriptor_curr_ );
    cout << "compute descriptors cost: " << timer.elapsed() << "s." << endl;
}

// match features

void VisualOdometry::matchFeature(){
    boost::timer timer;
    // select candidates in the map
    // reproject all mappoints to the reference image 
    // to check whether it's inside the frame or not
    // for loop in iterator mode, without '&', variables will be readonly.
    cv::Mat candidate_desp;
    vector<MapPoint::Ptr> candidate_mpts;
    for (auto& mpt:map_->map_points_){
        MapPoint::Ptr& pt_w = mpt.second;
        // check whether inside a frame
        if(ref_->isInFrame(pt_w->pos_)){
            pt_w->visible_times_++;
            candidate_mpts.push_back(pt_w);
            candidate_desp.push_back(pt_w->descriptor_);
        } 
    }
    // match features in current frame with candidate map point;
    vector<cv::DMatch> matches;
    matcher_bf_.match(candidate_desp, descriptor_curr_, matches);
    // min_element(FirstElement, EndElement, CopmareFunc)   
    double min_dist = std::min_element(matches.begin(), matches.end(), 
                            [](cv::DMatch& m1, cv::DMatch& m2)
                            {
                                return m1.distance < m2.distance;
                            })->distance;
    // select matched points according to match_ratio * distance, 
    // for example 2 * MIN_HAMMING and save matched 3d points with
    //  corresponding index for map optimization
    matched_index_.clear();
    matched_pts_3d_.clear();

    for (cv::DMatch m:matches){
        if (m.distance < max(min_dist * match_ratio_, 30.0)){
            matched_pts_3d_.push_back(candidate_mpts[m.queryIdx]);
            matched_index_.push_back(m.trainIdx);
        }
    }
    // output consume time
    double time_match = timer.elapsed();
    cout << "good match pts: " << matched_pts_3d_.size() << endl;
    cout << "match cost:" << time_match << "s." << endl;
}

// add keyframe to map_
void VisualOdometry::addKeyFrame(){
    // add all 3d points for first frame
    if (map_->key_frames_.empty()){
        for (int i = 0; i < (int)keypoints_curr_.size(); i++){
            double d = curr_->findDepth(keypoints_curr_[i]);
            if (d < 0){
                continue;
            }
            Vector3d pt_w = curr_->camera_->pixel2world(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                curr_->T_c_w_, d);
            Vector3d n = pt_w - ref_->getCenter();
            n.normalize();

            MapPoint::Ptr mappoint = MapPoint::createMapPoint(pt_w, n, curr_, descriptor_curr_.row(i).clone());
            map_->insertMapPoint(mappoint);
        }
    }
    // add current frame to map_ and use it as reference; 
    map_->insertKeyFrame(curr_);
    ref_ = curr_;
}

void VisualOdometry::addMapPoint(){
    vector<bool> matchFlag(keypoints_curr_.size(), false);
    for (int i:matched_index_){
        matchFlag[i] = true;
    }
    for (int i = 0; i < (int)keypoints_curr_.size(); i++){
        if (matchFlag[i] == true){continue;} // jump point already in the map

        double d = curr_->findDepth(keypoints_curr_[i]);
        if (d < 0){
            continue;
        }
        Vector3d pt_w = curr_->camera_->pixel2world(
            Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
            curr_->T_c_w_, d);
        Vector3d n = pt_w - ref_->getCenter();
        n.normalize();

        MapPoint::Ptr mappoint = MapPoint::createMapPoint(pt_w, n, curr_, descriptor_curr_.row(i).clone());
        map_->insertMapPoint(mappoint); 
   }
}

bool VisualOdometry::checkEstimatedPose(){
     // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::SE3<double> T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame(){
    Sophus::SE3<double> T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() > keyframe_rotation_ || trans.norm() > keyframe_translation_ )
        return true;
    return false;
}

void VisualOdometry::optimizeMap(){
     // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_status_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}

}

