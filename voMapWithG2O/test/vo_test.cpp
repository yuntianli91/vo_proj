/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-12 14:04:11
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 15:08:53
 */
#include "vo/config.h"
#include "vo/dataset_io.h"
#include "vo/traj_point.h"
#include "vo/visual_odometry.h"
using namespace Eigen;


int main(int argc, char** argv){
    // ======================= check input ========================== // 
    // if ( argc != 2 )
    // {
    //     cout<<"usage: run_vo parameter_file"<<endl;
    //     return 1;
    // }
    // ======================= read parameter file ============================ //
    // vo::Config::setParamFile(argv[1]);
    vo::Config::setParamFile("/home/yuntianli/gitRepository/odometry/voTwoFrame/config/default.yaml");
    vo::VisualOdometry::Ptr myVO(new vo::VisualOdometry);

    string dataset_dir = vo::Config::getParam<string>( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
  
    // ======================== read rgb and depth files ======================== //
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;

    vo::DatasetIO::readAssociateRGBD(dataset_dir, rgb_times, rgb_files, depth_times, depth_files);
    // ======================== read ground truth ================================ //
//     fin.open(dataset_dir + "/groundtruth.txt");

//     if ( !fin.is_open()){
//         cout << "can not open ground truth file !" << endl;
//     }
//     double time, tx, ty, tz, qx, qy, qz, qw;
    
//     int count = 0;
    
//     // vector<Eigen::Quaterniond> traj_quat;
//     // vector<Eigen::Vector3d> traj_translation;
//     vector<vo::TrajPoint::Ptr> traj_pts;

//     while ( fin.peek() != EOF){
//         if (count <3){
//             string file_head;
//             getline(fin, file_head);
//             count ++;   
//         }
//         else{
//             fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
//             vo::TrajPoint::Ptr traj_ptr(new vo::TrajPoint);
//             traj_ptr->time_stamp_ = time;
//             traj_ptr->translation_ = Vector3d(tx, ty, tz);
//             traj_ptr->quaternion_ = Quaterniond(qw, qx, qy, qz);
//             traj_pts.push_back(traj_ptr);
//         }
//    }
//     // release ifstream
//     fin.clear();
//     fin.close();
    // ======================== odometry main body =============================== //
    vo::Camera::Ptr camera(new vo::Camera);

    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    // ofstream fout(dataset_dir + "/estimated_g2o.txt");

    for ( unsigned long i=0; i<rgb_files.size()-1; i++ )
    {
        cv::Mat color = cv::imread ( rgb_files[i] );
        cv::Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        vo::Frame::Ptr pFrame = vo::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        myVO->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        
        if ( myVO->state_ == vo::VisualOdometry::LOST )
            break;
        SE3<double> Tcw = pFrame->T_c_w_.inverse();
        
        // cout << "translation is:" << pFrame->T_c_w_.translation() << endl;

        vo::DatasetIO::writeEstimatedRGBD(dataset_dir, pFrame->time_stamp_, pFrame->T_c_w_);
      
        // show the map and the camera pose 
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotationMatrix()(0,0), Tcw.rotationMatrix()(0,1), Tcw.rotationMatrix()(0,2),
                Tcw.rotationMatrix()(1,0), Tcw.rotationMatrix()(1,1), Tcw.rotationMatrix()(1,2),
                Tcw.rotationMatrix()(2,0), Tcw.rotationMatrix()(2,1), Tcw.rotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
        
        cv::Mat imgToShow = color.clone();
        // for (unsigned long i = 0; i < myVO->keypoints_curr_.size(); i++){
        //     cv::rectangle(imgToShow, myVO->keypoints_curr_[i].pt-cv::Point2f(5, 5), myVO->keypoints_curr_[i].pt+cv::Point2f(5, 5), cv::Scalar(0, 255, 0), 2);
        //     cv::circle(imgToShow, myVO->keypoints_curr_[i].pt, 1, cv::Scalar(0, 0, 255), 1);
        // }

        for (auto& mpt:myVO->map_->map_points_){
            vo::MapPoint::Ptr mpt_ptr = mpt.second;
            Vector2d mpt_uv = pFrame->camera_->world2pixel(mpt_ptr->pos_, pFrame->T_c_w_);
            cv::Point2f mpt_uv_cv(mpt_uv(0, 0), mpt_uv(1, 0));
            cv::rectangle(imgToShow, mpt_uv_cv - cv::Point2f(6, 6), mpt_uv_cv + cv::Point2f(6, 6), cv::Scalar(0, 255, 0), 2);
            cv::circle(imgToShow, mpt_uv_cv, 1, cv::Scalar(0, 0, 255), 1);
 
        }
        cv::imshow("image", imgToShow );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}