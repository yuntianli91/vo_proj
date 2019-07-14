/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 15:37:18
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 09:44:00
 */
#include "vo/config.h"

#include "vo/visual_odometry.h"


int main(int argc, char** argv){
    // ======================== Initialization ================================================== //
    // check input value
    // if(argc != 2){
        // cout << "usage: vo_test parameter_file." << endl;
        // return 1;
    // }
    // load parameter file
    vo::Config::setParamFile("/home/yuntianli/gitRepository/odometry/config/default.yaml");
    // vo::Config::setParamFile(argv[1]);
    // initiate vo object
    vo::VisualOdometry::Ptr myVO(new vo::VisualOdometry());
    // load dataset
    myVO->loadDataset();
    // camera model 
    vo::Camera::Ptr camera(new vo::Camera(myVO->fx_, myVO->fy_, myVO->cx_, myVO->cy_, myVO->depth_scale_));
    // visualization
    cv::viz::Viz3d vis("Visual Odometry"); // create visualization window
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.3); // create coordinate system with scale factor
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir (0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);// set affine3d camera pose
    vis.setViewerPose(cam_pose);    

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 ); // widget property
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1. ); //widget property
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );
    // ========================== Visual Odometry ================================================ //
    //  run vo
    cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
    for (int i=0; i < (int)min(myVO->rgb_files_.size(), myVO->depth_files_.size()-1); i++){
        cv::Mat color = cv::imread(myVO->rgb_files_[i], CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat depth = cv::imread(myVO->depth_files_[i], CV_LOAD_IMAGE_UNCHANGED);
        
        vo::Frame::Ptr capFrame = vo::Frame::createFrame();
        capFrame->color_ = color;
        capFrame->depth_ = depth;
        capFrame->camera_ = camera;
        capFrame->time_stamp_ = myVO->rgb_times_[i];
        
        myVO->addFrame(capFrame);
        if(myVO->state_ == vo::VisualOdometry::LOST){
            break;
        }



        Sophus::SE3<double> T_w_c = capFrame->T_c_w_.inverse();
        Eigen::Matrix3d currTcw = capFrame->T_c_w_.rotationMatrix();
        Eigen::Quaterniond q(currTcw);
        cout << "Quaternion of current frame is:" << endl << q.coeffs() << endl;
        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                T_w_c.rotationMatrix()(0, 0), T_w_c.rotationMatrix()(0, 1), T_w_c.rotationMatrix()(0, 2),
                T_w_c.rotationMatrix()(1, 0), T_w_c.rotationMatrix()(1, 1), T_w_c.rotationMatrix()(1, 2),
                T_w_c.rotationMatrix()(2, 0), T_w_c.rotationMatrix()(2, 1), T_w_c.rotationMatrix()(2, 2)
            ),
            cv::Affine3d::Vec3 (
                T_w_c.translation() ( 0,0 ), T_w_c.translation() ( 1,0 ), T_w_c.translation() ( 2,0 )
            )
        );
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
        // cv::waitKey();
        // show image 
        cv::Mat map_color; 
        cv::cvtColor(capFrame->color_, map_color, cv::COLOR_GRAY2BGR);
        for (auto& mpt:myVO->map_->map_points_){
            Vector2d pixel = capFrame->camera_->world2pixel(mpt.second->pos_, capFrame->T_c_w_);
            cv::circle(map_color, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(-1), 2);
        }

        cv::imshow("rgb", map_color);
        cv::imshow("depth", capFrame->depth_);
        cout << "show frame " << i << "/" << (int)myVO->rgb_files_.size() - 2 << " captured at " << capFrame->time_stamp_ << endl;
        cv::waitKey(1);



        cout << endl;
    }
}