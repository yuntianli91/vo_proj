/*
 * @Description: pose Estimator
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-10 15:40:12
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 10:15:54
 */
#include "vo/poseEstimator.h"
namespace vo
{   
    void PoseEstimator::poseEstimatorFeature(){
        // construc 3d and 2d points
            for(int index:matched_index_){
                pts_2d_.push_back(keypoints_curr_[index].pt);
            }
            for(MapPoint::Ptr mpt:matched_pts_3d_){
                pts_3d_.push_back(mpt->getPoseCV());
            }
        // obtain initiate value by solvePNPRansic
        cv::solvePnPRansac(pts_3d_, pts_2d_, K_, cv::Mat(), rvec0_, tvec0_, false, 100, 4.0f, 0.99, inliers_);
        num_inliers_ = inliers_.rows;
        cout << "inliers number of pnp: " << num_inliers_ << endl;
        // graphic optimization
        // rotation vector to rotation matrix
        cv::Rodrigues(rvec0_, R0_);
        // convert rotation matrix from cv::Mat to Eigen::Matrix3d 
        Eigen::Matrix3d R0;
        R0 << R0_.at<double>(0, 0), R0_.at<double>(0, 1), R0_.at<double>(0, 2), 
            R0_.at<double>(1, 0), R0_.at<double>(1, 1), R0_.at<double>(1, 2),
            R0_.at<double>(2, 0), R0_.at<double>(2, 1), R0_.at<double>(2, 2);
        // construct SE3 with SO3 and Translation vector
        T_c_w_estimated_ = Sophus::SE3<double>(
                                                Sophus::SO3<double>(R0), 
                                                Vector3d(tvec0_.at<double>(0, 0), tvec0_.at<double>(1, 0), tvec0_.at<double>(2.0))
                                              );        

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
            edge->setId(i+1);
            edge->setVertex(0, pose);
            edge->camera_ = curr_->camera_;
            edge->pt_ = Vector3d(pts_3d_[index].x, pts_3d_[index].y, pts_3d_[index].z);
            edge->setMeasurement(Vector2d(pts_2d_[index].x, pts_2d_[index].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
            matched_pts_3d_[index]->matched_times_++;
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        T_c_w_estimated_ = SE3<double>(
            pose->estimate().rotation(),
            pose->estimate().translation()
        );
        cout << "T_c_w_estimated after optimization is: " << endl << T_c_w_estimated_.matrix() << endl;

    }

    void PoseEstimator::poseEstimatorPhotometric(){
        cv::solvePnPRansac(pts_3d_, pts_2d_, K_, cv::Mat(), rvec0_, tvec0_, false, 100, 4.0f, 0.99, inliers_);
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    }

}