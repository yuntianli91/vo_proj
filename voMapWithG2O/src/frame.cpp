/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/OBKoro1
 * @Date: 2019-07-07 16:16:05
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 14:27:53
 */
#include "vo/frame.h"
namespace vo
{
    // default constructor
    Frame::Frame():id_(-1), kf_tag_(false), time_stamp_(-1), camera_(nullptr){}
    // constructor with real image
    Frame::Frame(unsigned long id, bool kf_tag, double time_stamp, SE3<double> T_c_w, 
        Camera::Ptr camera, cv::Mat color, cv::Mat depth):id_(id), kf_tag_(kf_tag), time_stamp_(time_stamp),
        camera_(camera), T_c_w_(T_c_w), color_(color), depth_(depth){}

    // create frame object;
    Frame::Ptr Frame::createFrame(){
        static long factory_index = 0;
        return Frame::Ptr(new Frame(factory_index++));
    }
    // find depth
    double Frame::findDepth(const cv::KeyPoint& kp)
    {
        int u = round(kp.pt.x);
        int v = round(kp.pt.y);

        ushort d = depth_.ptr<ushort>(v)[u];
        if(d!=0){
            return (double)d / camera_->depth_scale_;
        }
        else{
            // check the nearby points 
            int du[4] = {-1,0,1,0};
            int dv[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = depth_.ptr<ushort>( v+dv[i] )[u+du[i]];
                if ( d!=0 )
                {
                    return double(d)/camera_->depth_scale_;
                }
            } 
        }
        return -1.0;
    }
    // use const tag to declare read-only memeber function
    Vector3d Frame::getCenter() const{
        return Vector3d(T_c_w_.inverse().translation());
    }
    // // get sub pixel value
    // double Frame::getSubPixValue(const Vector3d& pt_w){
    //     Vector2d pt_p = camera_->world2pixel(pt_w, T_c_w_);
    //     int x = floor(pt_p(1, 0));
    //     int y = floor(pt_p(0, 0));
    //     double dx = pt_p(1, 0) - x;
    //     double dy = pt_p(0, 0) - y;

    //     return double((1.0 - dx) * (1.0 - dy) * color_.ptr<uchar>(x)[y]
    //                 + (1.0 -dx) * dy * color_.ptr<uchar>(x+1)[y]
    //                 + dx * (1.0 - dy) * color_.ptr<uchar>(x)[y+1]
    //                 + dx * dy * color_.ptr<uchar>(x+1)[y+1]);
    // }
    // check if a point is in a frame
    bool Frame::isInFrame(const Vector3d& pt_w){
        Vector3d pt_c = camera_->world2camera(pt_w, T_c_w_);
        if(pt_c(2, 0) < 0){return false;}
        Vector2d pt_p = camera_->world2pixel(pt_w, T_c_w_);
        if(pt_p(0, 0) < 0 || pt_p(0, 0) > color_.cols || pt_p(1,0) < 0 || pt_p(1,0) > color_.rows){
            return false;
        }
        else{
            return true;
        }
    }
} // namespace namevo