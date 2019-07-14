/*
 * @Description: MapPoint class
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntianli91
 * @Date: 2019-07-07 18:57:36
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 14:11:44
 */
#ifndef MAPPOINT_H_
#define MAPPOINT_H_
#include "vo/common_headers.h"
#include "vo/frame.h"

using namespace Eigen;

namespace vo
{
    class MapPoint
    {
        public:
        // ------------------------ variables --------------------- // 
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long id_;
        static unsigned long factory_id_;
        Vector3d pos_;
        Vector3d norm_;
        bool good_status_;
        cv::Mat descriptor_;

        list<Frame::Ptr> observed_frames_;

        int visible_times_;
        int matched_times_;
        // ------------------------ functions --------------------- //
        // constructor
        MapPoint();
        MapPoint(unsigned long id, 
                const Vector3d& pos, 
                const Vector3d& norm, 
                const Frame::Ptr frame = nullptr,
                const cv::Mat& descriptor = cv::Mat());
        // factory function
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint(const Vector3d& pt_w,
                            const Vector3d& norm,
                            const Frame::Ptr frame,
                            const cv::Mat& descriptor);
        // other function
        cv::Point3d getPoseCV(){
            return cv::Point3d(pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }
    };
}
#endif