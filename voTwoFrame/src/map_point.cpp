/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-10 09:46:29
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 14:17:19
 */
#include "vo/map_point.h"

namespace vo
{
    MapPoint::MapPoint():id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), good_status_(true), visible_times_(0), matched_times_(0)
    {

    } // -1 mean max value of corresponding type
    
    MapPoint::MapPoint(unsigned long id, const Vector3d& pos, const Vector3d& norm, const Frame::Ptr frame, const cv::Mat& descriptor)
    :id_(id), pos_(pos), norm_(norm), good_status_(true), descriptor_(descriptor), visible_times_(0), matched_times_(0)
    {
        observed_frames_.push_back(frame);
    }

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        return MapPoint::Ptr(new MapPoint(factory_id_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }
    
    MapPoint::Ptr MapPoint::createMapPoint(const Vector3d& pt_w, const Vector3d& norm, const Frame::Ptr frame, const cv::Mat& descriptor)
    {
        return MapPoint::Ptr(new MapPoint(factory_id_++, pt_w, norm, frame,descriptor));
    }
    
    unsigned long MapPoint::factory_id_ = 0;
}