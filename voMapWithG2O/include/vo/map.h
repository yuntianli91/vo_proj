/*
 * @Description: Map class for vo
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-07 19:20:41
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 14:20:45
 */
#ifndef MAP_H_
#define MAP_H_

#include "vo/common_headers.h"
#include "vo/frame.h"
#include "vo/map_point.h"

namespace vo 
{
    class Map
    {
    public:
        typedef shared_ptr<Map> Ptr;

        unordered_map<unsigned long, MapPoint::Ptr> map_points_;
        unordered_map<unsigned long, Frame::Ptr> key_frames_;

        Map(){};

        void insertKeyFrame(Frame::Ptr ky_frame);
        void insertMapPoint(MapPoint::Ptr mappoint);

    };
}
#endif