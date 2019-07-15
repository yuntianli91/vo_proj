/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 14:15:24
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-12 14:18:27
 */
#include "vo/map.h"

namespace vo
{
    
void Map::insertKeyFrame(Frame::Ptr ky_frame){
    // if current pair do not exist, insert it;
    // otherwise replace it
    if(key_frames_.find(ky_frame->id_) == key_frames_.end()){
        key_frames_.insert(make_pair(ky_frame->id_, ky_frame));
    }
    else{
        key_frames_[ky_frame->id_] = ky_frame;
    }

}

void Map::insertMapPoint(MapPoint::Ptr mappoint){
    if(map_points_.find(mappoint->id_) == map_points_.end()){
        map_points_.insert(make_pair(mappoint->id_, mappoint));
    }
    else{
        map_points_[mappoint->id_] = mappoint;
    }
}
}