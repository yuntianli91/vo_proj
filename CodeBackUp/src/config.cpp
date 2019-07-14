/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 11:10:40
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-08 16:02:50
 */
#include "vo/config.h"

namespace vo
{

void Config::setParamFile(const std::string& filepath){
    if (config_ == nullptr){
        config_ = shared_ptr<Config>(new Config());
    }
    config_->file_ = cv::FileStorage(filepath.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false){
        cerr<<"Parameter file " << filepath << " does not exist." << endl; 
        config_->file_.release();
        return;
    }
}

shared_ptr<Config> Config::config_ = nullptr;
}