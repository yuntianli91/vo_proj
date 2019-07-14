/*
 * @Description: parameter config
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-08 09:17:15
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-09 08:23:54
 */
#ifndef CONFIG_H_
#define CONFIG_H_
#include "vo/common_headers.h"

namespace vo{
    class Config{
    // use private constructor to declare a Singleton 
    private:
        Config(){};
        static shared_ptr<Config> config_;
        cv::FileStorage file_;

    public:
        // get instance of singleton
        // static shared_ptr<Config> getInstance(){
            // config_ = shared_ptr<Config>(new Config());
            // return config_;
        // }
        
         // destructor
        ~Config(){
           if (file_.isOpened()){
               file_.release();
           }
        }
        // set param file path
        static void setParamFile(const std::string& filepath);
        // get param from file
        template<typename T>
        static T getParam(const std::string& key){
            return T(config_->file_[key]);
        }
    };
}
#endif