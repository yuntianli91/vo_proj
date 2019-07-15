/*
 * @Description: dataset input and output
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-15 08:32:05
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 10:05:27
 */
#ifndef DATASETIO_H_
#define DATASETIO_H_
#include "vo/common_headers.h"

namespace vo
{
    class DatasetIO
    {
        private:
        DatasetIO(){};
        static std::shared_ptr<DatasetIO> io_ptr_;
        
        ifstream fin_;
        ofstream fout_;        


        public:
        // destructor
        ~DatasetIO(){
            // release ifstream object
            if (fin_.is_open()){
                fin_.clear();
                fin_.close();
            }
            // release ofstream object
            if (fout_.is_open()){
                fout_.clear();
                fout_.close();
            }
        };
        // input functions
        static bool readAssociateRGBD(const string& dataset_dir, vector<double>& rgb_times, vector<string>& rgb_files,
                        vector<double>& depth_times, vector<string>& depth_files); // RGB-D dataset

        // bool readGroundtruthRGBD(const string& dataset_dir, )                
                        
                        // Monocular VI dataset
        // output functions
        static bool writeEstimatedRGBD(const string& file_path, const double time_stamp, const Sophus::SE3d T_c_w);

        protected:
        // void readFileNames(vector<string>& filename);
        // void readTimeStamps(vector<double>& time_stamp);
        
        
    };
}
#endif