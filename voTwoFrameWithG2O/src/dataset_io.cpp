/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-07-15 08:49:29
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-15 15:16:53
 */
#include "vo/dataset_io.h"

namespace vo
{

    bool DatasetIO::readAssociateRGBD(const string& dataset_dir, vector<double>& rgb_times, vector<string>& rgb_files,
                        vector<double>& depth_times, vector<string>& depth_files)
    {   // read RGB-D dataset
        if (DatasetIO::io_ptr_ == nullptr){
            DatasetIO::io_ptr_ = shared_ptr<DatasetIO>(new DatasetIO());
        }

        io_ptr_->fin_.open(dataset_dir + "/associate.txt"); // openassociate file
        
        if (!io_ptr_->fin_.is_open()){
            cout << "Failed to read associate.txt !" << endl;
            return false;
        }

        while(io_ptr_->fin_.peek() != EOF){

            string rgb_time, rgb_file, depth_time, depth_file;
            io_ptr_->fin_>>rgb_time>>rgb_file>>depth_time>>depth_file;
            rgb_times.push_back ( atof ( rgb_time.c_str() ) );
            depth_times.push_back ( atof ( depth_time.c_str() ) );
            rgb_files.push_back ( dataset_dir+"/"+rgb_file );
            depth_files.push_back ( dataset_dir+"/"+depth_file );

            if ( io_ptr_->fin_.good() == false )
                break;  
        }

        io_ptr_->fin_.clear();
        io_ptr_->fin_.close();

        return true;
    }



    // output functions
    bool DatasetIO::writeEstimatedRGBD(const string& file_path, const double time_stamp, const Sophus::SE3d T_c_w){
        if (DatasetIO::io_ptr_ == nullptr){
            DatasetIO::io_ptr_ = shared_ptr<DatasetIO>(new DatasetIO());
        }
        
        if (!io_ptr_->fout_.is_open()){
            io_ptr_->fout_.open(file_path + "/estimated_g2o.txt");        

            if (!io_ptr_->fout_.is_open()){
            cout << "Failed to write, please check file path !" << endl;
            return false;
            }        
        }

        Eigen::Vector3d estimated_trans = T_c_w.translation();
        Eigen::Quaterniond estimated_quat(T_c_w.rotationMatrix());

        io_ptr_->fout_ << fixed << time_stamp << " " 
            << estimated_trans(0, 0) << " " << estimated_trans(1, 0) << " " << estimated_trans(2, 0) << " "
            << estimated_quat.x() << " " << estimated_quat.y() << " " << estimated_quat.z() << " "
            << estimated_quat.w() << endl;

        io_ptr_->fout_.clear();
        io_ptr_->fout_.close();

        return true;
    }

    // void readFileNames(vector<string>& filename){
    //     // read filenames from associate file
    // }

    // void readTimeStamps(vector<double>& time_stamp){
    //     // read time stamp from associate file
    // }

    // initialization of all static variable members
    std::shared_ptr<DatasetIO> DatasetIO::io_ptr_ = nullptr;
}