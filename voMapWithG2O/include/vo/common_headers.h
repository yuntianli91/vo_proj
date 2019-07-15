/*
 * @Description: common headers for vo project
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntianli91
 * @Date: 2019-07-07 09:06:46
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-07-11 15:02:28
 */
#ifndef COMMON_H_
#define COMMON_H_
// ====================== C++ headers ============================= //
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <boost/timer.hpp>
#include <unordered_map>
using namespace std;
// ====================== Eigen =================================== // 
// Dense module inlcude core/geometry/LU/cholesky/SVD/QR/EigenValue
#include <Eigen/Dense>
// ====================== Opencv ================================== //
#include <opencv2/opencv.hpp>
// ====================== Sophus ================================== //
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
// ====================== Ceres =================================== //
#include <ceres/ceres.h>
// ====================== g2o ===================================== //
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#endif