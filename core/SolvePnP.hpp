/**
 * @file SolvePnP.hpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 *
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 *
 */
#ifndef SOLVEPNP_HPP
#define SOLVEPNP_HPP

#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

/**
 * @brief use cv::solvePnP to get initial calibration parameters, then use g2o
 * to optimize PnP problem
 *
 * @param _points_2d 2d points
 * @param _points_3d 3d points
 * @param fx intrinsic parameters
 * @param fy
 * @param cx
 * @param cy
 * @param debug_flag output debug log
 * @param verbose_flag output verbose debug log
 * @return Eigen::Matrix4d transformation from lidar to camera
 */
Eigen::Matrix4d solvePnP(const Eigen::Matrix3Xd &_points_2d,
                              const Eigen::Matrix4Xd &_points_3d, double fx,
                              double fy, double cx, double cy,
                              bool debug_flag = false,
                              bool verbose_flag = false) {
  assert(_points_2d.cols() == _points_3d.cols());
  int data_num = _points_2d.cols();
  if (debug_flag) {
    cout << "fx: " << fx << endl;
    cout << "fy: " << fy << endl;
    cout << "cx: " << cx << endl;
    cout << "cy: " << cy << endl;
    cout << _points_2d.matrix() << endl;
    cout << _points_3d.matrix() << endl;
  }
  vector<Point3f> pts_3d, pts_3d_initial;
  vector<Point2f> pts_2d, pts_2d_initial;
  int initial_data_num = min(data_num, 4);
  for (int i = 0; i < initial_data_num; ++i) {
    pts_3d_initial.emplace_back(_points_3d(0, i), _points_3d(1, i),
                                _points_3d(2, i));
    pts_2d_initial.emplace_back(_points_2d(0, i), _points_2d(1, i));
  }
  for (int i = 0; i < data_num; ++i) {
    pts_3d.emplace_back(_points_3d(0, i), _points_3d(1, i), _points_3d(2, i));
    pts_2d.emplace_back(_points_2d(0, i), _points_2d(1, i));
  }

  Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  Mat r(3, 1, CV_64FC1), t(3, 1, CV_64FC1);
  Mat inliers;
  cv::solvePnPRansac(pts_3d, pts_2d, K, Mat(), r, t, false, 1000, 10.0, 0.99,
                     inliers, cv::SOLVEPNP_ITERATIVE);

  Mat R;
  cv::Rodrigues(r, R);

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);

  Eigen::Matrix3d R_temp;
  Eigen::Vector3d t_temp;
  cv::cv2eigen(R, R_temp);
  cv::cv2eigen(t, t_temp);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = R_temp;
  T.block(0, 3, 3, 1) = t_temp;

  if (debug_flag) {
    cout << "SolvePnP:\t"
         << "solve time cost = " << time_used.count() << " seconds. " << endl;
    cout << "SolvePnP:\t"
         << "estimated model: \n"
         << T << endl;
  }

  return T;
}

#endif // SOLVEPNP_HPP