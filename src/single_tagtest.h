/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 ********************************************************************/

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "board_corner_detection.h"

#define DEFAULT_TAG_FAMILY "Tag36h11"

void getInitCorners(const cv::Mat &im, cv::Point2d corners[]) {
  // up , down
  for (int i = 0; i < im.rows - 1; i++) {
    cv::Scalar current_mean_temp = cv::mean(im.rowRange(i, i + 1));
    cv::Scalar next_mean_temp = cv::mean(im.rowRange(i + 1, i + 2));
    double current_sum(0), next_sum(0);
    for (int j = 0; j < 4; j++) {
      current_sum += current_mean_temp.val[j];
      next_sum += next_mean_temp.val[j];
    }
    // up
    if (current_sum == 0 && next_sum > 0) {
      int j;
      for (j = 0; j < im.cols; j++) {
        if (im.at<cv::Vec3b>(i + 1, j)[0] || im.at<cv::Vec3b>(i + 1, j)[1] ||
            im.at<cv::Vec3b>(i + 1, j)[2]) {
          break;
        }
      }
      corners[0] = cv::Point2d(j, i + 1);
    }
    // down
    if (current_sum > 0 && next_sum == 0) {
      int j;
      for (j = 0; j < im.cols; j++) {
        if (im.at<cv::Vec3b>(i, j)[0] || im.at<cv::Vec3b>(i, j)[1] ||
            im.at<cv::Vec3b>(i, j)[2]) {
          break;
        }
      }
      corners[2] = cv::Point2d(j, i);
    }
  }
  // left right
  for (int i = 0; i < im.cols - 1; i++) {
    cv::Scalar current_mean_temp = cv::mean(im.colRange(i, i + 1));
    cv::Scalar next_mean_temp = cv::mean(im.colRange(i + 1, i + 2));
    double current_sum(0), next_sum(0);
    for (int j = 0; j < 4; j++) {
      current_sum += current_mean_temp.val[j];
      next_sum += next_mean_temp.val[j];
    }
    // left
    if (current_sum == 0 && next_sum > 0) {
      int j;
      for (j = 0; j < im.rows; j++) {
        if (im.at<cv::Vec3b>(j, i + 1)[0] || im.at<cv::Vec3b>(j, i + 1)[1] ||
            im.at<cv::Vec3b>(j, i + 1)[2]) {
          break;
        }
      }
      corners[3] = cv::Point2d(i + 1, j);
    }
    // down
    if (current_sum > 0 && next_sum == 0) {
      int j;
      for (j = 0; j < im.cols; j++) {
        if (im.at<cv::Vec3b>(j, i)[0] || im.at<cv::Vec3b>(j, i)[1] ||
            im.at<cv::Vec3b>(j, i)[2]) {
          break;
        }
      }
      corners[1] = cv::Point2d(i, j);
    }
  }
}

bool compareCorners(const cv::Point2d corners1[],
                    const std::vector<cv::Point2d> &corners2) {
  cv::Point2d center1 = (corners1[0] + corners1[2]) / 2;
  cv::Point2d distance1_vector = corners1[0] - center1;
  double distance1 = cv::sqrt(distance1_vector.x * distance1_vector.x +
                              distance1_vector.y * distance1_vector.y);
  cv::Point2d center2 = (corners2[0] + corners2[2]) / 2;
  cv::Point2d distance2_vector = corners2[0] - center2;
  double distance2 = cv::sqrt(distance2_vector.x * distance2_vector.x +
                              distance2_vector.y * distance2_vector.y);
  return distance1 > distance2;
}

void getCorners(const cv::Mat &src, const Eigen::Matrix3d &camera_matrix,
                const Eigen::Matrix<double, 5, 1> &dist_coeffs,
                std::vector<cv::Point2d> &corners_max) {
  BoardCornerDetection detector("config/board_params.yaml");
  detector.SetCameraModel(camera_matrix, dist_coeffs);
  detector.SetDictionary(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  std::vector<Eigen::Vector3d> board_corners =
      detector.DetectMarkersInImage(src);

  // custom order
    std::vector<int> order = {0, 3, 2, 1};
//  std::vector<int> order = {1, 0, 3, 2};
  std::vector<Eigen::Vector3d> board_corners_temp = board_corners;
  std::transform(
      order.begin(), order.end(), board_corners.begin(),
      [&board_corners_temp](int index) { return board_corners_temp[index]; });

  std::vector<cv::Point3d> corners3d;
  for (size_t i = 0; i < board_corners.size(); ++i) {
    corners3d.push_back(cv::Point3d(board_corners[i][0], board_corners[i][1],
                                    board_corners[i][2]));
  }

  cv::Mat cv_camera_matrix, cv_dist_coeffs;
  cv::eigen2cv(camera_matrix, cv_camera_matrix);
  cv::eigen2cv(dist_coeffs, cv_dist_coeffs);
  cv::projectPoints(corners3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                    cv_camera_matrix, cv_dist_coeffs, corners_max);

  return;
}
// int main(int argc, char** argv) {

//    const std::string win = "Single tag test";
//    double error_fraction ;
//    std::stringstream ss;
//    ss << argv[1];
//    ss >> error_fraction;

//    char file_name[100] ;
//    for (int i = 0; i < 9; ++i) {
//        cv::Point2d corners[4];
//        sprintf(file_name,
//        "/home/wang/wang/git_files/apriltags-cpp/images/image_orig/%d.jpg",
//        i); cv::Mat src = cv::imread(file_name); if (src.empty()) { continue;
//        } getCorners(src,corners,error_fraction); for (uint32_t i = 0; i < 4;
//        i++)
//        {
//            uint32_t next = (i== 4-1)?0:i+1;
//            cv::Point2d p0 = corners[i];
//            cv::Point2d p1 = corners[next];
//            // draw vertex
//            cv::circle(src, p0, double(src.rows) /150 ,
//            cv::Scalar(0,0,255),src.rows/150 + 3);
//            // draw line
//            cv::line(src,p0, p1, cv::Scalar(0,255,0) , src.rows/200);
//        }
//        labelAndWaitForKey(win, "Detected", src, ScaleNone, true);

//    }
//    std::cout<<std::endl;
//    return 0;
//}
