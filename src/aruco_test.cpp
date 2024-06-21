//
// Created by wjh on 24-3-30.
//

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "board_corner_detection.h"

int main(int argc, char **argv) {
  //  for (size_t i = 1; i < 5; ++i) {
  //    cv::Mat marker_image;
  //    cv::Ptr<cv::aruco::Dictionary> dictionary_test =
  //        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  //    cv::aruco::drawMarker(dictionary_test, i, 1200, marker_image, 1);
  //    cv::imwrite("marker" + std::to_string(i) + ".png", marker_image);
  //  }

  cv::Mat inputImage = cv::imread(argv[1]);

  Eigen::Matrix3d camera_matrix;
  Eigen::Matrix<double, 5, 1> dist_coeffs;
  camera_matrix << 1311.05972093921, 0.0, 749.818357327634, 0.0,
      1313.67270907052, 534.587402816582, 0.0, 0.0, 1.0;
  dist_coeffs << -0.0654429523802683, 0.0998393523310624, 0.00107153425553708,
      -0.00283428566558518, 0.0;

  BoardCornerDetection board_corner_detection;
  board_corner_detection.SetDictionary(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50));
  board_corner_detection.SetCameraModel(camera_matrix, dist_coeffs);

  board_corner_detection.DetectMarkersInImage(inputImage);

  return 0;
}