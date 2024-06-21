//
// Created by wjh on 24-4-2.
//

#ifndef LIDAR_CAMERA_CALIBRATOR_BOARD_CORNER_DETECTION_H
#define LIDAR_CAMERA_CALIBRATOR_BOARD_CORNER_DETECTION_H

#include <Eigen/Eigen>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

class BoardCornerDetection {
public:
  struct BoardParams {
    float board_width = 1.4;
    float board_height = 1.0;

    float marker_size = 0.2;
    float marker_width_interval = 1.1;
    float marker_height_interval = 0.7;
  };

public:
  BoardCornerDetection();
  virtual ~BoardCornerDetection();

public:
  void SetDictionary(const cv::Ptr<cv::aruco::Dictionary> &dictionary);

  void SetCameraModel(const Eigen::Matrix3d &camera_matrix,
                      const Eigen::Matrix<double, 5, 1> &dist_coeffs);

  std::vector<Eigen::Vector3d> DetectMarkersInImage(const cv::Mat &input_image);

private:
  void
  DetectMarkers(const cv::Mat &input_image,
                std::vector<std::vector<cv::Point2f>> &marker_corners,
                std::vector<int> &marker_ids,
                std::vector<std::vector<cv::Point2f>> &rejected_candidates);

  void CalculateAverageRot(const std::vector<cv::Vec3d> &rvecs,
                           cv::Vec3d &average_rvec);

  void CalculateAverageTrans(const std::vector<cv::Vec3d> &tvecs,
                             cv::Vec3d &average_tvec);

  void CalculateMarkerPhysicalCorners(
      std::vector<std::vector<cv::Point3f>> &marker_physical_corners);

  void CalculateBoardPhysicalCorners(
      const cv::Vec3d &rvec, const cv::Vec3d &tvec,
      std::vector<Eigen::Vector3d> &board_physical_corners);

private:
  BoardParams board_params_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;

  cv::Mat camera_matrix_, dist_coeffs_;
};

#endif // LIDAR_CAMERA_CALIBRATOR_BOARD_CORNER_DETECTION_H
