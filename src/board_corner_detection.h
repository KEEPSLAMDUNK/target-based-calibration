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

#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <stdexcept>

class BoardCornerDetection {
public:
  struct BoardParams {
    float board_width = 1.4;
    float board_height = 1.0;

    float marker_size = 0.2;
    float marker_width_interval = 1.1;
    float marker_height_interval = 0.7;

    // 从YAML文件加载参数
    void LoadFromYAML(const std::string& yaml_file_path) {
      try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        
        if (config["board_params"]) {
          auto board_config = config["board_params"];
          
          if (board_config["board_width"]) {
            board_width = board_config["board_width"].as<float>();
          }
          if (board_config["board_height"]) {
            board_height = board_config["board_height"].as<float>();
          }
          if (board_config["marker_size"]) {
            marker_size = board_config["marker_size"].as<float>();
          }
          if (board_config["marker_width_interval"]) {
            marker_width_interval = board_config["marker_width_interval"].as<float>();
          }
          if (board_config["marker_height_interval"]) {
            marker_height_interval = board_config["marker_height_interval"].as<float>();
          }
          
          std::cout << "Successfully loaded board parameters from: " << yaml_file_path << std::endl;
        } else {
          std::cerr << "Warning: 'board_params' section not found in YAML file. Using default values." << std::endl;
        }
      } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file '" << yaml_file_path << "': " << e.what() << std::endl;
        std::cerr << "Using default parameters." << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Using default parameters." << std::endl;
      }
    }
  };

public:
  BoardCornerDetection();
  BoardCornerDetection(const std::string& config_file_path);
  virtual ~BoardCornerDetection();

  // 加载配置文件
  void LoadConfig(const std::string& config_file_path);

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
