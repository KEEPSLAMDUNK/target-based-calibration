//
// Created by wjh on 24-4-2.
//

#include "board_corner_detection.h"

BoardCornerDetection::BoardCornerDetection() {
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  parameters_ = cv::aruco::DetectorParameters::create();
}

BoardCornerDetection::~BoardCornerDetection() {}

void BoardCornerDetection::SetDictionary(
    const cv::Ptr<cv::aruco::Dictionary> &dictionary) {
  dictionary_ = dictionary;
}

void BoardCornerDetection::DetectMarkers(
    const cv::Mat &input_image,
    std::vector<std::vector<cv::Point2f>> &marker_corners,
    std::vector<int> &marker_ids,
    std::vector<std::vector<cv::Point2f>> &rejected_candidates) {
  cv::aruco::detectMarkers(input_image, this->dictionary_, marker_corners,
                           marker_ids, this->parameters_, rejected_candidates);
}

void BoardCornerDetection::SetCameraModel(
    const Eigen::Matrix3d &camera_matrix,
    const Eigen::Matrix<double, 5, 1> &dist_coeffs) {
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(dist_coeffs, dist_coeffs_);
}

std::vector<Eigen::Vector3d>
BoardCornerDetection::DetectMarkersInImage(const cv::Mat &input_image) {
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  this->DetectMarkers(input_image, marker_corners, marker_ids,
                      rejected_candidates);

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
      marker_corners, this->board_params_.marker_size, camera_matrix_,
      dist_coeffs_, rvecs, tvecs);

  std::cout << dist_coeffs_ << std::endl;

  cv::Vec3d average_rvec, average_tvec;
  this->CalculateAverageRot(rvecs, average_rvec);
  this->CalculateAverageTrans(tvecs, average_tvec);

  std::vector<std::vector<cv::Point3f>> marker_physical_corners;
  this->CalculateMarkerPhysicalCorners(marker_physical_corners);

  std::vector<int> board_ids{1, 2, 3, 4}; // IDs order as explained above
  cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(
      marker_physical_corners, this->dictionary_, board_ids);
  int valid = cv::aruco::estimatePoseBoard(
      marker_corners, marker_ids, board, this->camera_matrix_,
      this->dist_coeffs_, average_rvec, average_tvec, true);

  std::vector<Eigen::Vector3d> board_physical_corners;
  this->CalculateBoardPhysicalCorners(average_rvec, average_tvec,
                                      board_physical_corners);

  // Draw center (DEBUG)
//  std::vector<cv::Point3d> corners3d;
//  for (auto &corner : board_physical_corners) {
//    corners3d.push_back(cv::Point3d(corner[0], corner[1], corner[2]));
//  }
//
//  cv::Mat outputImage = input_image.clone();
//  cv::drawFrameAxes(outputImage, camera_matrix_, dist_coeffs_, average_rvec,
//                    average_tvec, 0.1);
//
//  std::vector<cv::Point2d> uv;
//  cv::projectPoints(corners3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
//                    camera_matrix_, dist_coeffs_, uv);
//
//  for (auto &pixel : uv) {
//    cv::circle(outputImage, pixel, 5, cv::Scalar(0, 255, 0), -1);
//  }
//
//  cv::imshow("outputImage", outputImage);
//  cv::waitKey(0);

  return board_physical_corners;
}

void BoardCornerDetection::CalculateAverageRot(
    const std::vector<cv::Vec3d> &rvecs, cv::Vec3d &average_rvec) {
  Eigen::Quaterniond quaternion_sum(0.0, 0.0, 0.0, 0.0);
  for (const auto &rotVec : rvecs) {
    cv::Mat R;
    cv::Rodrigues(rotVec, R);
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R, R_eigen);
    Eigen::Quaterniond quaternion(R_eigen);
    // 这里简单地对四元数进行算术平均
    quaternion_sum.coeffs() += quaternion.coeffs();
  }
  quaternion_sum.coeffs() = quaternion_sum.coeffs() / static_cast<double>(rvecs.size());

  // 归一化平均四元数
  quaternion_sum.normalize();

  cv::Mat rot;
  cv::eigen2cv(quaternion_sum.toRotationMatrix(), rot);

  cv::Rodrigues(rot, average_rvec);
}

void BoardCornerDetection::CalculateAverageTrans(
    const std::vector<cv::Vec3d> &tvecs, cv::Vec3d &average_tvec) {
  average_tvec = cv::Vec3d(0, 0, 0);
  for (size_t i = 0; i < tvecs.size(); ++i) {
    average_tvec += tvecs[i];
  }
  average_tvec = average_tvec / int(tvecs.size());
}

void BoardCornerDetection::CalculateMarkerPhysicalCorners(
    std::vector<std::vector<cv::Point3f>> &marker_physical_corners) {
  marker_physical_corners.resize(4);
  for (int i = 0; i < 4; ++i) {
    int x_qr_center = (i % 3) == 0 ? -1 : 1; // x distances are substracted for
                                             // QRs on the left, added otherwise
    int y_qr_center =
        (i < 2) ? 1 : -1; // y distances are added for QRs above target's
                          // center, substracted otherwise
    float x_center =
        x_qr_center * (this->board_params_.marker_width_interval / 2.);
    float y_center =
        y_qr_center * (this->board_params_.marker_height_interval / 2.);

    for (int j = 0; j < 4; ++j) {
      int x_qr = (j % 3) == 0 ? -1 : 1; // x distances are added for QRs 0 and
                                        // 3, substracted otherwise
      int y_qr = (j < 2) ? 1 : -1; // y distances are added for QRs 0 and 1,
                                   // substracted otherwise
      cv::Point3f pt3d(x_center + x_qr * this->board_params_.marker_size / 2., y_center + y_qr * this->board_params_.marker_size / 2.,
                       0);

      marker_physical_corners[i].push_back(pt3d);
    }
  }
}

void BoardCornerDetection::CalculateBoardPhysicalCorners(
    const cv::Vec3d &rvec, const cv::Vec3d &tvec,
    std::vector<Eigen::Vector3d> &board_physical_corners) {
  board_physical_corners.resize(4);

  // Build transformation matrix to calibration target axis
  cv::Mat R(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, R);

  Eigen::Matrix3d R_eigen;
  Eigen::Vector3d t;
  cv::cv2eigen(R, R_eigen);
  cv::cv2eigen(tvec, t);

  std::vector<Eigen::Vector3d> board_corners;
  board_corners.push_back(
      Eigen::Vector3d(-this->board_params_.board_width / 2.0,
                      this->board_params_.board_height / 2.0, 0));
  board_corners.push_back(
      Eigen::Vector3d(this->board_params_.board_width / 2.0,
                      this->board_params_.board_height / 2.0, 0));
  board_corners.push_back(
      Eigen::Vector3d(this->board_params_.board_width / 2.0,
                      -this->board_params_.board_height / 2.0, 0));
  board_corners.push_back(
      Eigen::Vector3d(-this->board_params_.board_width / 2.0,
                      -this->board_params_.board_height / 2.0, 0));

  for (int i = 0; i < board_corners.size(); ++i) {
    board_physical_corners[i] = R_eigen * board_corners[i] + t;
  }
}
