/**
 * @file PCFeatureExtra.hpp
 * @author Yanliang Wang (wyl410922@qq.com)
 * @date 2022-02-07
 *
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 *
 */
#ifndef PCFEATUREEXTRA_HPP
#define PCFEATUREEXTRA_HPP

#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;
using namespace cv;

namespace gtsam {

class PcPolygonFitFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
  gtsam::Vector3 point_;
  const std::vector<double> tag_size_;

  double error_function(double x, double y, double z) const {
    return checkCost(x, -tag_size_[2] / 2, tag_size_[2] / 2) +
           checkCost(y, -tag_size_[1] / 2, tag_size_[1] / 2) +
           checkCost(z, -tag_size_[0] / 2, tag_size_[0] / 2);
  }
  double checkCost(double value, double min_threshold,
                   double max_threshold) const {
    if (value >= min_threshold && value <= max_threshold)
      return 0;
    else {
      return min(abs(value - min_threshold), abs(value - max_threshold));
    }
  }

public:
  PcPolygonFitFactor(gtsam::Key pose_key, gtsam::Vector3 point,
                     const std::vector<double> &tag_size,
                     gtsam::SharedNoiseModel noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        point_(point), tag_size_(tag_size) {}

  gtsam::Vector
  evaluateError(const Pose3 &pose,
                boost::optional<Matrix &> H1 = boost::none) const override {
    gtsam::Point3 point(point_(0), point_(1), point_(2));

    gtsam::Point3 point_trans = pose.transformFrom(point);

    gtsam::Vector1 residual;
    residual[0] =
        error_function(point_trans[0], point_trans[1], point_trans[2]);

    // if we need jaccobians
    if (H1) {
      *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
          std::bind(&PcPolygonFitFactor::evaluateError, this,
                    std::placeholders::_1, boost::none),
          pose, 1e-8);
    }

    return residual;
  }
};

} // namespace gtsam

Eigen::Matrix4d extractPCFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
                                 const Eigen::Vector4d &plane_coeff,
                                 const std::vector<double> &tag_size,
                                 bool debug_flag = true,
                                 bool verbose_flag = false) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*_cloud, *cloud_copy);
  
  int size = cloud_copy->points.size();

  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_copy);
  chull.setAlpha(0.5);
  chull.reconstruct(*cloud_copy);

  pcl::io::savePCDFile("chull.pcd", *cloud_copy);

  gtsam::NonlinearFactorGraph graph;

  gtsam::noiseModel::Isotropic::shared_ptr gaussian_model(
      gtsam::noiseModel::Isotropic::Sigma(1, 1));
  gtsam::noiseModel::Robust::shared_ptr noise_model =
      gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(0.05), gaussian_model);

  for (size_t i = 0; i < size; ++i) {
    gtsam::PcPolygonFitFactor factor(gtsam::Key(0),
                                     gtsam::Vector3(cloud_copy->points[i].x,
                                                    cloud_copy->points[i].y,
                                                    cloud_copy->points[i].z),
                                     tag_size, noise_model);
    graph.add(factor);
  }

  Eigen::Vector3d start(plane_coeff(0), plane_coeff(1), plane_coeff(2));
  Eigen::Vector3d end(1, 0, 0);

  gtsam::Values initial_values;
  Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();
  initial_pose.block<3, 3>(0, 0) =
      Eigen::Quaterniond::FromTwoVectors(start, end).toRotationMatrix();
  initial_pose(0, 3) = plane_coeff(3);
  initial_values.insert(0, gtsam::Pose3(initial_pose));

  gtsam::LevenbergMarquardtParams optimizer_params;
  optimizer_params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  optimizer_params.setRelativeErrorTol(1e-4);

  // optimize
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values,
                                               optimizer_params);
  gtsam::Values result = optimizer.optimize();

  double error = graph.error(result);
  if (error > 0.1) {
    Eigen::Matrix4d T_axis = Eigen::Matrix4d::Identity();
    double angle = M_PI / 2; // 90 度转换为弧度
    T_axis.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();

    initial_pose = T_axis * initial_pose;
    initial_values.clear();
    initial_values.insert(0, gtsam::Pose3(initial_pose));

    gtsam::LevenbergMarquardtOptimizer optimizer_again(graph, initial_values,
                                                 optimizer_params);
    result = optimizer_again.optimize();
  }

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);

  if (debug_flag) {
    cout << "\033[1;32mPointCloudExtract\033[0m:\t"
         << "solve time cost = " << time_used.count() << " seconds. " << endl;
  }

//  std::cout << centroid.transpose() << std::endl << std::endl;
//  std::cout << plane_coeff.transpose() << std::endl << std::endl;
//  std::cout << initial_pose << std::endl << std::endl;
//  std::cout << result.at<gtsam::Pose3>(0).matrix() << std::endl;

  return result.at<gtsam::Pose3>(0).matrix();
}

#endif // PCFEATUREEXTRA_HPP