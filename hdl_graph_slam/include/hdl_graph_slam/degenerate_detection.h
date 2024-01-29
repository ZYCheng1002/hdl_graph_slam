//
// Created by czy on 24-1-27.
//

#pragma once
#include "keyframe.hpp"
namespace hdl_graph_slam {
void DegenerateDetect(KeyFrame& keyframe) {
  Eigen::Vector4d centroid;  // 质心
  pcl::compute3DCentroid(*keyframe.cloud, centroid);
  Eigen::Matrix3d covariance_matrix;
  pcl::computeCovarianceMatrix(*keyframe.cloud, covariance_matrix);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto A_2 = svd.singularValues();
  keyframe.cov = covariance_matrix / A_2(0, 0);
  keyframe.degenerate = A_2(0, 0) > 3 * A_2(1, 0) ; // || A_2(1, 0) > 25 * A_2(2, 0)
}

}