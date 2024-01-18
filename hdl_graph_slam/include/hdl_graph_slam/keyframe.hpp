// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <boost/optional.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief 关键帧
 */
struct KeyFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time& stamp,
           const Eigen::Isometry3d& odom,
           double accum_distance,
           const pcl::PointCloud<PointT>::ConstPtr& cloud);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  void save(const std::string& directory);
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  long id() const;
  Eigen::Isometry3d estimate() const;

 public:
  ros::Time stamp;                                // 时间辍
  Eigen::Isometry3d odom;                         // 激光里程计位姿
  double accum_distance;                          // 累积距离(第一个node)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // 点云
  boost::optional<Eigen::Vector4d> floor_coeffs;  // 平面拟合参数
  boost::optional<Eigen::Vector3d> utm_coord;     // utm坐标系

  boost::optional<Eigen::Vector3d> acceleration;    // 加速度
  boost::optional<Eigen::Quaterniond> orientation;  // 陀螺仪旋转

  g2o::VertexSE3* node;  // node instance
};

/**
 * @brief 无时序只有空间顺序的关键帧
 */
struct KeyFrameSnapshot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

 public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_HPP
