// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>

#include <Eigen/Dense>

namespace hdl_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(ros::NodeHandle& pnh) : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 2.0);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 2.0);

    accum_distance = 0.0;
  }

  /**
   * @brief 关键帧判断
   * @param pose 位姿
   */
  bool update(const Eigen::Isometry3d& pose) {
    /// 首帧直接是关键帧
    if (is_first) {
      is_first = false;
      prev_keypose = pose;
      return true;
    }
    /// 上一关键帧到当前关键帧的相对变化量
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();                 // 位移变化量
    double da = Eigen::AngleAxisd(delta.linear()).angle();  // 角度变化量

    /// 变化太小不作为关键帧
    if (dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }
    accum_distance += dx;
    prev_keypose = pose;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const { return accum_distance; }

 private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
