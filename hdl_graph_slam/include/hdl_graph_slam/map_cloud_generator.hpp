#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <hdl_graph_slam/keyframe.hpp>
#include <vector>

namespace hdl_graph_slam {

/**
 * @brief this class generates a map point cloud from registered keyframes
 */
class MapCloudGenerator {
 public:
  using PointT = pcl::PointXYZI;

  MapCloudGenerator();
  ~MapCloudGenerator();

  /**
   * @brief 点云地图生成器
   */
  pcl::PointCloud<PointT>::Ptr generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution);

 private:
  pcl::VoxelGrid<PointT> voxel_filter;
};

}  // namespace hdl_graph_slam
