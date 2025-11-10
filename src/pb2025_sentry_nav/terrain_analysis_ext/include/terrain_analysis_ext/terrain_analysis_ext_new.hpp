// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#ifndef TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NEW_HPP_
#define TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NEW_HPP_

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace terrain_analysis
{

/**
 * @class TerrainAnalysisExtNode
 * @brief 拓展尺度地形分析节点 - 组件化定时器驱动版本
 *
 * @details
 * 该节点在原始 `terrainAnalysisExt` 可执行程序的基础上重构为 ROS 2 组件，
 * 将原先的主循环迁移到 100Hz 的定时器回调中，并保持所有算法逻辑不变。
 * 主要功能包括：
 * - 点云堆叠与体素降采样
 * - 地形地面估计与连通性检查
 * - 远/近景地形图融合与发布
 *
 * @note 该类设计用于 Nav2 组件容器，可通过 `LoadComposableNodes` 动态加载。
 */
class TerrainAnalysisExtNode : public rclcpp::Node
{
public:
  /// @brief 构造函数，初始化参数、订阅发布器以及定时器
  explicit TerrainAnalysisExtNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // ==================== 回调函数 ====================

  /// @brief 里程计回调 - 更新车辆位姿
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  /// @brief 点云回调 - 截取 ROI 点云并标记新数据
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud_msg);

  /// @brief 本地地形图回调 - 缓存局部地图用于融合
  void terrainCloudLocalHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_cloud_msg);

  /// @brief 手柄回调 - 触发地图清空
  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  /// @brief 清空距离回调 - 更新清空半径并触发清空
  void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr distance);

  /// @brief 定时器回调 - 主处理循环，100Hz
  void timerCallback();

  // ==================== 参数与状态 ====================

  // ---- 参数 ----
  double scan_voxel_size_;
  double decay_time_;
  double no_decay_dis_;
  double clearing_dis_;
  bool clearing_cloud_;
  bool use_sorting_;
  double quantile_z_;
  double vehicle_height_;
  int voxel_point_update_thre_;
  double voxel_time_update_thre_;
  double lower_bound_z_;
  double upper_bound_z_;
  double dis_ratio_z_;
  bool check_terrain_conn_;
  double terrain_under_vehicle_;
  double terrain_conn_thre_;
  double ceiling_filtering_thre_;
  double local_terrain_map_radius_;

  // ---- 系统状态 ----
  bool system_inited_;
  bool new_laser_cloud_;
  double system_init_time_;
  double laser_cloud_time_;

  double vehicle_roll_;
  double vehicle_pitch_;
  double vehicle_yaw_;
  double vehicle_x_;
  double vehicle_y_;
  double vehicle_z_;

  int terrain_voxel_shift_x_;
  int terrain_voxel_shift_y_;

  // ---- 常量 ----
  constexpr static float kTerrainVoxelSize = 2.0f;
  constexpr static int kTerrainVoxelWidth = 41;
  constexpr static int kTerrainVoxelHalfWidth = (kTerrainVoxelWidth - 1) / 2;
  constexpr static int kTerrainVoxelNum = kTerrainVoxelWidth * kTerrainVoxelWidth;

  constexpr static float kPlanarVoxelSize = 0.4f;
  constexpr static int kPlanarVoxelWidth = 101;
  constexpr static int kPlanarVoxelHalfWidth = (kPlanarVoxelWidth - 1) / 2;
  constexpr static int kPlanarVoxelNum = kPlanarVoxelWidth * kPlanarVoxelWidth;

  // ---- 体素存储 ----
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_downsampled_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_local_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_voxel_cloud_[kTerrainVoxelNum];

  int terrain_voxel_update_num_[kTerrainVoxelNum];
  float terrain_voxel_update_time_[kTerrainVoxelNum];
  float planar_voxel_elev_[kPlanarVoxelNum];
  int planar_voxel_conn_[kPlanarVoxelNum];
  std::vector<float> planar_point_elev_[kPlanarVoxelNum];
  std::queue<int> planar_voxel_queue_;

  // ---- 工具对象 ----
  pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_;

  // ---- ROS 接口 ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_cloud_local_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace terrain_analysis

#endif  // TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NEW_HPP_