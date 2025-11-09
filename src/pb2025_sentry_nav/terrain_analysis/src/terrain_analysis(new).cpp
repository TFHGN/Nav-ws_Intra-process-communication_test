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

/**
 * @file terrain_analysis(new).cpp
 * @brief 地形分析节点实现 - 定时器驱动的可组合节点版本
 * @author Hongbiao Zhu
 * @date 2024
 * 
 * @details 
 * 该文件实现了用于机器人导航的地形分析节点，采用定时器驱动架构以支持组件化部署。
 * 主要功能包括：
 * - 点云数据采集与预处理
 * - 基于体素的地形网格管理
 * - 地面高程估计
 * - 障碍物检测与动态障碍物过滤
 * - 地形可通行性评估
 * 
 * 架构特性：
 * - 支持进程内通信（Intra-Process Communication）
 * - 100Hz定时器驱动的非阻塞处理
 * - 适配 Nav2 组件容器部署
 * - 支持实时地形滚动更新
 */

#include <cmath>
#include <functional>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

/// @brief 地形分析命名空间
namespace terrain_analysis
{

  /**
   * @class TerrainAnalysisNode
   * @brief 地形分析节点类 - 用于实时地形评估和可通行性分析
   * 
   * @details
   * 该类实现了基于点云的地形分析功能，通过维护一个滚动的体素网格来
   * 跟踪机器人周围的地形信息。主要特性包括：
   * 
   * **数据流程**：
   * 1. 接收激光雷达点云 (registered_scan)
   * 2. 接收里程计位姿 (lidar_odometry)
   * 3. 进行体素化与地形网格更新
   * 4. 计算地面高程与障碍物分布
   * 5. 发布带高程信息的地形图 (terrain_map)
   * 
   * **处理模式**：
   * - 100Hz 定时器驱动主处理循环
   * - 回调函数仅更新缓冲区标志
   * - 所有计算在定时器回调中完成
   * 
   * **坐标系**：
   * - 输入/输出均使用 "odom" 坐标系
   * - 相对于机器人基座的地形评估
   * 
   * @note 该节点设计为可组合组件，可加载到 Nav2 容器中
   */
  class TerrainAnalysisNode : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数 - 初始化地形分析节点
     * 
     * @param options ROS2 节点选项，用于配置节点行为
     * 
     * @details
     * 构造函数执行以下初始化步骤：
     * 1. **参数声明与加载**：声明并读取所有配置参数
     * 2. **体素网格初始化**：创建地形体素云数组
     * 3. **订阅器创建**：
     *    - lidar_odometry：接收机器人位姿
     *    - registered_scan：接收配准后的点云
     *    - joy：接收手柄清空指令
     *    - map_clearing：接收地图清空指令
     * 4. **发布器创建**：terrain_map - 发布带高程的地形图
     * 5. **定时器创建**：100Hz 主处理循环
     * 
     * @note 使用 SensorDataQoS 以支持高频传感器数据
     * @warning 构造函数设计为非阻塞，适配组件容器环境
     */
    explicit TerrainAnalysisNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        // Constructor with member initializer list
        // 初始化列表
        : Node("terrain_analysis", options),
          scan_voxel_size_(0.05),
          decay_time_(2.0),
          no_decay_dis_(4.0),
          clearing_dis_(8.0),
          clearing_cloud_(false),
          use_sorting_(true),
          quantile_z_(0.25),
          consider_drop_(false),
          limit_ground_lift_(false),
          max_ground_lift_(0.15),
          clear_dynamic_obs_(false),
          min_dy_obs_dis_(0.3),
          min_dy_obs_angle_(0.0),
          min_dy_obs_rel_z_(-0.5),
          abs_dy_obs_rel_z_thre_(0.2),
          min_dy_obs_vfov_(-16.0),
          max_dy_obs_vfov_(16.0),
          min_dy_obs_point_num_(1),
          no_data_obstacle_(false),
          no_data_block_skip_num_(0),
          min_block_point_num_(10),
          vehicle_height_(1.5),
          voxel_point_update_thre_(100),
          voxel_time_update_thre_(2.0),
          min_rel_z_(-1.5),
          max_rel_z_(0.2),
          dis_ratio_z_(0.2),
          system_inited_(false),
          no_data_inited_(0),
          new_laser_cloud_(false),
          system_init_time_(0),
          laser_cloud_time_(0),
          vehicleX(0), vehicleY(0), vehicleZ(0),
          vehicleRoll(0), vehiclePitch(0), vehicleYaw(0),
          sinVehicleRoll(0), cosVehicleRoll(0),
          sinVehiclePitch(0), cosVehiclePitch(0),
          sinVehicleYaw(0), cosVehicleYaw(0),
          vehicleXRec(0), vehicleYRec(0),
          terrain_voxel_shift_x_(0), terrain_voxel_shift_y_(0),
          laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
          laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()),
          laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()),
          terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()),
          terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>())
    {
      // Declare parameters
      this->declare_parameter<double>("scan_voxel_size", scan_voxel_size_);
      this->declare_parameter<double>("decay_time", decay_time_);
      this->declare_parameter<double>("no_decay_dis", no_decay_dis_);
      this->declare_parameter<double>("clearing_dis", clearing_dis_);
      this->declare_parameter<bool>("use_sorting", use_sorting_);
      this->declare_parameter<double>("quantile_z", quantile_z_);
      this->declare_parameter<bool>("consider_drop", consider_drop_);
      this->declare_parameter<bool>("limit_ground_lift", limit_ground_lift_);
      this->declare_parameter<double>("max_ground_lift", max_ground_lift_);
      this->declare_parameter<bool>("clear_dynamic_obs", clear_dynamic_obs_);
      this->declare_parameter<double>("min_dy_obs_dis", min_dy_obs_dis_);
      this->declare_parameter<double>("min_dy_obs_angle", min_dy_obs_angle_);
      this->declare_parameter<double>("min_dy_obs_rel_z", min_dy_obs_rel_z_);
      this->declare_parameter<double>("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
      this->declare_parameter<double>("min_dy_obs_vfov", min_dy_obs_vfov_);
      this->declare_parameter<double>("max_dy_obs_vfov", max_dy_obs_vfov_);
      this->declare_parameter<int>("min_dy_obs_point_num", min_dy_obs_point_num_);
      this->declare_parameter<bool>("no_data_obstacle", no_data_obstacle_);
      this->declare_parameter<int>("no_data_block_skip_num", no_data_block_skip_num_);
      this->declare_parameter<int>("min_block_point_num", min_block_point_num_);
      this->declare_parameter<double>("vehicle_height", vehicle_height_);
      this->declare_parameter<int>("voxel_point_update_thre", voxel_point_update_thre_);
      this->declare_parameter<double>("voxel_time_update_thre", voxel_time_update_thre_);
      this->declare_parameter<double>("min_rel_z", min_rel_z_);
      this->declare_parameter<double>("max_rel_z", max_rel_z_);
      this->declare_parameter<double>("dis_ratio_z", dis_ratio_z_);

      // Get parameters
      this->get_parameter("scan_voxel_size", scan_voxel_size_);
      this->get_parameter("decay_time", decay_time_);
      this->get_parameter("no_decay_dis", no_decay_dis_);
      this->get_parameter("clearing_dis", clearing_dis_);
      this->get_parameter("use_sorting", use_sorting_);
      this->get_parameter("quantile_z", quantile_z_);
      this->get_parameter("consider_drop", consider_drop_);
      this->get_parameter("limit_ground_lift", limit_ground_lift_);
      this->get_parameter("max_ground_lift", max_ground_lift_);
      this->get_parameter("clear_dynamic_obs", clear_dynamic_obs_);
      this->get_parameter("min_dy_obs_dis", min_dy_obs_dis_);
      this->get_parameter("min_dy_obs_angle", min_dy_obs_angle_);
      this->get_parameter("min_dy_obs_rel_z", min_dy_obs_rel_z_);
      this->get_parameter("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
      this->get_parameter("min_dy_obs_vfov", min_dy_obs_vfov_);
      this->get_parameter("max_dy_obs_vfov", max_dy_obs_vfov_);
      this->get_parameter("min_dy_obs_point_num", min_dy_obs_point_num_);
      this->get_parameter("no_data_obstacle", no_data_obstacle_);
      this->get_parameter("no_data_block_skip_num", no_data_block_skip_num_);
      this->get_parameter("min_block_point_num", min_block_point_num_);
      this->get_parameter("vehicle_height", vehicle_height_);
      this->get_parameter("voxel_point_update_thre", voxel_point_update_thre_);
      this->get_parameter("voxel_time_update_thre", voxel_time_update_thre_);
      this->get_parameter("min_rel_z", min_rel_z_);
      this->get_parameter("max_rel_z", max_rel_z_);
      this->get_parameter("dis_ratio_z", dis_ratio_z_);

      RCLCPP_INFO(this->get_logger(), "Parameters loaded.");

      // Initialize terrain voxel clouds
      // 初始化地形体素云
      for (int i = 0; i < kTerrainVoxelNum; i++)
      {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      }

      // Configure voxel grid filter
      downSizeFilter.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);

      // ROS2 publishers and subscribers
      // 定义ros成员
      auto sensor_qos = rclcpp::SensorDataQoS();

      subOdometry_ =
          this->create_subscription<nav_msgs::msg::Odometry>(
              "lidar_odometry",
              sensor_qos,
              [this](const nav_msgs::msg::Odometry::ConstSharedPtr odom) -> void
              {
                this->odometryHandler(odom);
              });

      sub_laser_cloud_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              "registered_scan",
              sensor_qos,
              [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) -> void
              {
                this->laserCloudHandler(laserCloud2);
              });

      sub_joystick_ =
          this->create_subscription<sensor_msgs::msg::Joy>(
              "joy",
              5,
              [this](const sensor_msgs::msg::Joy::ConstSharedPtr joy) -> void
              {
                this->joystickHandler(joy);
              });

      sub_clearing_ =
          this->create_subscription<std_msgs::msg::Float32>(
              "map_clearing",
              5,
              [this](const std_msgs::msg::Float32::ConstSharedPtr dis) -> void
              {
                this->clearingHandler(dis);
              });

      pub_laser_cloud_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>(
              "terrain_map",
              5);

      // Create timer for 100Hz processing loop (10ms period)
      //100Hz处理循环的定时器（10毫秒周期）
      timer_ = this->create_wall_timer(
          10ms,
          [this]()->void
          {
            this->timerCallback();
          });
    }

  private:
    // ==================== 回调函数 Callback Functions ====================

    /**
     * @brief 里程计回调函数 - 更新机器人位姿状态
     * 
     * @param odom 里程计消息，包含机器人的位置和姿态信息
     * 
     * @details
     * 该函数从里程计消息中提取并更新以下状态：
     * - 机器人位置 (vehicleX, vehicleY, vehicleZ)
     * - 机器人姿态角 (vehicleRoll, vehiclePitch, vehicleYaw)
     * - 预计算的三角函数值（用于坐标变换优化）
     * - 初始化无数据障碍检测的参考位置
     * 
     * @note 姿态角通过四元数转换为欧拉角（RPY）
     * @note 三角函数值预计算以减少主处理循环的计算负担
     */
    void odometryHandler(
        const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
      double roll, pitch, yaw;
      geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
      tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
          .getRPY(roll, pitch, yaw);

      vehicleRoll = roll;
      vehiclePitch = pitch;
      vehicleYaw = yaw;
      vehicleX = odom->pose.pose.position.x;
      vehicleY = odom->pose.pose.position.y;
      vehicleZ = odom->pose.pose.position.z;

      sinVehicleRoll = sin(vehicleRoll);
      cosVehicleRoll = cos(vehicleRoll);
      sinVehiclePitch = sin(vehiclePitch);
      cosVehiclePitch = cos(vehiclePitch);
      sinVehicleYaw = sin(vehicleYaw);
      cosVehicleYaw = cos(vehicleYaw);

      if (no_data_inited_ == 0)
      {
        vehicleXRec = vehicleX;
        vehicleYRec = vehicleY;
        no_data_inited_ = 1;
      }
      if (no_data_inited_ == 1)
      {
        float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                         (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
        if (dis >= no_decay_dis_)
          no_data_inited_ = 2;
      }
    }

    /**
     * @brief 点云回调函数 - 接收并预处理激光雷达点云
     * 
     * @param laserCloud2 激光雷达点云消息（ROS2格式）
     * 
     * @details
     * 该函数执行点云的初步处理：
     * 1. **时间戳管理**：提取点云时间戳，初始化系统时钟
     * 2. **格式转换**：将 ROS2 消息转换为 PCL 点云
     * 3. **空间裁剪**：过滤出机器人周围感兴趣区域的点
     *    - 高度范围：min_rel_z_ 到 max_rel_z_（相对于机器人）
     *    - 水平范围：terrainVoxelSize * (terrainVoxelHalfWidth + 1)
     * 4. **时间戳标记**：为每个点标记相对于系统初始化的时间
     * 5. **标志设置**：设置 new_laser_cloud_ 标志触发主处理
     * 
     * @note 裁剪后的点云存储在 laserCloudCrop 中
     * @note 采用动态高度范围（与距离成正比）以适应地形起伏
     */
    // registered laser scan callback function
    void laserCloudHandler(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
    {
      laser_cloud_time_ = rclcpp::Time(laserCloud2->header.stamp).seconds();
      if (!system_inited_)
      {
        system_init_time_ = laser_cloud_time_;
        system_inited_ = true;
      }

      laserCloud->clear();
      pcl::fromROSMsg(*laserCloud2, *laserCloud);

      pcl::PointXYZI point;
      laserCloudCrop->clear();
      int laserCloudSize = laserCloud->points.size();
      for (int i = 0; i < laserCloudSize; i++)
      {
        point = laserCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                         (pointY - vehicleY) * (pointY - vehicleY));
        if (pointZ - vehicleZ > min_rel_z_ - dis_ratio_z_ * dis &&
            pointZ - vehicleZ < max_rel_z_ + dis_ratio_z_ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1))
        {
          point.x = pointX;
          point.y = pointY;
          point.z = pointZ;
          point.intensity = laser_cloud_time_ - system_init_time_;
          laserCloudCrop->push_back(point);
        }
      }

      new_laser_cloud_ = true;
    }

    /**
     * @brief 手柄回调函数 - 处理手动清空地图指令
     * 
     * @param joy 手柄消息
     * 
     * @details
     * 监听手柄的按钮5（索引5），按下时触发地图清空：
     * - 重置无数据障碍检测状态 (no_data_inited_ = 0)
     * - 设置清空标志 (clearing_cloud_ = true)
     * 
     * @note 通常用于调试或重置场景
     */
    // joystick callback function
    void joystickHandler(
        const sensor_msgs::msg::Joy::ConstSharedPtr joy)
    {
      if (joy->buttons[5] > 0.5)
      {
        no_data_inited_ = 0;
        clearing_cloud_ = true;
      }
    }

    /**
     * @brief 地图清空回调函数 - 处理自动清空指令
     * 
     * @param dis 清空距离消息
     * 
     * @details
     * 接收并应用新的清空距离参数：
     * - 更新 clearing_dis_（清空半径）
     * - 重置无数据障碍检测状态
     * - 触发下一次处理时清空指定半径内的旧数据
     * 
     * @note 与手柄清空不同，该方法支持动态调整清空范围
     */
    // cloud clearing callback function
    void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
    {
      no_data_inited_ = 0;
      clearing_dis_ = dis->data;
      clearing_cloud_ = true;
    }

    /**
     * @brief 定时器回调函数 - 主处理循环（100Hz）
     * 
     * @details
     * 该函数是地形分析的核心处理逻辑，每10ms执行一次，包含以下步骤：
     * 
     * **1. 体素网格滚动更新**：
     *    - 根据机器人移动动态调整地形网格原点
     *    - 移出旧体素，清空并重用内存
     *    - 保持机器人始终在网格中心区域
     * 
     * **2. 点云堆叠与体素化**：
     *    - 将新点云按位置映射到对应体素
     *    - 累积多帧点云以提高密度
     *    - 记录体素更新次数和时间
     * 
     * **3. 体素降采样与衰减**：
     *    - 对达到更新阈值的体素执行降采样
     *    - 应用时间衰减规则移除过期点
     *    - 根据距离和时间保留有效数据
     * 
     * **4. 地面高程估计**：
     *    - 将点云投影到平面体素网格
     *    - 使用分位数或最小值法估计地面高度
     *    - 限制地面抬升以过滤异常值
     * 
     * **5. 动态障碍物过滤**（可选）：
     *    - 检测高于地面且在特定视场角内的点
     *    - 累积障碍物点数判断是否为动态物体
     *    - 过滤掉可能的移动障碍物
     * 
     * **6. 可通行性评估**：
     *    - 计算点相对于地面的高度差
     *    - 过滤出车辆高度范围内的障碍
     *    - 标记无数据区域为潜在障碍
     * 
     * **7. 地形图发布**：
     *    - 将带高程信息的点云转换为ROS2消息
     *    - 设置正确的时间戳和坐标系
     *    - 通过 terrain_map 话题发布
     * 
     * @note 仅在 new_laser_cloud_ 为 true 时执行处理
     * @note 所有计算均在 "odom" 坐标系下进行
     * @warning 确保处理时间 < 10ms 以避免定时器堆积
     */
    void timerCallback()
    {
      if (new_laser_cloud_)
      {
        new_laser_cloud_ = false;

        // terrain voxel roll over
        float terrainVoxelCenX = terrainVoxelSize * terrain_voxel_shift_x_;
        float terrainVoxelCenY = terrainVoxelSize * terrain_voxel_shift_y_;

        while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
        {
          for (int indY = 0; indY < terrainVoxelWidth; indY++)
          {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                  indY];
            for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
            {
              terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                  terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
            }
            terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
            terrainVoxelCloud[indY]->clear();
          }
          terrain_voxel_shift_x_--;
          terrainVoxelCenX = terrainVoxelSize * terrain_voxel_shift_x_;
        }

        while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
        {
          for (int indY = 0; indY < terrainVoxelWidth; indY++)
          {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[indY];
            for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
            {
              terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                  terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
            }
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                              indY] = terrainVoxelCloudPtr;
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
                ->clear();
          }
          terrain_voxel_shift_x_++;
          terrainVoxelCenX = terrainVoxelSize * terrain_voxel_shift_x_;
        }

        while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
        {
          for (int indX = 0; indX < terrainVoxelWidth; indX++)
          {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * indX +
                                  (terrainVoxelWidth - 1)];
            for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
            {
              terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                  terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
            }
            terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
            terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
          }
          terrain_voxel_shift_y_--;
          terrainVoxelCenY = terrainVoxelSize * terrain_voxel_shift_y_;
        }

        while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
        {
          for (int indX = 0; indX < terrainVoxelWidth; indX++)
          {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[terrainVoxelWidth * indX];
            for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
            {
              terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                  terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
            }
            terrainVoxelCloud[terrainVoxelWidth * indX +
                              (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
            terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
                ->clear();
          }
          terrain_voxel_shift_y_++;
          terrainVoxelCenY = terrainVoxelSize * terrain_voxel_shift_y_;
        }

        // stack registered laser scans
        pcl::PointXYZI point;
        int laserCloudCropSize = laserCloudCrop->points.size();
        for (int i = 0; i < laserCloudCropSize; i++)
        {
          point = laserCloudCrop->points[i];

          int indX =
              static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) /
                               terrainVoxelSize) +
              terrainVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                               terrainVoxelSize) +
              terrainVoxelHalfWidth;

          if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
              indY < terrainVoxelWidth)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
          }
        }

        for (int ind = 0; ind < kTerrainVoxelNum; ind++)
        {
          if (terrainVoxelUpdateNum[ind] >= voxel_point_update_thre_ ||
              laser_cloud_time_ - system_init_time_ - terrainVoxelUpdateTime[ind] >=
                  voxel_time_update_thre_ ||
              clearing_cloud_)
          {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                terrainVoxelCloud[ind];

            laserCloudDwz->clear();
            downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
            downSizeFilter.filter(*laserCloudDwz);

            terrainVoxelCloudPtr->clear();
            int laserCloudDwzSize = laserCloudDwz->points.size();
            for (int i = 0; i < laserCloudDwzSize; i++)
            {
              point = laserCloudDwz->points[i];
              float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                               (point.y - vehicleY) * (point.y - vehicleY));
              if (point.z - vehicleZ > min_rel_z_ - dis_ratio_z_ * dis &&
                  point.z - vehicleZ < max_rel_z_ + dis_ratio_z_ * dis &&
                  (laser_cloud_time_ - system_init_time_ - point.intensity <
                       decay_time_ ||
                   dis < no_decay_dis_) &&
                  !(dis < clearing_dis_ && clearing_cloud_))
              {
                terrainVoxelCloudPtr->push_back(point);
              }
            }

            terrainVoxelUpdateNum[ind] = 0;
            terrainVoxelUpdateTime[ind] = laser_cloud_time_ - system_init_time_;
          }
        }

        terrainCloud->clear();
        for (int indX = terrainVoxelHalfWidth - 5;
             indX <= terrainVoxelHalfWidth + 5; indX++)
        {
          for (int indY = terrainVoxelHalfWidth - 5;
               indY <= terrainVoxelHalfWidth + 5; indY++)
          {
            *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
          }
        }

        // estimate ground and compute elevation for each point
        for (int i = 0; i < kPlanarVoxelNum; i++)
        {
          planarVoxelElev[i] = 0;
          planarVoxelEdge[i] = 0;
          planarVoxelDyObs[i] = 0;
          planarPointElev[i].clear();
        }

        int terrainCloudSize = terrainCloud->points.size();
        for (int i = 0; i < terrainCloudSize; i++)
        {
          point = terrainCloud->points[i];

          int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                      planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                      planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (point.z - vehicleZ > min_rel_z_ && point.z - vehicleZ < max_rel_z_)
          {
            for (int dX = -1; dX <= 1; dX++)
            {
              for (int dY = -1; dY <= 1; dY++)
              {
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                    indY + dY >= 0 && indY + dY < planarVoxelWidth)
                {
                  planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                      .push_back(point.z);
                }
              }
            }
          }

          if (clear_dynamic_obs_)
          {
            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                indY < planarVoxelWidth)
            {
              float pointX1 = point.x - vehicleX;
              float pointY1 = point.y - vehicleY;
              float pointZ1 = point.z - vehicleZ;

              float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
              if (dis1 > min_dy_obs_dis_)
              {
                float angle1 = atan2(pointZ1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
                if (angle1 > min_dy_obs_angle_)
                {
                  float pointX2 =
                      pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                  float pointY2 =
                      -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                  float pointZ2 = pointZ1;

                  float pointX3 =
                      pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                  float pointY3 = pointY2;
                  float pointZ3 =
                      pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                  float pointX4 = pointX3;
                  float pointY4 =
                      pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                  float pointZ4 =
                      -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                  float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                  float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
                  if ((angle4 > min_dy_obs_vfov_ && angle4 < max_dy_obs_vfov_) ||
                      fabs(pointZ4) < abs_dy_obs_rel_z_thre_)
                  {
                    planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                  }
                }
              }
              else
              {
                planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                    min_dy_obs_point_num_;
              }
            }
          }
        }

        if (clear_dynamic_obs_)
        {
          for (int i = 0; i < laserCloudCropSize; i++)
          {
            point = laserCloudCrop->points[i];

            int indX =
                static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                 planarVoxelSize) +
                planarVoxelHalfWidth;
            int indY =
                static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                 planarVoxelSize) +
                planarVoxelHalfWidth;

            if (point.x - vehicleX + planarVoxelSize / 2 < 0)
              indX--;
            if (point.y - vehicleY + planarVoxelSize / 2 < 0)
              indY--;

            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                indY < planarVoxelWidth)
            {
              float pointX1 = point.x - vehicleX;
              float pointY1 = point.y - vehicleY;
              float pointZ1 = point.z - vehicleZ;

              float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
              float angle1 = atan2(pointZ1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
              if (angle1 > min_dy_obs_angle_)
              {
                planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
              }
            }
          }
        }

        if (use_sorting_)
        {
          for (int i = 0; i < kPlanarVoxelNum; i++)
          {
            int planarPointElevSize = planarPointElev[i].size();
            if (planarPointElevSize > 0)
            {
              sort(planarPointElev[i].begin(), planarPointElev[i].end());

              int quantileID = static_cast<int>(quantile_z_ * planarPointElevSize);
              if (quantileID < 0)
                quantileID = 0;
              else if (quantileID >= planarPointElevSize)
                quantileID = planarPointElevSize - 1;

              if (planarPointElev[i][quantileID] >
                      planarPointElev[i][0] + max_ground_lift_ &&
                  limit_ground_lift_)
              {
                planarVoxelElev[i] = planarPointElev[i][0] + max_ground_lift_;
              }
              else
              {
                planarVoxelElev[i] = planarPointElev[i][quantileID];
              }
            }
          }
        }
        else
        {
          for (int i = 0; i < kPlanarVoxelNum; i++)
          {
            int planarPointElevSize = planarPointElev[i].size();
            if (planarPointElevSize > 0)
            {
              float minZ = 1000.0;
              int minID = -1;
              for (int j = 0; j < planarPointElevSize; j++)
              {
                if (planarPointElev[i][j] < minZ)
                {
                  minZ = planarPointElev[i][j];
                  minID = j;
                }
              }

              if (minID != -1)
              {
                planarVoxelElev[i] = planarPointElev[i][minID];
              }
            }
          }
        }

        terrainCloudElev->clear();
        int terrainCloudElevSize = 0;
        for (int i = 0; i < terrainCloudSize; i++)
        {
          point = terrainCloud->points[i];
          if (point.z - vehicleZ > min_rel_z_ && point.z - vehicleZ < max_rel_z_)
          {
            int indX =
                static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                 planarVoxelSize) +
                planarVoxelHalfWidth;
            int indY =
                static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                 planarVoxelSize) +
                planarVoxelHalfWidth;

            if (point.x - vehicleX + planarVoxelSize / 2 < 0)
              indX--;
            if (point.y - vehicleY + planarVoxelSize / 2 < 0)
              indY--;

            if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                indY < planarVoxelWidth)
            {
              if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                      min_dy_obs_point_num_ ||
                  !clear_dynamic_obs_)
              {
                float disZ =
                    point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                if (consider_drop_)
                  disZ = fabs(disZ);
                int planarPointElevSize =
                    planarPointElev[planarVoxelWidth * indX + indY].size();
                if (disZ >= 0 && disZ < vehicle_height_ &&
                    planarPointElevSize >= min_block_point_num_)
                {
                  terrainCloudElev->push_back(point);
                  terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                  terrainCloudElevSize++;
                }
              }
            }
          }
        }

        if (no_data_obstacle_ && no_data_inited_ == 2)
        {
          for (int i = 0; i < kPlanarVoxelNum; i++)
          {
            int planarPointElevSize = planarPointElev[i].size();
            if (planarPointElevSize < min_block_point_num_)
            {
              planarVoxelEdge[i] = 1;
            }
          }

          for (int noDataBlockSkipCount = 0;
               noDataBlockSkipCount < no_data_block_skip_num_;
               noDataBlockSkipCount++)
          {
            for (int i = 0; i < kPlanarVoxelNum; i++)
            {
              if (planarVoxelEdge[i] >= 1)
              {
                int indX = static_cast<int>(i / planarVoxelWidth);
                int indY = i % planarVoxelWidth;
                bool edgeVoxel = false;
                for (int dX = -1; dX <= 1; dX++)
                {
                  for (int dY = -1; dY <= 1; dY++)
                  {
                    if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                        indY + dY >= 0 && indY + dY < planarVoxelWidth)
                    {
                      if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                          dY] < planarVoxelEdge[i])
                      {
                        edgeVoxel = true;
                      }
                    }
                  }
                }

                if (!edgeVoxel)
                  planarVoxelEdge[i]++;
              }
            }
          }

          for (int i = 0; i < kPlanarVoxelNum; i++)
          {
            if (planarVoxelEdge[i] > no_data_block_skip_num_)
            {
              int indX = static_cast<int>(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;

              point.x =
                  planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
              point.y =
                  planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
              point.z = vehicleZ;
              point.intensity = vehicle_height_;

              point.x -= planarVoxelSize / 4.0;
              point.y -= planarVoxelSize / 4.0;
              terrainCloudElev->push_back(point);

              point.x += planarVoxelSize / 2.0;
              terrainCloudElev->push_back(point);

              point.y += planarVoxelSize / 2.0;
              terrainCloudElev->push_back(point);

              point.x -= planarVoxelSize / 2.0;
              terrainCloudElev->push_back(point);
            }
          }
        }

        clearing_cloud_ = false;

        // publish points with elevation
        sensor_msgs::msg::PointCloud2 terrainCloud2;
        pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
        terrainCloud2.header.stamp =
            rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
        terrainCloud2.header.frame_id = "odom";
        pub_laser_cloud_->publish(terrainCloud2);
      }
    }

    // ==================== 成员变量 Member Variables ====================

    // -------- 参数存储 Parameter Storage --------
    
    /// @brief 扫描体素大小 (米)，用于点云降采样
    double scan_voxel_size_;
    
    /// @brief 点云衰减时间 (秒)，超过此时间的点将被移除
    double decay_time_;
    
    /// @brief 无衰减距离 (米)，此范围内的点不受时间衰减影响
    double no_decay_dis_;
    
    /// @brief 清空距离 (米)，执行清空时的有效半径
    double clearing_dis_;
    
    /// @brief 清空标志，true时在下次处理中清空点云
    bool clearing_cloud_;
    
    /// @brief 是否使用排序法估计地面高程（false则使用最小值法）
    bool use_sorting_;
    
    /// @brief 地面高程分位数 (0-1)，用于排序法时选择的分位点
    double quantile_z_;
    
    /// @brief 是否考虑下降沿（凹陷地形）作为障碍
    bool consider_drop_;
    
    /// @brief 是否限制地面抬升幅度
    bool limit_ground_lift_;
    
    /// @brief 最大地面抬升 (米)，防止异常高点影响地面估计
    double max_ground_lift_;
    
    /// @brief 是否启用动态障碍物清除功能
    bool clear_dynamic_obs_;
    
    /// @brief 动态障碍物最小检测距离 (米)
    double min_dy_obs_dis_;
    
    /// @brief 动态障碍物最小角度 (度)，相对于水平面
    double min_dy_obs_angle_;
    
    /// @brief 动态障碍物最小相对高度 (米)
    double min_dy_obs_rel_z_;
    
    /// @brief 动态障碍物绝对相对高度阈值 (米)
    double abs_dy_obs_rel_z_thre_;
    
    /// @brief 动态障碍物最小垂直视场角 (度)
    double min_dy_obs_vfov_;
    
    /// @brief 动态障碍物最大垂直视场角 (度)
    double max_dy_obs_vfov_;
    
    /// @brief 判定为动态障碍物所需的最小点数
    int min_dy_obs_point_num_;
    
    /// @brief 是否将无数据区域标记为障碍物
    bool no_data_obstacle_;
    
    /// @brief 无数据块跳过数量，用于膨胀无数据区域
    int no_data_block_skip_num_;
    
    /// @brief 体素有效所需的最小点数
    int min_block_point_num_;
    
    /// @brief 车辆高度 (米)，用于确定可通行性判断范围
    double vehicle_height_;
    
    /// @brief 体素点数更新阈值，达到此值触发降采样
    int voxel_point_update_thre_;
    
    /// @brief 体素时间更新阈值 (秒)，超过此时间强制更新
    double voxel_time_update_thre_;
    
    /// @brief 最小相对高度 (米)，低于此值的点被过滤
    double min_rel_z_;
    
    /// @brief 最大相对高度 (米)，高于此值的点被过滤
    double max_rel_z_;
    
    /// @brief 距离-高度比例系数，用于动态调整高度范围
    double dis_ratio_z_;

    // -------- 状态变量 State Variables (蛇形命名) --------
    
    /// @brief 系统初始化标志
    bool system_inited_;
    
    /// @brief 无数据障碍检测初始化状态 (0:未初始化, 1:已记录起点, 2:已移动足够距离)
    int no_data_inited_;
    
    /// @brief 新点云到达标志，触发主处理循环
    bool new_laser_cloud_;
    
    /// @brief 系统初始化时间戳 (秒)
    double system_init_time_;
    
    /// @brief 最新点云时间戳 (秒)
    double laser_cloud_time_;

    // -------- 几何计算变量 Geometric Variables (驼峰命名) --------
    
    /// @brief 机器人位置 - X坐标 (米)
    double vehicleX;
    /// @brief 机器人位置 - Y坐标 (米)
    double vehicleY;
    /// @brief 机器人位置 - Z坐标 (米)
    double vehicleZ;
    
    /// @brief 机器人姿态 - 横滚角 Roll (弧度)
    double vehicleRoll;
    /// @brief 机器人姿态 - 俯仰角 Pitch (弧度)
    double vehiclePitch;
    /// @brief 机器人姿态 - 偏航角 Yaw (弧度)
    double vehicleYaw;
    
    /// @brief 预计算的 sin(Roll)，用于坐标变换优化
    double sinVehicleRoll;
    /// @brief 预计算的 cos(Roll)，用于坐标变换优化
    double cosVehicleRoll;
    /// @brief 预计算的 sin(Pitch)，用于坐标变换优化
    double sinVehiclePitch;
    /// @brief 预计算的 cos(Pitch)，用于坐标变换优化
    double cosVehiclePitch;
    /// @brief 预计算的 sin(Yaw)，用于坐标变换优化
    double sinVehicleYaw;
    /// @brief 预计算的 cos(Yaw)，用于坐标变换优化
    double cosVehicleYaw;
    
    /// @brief 记录的机器人X坐标，用于无数据障碍检测
    double vehicleXRec;
    /// @brief 记录的机器人Y坐标，用于无数据障碍检测
    double vehicleYRec;

    // -------- 地形体素参数 Terrain Voxel Parameters --------
    
    /// @brief 地形体素网格X方向偏移量（体素单位）
    int terrain_voxel_shift_x_;
    /// @brief 地形体素网格Y方向偏移量（体素单位）
    int terrain_voxel_shift_y_;
    
    /// @brief 地形体素大小 (米) - 每个体素边长1.0米
    constexpr static float terrainVoxelSize = 1.0;
    /// @brief 地形体素网格宽度 (体素数) - 21x21网格
    constexpr static int terrainVoxelWidth = 21;
    /// @brief 地形体素网格半宽 - 网格中心到边缘的体素数
    constexpr static int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
    /// @brief 地形体素总数 - 21x21 = 441个体素
    constexpr static int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

    // -------- 平面体素参数 Planar Voxel Parameters --------
    
    /// @brief 平面体素大小 (米) - 用于精细地面高程估计
    constexpr static float planarVoxelSize = 0.2;
    /// @brief 平面体素网格宽度 (体素数) - 51x51网格
    constexpr static int planarVoxelWidth = 51;
    /// @brief 平面体素网格半宽
    constexpr static int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
    /// @brief 平面体素总数 - 51x51 = 2601个体素
    constexpr static int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

    // -------- 体素数据数组 Voxel Data Arrays --------
    
    /// @brief 地形体素更新次数数组，记录每个体素累积的点数
    int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
    /// @brief 地形体素更新时间数组，记录每个体素最后更新时间
    float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
    /// @brief 平面体素高程数组，存储每个体素的估计地面高度
    float planarVoxelElev[kPlanarVoxelNum] = {0};
    /// @brief 平面体素边缘标记数组，用于无数据障碍检测
    int planarVoxelEdge[kPlanarVoxelNum] = {0};
    /// @brief 平面体素动态障碍物计数数组
    int planarVoxelDyObs[kPlanarVoxelNum] = {0};
    /// @brief 平面体素点高程向量数组，存储每个体素内所有点的Z值
    std::vector<float> planarPointElev[kPlanarVoxelNum];

    // -------- 点云数据 Point Cloud Data --------
    
    /// @brief 原始激光雷达点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
    /// @brief 裁剪后的点云（ROI内）
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
    /// @brief 降采样后的点云缓冲
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
    /// @brief 地形点云（用于高程计算）
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
    /// @brief 带高程信息的地形点云（最终输出）
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
    /// @brief 地形体素点云数组 - 21x21个体素容器
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

    /// @brief PCL体素网格滤波器，用于点云降采样
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

    // -------- ROS2 接口 ROS2 Interfaces --------
    
    /// @brief 里程计订阅器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
    /// @brief 点云订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
    /// @brief 手柄订阅器
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
    /// @brief 地图清空订阅器
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_clearing_;
    /// @brief 地形图发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
    /// @brief 主处理定时器 (100Hz)
    rclcpp::TimerBase::SharedPtr timer_;
  };

} // namespace terrain_analysis

/**
 * @brief 组件注册宏 - 将 TerrainAnalysisNode 注册为 ROS2 组件
 * 
 * @details
 * 该宏使节点可以被动态加载到组件容器中，支持：
 * - 进程内通信（零拷贝）
 * - 运行时动态加载/卸载
 * - 与其他组件共享执行器和线程池
 * 
 * @note 组件化部署是 ROS2 的推荐做法，可显著提升性能
 */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisNode)