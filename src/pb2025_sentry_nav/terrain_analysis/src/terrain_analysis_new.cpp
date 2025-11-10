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

#include "terrain_analysis/terrain_analysis_new.hpp"

namespace terrain_analysis
{

TerrainAnalysisNode::TerrainAnalysisNode(const rclcpp::NodeOptions &options)
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

  std::fill(std::begin(terrainVoxelUpdateNum), std::end(terrainVoxelUpdateNum), 0);
  std::fill(std::begin(terrainVoxelUpdateTime), std::end(terrainVoxelUpdateTime), 0.0f);
  std::fill(std::begin(planarVoxelElev), std::end(planarVoxelElev), 0.0f);
  std::fill(std::begin(planarVoxelEdge), std::end(planarVoxelEdge), 0);
  std::fill(std::begin(planarVoxelDyObs), std::end(planarVoxelDyObs), 0);
  for (auto &elev_vec : planarPointElev)
  {
    elev_vec.clear();
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
  // 100Hz处理循环的定时器（10毫秒周期）
  timer_ = this->create_wall_timer(
      10ms,
      [this]() -> void
      {
        this->timerCallback();
      });
}

void TerrainAnalysisNode::odometryHandler(
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

void TerrainAnalysisNode::laserCloudHandler(
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

void TerrainAnalysisNode::joystickHandler(
    const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons[5] > 0.5)
  {
    no_data_inited_ = 0;
    clearing_cloud_ = true;
  }
    }

void TerrainAnalysisNode::clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
{
  no_data_inited_ = 0;
  clearing_dis_ = dis->data;
  clearing_cloud_ = true;
}

void TerrainAnalysisNode::timerCallback()
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
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisNode)