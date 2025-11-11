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

#include "terrain_analysis_ext/terrain_analysis_ext_new.hpp"


using namespace std::chrono_literals;

namespace terrain_analysis
{

TerrainAnalysisExtNode::TerrainAnalysisExtNode(const rclcpp::NodeOptions &options)
: Node("terrain_analysis_ext", options),
	scan_voxel_size_(0.1),
	decay_time_(10.0),
	no_decay_dis_(0.0),
	clearing_dis_(30.0),
	clearing_cloud_(false),
	use_sorting_(false),
	quantile_z_(0.25),
	vehicle_height_(1.5),
	voxel_point_update_thre_(100),
	voxel_time_update_thre_(2.0),
	lower_bound_z_(-1.5),
	upper_bound_z_(1.0),
	dis_ratio_z_(0.1),
	check_terrain_conn_(true),
	terrain_under_vehicle_(-0.75),
	terrain_conn_thre_(0.5),
	ceiling_filtering_thre_(2.0),
	local_terrain_map_radius_(4.0),
	system_inited_(false),
	new_laser_cloud_(false),
	system_init_time_(0.0),
	laser_cloud_time_(0.0),
	vehicle_roll_(0.0),
	vehicle_pitch_(0.0),
	vehicle_yaw_(0.0),
	vehicle_x_(0.0),
	vehicle_y_(0.0),
	vehicle_z_(0.0),
	terrain_voxel_shift_x_(0),
	terrain_voxel_shift_y_(0),
	laser_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
	laser_cloud_crop_(new pcl::PointCloud<pcl::PointXYZI>()),
	laser_cloud_downsampled_(new pcl::PointCloud<pcl::PointXYZI>()),
	terrain_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
	terrain_cloud_elev_(new pcl::PointCloud<pcl::PointXYZI>()),
	terrain_cloud_local_(new pcl::PointCloud<pcl::PointXYZI>())
{
	// Declare parameters with snake_case naming to match terrain_analysis style
	this->declare_parameter<double>("scan_voxel_size", scan_voxel_size_);
	this->declare_parameter<double>("decay_time", decay_time_);
	this->declare_parameter<double>("no_decay_dis", no_decay_dis_);
	this->declare_parameter<double>("clearing_dis", clearing_dis_);
	this->declare_parameter<bool>("use_sorting", use_sorting_);
	this->declare_parameter<double>("quantile_z", quantile_z_);
	this->declare_parameter<double>("vehicle_height", vehicle_height_);
	this->declare_parameter<int>("voxel_point_update_thre", voxel_point_update_thre_);
	this->declare_parameter<double>("voxel_time_update_thre", voxel_time_update_thre_);
	this->declare_parameter<double>("lower_bound_z", lower_bound_z_);
	this->declare_parameter<double>("upper_bound_z", upper_bound_z_);
	this->declare_parameter<double>("dis_ratio_z", dis_ratio_z_);
	this->declare_parameter<bool>("check_terrain_conn", check_terrain_conn_);
	this->declare_parameter<double>("terrain_under_vehicle", terrain_under_vehicle_);
	this->declare_parameter<double>("terrain_conn_thre", terrain_conn_thre_);
	this->declare_parameter<double>("ceiling_filtering_thre", ceiling_filtering_thre_);
	this->declare_parameter<double>("local_terrain_map_radius", local_terrain_map_radius_);

	// Load parameters
	this->get_parameter("scan_voxel_size", scan_voxel_size_);
	this->get_parameter("decay_time", decay_time_);
	this->get_parameter("no_decay_dis", no_decay_dis_);
	this->get_parameter("clearing_dis", clearing_dis_);
	this->get_parameter("use_sorting", use_sorting_);
	this->get_parameter("quantile_z", quantile_z_);
	this->get_parameter("vehicle_height", vehicle_height_);
	this->get_parameter("voxel_point_update_thre", voxel_point_update_thre_);
	this->get_parameter("voxel_time_update_thre", voxel_time_update_thre_);
	this->get_parameter("lower_bound_z", lower_bound_z_);
	this->get_parameter("upper_bound_z", upper_bound_z_);
	this->get_parameter("dis_ratio_z", dis_ratio_z_);
	this->get_parameter("check_terrain_conn", check_terrain_conn_);
	this->get_parameter("terrain_under_vehicle", terrain_under_vehicle_);
	this->get_parameter("terrain_conn_thre", terrain_conn_thre_);
	this->get_parameter("ceiling_filtering_thre", ceiling_filtering_thre_);
	this->get_parameter("local_terrain_map_radius", local_terrain_map_radius_);

	RCLCPP_INFO(get_logger(), "terrain_analysis_ext parameters loaded.");

	// Initialise voxel containers
	for (int i = 0; i < kTerrainVoxelNum; ++i) {
		terrain_voxel_cloud_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
		terrain_voxel_update_num_[i] = 0;
		terrain_voxel_update_time_[i] = 0.0f;
	}

	std::fill(std::begin(planar_voxel_elev_), std::end(planar_voxel_elev_), 0.0f);
	std::fill(std::begin(planar_voxel_conn_), std::end(planar_voxel_conn_), 0);
	for (auto &vec : planar_point_elev_) {
		vec.clear();
	}

	downsize_filter_.setLeafSize(
		static_cast<float>(scan_voxel_size_),
		static_cast<float>(scan_voxel_size_),
		static_cast<float>(scan_voxel_size_));

	auto sensor_qos = rclcpp::SensorDataQoS();

	sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"lidar_odometry",
		sensor_qos,
		std::bind(&TerrainAnalysisExtNode::odometryHandler, this, std::placeholders::_1));

	sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"registered_scan",
		sensor_qos,
		std::bind(&TerrainAnalysisExtNode::laserCloudHandler, this, std::placeholders::_1));

	sub_terrain_cloud_local_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"terrain_map",
		rclcpp::QoS(2),
		std::bind(&TerrainAnalysisExtNode::terrainCloudLocalHandler, this, std::placeholders::_1));

	sub_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"joy",
		rclcpp::QoS(5),
		std::bind(&TerrainAnalysisExtNode::joystickHandler, this, std::placeholders::_1));

	sub_clearing_ = this->create_subscription<std_msgs::msg::Float32>(
		"cloud_clearing",
		rclcpp::QoS(5),
		std::bind(&TerrainAnalysisExtNode::clearingHandler, this, std::placeholders::_1));

	pub_terrain_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		"terrain_map_ext",
		rclcpp::QoS(2));

	timer_ = this->create_wall_timer(
		10ms,
		std::bind(&TerrainAnalysisExtNode::timerCallback, this));
}

void TerrainAnalysisExtNode::odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
	double roll, pitch, yaw;
	const auto &geo_quat = odom->pose.pose.orientation;
	tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

	vehicle_roll_ = roll;
	vehicle_pitch_ = pitch;
	vehicle_yaw_ = yaw;
	vehicle_x_ = odom->pose.pose.position.x;
	vehicle_y_ = odom->pose.pose.position.y;
	vehicle_z_ = odom->pose.pose.position.z;
}

void TerrainAnalysisExtNode::laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud_msg)
{
	laser_cloud_time_ = rclcpp::Time(laser_cloud_msg->header.stamp).seconds();

	if (!system_inited_) {
		system_init_time_ = laser_cloud_time_;
		system_inited_ = true;
	}

	laser_cloud_->clear();
	pcl::fromROSMsg(*laser_cloud_msg, *laser_cloud_);

	pcl::PointXYZI point;
	laser_cloud_crop_->clear();
	const int laser_cloud_size = static_cast<int>(laser_cloud_->points.size());
	for (int i = 0; i < laser_cloud_size; ++i) {
		point = laser_cloud_->points[i];

		const float point_x = point.x;
		const float point_y = point.y;
		const float point_z = point.z;

		const float dis = std::hypot(point_x - vehicle_x_, point_y - vehicle_y_);
		if (point_z - vehicle_z_ > lower_bound_z_ - dis_ratio_z_ * dis &&
				point_z - vehicle_z_ < upper_bound_z_ + dis_ratio_z_ * dis &&
				dis < kTerrainVoxelSize * (kTerrainVoxelHalfWidth + 1))
		{
			point.x = point_x;
			point.y = point_y;
			point.z = point_z;
			point.intensity = laser_cloud_time_ - system_init_time_;
			laser_cloud_crop_->push_back(point);
		}
	}

	new_laser_cloud_ = true;
}

void TerrainAnalysisExtNode::terrainCloudLocalHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_cloud_msg)
{
	terrain_cloud_local_->clear();
	pcl::fromROSMsg(*terrain_cloud_msg, *terrain_cloud_local_);
}

void TerrainAnalysisExtNode::joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
	if (!joy->buttons.empty() && joy->buttons[5] > 0.5) {
		clearing_cloud_ = true;
	}
}

void TerrainAnalysisExtNode::clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr distance)
{
	clearing_dis_ = distance->data;
	clearing_cloud_ = true;
}

void TerrainAnalysisExtNode::timerCallback()
{
	if (!new_laser_cloud_) {
		return;
	}
	new_laser_cloud_ = false;

	// ===== Terrain voxel roll over =====
	float terrain_voxel_center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
	float terrain_voxel_center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;

	while (vehicle_x_ - terrain_voxel_center_x < -kTerrainVoxelSize) {
		for (int ind_y = 0; ind_y < kTerrainVoxelWidth; ++ind_y) {
			auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + ind_y];
			for (int ind_x = kTerrainVoxelWidth - 1; ind_x >= 1; --ind_x) {
				terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y] =
					terrain_voxel_cloud_[kTerrainVoxelWidth * (ind_x - 1) + ind_y];
			}
			terrain_voxel_cloud_[ind_y] = terrain_voxel_cloud_ptr;
			terrain_voxel_cloud_[ind_y]->clear();
		}
		--terrain_voxel_shift_x_;
		terrain_voxel_center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
	}

	while (vehicle_x_ - terrain_voxel_center_x > kTerrainVoxelSize) {
		for (int ind_y = 0; ind_y < kTerrainVoxelWidth; ++ind_y) {
			auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind_y];
			for (int ind_x = 0; ind_x < kTerrainVoxelWidth - 1; ++ind_x) {
				terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y] =
					terrain_voxel_cloud_[kTerrainVoxelWidth * (ind_x + 1) + ind_y];
			}
			terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + ind_y] = terrain_voxel_cloud_ptr;
			terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + ind_y]->clear();
		}
		++terrain_voxel_shift_x_;
		terrain_voxel_center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
	}

	while (vehicle_y_ - terrain_voxel_center_y < -kTerrainVoxelSize) {
		for (int ind_x = 0; ind_x < kTerrainVoxelWidth; ++ind_x) {
			auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + (kTerrainVoxelWidth - 1)];
			for (int ind_y = kTerrainVoxelWidth - 1; ind_y >= 1; --ind_y) {
				terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y] =
					terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + (ind_y - 1)];
			}
			terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x] = terrain_voxel_cloud_ptr;
			terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x]->clear();
		}
		--terrain_voxel_shift_y_;
		terrain_voxel_center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;
	}

	while (vehicle_y_ - terrain_voxel_center_y > kTerrainVoxelSize) {
		for (int ind_x = 0; ind_x < kTerrainVoxelWidth; ++ind_x) {
			auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x];
			for (int ind_y = 0; ind_y < kTerrainVoxelWidth - 1; ++ind_y) {
				terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y] =
					terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + (ind_y + 1)];
			}
			terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + (kTerrainVoxelWidth - 1)] = terrain_voxel_cloud_ptr;
			terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + (kTerrainVoxelWidth - 1)]->clear();
		}
		++terrain_voxel_shift_y_;
		terrain_voxel_center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;
	}

	// ===== Stack registered laser scans =====
	pcl::PointXYZI point;
	const int laser_cloud_crop_size = static_cast<int>(laser_cloud_crop_->points.size());
	for (int i = 0; i < laser_cloud_crop_size; ++i) {
		point = laser_cloud_crop_->points[i];

		int ind_x = static_cast<int>((point.x - vehicle_x_ + kTerrainVoxelSize / 2.0f) / kTerrainVoxelSize) +
			kTerrainVoxelHalfWidth;
		int ind_y = static_cast<int>((point.y - vehicle_y_ + kTerrainVoxelSize / 2.0f) / kTerrainVoxelSize) +
			kTerrainVoxelHalfWidth;

		if (point.x - vehicle_x_ + kTerrainVoxelSize / 2.0f < 0) {
			--ind_x;
		}
		if (point.y - vehicle_y_ + kTerrainVoxelSize / 2.0f < 0) {
			--ind_y;
		}

		if (ind_x >= 0 && ind_x < kTerrainVoxelWidth && ind_y >= 0 && ind_y < kTerrainVoxelWidth) {
			terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y]->push_back(point);
			++terrain_voxel_update_num_[kTerrainVoxelWidth * ind_x + ind_y];
		}
	}

	for (int ind = 0; ind < kTerrainVoxelNum; ++ind) {
		if (terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
				laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >= voxel_time_update_thre_ ||
				clearing_cloud_)
		{
			auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind];

			laser_cloud_downsampled_->clear();
			downsize_filter_.setInputCloud(terrain_voxel_cloud_ptr);
			downsize_filter_.filter(*laser_cloud_downsampled_);

			terrain_voxel_cloud_ptr->clear();
			const int laser_cloud_dwz_size = static_cast<int>(laser_cloud_downsampled_->points.size());
			for (int i = 0; i < laser_cloud_dwz_size; ++i) {
				point = laser_cloud_downsampled_->points[i];
				const float dis = std::hypot(point.x - vehicle_x_, point.y - vehicle_y_);
				if (point.z - vehicle_z_ > lower_bound_z_ - dis_ratio_z_ * dis &&
						point.z - vehicle_z_ < upper_bound_z_ + dis_ratio_z_ * dis &&
						((laser_cloud_time_ - system_init_time_ - point.intensity < decay_time_) || (dis < no_decay_dis_)) &&
						!(dis < clearing_dis_ && clearing_cloud_))
				{
					terrain_voxel_cloud_ptr->push_back(point);
				}
			}

			terrain_voxel_update_num_[ind] = 0;
			terrain_voxel_update_time_[ind] = static_cast<float>(laser_cloud_time_ - system_init_time_);
		}
	}

	terrain_cloud_->clear();
	for (int ind_x = kTerrainVoxelHalfWidth - 10; ind_x <= kTerrainVoxelHalfWidth + 10; ++ind_x) {
		for (int ind_y = kTerrainVoxelHalfWidth - 10; ind_y <= kTerrainVoxelHalfWidth + 10; ++ind_y) {
			*terrain_cloud_ += *terrain_voxel_cloud_[kTerrainVoxelWidth * ind_x + ind_y];
		}
	}

	// ===== Estimate ground and compute elevation =====
	for (int i = 0; i < kPlanarVoxelNum; ++i) {
		planar_voxel_elev_[i] = 0.0f;
		planar_voxel_conn_[i] = 0;
		planar_point_elev_[i].clear();
	}

			const int terrain_cloud_size = static_cast<int>(terrain_cloud_->points.size());
			for (int i = 0; i < terrain_cloud_size; ++i) {
				point = terrain_cloud_->points[i];

				if (point.z - vehicle_z_ > lower_bound_z_ &&
						point.z - vehicle_z_ < upper_bound_z_)
		{
					int ind_x = static_cast<int>((point.x - vehicle_x_ + kPlanarVoxelSize / 2.0f) / kPlanarVoxelSize) +
				kPlanarVoxelHalfWidth;
			int ind_y = static_cast<int>((point.y - vehicle_y_ + kPlanarVoxelSize / 2.0f) / kPlanarVoxelSize) +
				kPlanarVoxelHalfWidth;

			if (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0f < 0) {
				--ind_x;
			}
			if (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0f < 0) {
				--ind_y;
			}

			if (ind_x >= 0 && ind_x < kPlanarVoxelWidth && ind_y >= 0 && ind_y < kPlanarVoxelWidth) {
				for (int dx = -1; dx <= 1; ++dx) {
					for (int dy = -1; dy <= 1; ++dy) {
						if (ind_x + dx >= 0 && ind_x + dx < kPlanarVoxelWidth &&
								ind_y + dy >= 0 && ind_y + dy < kPlanarVoxelWidth)
						{
							planar_point_elev_[kPlanarVoxelWidth * (ind_x + dx) + ind_y + dy].push_back(point.z);
						}
					}
				}
			}
		}
	}

	if (use_sorting_) {
		for (int i = 0; i < kPlanarVoxelNum; ++i) {
			auto &elev_vec = planar_point_elev_[i];
			if (!elev_vec.empty()) {
				std::sort(elev_vec.begin(), elev_vec.end());
					int quantile_id = static_cast<int>(quantile_z_ * static_cast<double>(elev_vec.size()));
					if (quantile_id < 0) {
						quantile_id = 0;
					} else {
						const int max_index = static_cast<int>(elev_vec.size()) - 1;
						if (quantile_id > max_index) {
							quantile_id = max_index;
						}
					}
				planar_voxel_elev_[i] = elev_vec[quantile_id];
			}
		}
	} else {
		for (int i = 0; i < kPlanarVoxelNum; ++i) {
			auto &elev_vec = planar_point_elev_[i];
			if (!elev_vec.empty()) {
				const auto min_it = std::min_element(elev_vec.begin(), elev_vec.end());
				planar_voxel_elev_[i] = *min_it;
			}
		}
	}

	if (check_terrain_conn_) {
		const int center_index = kPlanarVoxelWidth * kPlanarVoxelHalfWidth + kPlanarVoxelHalfWidth;
		if (planar_point_elev_[center_index].empty()) {
			planar_voxel_elev_[center_index] = static_cast<float>(vehicle_z_ + terrain_under_vehicle_);
		}

		std::queue<int> empty_queue;
		std::swap(planar_voxel_queue_, empty_queue);
		planar_voxel_queue_.push(center_index);
		planar_voxel_conn_[center_index] = 1;

		while (!planar_voxel_queue_.empty()) {
			const int front = planar_voxel_queue_.front();
			planar_voxel_queue_.pop();
			planar_voxel_conn_[front] = 2;

			const int ind_x = front / kPlanarVoxelWidth;
			const int ind_y = front % kPlanarVoxelWidth;
			for (int dx = -10; dx <= 10; ++dx) {
				for (int dy = -10; dy <= 10; ++dy) {
					if (ind_x + dx >= 0 && ind_x + dx < kPlanarVoxelWidth &&
							ind_y + dy >= 0 && ind_y + dy < kPlanarVoxelWidth)
					{
						const int neighbour = kPlanarVoxelWidth * (ind_x + dx) + ind_y + dy;
						if (planar_voxel_conn_[neighbour] == 0 && !planar_point_elev_[neighbour].empty()) {
							const float diff = std::fabs(planar_voxel_elev_[front] - planar_voxel_elev_[neighbour]);
							if (diff < terrain_conn_thre_) {
								planar_voxel_queue_.push(neighbour);
								planar_voxel_conn_[neighbour] = 1;
							} else if (diff > ceiling_filtering_thre_) {
								planar_voxel_conn_[neighbour] = -1;
							}
						}
					}
				}
			}
		}
	}

	// ===== Compute terrain map =====
		terrain_cloud_elev_->clear();
		int terrain_cloud_elev_size = 0;
	for (int i = 0; i < terrain_cloud_size; ++i) {
		point = terrain_cloud_->points[i];
			if (point.z - vehicle_z_ > lower_bound_z_ && point.z - vehicle_z_ < upper_bound_z_) {
				const float dis = std::hypot(point.x - vehicle_x_, point.y - vehicle_y_);
				if (dis <= local_terrain_map_radius_) {
					continue;
				}
			int ind_x = static_cast<int>((point.x - vehicle_x_ + kPlanarVoxelSize / 2.0f) / kPlanarVoxelSize) +
				kPlanarVoxelHalfWidth;
			int ind_y = static_cast<int>((point.y - vehicle_y_ + kPlanarVoxelSize / 2.0f) / kPlanarVoxelSize) +
				kPlanarVoxelHalfWidth;

			if (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0f < 0) {
				--ind_x;
			}
			if (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0f < 0) {
				--ind_y;
			}

					if (ind_x >= 0 && ind_x < kPlanarVoxelWidth && ind_y >= 0 && ind_y < kPlanarVoxelWidth) {
						const int planar_index = kPlanarVoxelWidth * ind_x + ind_y;
						if (planar_voxel_conn_[planar_index] == 2 || !check_terrain_conn_) {
							const float dis_z = std::fabs(point.z - planar_voxel_elev_[planar_index]);
							if (dis_z < vehicle_height_) {
								terrain_cloud_elev_->push_back(point);
								(*terrain_cloud_elev_)[terrain_cloud_elev_size].intensity = dis_z;
								++terrain_cloud_elev_size;
							}
				}
			}
		}
	}

	const int terrain_cloud_local_size = static_cast<int>(terrain_cloud_local_->points.size());
	for (int i = 0; i < terrain_cloud_local_size; ++i) {
		point = terrain_cloud_local_->points[i];
		const float dis = std::hypot(point.x - vehicle_x_, point.y - vehicle_y_);
		if (dis <= local_terrain_map_radius_) {
			terrain_cloud_elev_->push_back(point);
		}
	}

	clearing_cloud_ = false;

	sensor_msgs::msg::PointCloud2 terrain_cloud_msg;
	pcl::toROSMsg(*terrain_cloud_elev_, terrain_cloud_msg);
	terrain_cloud_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
	terrain_cloud_msg.header.frame_id = "odom";
	pub_terrain_cloud_->publish(terrain_cloud_msg);
}

}  // namespace terrain_analysis

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisExtNode)