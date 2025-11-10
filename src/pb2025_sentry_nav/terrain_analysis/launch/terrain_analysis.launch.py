"""
Terrain Analysis Launch File (Python Version)
地形分析节点启动文件（Python 版本）

该启动文件支持两种部署模式：
1. 独立节点模式 (Standalone Node)
2. 可组合节点模式 (Composable Node)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    生成 launch 描述
    
    Returns:
        LaunchDescription: 完整的启动配置
    """
    
    # ========== 声明启动参数 ==========
    
    # 部署模式选择
    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='true',
        description='是否使用可组合节点模式部署。true: 组合节点, false: 独立节点, 默认 true'
    )
    
    # 容器名称（仅在可组合模式下使用）
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='terrain_analysis_container',
        description='组合节点容器名称（仅在 use_composable=true 时有效）'
    )
    
    # 进程内通信
    use_intra_process_arg = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='true',
        description='是否启用进程内通信（零拷贝）'
    )
    
    # 命名空间
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='节点命名空间'
    )
    
    # 输出模式
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='日志输出目标：screen, log, both'
    )
    
    # ========== 地形分析参数 ==========
    
    # 体素化参数
    scan_voxel_size_arg = DeclareLaunchArgument(
        'scan_voxel_size',
        default_value='0.05',
        description='扫描体素大小 (米)，用于点云降采样'
    )
    
    decay_time_arg = DeclareLaunchArgument(
        'decay_time',
        default_value='1.0',
        description='点云衰减时间 (秒)，超过此时间的点将被移除'
    )
    
    no_decay_dis_arg = DeclareLaunchArgument(
        'no_decay_dis',
        default_value='1.75',
        description='无衰减距离 (米)，此范围内的点不受时间衰减影响'
    )
    
    clearing_dis_arg = DeclareLaunchArgument(
        'clearing_dis',
        default_value='8.0',
        description='清空距离 (米)，执行清空时的有效半径'
    )
    
    # 地面估计参数
    use_sorting_arg = DeclareLaunchArgument(
        'use_sorting',
        default_value='true',
        description='是否使用排序法估计地面高程（false 则使用最小值法）'
    )
    
    quantile_z_arg = DeclareLaunchArgument(
        'quantile_z',
        default_value='0.25',
        description='地面高程分位数 (0-1)，用于排序法时选择的分位点'
    )
    
    consider_drop_arg = DeclareLaunchArgument(
        'consider_drop',
        default_value='false',
        description='是否考虑下降沿（凹陷地形）作为障碍'
    )
    
    limit_ground_lift_arg = DeclareLaunchArgument(
        'limit_ground_lift',
        default_value='false',
        description='是否限制地面抬升幅度'
    )
    
    max_ground_lift_arg = DeclareLaunchArgument(
        'max_ground_lift',
        default_value='0.15',
        description='最大地面抬升 (米)，防止异常高点影响地面估计'
    )
    
    # 动态障碍物过滤参数
    clear_dynamic_obs_arg = DeclareLaunchArgument(
        'clear_dynamic_obs',
        default_value='true',
        description='是否启用动态障碍物清除功能'
    )
    
    min_dy_obs_dis_arg = DeclareLaunchArgument(
        'min_dy_obs_dis',
        default_value='0.3',
        description='动态障碍物最小检测距离 (米)'
    )
    
    min_dy_obs_angle_arg = DeclareLaunchArgument(
        'min_dy_obs_angle',
        default_value='0.0',
        description='动态障碍物最小角度 (度)，相对于水平面'
    )
    
    min_dy_obs_rel_z_arg = DeclareLaunchArgument(
        'min_dy_obs_rel_z',
        default_value='-0.3',
        description='动态障碍物最小相对高度 (米)'
    )
    
    abs_dy_obs_rel_z_thre_arg = DeclareLaunchArgument(
        'abs_dy_obs_rel_z_thre',
        default_value='0.2',
        description='动态障碍物绝对相对高度阈值 (米)'
    )
    
    min_dy_obs_vfov_arg = DeclareLaunchArgument(
        'min_dy_obs_vfov',
        default_value='-28.0',
        description='动态障碍物最小垂直视场角 (度)'
    )
    
    max_dy_obs_vfov_arg = DeclareLaunchArgument(
        'max_dy_obs_vfov',
        default_value='33.0',
        description='动态障碍物最大垂直视场角 (度)'
    )
    
    min_dy_obs_point_num_arg = DeclareLaunchArgument(
        'min_dy_obs_point_num',
        default_value='1',
        description='判定为动态障碍物所需的最小点数'
    )
    
    # 无数据障碍检测参数
    no_data_obstacle_arg = DeclareLaunchArgument(
        'no_data_obstacle',
        default_value='false',
        description='是否将无数据区域标记为障碍物'
    )
    
    no_data_block_skip_num_arg = DeclareLaunchArgument(
        'no_data_block_skip_num',
        default_value='0',
        description='无数据块跳过数量，用于膨胀无数据区域'
    )
    
    min_block_point_num_arg = DeclareLaunchArgument(
        'min_block_point_num',
        default_value='10',
        description='体素有效所需的最小点数'
    )
    
    # 车辆参数
    vehicle_height_arg = DeclareLaunchArgument(
        'vehicle_height',
        default_value='1.5',
        description='车辆高度 (米)，用于确定可通行性判断范围'
    )
    
    # 更新阈值参数
    voxel_point_update_thre_arg = DeclareLaunchArgument(
        'voxel_point_update_thre',
        default_value='100',
        description='体素点数更新阈值，达到此值触发降采样'
    )
    
    voxel_time_update_thre_arg = DeclareLaunchArgument(
        'voxel_time_update_thre',
        default_value='2.0',
        description='体素时间更新阈值 (秒)，超过此时间强制更新'
    )
    
    # 高度范围参数
    min_rel_z_arg = DeclareLaunchArgument(
        'min_rel_z',
        default_value='-1.5',
        description='最小相对高度 (米)，低于此值的点被过滤'
    )
    
    max_rel_z_arg = DeclareLaunchArgument(
        'max_rel_z',
        default_value='0.3',
        description='最大相对高度 (米)，高于此值的点被过滤'
    )
    
    dis_ratio_z_arg = DeclareLaunchArgument(
        'dis_ratio_z',
        default_value='0.2',
        description='距离-高度比例系数，用于动态调整高度范围'
    )
    
    # ========== 构建参数字典 ==========
    
    def get_terrain_params(context):
        """从 LaunchConfiguration 获取参数字典"""
        return {
            'scan_voxel_size': float(LaunchConfiguration('scan_voxel_size').perform(context)),
            'decay_time': float(LaunchConfiguration('decay_time').perform(context)),
            'no_decay_dis': float(LaunchConfiguration('no_decay_dis').perform(context)),
            'clearing_dis': float(LaunchConfiguration('clearing_dis').perform(context)),
            'use_sorting': LaunchConfiguration('use_sorting').perform(context).lower() == 'true',
            'quantile_z': float(LaunchConfiguration('quantile_z').perform(context)),
            'consider_drop': LaunchConfiguration('consider_drop').perform(context).lower() == 'true',
            'limit_ground_lift': LaunchConfiguration('limit_ground_lift').perform(context).lower() == 'true',
            'max_ground_lift': float(LaunchConfiguration('max_ground_lift').perform(context)),
            'clear_dynamic_obs': LaunchConfiguration('clear_dynamic_obs').perform(context).lower() == 'true',
            'min_dy_obs_dis': float(LaunchConfiguration('min_dy_obs_dis').perform(context)),
            'min_dy_obs_angle': float(LaunchConfiguration('min_dy_obs_angle').perform(context)),
            'min_dy_obs_rel_z': float(LaunchConfiguration('min_dy_obs_rel_z').perform(context)),
            'abs_dy_obs_rel_z_thre': float(LaunchConfiguration('abs_dy_obs_rel_z_thre').perform(context)),
            'min_dy_obs_vfov': float(LaunchConfiguration('min_dy_obs_vfov').perform(context)),
            'max_dy_obs_vfov': float(LaunchConfiguration('max_dy_obs_vfov').perform(context)),
            'min_dy_obs_point_num': int(LaunchConfiguration('min_dy_obs_point_num').perform(context)),
            'no_data_obstacle': LaunchConfiguration('no_data_obstacle').perform(context).lower() == 'true',
            'no_data_block_skip_num': int(LaunchConfiguration('no_data_block_skip_num').perform(context)),
            'min_block_point_num': int(LaunchConfiguration('min_block_point_num').perform(context)),
            'vehicle_height': float(LaunchConfiguration('vehicle_height').perform(context)),
            'voxel_point_update_thre': int(LaunchConfiguration('voxel_point_update_thre').perform(context)),
            'voxel_time_update_thre': float(LaunchConfiguration('voxel_time_update_thre').perform(context)),
            'min_rel_z': float(LaunchConfiguration('min_rel_z').perform(context)),
            'max_rel_z': float(LaunchConfiguration('max_rel_z').perform(context)),
            'dis_ratio_z': float(LaunchConfiguration('dis_ratio_z').perform(context)),
        }
    
    # ========== 生成节点配置 ==========
    
    def launch_setup(context, *args, **kwargs):
        """根据参数选择启动模式"""
        
        use_composable = LaunchConfiguration('use_composable').perform(context).lower() == 'true'
        container_name = LaunchConfiguration('container_name').perform(context)
        use_intra_process = LaunchConfiguration('use_intra_process_comms').perform(context).lower() == 'true'
        namespace = LaunchConfiguration('namespace').perform(context)
        output = LaunchConfiguration('output').perform(context)
        
        # 获取参数
        terrain_params = get_terrain_params(context)
        
        nodes_to_launch = []
        
            # 组合节点描述
        composable_node = ComposableNode(
            package='terrain_analysis',
            plugin='terrain_analysis::TerrainAnalysisNode',
            name='terrain_analysis',
            namespace=namespace,
            parameters=[terrain_params],
            extra_arguments=[{'use_intra_process_comms': use_intra_process}],
            remappings=[
                ('lidar_odometry', 'lidar_odometry'),
                ('registered_scan', 'registered_scan'),
                ('joy', 'joy'),
                ('map_clearing', 'map_clearing'),
                ('terrain_map', 'terrain_map'),
            ]
        )

        if use_composable:
            # ========== 模式 1: 可组合节点（加载到现有容器）==========
            
            # 加载到指定的容器（例如 nav2_container）
            # 前置条件：容器必须已经存在
            load_nodes = LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[composable_node],
            )
            
            nodes_to_launch.append(load_nodes)
            
        else:
            # ========== 模式 2: 独立容器（创建新容器）==========
            
            # 创建一个新的独立容器
            container = ComposableNodeContainer(
                name=container_name,
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[composable_node],
                output=output,
            )
            nodes_to_launch.append(container)
        
        return nodes_to_launch
    
    # ========== 返回 LaunchDescription ==========
    
    return LaunchDescription([
        # 声明所有参数
        use_composable_arg,
        container_name_arg,
        use_intra_process_arg,
        namespace_arg,
        output_arg,
        
        # 地形分析参数
        scan_voxel_size_arg,
        decay_time_arg,
        no_decay_dis_arg,
        clearing_dis_arg,
        use_sorting_arg,
        quantile_z_arg,
        consider_drop_arg,
        limit_ground_lift_arg,
        max_ground_lift_arg,
        clear_dynamic_obs_arg,
        min_dy_obs_dis_arg,
        min_dy_obs_angle_arg,
        min_dy_obs_rel_z_arg,
        abs_dy_obs_rel_z_thre_arg,
        min_dy_obs_vfov_arg,
        max_dy_obs_vfov_arg,
        min_dy_obs_point_num_arg,
        no_data_obstacle_arg,
        no_data_block_skip_num_arg,
        min_block_point_num_arg,
        vehicle_height_arg,
        voxel_point_update_thre_arg,
        voxel_time_update_thre_arg,
        min_rel_z_arg,
        max_rel_z_arg,
        dis_ratio_z_arg,
        
        # 执行启动逻辑
        OpaqueFunction(function=launch_setup)
    ])
