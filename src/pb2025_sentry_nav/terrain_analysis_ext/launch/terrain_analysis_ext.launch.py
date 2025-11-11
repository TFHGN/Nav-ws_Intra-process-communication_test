"""
Terrain Analysis Extended Launch File (Python Version)
拓展尺度地形分析节点启动文件（Python 版本）

该启动文件支持两种部署模式：
1. 独立节点模式 (Standalone Node)
2. 可组合节点模式 (Composable Node)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


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
        default_value='terrain_analysis_ext_container',
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
        default_value='0.1',
        description='扫描体素大小 (米)，用于点云降采样'
    )
    
    decay_time_arg = DeclareLaunchArgument(
        'decay_time',
        default_value='10.0',
        description='点云衰减时间 (秒)'
    )
    
    no_decay_dis_arg = DeclareLaunchArgument(
        'no_decay_dis',
        default_value='0.0',
        description='无衰减距离 (米)'
    )
    
    clearing_dis_arg = DeclareLaunchArgument(
        'clearing_dis',
        default_value='30.0',
        description='清空距离 (米)'
    )
    
    # 地面估计参数
    use_sorting_arg = DeclareLaunchArgument(
        'use_sorting',
        default_value='false',
        description='是否在地面估计中使用排序法'
    )
    
    quantile_z_arg = DeclareLaunchArgument(
        'quantile_z',
        default_value='0.25',
        description='排序法选择的分位点'
    )
    
    vehicle_height_arg = DeclareLaunchArgument(
        'vehicle_height',
        default_value='1.5',
        description='车辆高度 (米)'
    )
    
    # 体素更新阈值
    voxel_point_update_thre_arg = DeclareLaunchArgument(
        'voxel_point_update_thre',
        default_value='100',
        description='体素点数更新阈值'
    )
    
    voxel_time_update_thre_arg = DeclareLaunchArgument(
        'voxel_time_update_thre',
        default_value='2.0',
        description='体素时间更新阈值 (秒)'
    )
    
    # 高度边界
    lower_bound_z_arg = DeclareLaunchArgument(
        'lower_bound_z',
        default_value='-1.5',
        description='点云最低高度 (米)'
    )
    
    upper_bound_z_arg = DeclareLaunchArgument(
        'upper_bound_z',
        default_value='1.0',
        description='点云最高高度 (米)'
    )
    
    dis_ratio_z_arg = DeclareLaunchArgument(
        'dis_ratio_z',
        default_value='0.1',
        description='距离-高度比例系数'
    )
    
    # 连通性检查参数
    check_terrain_conn_arg = DeclareLaunchArgument(
        'check_terrain_conn',
        default_value='true',
        description='是否启用地形连通性检查'
    )
    
    terrain_under_vehicle_arg = DeclareLaunchArgument(
        'terrain_under_vehicle',
        default_value='-0.75',
        description='车辆正下方地形默认高度 (米)'
    )
    
    terrain_conn_thre_arg = DeclareLaunchArgument(
        'terrain_conn_thre',
        default_value='0.5',
        description='地形连通高度阈值 (米)'
    )
    
    ceiling_filtering_thre_arg = DeclareLaunchArgument(
        'ceiling_filtering_thre',
        default_value='2.0',
        description='天花板过滤高度阈值 (米)'
    )
    
    local_terrain_map_radius_arg = DeclareLaunchArgument(
        'local_terrain_map_radius',
        default_value='4.0',
        description='局部地形图半径 (米)'
    )
    
    # ========== 辅助函数 ==========
    
    def get_terrain_params(context):
        """
        从 LaunchConfiguration 获取地形分析参数
        
        Args:
            context: Launch context
            
        Returns:
            dict: 地形分析节点参数字典
        """
        return {
            "scanVoxelSize": float(LaunchConfiguration("scan_voxel_size").perform(context)),
            "decayTime": float(LaunchConfiguration("decay_time").perform(context)),
            "noDecayDis": float(LaunchConfiguration("no_decay_dis").perform(context)),
            "clearingDis": float(LaunchConfiguration("clearing_dis").perform(context)),
            "useSorting": LaunchConfiguration("use_sorting").perform(context).lower() == 'true',
            "quantileZ": float(LaunchConfiguration("quantile_z").perform(context)),
            "vehicleHeight": float(LaunchConfiguration("vehicle_height").perform(context)),
            "voxelPointUpdateThre": int(LaunchConfiguration("voxel_point_update_thre").perform(context)),
            "voxelTimeUpdateThre": float(LaunchConfiguration("voxel_time_update_thre").perform(context)),
            "lowerBoundZ": float(LaunchConfiguration("lower_bound_z").perform(context)),
            "upperBoundZ": float(LaunchConfiguration("upper_bound_z").perform(context)),
            "disRatioZ": float(LaunchConfiguration("dis_ratio_z").perform(context)),
            "checkTerrainConn": LaunchConfiguration("check_terrain_conn").perform(context).lower() == 'true',
            "terrainUnderVehicle": float(LaunchConfiguration("terrain_under_vehicle").perform(context)),
            "terrainConnThre": float(LaunchConfiguration("terrain_conn_thre").perform(context)),
            "ceilingFilteringThre": float(LaunchConfiguration("ceiling_filtering_thre").perform(context)),
            "localTerrainMapRadius": float(LaunchConfiguration("local_terrain_map_radius").perform(context)),
        }
    
    def _launch_setup(context, *args, **kwargs):
        """
        根据启动参数动态构建节点配置
        
        Args:
            context: Launch context
            
        Returns:
            list: 待启动的节点列表
        """
        use_composable = LaunchConfiguration('use_composable').perform(context).lower() == 'true'
        container_name = LaunchConfiguration('container_name').perform(context)
        use_intra_process = LaunchConfiguration('use_intra_process_comms').perform(context).lower() == 'true'
        namespace = LaunchConfiguration('namespace').perform(context)
        output = LaunchConfiguration('output').perform(context)
        
        terrain_params = get_terrain_params(context)
        
        nodes_to_launch = []
        
        # 创建可组合节点描述
        composable_node = ComposableNode(
            package="terrain_analysis_ext",
            plugin="terrain_analysis::TerrainAnalysisExtNode",
            name="terrain_analysis_ext",
            namespace=namespace,
            parameters=[terrain_params],
            extra_arguments=[{"use_intra_process_comms": use_intra_process}],
            remappings=[
                ("lidar_odometry", "lidar_odometry"),
                ("registered_scan", "registered_scan"),
                ("terrain_map", "terrain_map"),
                ("terrain_map_ext", "terrain_map_ext"),
                ("cloud_clearing", "cloud_clearing"),
            ],
        )
        
        if use_composable:
            # 加载到已有容器
            load_nodes = LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[composable_node],
            )
            nodes_to_launch.append(load_nodes)
        else:
            # 创建新容器
            container = ComposableNodeContainer(
                name=container_name,
                namespace=namespace,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[composable_node],
                output=output,
            )
            nodes_to_launch.append(container)
        
        return nodes_to_launch
    
    # ========== 返回 LaunchDescription ==========
    
    return LaunchDescription([
        use_composable_arg,
        container_name_arg,
        use_intra_process_arg,
        namespace_arg,
        output_arg,
        scan_voxel_size_arg,
        decay_time_arg,
        no_decay_dis_arg,
        clearing_dis_arg,
        use_sorting_arg,
        quantile_z_arg,
        vehicle_height_arg,
        voxel_point_update_thre_arg,
        voxel_time_update_thre_arg,
        lower_bound_z_arg,
        upper_bound_z_arg,
        dis_ratio_z_arg,
        check_terrain_conn_arg,
        terrain_under_vehicle_arg,
        terrain_conn_thre_arg,
        ceiling_filtering_thre_arg,
        local_terrain_map_radius_arg,
        OpaqueFunction(function=_launch_setup),
    ])
