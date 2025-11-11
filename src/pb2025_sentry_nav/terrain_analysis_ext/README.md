# terrain_analysis_ext

重构后的拓展尺度地形分析组件节点。该包与 `terrain_analysis` 采用一致的结构：

- **组件化节点**：`terrain_analysis::TerrainAnalysisExtNode`
- **共享库目标**：`libterrain_analysis_ext.so`
- **可执行包装器**：通过 `ros2 run terrain_analysis_ext terrain_analysis_ext_node` 或组合式容器加载
- **Python Launch**：`launch/terrain_analysis_ext.launch.py` 支持组合/独立两种部署方式

## 主要话题

| 话题 | 方向 | 描述 |
| --- | --- | --- |
| `registered_scan` | 订阅 | 注册后的点云输入 |
| `lidar_odometry` | 订阅 | 里程计位姿 |
| `terrain_map` | 订阅 | 局部地形图融合输入 |
| `cloud_clearing` | 订阅 | 清空半径指令 |
| `terrain_map_ext` | 发布 | 含高程信息的拓展地形输出 |

## 示例

```bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch.py use_composable:=false output:=screen
```

## 迁移说明

本次重构保持所有算法逻辑不变，仅重整为类封装：

- 主循环迁移至 100Hz 定时器回调
- 参数仍沿用原 CamelCase 命名以兼容既有配置
- 发布/订阅接口保持原话题名
- Launch 接口与 `terrain_analysis` 保持一致的参数命名风格
