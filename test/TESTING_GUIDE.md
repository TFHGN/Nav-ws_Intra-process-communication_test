# 导航系统性能测试指南

## 📋 测试工具说明

本测试套件包含三个脚本：

### 1. `quick_check.sh` - 快速检查 ⚡
**用途**: 快速检查系统当前状态（2秒内完成）

**检查内容**:
- ✅ 导航系统是否运行
- 📊 进程数量统计
- 💻 CPU和内存快照
- 🔧 ROS2节点数量
- 🔍 Composable容器状态

**使用方法**:
```bash
./quick_check.sh
```

---

### 2. `performance_test.sh` - 完整性能测试 🔬
**用途**: 完整的性能基准测试（约30-60秒）

**测试内容**:
- 📊 详细进程统计
- 📋 完整节点列表
- 📦 Composable节点清单
- 💻 30秒资源监控（CPU、内存）
- 📡 话题延迟测试
- 🌲 TF树性能分析
- 📈 生成CSV数据和PDF报告

**使用方法**:
```bash
./performance_test.sh
```

**输出**: 创建 `performance_test_results_YYYYMMDD_HHMMSS/` 目录，包含详细报告

---

### 3. `process_analysis.md` - 优化方案文档 📖
**用途**: 理论分析和优化建议

**内容**:
- 当前架构分析
- 可合并节点识别
- 三种优化方案对比
- 实施步骤指南

---

## 🚀 测试步骤

### Step 1: 启动导航系统

#### Terminal 1 - 启动仿真环境
```bash
cd ~/nav2_ws
source ./install/setup.bash

# 启动Gazebo仿真器
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

**⚠️ 重要**: 点击Gazebo界面左下角的**橙红色启动按钮**

---

#### Terminal 2 - 启动导航系统

**导航模式**（推荐用于性能测试）:
```bash
cd ~/nav2_ws
source ./install/setup.bash

# 使用Composable模式（默认）
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmul_2025 \
    slam:=False \
    use_composition:=True
```

**SLAM建图模式**（可选）:
```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmul_2025 \
    slam:=True \
    use_composition:=True
```

---

### Step 2: 快速检查系统状态

#### Terminal 3 - 快速检查
```bash
cd ~/nav2_ws
./quick_check.sh
```

**预期输出示例**:
```
========================================
  快速性能检查
========================================

✓ 检测到导航系统正在运行

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 进程统计
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  nav2_container       : 1
  point_lio           : 1
  terrain_analysis    : 2
  joy相关             : 2
  rviz2               : 1
  gazebo              : 2
  ────────────────────────
  总ROS2进程数     : 9

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
💻 资源使用快照
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

进程                        CPU%     MEM%    内存(MB)
────────────────────────────────────────────────────
component_container        15.2%     2.3%      450.2
pointlio_mapping            8.5%     1.8%      320.5
terrainAnalysis             3.2%     0.5%       85.3
rviz2                      12.1%     3.5%      680.1
gzserver                   35.8%     5.2%     1024.8
────────────────────────────────────────────────────
总计                       74.8%        -     2560.9

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔧 ROS2节点状态
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ROS2节点总数: 18

  ✓ 检测到Composable容器
    容器数量: 1
```

---

### Step 3: 运行完整性能测试

#### Terminal 3 - 完整测试
```bash
cd ~/nav2_ws
./performance_test.sh
```

**测试将持续约30-60秒**

**输出示例**:
```
========================================
  导航系统性能测试
========================================

[1/8] 检查ROS2环境...
✓ ROS2环境已加载

[2/8] 统计进程数量...
✓ 进程统计完成

[3/8] 列出所有ROS2节点...
✓ 节点列表已保存

[4/8] 检查Composable节点...
✓ Composable节点检查完成

[5/8] 监控CPU和内存使用 (持续 30秒)...
  进度: 30 / 30 秒
✓ 资源监控完成

[6/8] 测试话题延迟...
✓ 话题延迟测试完成

[7/8] 测试TF树性能...
✓ TF性能测试完成

[8/8] 生成最终性能报告...
✓ 性能报告已生成

========================================
  测试完成！
========================================

测试结果已保存到: performance_test_results_20251109_143025

主要文件:
  📊 PERFORMANCE_REPORT.md    - 综合性能报告
  📈 resource_usage.csv       - 详细资源使用数据
  📋 process_count.txt        - 进程统计
  📋 node_list.txt            - 节点列表
  📋 topic_latency.txt        - 话题延迟
  🌲 tf_frames.pdf            - TF树可视化

查看报告: cat performance_test_results_20251109_143025/PERFORMANCE_REPORT.md
```

---

## 📊 查看测试结果

### 查看综合报告
```bash
# 假设输出目录为 performance_test_results_20251109_143025
cd performance_test_results_20251109_143025

# 查看Markdown报告
cat PERFORMANCE_REPORT.md

# 或使用文本编辑器
code PERFORMANCE_REPORT.md
```

### 查看详细数据
```bash
# CSV格式的资源使用数据
cat resource_usage.csv

# 使用Python分析（需要pandas）
python3 << EOF
import pandas as pd
df = pd.read_csv('resource_usage.csv')
print(df.groupby('Process')[['CPU%', 'MEM%', 'RSS_MB']].describe())
EOF
```

### 查看TF树可视化
```bash
# 使用PDF阅读器打开
xdg-open tf_frames.pdf
# 或
evince tf_frames.pdf
```

---

## 📈 性能指标解读

### 1. 进程数量
- **当前配置**: 9-10个进程
- **理想目标**: 6-7个进程（减少30%）
- **评估**: 进程越少，上下文切换开销越小

### 2. CPU使用率
- **nav2_container**: 应在 10-20%
- **point_lio**: 应在 5-15%
- **terrain_analysis**: 应在 3-8%
- **gazebo**: 通常 20-40%（正常）
- **总计**: 应低于 100%（单核）

### 3. 内存使用
- **总RSS**: 通常 2-4 GB
- **nav2_container**: 400-600 MB
- **gazebo**: 800-1500 MB
- **评估**: 内存占用应稳定，无明显增长

### 4. 话题频率
- `/cloud_registered`: ~10 Hz
- `/terrain_map`: ~5-10 Hz
- `/odom`: ~50-100 Hz
- `/cmd_vel`: ~20-50 Hz
- **评估**: 频率应稳定，无丢帧

### 5. TF树延迟
- 大部分变换应 < 10 ms
- `/map` → `/odom`: < 50 ms
- **评估**: 延迟应稳定，无警告

---

## 🔧 对比测试（优化前后）

### 测试场景
1. **基准测试**（当前配置）
   ```bash
   # use_composition:=True（默认）
   ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
       world:=rmul_2025 slam:=False use_composition:=True
   
   ./performance_test.sh
   # 保存结果到 baseline/
   ```

2. **独立进程模式测试**（对比）
   ```bash
   # use_composition:=False
   ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
       world:=rmul_2025 slam:=False use_composition:=False
   
   ./performance_test.sh
   # 保存结果到 non_composable/
   ```

3. **优化后测试**（实施合并方案后）
   ```bash
   # 修改launch文件后
   ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
       world:=rmul_2025 slam:=False use_composition:=True
   
   ./performance_test.sh
   # 保存结果到 optimized/
   ```

### 对比指标
```bash
# 创建对比脚本
cat > compare_results.sh << 'EOF'
#!/bin/bash
echo "性能对比"
echo "===================="
echo ""
echo "进程数量:"
grep "总ROS2进程数" baseline/process_count.txt
grep "总ROS2进程数" non_composable/process_count.txt
grep "总ROS2进程数" optimized/process_count.txt
echo ""
echo "平均CPU使用:"
# ... 添加更多对比逻辑
EOF

chmod +x compare_results.sh
./compare_results.sh
```

---

## 🎯 测试检查清单

### 测试前
- [ ] Gazebo仿真器已启动
- [ ] 点击了Gazebo的启动按钮
- [ ] 导航系统已完全启动（等待3-5秒）
- [ ] RViz显示正常
- [ ] 无明显错误日志

### 测试中
- [ ] 观察进程数量是否符合预期
- [ ] 检查CPU使用率是否合理
- [ ] 观察内存是否稳定
- [ ] 话题是否有数据发布

### 测试后
- [ ] 查看生成的报告
- [ ] 分析资源使用CSV数据
- [ ] 检查TF树是否完整
- [ ] 记录性能基准数据

---

## 🐛 常见问题

### Q1: 脚本提示"导航系统未运行"
**A**: 确保已在另一个终端启动导航系统，并等待3-5秒让所有节点启动完成。

### Q2: "ros2: command not found"
**A**: 需要先source环境：
```bash
source ./install/setup.bash
```

### Q3: TF树生成失败
**A**: 确保安装了tf2_tools：
```bash
sudo apt install ros-humble-tf2-tools
```

### Q4: Python分析报错
**A**: 安装pandas：
```bash
pip install pandas
```

### Q5: 进程数量与预期不符
**A**: 
- 检查是否正确使用了 `use_composition:=True`
- 确认所有节点已启动（`ros2 node list`）
- 某些节点可能延迟启动

---

## 📝 性能优化流程

1. **建立基准** → 运行 `./performance_test.sh` 记录当前性能
2. **分析瓶颈** → 查看报告，识别资源消耗大的节点
3. **实施优化** → 参考 `process_analysis.md` 合并节点
4. **对比测试** → 再次运行测试，对比优化效果
5. **迭代改进** → 根据结果继续优化

---

## 📚 相关文档

- `process_analysis.md` - 详细的架构分析和优化方案
- `PERFORMANCE_REPORT.md` - 测试生成的性能报告
- `resource_usage.csv` - 原始性能数据

---

## 🔗 有用的命令

```bash
# 实时监控进程
watch -n 1 './quick_check.sh'

# 查看特定进程详细信息
top -p $(pgrep -d',' -f nav2_container)

# 查看ROS2话题列表
ros2 topic list

# 查看节点图
rqt_graph

# 检查TF
ros2 run tf2_ros tf2_echo map base_link

# 查看计算图
ros2 run rqt_graph rqt_graph
```

---

**最后更新**: 2025-11-09
**测试环境**: Ubuntu 22.04 + ROS2 Humble
