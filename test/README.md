# 性能测试工具使用指南

## 📁 测试文件夹说明

本目录 (`test/`) 包含完整的导航系统性能测试工具套件。

---

## 🚀 快速开始

### 方法1: 使用主测试脚本（推荐）

```bash
cd ~/nav2_ws
./test/run_test.sh
```

这个脚本会：
1. ✅ 检查系统是否运行
2. 📋 显示启动指南（如果未运行）
3. 📊 提供测试菜单供选择
4. 🎯 自动执行测试并显示结果

---

### 方法2: 直接运行测试

```bash
# 快速检查（2秒）
cd ~/nav2_ws/test
./quick_check.sh

# 完整测试（30-60秒）
cd ~/nav2_ws/test
./performance_test.sh
```

---

## 📂 文件说明

### 🔧 可执行脚本

| 文件 | 说明 | 用途 |
|------|------|------|
| `run_test.sh` | 主测试入口 | 交互式测试菜单，推荐使用 |
| `quick_check.sh` | 快速检查 | 2秒内查看系统状态 |
| `performance_test.sh` | 完整测试 | 30-60秒详细性能分析 |

### 📖 文档文件

| 文件 | 说明 |
|------|------|
| `README.md` | 本文件，使用指南 |
| `PERFORMANCE_TESTING_README.md` | 测试套件总览 |
| `TESTING_GUIDE.md` | 详细测试指南 |
| `process_analysis.md` | 架构分析与优化方案 |
| `FILES_SUMMARY.md` | 完整文件清单 |

---

## 📊 测试流程

### 第一步：启动系统

#### Terminal 1 - 仿真器
```bash
cd ~/nav2_ws
source ./install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```
**⚠️ 重要：点击Gazebo左下角的启动按钮！**

#### Terminal 2 - 导航系统
```bash
cd ~/nav2_ws
source ./install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmul_2025 \
    slam:=False \
    use_composition:=True
```

### 第二步：运行测试

#### Terminal 3 - 测试
```bash
cd ~/nav2_ws
./test/run_test.sh
```

然后按菜单提示选择测试类型。

---

## 🎯 测试选项说明

### 选项1: 快速检查 ⚡
- **耗时**: 约2秒
- **功能**: 
  - 检测导航系统是否运行
  - 统计进程数量
  - 显示CPU和内存快照
  - 查看ROS2节点状态
- **适用场景**: 日常快速诊断、验证系统启动

### 选项2: 完整性能测试 🔬
- **耗时**: 30-60秒
- **功能**:
  - 详细进程统计
  - 完整节点列表
  - 30秒资源监控
  - 话题延迟测试
  - TF树性能分析
  - 生成CSV数据和PDF报告
- **适用场景**: 性能基准建立、优化效果验证

### 选项3: 两者都运行 🎯
- 先执行快速检查
- 再执行完整测试
- 适合完整的性能评估

---

## 📈 测试输出

### 快速检查输出
- 直接在终端显示结果
- 包含进程统计、资源使用快照

### 完整测试输出
- 创建目录: `performance_test_results_YYYYMMDD_HHMMSS/`
- 包含文件:
  - `PERFORMANCE_REPORT.md` - 综合报告
  - `resource_usage.csv` - 详细数据
  - `process_count.txt` - 进程统计
  - `node_list.txt` - 节点列表
  - `topic_latency.txt` - 话题延迟
  - `tf_frames.pdf` - TF树可视化

---

## 📖 查看文档

```bash
# 查看测试套件总览（推荐首读）
cat test/PERFORMANCE_TESTING_README.md

# 查看详细测试指南
cat test/TESTING_GUIDE.md

# 查看架构分析和优化方案
cat test/process_analysis.md

# 在编辑器中打开
code test/PERFORMANCE_TESTING_README.md
```

---

## 🔧 权限问题

如果脚本无法执行：

```bash
cd ~/nav2_ws/test
chmod +x run_test.sh quick_check.sh performance_test.sh
```

---

## 🐛 常见问题

### Q: 提示"导航系统未运行"
**A**: 按照上述步骤1启动系统，等待3-5秒后再运行测试

### Q: 找不到ros2命令
**A**: 需要先source环境：
```bash
source ~/nav2_ws/install/setup.bash
```

### Q: 脚本执行出错
**A**: 检查：
1. 是否在正确的目录（`~/nav2_ws`）
2. 脚本是否有执行权限
3. 系统是否正确启动

---

## 💡 使用技巧

### 实时监控
```bash
# 持续监控系统状态（每秒刷新）
watch -n 1 'cd ~/nav2_ws/test && ./quick_check.sh'
```

### 保存测试历史
```bash
# 创建历史目录
mkdir -p ~/nav2_ws/test_history

# 运行测试后保存
cd ~/nav2_ws/test
./performance_test.sh
cp -r performance_test_results_* ../test_history/
```

### 对比不同配置
```bash
# 测试use_composition=True
./run_test.sh
# 保存结果为 baseline_composition

# 测试use_composition=False
# 修改启动参数后重新测试
./run_test.sh
# 保存结果为 baseline_no_composition

# 对比结果
diff test_history/baseline_*/PERFORMANCE_REPORT.md
```

---

## 📞 获取帮助

1. **查看详细文档**: 
   ```bash
   cat test/TESTING_GUIDE.md
   ```

2. **查看优化建议**: 
   ```bash
   cat test/process_analysis.md
   ```

3. **查看所有文件**: 
   ```bash
   cat test/FILES_SUMMARY.md
   ```

---

## 🎓 测试工作流程建议

```
启动系统 → 快速检查 → 确认正常 → 完整测试 → 分析报告 → 实施优化 → 再次测试
```

详细流程请参考 `TESTING_GUIDE.md`

---

## 📊 性能基准参考

### 当前配置（use_composition:=True）
- **总进程数**: 9-10个
- **ROS2节点**: ~18个
- **总内存**: 2.5-4 GB
- **容器内节点**: 14个

### 优化目标
- **进程数**: 6-7个（减少30%）
- **内存**: 2-3 GB（节省15%）
- **延迟**: 降低20-30%

详细分析见 `process_analysis.md`

---

**最后更新**: 2025-11-09  
**版本**: v1.0  
**维护者**: Transistor战队导航组
