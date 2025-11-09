#!/bin/bash

# =============================================================================
# 导航系统性能测试脚本
# 生成日期: 2025-11-09
# 功能: 测试当前导航系统的进程数、CPU、内存、通信延迟等指标
# =============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 测试配置
TEST_DURATION=30  # 测试持续时间（秒）
SAMPLE_INTERVAL=1 # 采样间隔（秒）
NAMESPACE="red_standard_robot1"

# 输出目录
OUTPUT_DIR="performance_test_results_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  导航系统性能测试${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# =============================================================================
# 函数: 检查ROS2环境
# =============================================================================
check_ros_environment() {
    echo -e "${YELLOW}[1/8] 检查ROS2环境...${NC}"
    
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到ros2命令${NC}"
        exit 1
    fi
    
    # Source环境
    if [ -f "./install/setup.bash" ]; then
        source ./install/setup.bash
        echo -e "${GREEN}✓ ROS2环境已加载${NC}"
    else
        echo -e "${RED}错误: 未找到 ./install/setup.bash${NC}"
        exit 1
    fi
    echo ""
}

# =============================================================================
# 函数: 统计进程数量
# =============================================================================
count_processes() {
    echo -e "${YELLOW}[2/8] 统计进程数量...${NC}"
    
    # 查找所有ROS2相关进程
    local total_processes=$(pgrep -f "ros2|point_lio|rviz2|gazebo|joy" | wc -l)
    
    echo "总ROS2相关进程数: $total_processes" | tee "$OUTPUT_DIR/process_count.txt"
    
    # 详细进程列表
    echo "" >> "$OUTPUT_DIR/process_count.txt"
    echo "详细进程列表:" >> "$OUTPUT_DIR/process_count.txt"
    ps aux | grep -E "ros2|point_lio|rviz2|gazebo|joy" | grep -v grep >> "$OUTPUT_DIR/process_count.txt"
    
    echo -e "${GREEN}✓ 进程统计完成${NC}"
    echo ""
}

# =============================================================================
# 函数: 列出所有ROS2节点
# =============================================================================
list_ros_nodes() {
    echo -e "${YELLOW}[3/8] 列出所有ROS2节点...${NC}"
    
    # 等待节点启动
    sleep 3
    
    ros2 node list > "$OUTPUT_DIR/node_list.txt" 2>&1 || echo "无法获取节点列表"
    
    local node_count=$(cat "$OUTPUT_DIR/node_list.txt" | wc -l)
    echo "ROS2节点总数: $node_count" | tee -a "$OUTPUT_DIR/node_list.txt"
    
    echo -e "${GREEN}✓ 节点列表已保存${NC}"
    echo ""
}

# =============================================================================
# 函数: 检查Composable节点
# =============================================================================
check_composable_nodes() {
    echo -e "${YELLOW}[4/8] 检查Composable节点...${NC}"
    
    # 查找容器
    if ros2 component list > "$OUTPUT_DIR/component_list.txt" 2>&1; then
        echo "Composable容器内容:" | tee -a "$OUTPUT_DIR/component_list.txt"
        cat "$OUTPUT_DIR/component_list.txt"
        echo -e "${GREEN}✓ Composable节点检查完成${NC}"
    else
        echo -e "${YELLOW}⚠ 未找到Composable容器或容器未运行${NC}"
    fi
    echo ""
}

# =============================================================================
# 函数: 监控CPU和内存使用
# =============================================================================
monitor_resources() {
    echo -e "${YELLOW}[5/8] 监控CPU和内存使用 (持续 ${TEST_DURATION}秒)...${NC}"
    
    # CSV表头
    echo "Timestamp,Process,PID,CPU%,MEM%,RSS_MB,VSZ_MB" > "$OUTPUT_DIR/resource_usage.csv"
    
    local count=0
    while [ $count -lt $TEST_DURATION ]; do
        timestamp=$(date +%Y-%m-%d\ %H:%M:%S)
        
        # 监控主要进程
        for proc_pattern in "nav2_container" "point_lio" "rviz2" "gazebo" "joy"; do
            pid=$(pgrep -f "$proc_pattern" | head -1)
            if [ ! -z "$pid" ]; then
                # 获取进程信息
                ps_info=$(ps -p $pid -o %cpu,%mem,rss,vsz --no-headers 2>/dev/null || echo "0 0 0 0")
                cpu=$(echo $ps_info | awk '{print $1}')
                mem=$(echo $ps_info | awk '{print $2}')
                rss=$(echo $ps_info | awk '{print $3/1024}') # 转换为MB
                vsz=$(echo $ps_info | awk '{print $4/1024}') # 转换为MB
                
                echo "$timestamp,$proc_pattern,$pid,$cpu,$mem,$rss,$vsz" >> "$OUTPUT_DIR/resource_usage.csv"
            fi
        done
        
        sleep $SAMPLE_INTERVAL
        count=$((count + SAMPLE_INTERVAL))
        echo -ne "\r  进度: $count / $TEST_DURATION 秒"
    done
    
    echo ""
    echo -e "${GREEN}✓ 资源监控完成${NC}"
    
    # 生成统计摘要
    generate_resource_summary
    echo ""
}

# =============================================================================
# 函数: 生成资源使用摘要
# =============================================================================
generate_resource_summary() {
    echo -e "${YELLOW}生成资源使用摘要...${NC}"
    
    python3 << 'EOF' > "$OUTPUT_DIR/resource_summary.txt" 2>/dev/null || echo "Python分析失败"
import pandas as pd
import sys

try:
    df = pd.read_csv('$OUTPUT_DIR/resource_usage.csv')
    
    summary = df.groupby('Process').agg({
        'CPU%': ['mean', 'max', 'min'],
        'MEM%': ['mean', 'max', 'min'],
        'RSS_MB': ['mean', 'max', 'min']
    }).round(2)
    
    print("=" * 70)
    print("资源使用统计摘要")
    print("=" * 70)
    print(summary)
    print("\n总内存使用 (RSS):")
    print(f"  平均: {df['RSS_MB'].sum() / len(df['Timestamp'].unique()):.2f} MB")
    print(f"  峰值: {df.groupby('Timestamp')['RSS_MB'].sum().max():.2f} MB")
except Exception as e:
    print(f"分析失败: {e}")
EOF
    
    cat "$OUTPUT_DIR/resource_summary.txt" 2>/dev/null || echo "摘要生成失败"
}

# =============================================================================
# 函数: 测试话题延迟
# =============================================================================
test_topic_latency() {
    echo -e "${YELLOW}[6/8] 测试话题通信延迟...${NC}"
    
    # 关键话题列表
    local topics=(
        "/${NAMESPACE}/cloud_registered"
        "/${NAMESPACE}/terrain_map"
        "/${NAMESPACE}/terrain_map_ext"
        "/${NAMESPACE}/cmd_vel"
        "/${NAMESPACE}/odom"
    )
    
    echo "话题延迟测试结果:" > "$OUTPUT_DIR/topic_latency.txt"
    echo "=====================" >> "$OUTPUT_DIR/topic_latency.txt"
    
    for topic in "${topics[@]}"; do
        echo -e "\n测试话题: $topic" | tee -a "$OUTPUT_DIR/topic_latency.txt"
        
        # 检查话题是否存在
        if ros2 topic list | grep -q "^${topic}$"; then
            # 获取话题频率
            timeout 5s ros2 topic hz "$topic" 2>&1 | head -10 >> "$OUTPUT_DIR/topic_latency.txt" || echo "  无数据" >> "$OUTPUT_DIR/topic_latency.txt"
        else
            echo "  话题不存在" >> "$OUTPUT_DIR/topic_latency.txt"
        fi
    done
    
    echo -e "${GREEN}✓ 话题延迟测试完成${NC}"
    echo ""
}

# =============================================================================
# 函数: 测试TF树性能
# =============================================================================
test_tf_performance() {
    echo -e "${YELLOW}[7/8] 测试TF树性能...${NC}"
    
    # 查看TF树延迟
    timeout 10s ros2 run tf2_ros tf2_monitor > "$OUTPUT_DIR/tf_monitor.txt" 2>&1 || echo "TF监控超时"
    
    # 保存TF树
    ros2 run tf2_tools view_frames --ros-args -r __ns:=/${NAMESPACE} 2>&1 | tee "$OUTPUT_DIR/tf_tree_generation.log"
    
    if [ -f "frames.pdf" ]; then
        mv frames.pdf "$OUTPUT_DIR/tf_frames.pdf"
        echo -e "${GREEN}✓ TF树已保存到 $OUTPUT_DIR/tf_frames.pdf${NC}"
    fi
    
    echo -e "${GREEN}✓ TF性能测试完成${NC}"
    echo ""
}

# =============================================================================
# 函数: 生成最终报告
# =============================================================================
generate_final_report() {
    echo -e "${YELLOW}[8/8] 生成最终性能报告...${NC}"
    
    cat > "$OUTPUT_DIR/PERFORMANCE_REPORT.md" << 'REPORT_EOF'
# 导航系统性能测试报告

**测试日期**: $(date +%Y-%m-%d\ %H:%M:%S)
**测试时长**: $TEST_DURATION 秒
**命名空间**: $NAMESPACE

---

## 一、进程统计

```
$(cat "$OUTPUT_DIR/process_count.txt" | head -20)
```

---

## 二、ROS2节点列表

```
$(cat "$OUTPUT_DIR/node_list.txt" | head -30)
```

---

## 三、Composable节点

```
$(cat "$OUTPUT_DIR/component_list.txt" 2>/dev/null || echo "未找到Composable容器")
```

---

## 四、资源使用统计

```
$(cat "$OUTPUT_DIR/resource_summary.txt" 2>/dev/null || echo "统计数据不可用")
```

详细数据请查看: `resource_usage.csv`

---

## 五、话题延迟测试

```
$(cat "$OUTPUT_DIR/topic_latency.txt" | head -50)
```

---

## 六、TF树监控

```
$(cat "$OUTPUT_DIR/tf_monitor.txt" 2>/dev/null | head -30 || echo "TF监控数据不可用")
```

TF树可视化: `tf_frames.pdf`

---

## 七、测试结论

### 性能基准
- **总进程数**: $(pgrep -f "ros2|point_lio|rviz2" | wc -l)
- **ROS2节点数**: $(cat "$OUTPUT_DIR/node_list.txt" | wc -l)
- **测试时间**: $(date +%Y-%m-%d\ %H:%M:%S)

### 优化建议
参考 `process_analysis.md` 进行进程合并优化。

---

**报告生成**: $(date)
REPORT_EOF

    # 展开变量
    eval "cat > \"$OUTPUT_DIR/PERFORMANCE_REPORT.md\" << 'REPORT_EOF'
$(cat "$OUTPUT_DIR/PERFORMANCE_REPORT.md")
REPORT_EOF"
    
    echo -e "${GREEN}✓ 性能报告已生成${NC}"
    echo ""
}

# =============================================================================
# 函数: 显示测试结果路径
# =============================================================================
show_results() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  测试完成！${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "测试结果已保存到: ${GREEN}$OUTPUT_DIR${NC}"
    echo ""
    echo "主要文件:"
    echo "  📊 PERFORMANCE_REPORT.md    - 综合性能报告"
    echo "  📈 resource_usage.csv       - 详细资源使用数据"
    echo "  📋 process_count.txt        - 进程统计"
    echo "  📋 node_list.txt            - 节点列表"
    echo "  📋 topic_latency.txt        - 话题延迟"
    echo "  🌲 tf_frames.pdf            - TF树可视化"
    echo ""
    echo -e "查看报告: ${YELLOW}cat $OUTPUT_DIR/PERFORMANCE_REPORT.md${NC}"
    echo ""
}

# =============================================================================
# 主执行流程
# =============================================================================
main() {
    # 检查是否有导航系统在运行
    if ! pgrep -f "nav2_container" > /dev/null; then
        echo -e "${RED}错误: 未检测到导航系统运行${NC}"
        echo "请先启动导航系统："
        echo "  ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py"
        exit 1
    fi
    
    check_ros_environment
    count_processes
    list_ros_nodes
    check_composable_nodes
    monitor_resources
    test_topic_latency
    test_tf_performance
    generate_final_report
    show_results
}

# 捕获 Ctrl+C
trap 'echo -e "\n${RED}测试被中断${NC}"; exit 1' INT

# 执行主函数
main "$@"
