#!/bin/bash

# =============================================================================
# 快速性能检查脚本
# 用于快速检查当前系统状态，无需等待完整测试
# =============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  快速性能检查${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查导航系统是否运行
if ! pgrep -f "component_container_isolated" > /dev/null && \
   ! pgrep -f "pointlio_mapping" > /dev/null; then
    echo -e "${YELLOW}⚠ 导航系统未运行${NC}"
    echo ""
    echo "请先启动导航系统："
    echo -e "${GREEN}Terminal 1:${NC}"
    echo "  ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
    echo ""
    echo -e "${GREEN}Terminal 2:${NC}"
    echo "  source ./install/setup.bash"
    echo "  ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\"
    echo "      world:=rmul_2025 slam:=False use_composition:=True"
    echo ""
    echo "启动后再运行: ./performance_test.sh"
    exit 0
fi

echo -e "${GREEN}✓ 检测到导航系统正在运行${NC}"
echo ""

# 统计进程
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}📊 进程统计${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

nav2_container=$(pgrep -f "component_container" | wc -l)
point_lio=$(pgrep -f "pointlio_mapping" | wc -l)
terrain=$(pgrep -f "terrain" | wc -l)
joy=$(pgrep -f "joy" | wc -l)
rviz=$(pgrep -f "rviz2" | wc -l)
gazebo=$(pgrep -f "gazebo" | wc -l)
total_ros=$(pgrep -f "ros2\|point_lio\|rviz2\|gazebo\|joy\|terrain" | wc -l)

echo "  nav2_container       : $nav2_container"
echo "  point_lio           : $point_lio"
echo "  terrain_analysis    : $terrain"
echo "  joy相关             : $joy"
echo "  rviz2               : $rviz"
echo "  gazebo              : $gazebo"
echo "  ────────────────────────"
echo -e "  ${GREEN}总ROS2进程数     : $total_ros${NC}"
echo ""

# CPU和内存快照
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}💻 资源使用快照${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 表头
printf "%-25s %8s %8s %10s\n" "进程" "CPU%" "MEM%" "内存(MB)"
echo "────────────────────────────────────────────────────"

total_cpu=0
total_mem=0

for proc_pattern in "nav2_container" "pointlio_mapping" "terrainAnalysis" "rviz2" "gzserver"; do
    pid=$(pgrep -f "$proc_pattern" | head -1)
    if [ ! -z "$pid" ]; then
        ps_info=$(ps -p $pid -o comm=,pcpu=,pmem=,rss= --no-headers 2>/dev/null)
        if [ ! -z "$ps_info" ]; then
            comm=$(echo $ps_info | awk '{print $1}')
            cpu=$(echo $ps_info | awk '{print $2}')
            mem=$(echo $ps_info | awk '{print $3}')
            rss=$(echo $ps_info | awk '{print $4/1024}')
            
            printf "%-25s %7.1f%% %7.1f%% %9.1f\n" "$comm" "$cpu" "$mem" "$rss"
            
            total_cpu=$(echo "$total_cpu + $cpu" | bc)
            total_mem=$(echo "$total_mem + $rss" | bc)
        fi
    fi
done

echo "────────────────────────────────────────────────────"
printf "%-25s %7.1f%% %8s %9.1f\n" "总计" "$total_cpu" "-" "$total_mem"
echo ""

# Source ROS2环境
WORKSPACE_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash" 2>/dev/null
    
    # ROS2节点检查
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}🔧 ROS2节点状态${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    node_count=$(timeout 3s ros2 node list 2>/dev/null | wc -l)
    echo "  ROS2节点总数: $node_count"
    echo ""
    
    # Composable检查
    if timeout 3s ros2 component list &>/dev/null; then
        echo -e "${GREEN}  ✓ 检测到Composable容器${NC}"
        container_count=$(timeout 3s ros2 component list 2>/dev/null | grep -c "ComponentManager" || echo "0")
        echo "    容器数量: $container_count"
    else
        echo -e "${YELLOW}  ⚠ 未检测到Composable容器${NC}"
    fi
    echo ""
fi

# 系统总览
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}🔍 系统总览${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# 使用use_composition的检测
if pgrep -af "component_container" > /dev/null; then
    echo -e "  模式: ${GREEN}Composable (已启用)${NC}"
else
    echo -e "  模式: ${YELLOW}独立进程模式${NC}"
fi

echo "  系统负载: $(uptime | awk -F'load average:' '{print $2}')"
echo "  可用内存: $(free -h | awk '/^Mem:/ {print $7}')"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  快速检查完成${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "运行完整性能测试："
echo -e "  ${GREEN}./performance_test.sh${NC}"
echo ""
