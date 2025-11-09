#!/bin/bash

# =============================================================================
# 导航系统性能测试 - 主入口脚本
# 位置: test/run_test.sh
# 用途: 自动化测试流程，引导用户完成测试
# =============================================================================

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

clear

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         导航系统性能测试 - 自动化测试流程                      ║${NC}"
echo -e "${BLUE}║                    Test Suite v1.0                            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# =============================================================================
# 函数: 检查系统是否运行
# =============================================================================
check_system_running() {
    if pgrep -f "component_container_isolated" > /dev/null || \
       pgrep -f "pointlio_mapping" > /dev/null; then
        return 0
    else
        return 1
    fi
}

# =============================================================================
# 函数: 显示启动指南
# =============================================================================
show_startup_guide() {
    echo -e "${YELLOW}⚠  导航系统未运行${NC}"
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}步骤 1: 启动仿真环境（Terminal 1）${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${GREEN}cd ~/nav2_ws${NC}"
    echo -e "${GREEN}source ./install/setup.bash${NC}"
    echo -e "${GREEN}ros2 launch rmu_gazebo_simulator bringup_sim.launch.py${NC}"
    echo ""
    echo -e "${RED}⚠️  重要: 启动后点击 Gazebo 左下角的【启动】按钮！${NC}"
    echo ""
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}步骤 2: 启动导航系统（Terminal 2）${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${GREEN}cd ~/nav2_ws${NC}"
    echo -e "${GREEN}source ./install/setup.bash${NC}"
    echo -e "${GREEN}ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\${NC}"
    echo -e "${GREEN}    world:=rmul_2025 \\${NC}"
    echo -e "${GREEN}    slam:=False \\${NC}"
    echo -e "${GREEN}    use_composition:=True${NC}"
    echo ""
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}步骤 3: 等待系统稳定（5-10秒）${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "确认以下内容："
    echo "  ✓ RViz 界面正常显示"
    echo "  ✓ 无明显错误日志"
    echo "  ✓ 机器人模型加载完成"
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# =============================================================================
# 函数: 显示测试菜单
# =============================================================================
show_menu() {
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}请选择测试类型:${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "  1) 快速检查 (2秒)           - 查看当前系统状态"
    echo "  2) 完整性能测试 (30-60秒)    - 生成详细报告"
    echo "  3) 两者都运行                - 先快速检查，再完整测试"
    echo "  4) 查看测试文档              - 显示帮助信息"
    echo "  5) 退出"
    echo ""
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo ""
}

# =============================================================================
# 函数: 运行快速检查
# =============================================================================
run_quick_check() {
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}运行快速检查...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    
    cd "$SCRIPT_DIR"
    if [ -x "./quick_check.sh" ]; then
        ./quick_check.sh
    else
        echo -e "${RED}错误: quick_check.sh 不存在或无执行权限${NC}"
        return 1
    fi
}

# =============================================================================
# 函数: 运行完整测试
# =============================================================================
run_full_test() {
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}运行完整性能测试...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    
    cd "$SCRIPT_DIR"
    if [ -x "./performance_test.sh" ]; then
        ./performance_test.sh
    else
        echo -e "${RED}错误: performance_test.sh 不存在或无执行权限${NC}"
        return 1
    fi
}

# =============================================================================
# 函数: 显示文档
# =============================================================================
show_docs() {
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}测试文档${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "可用文档:"
    echo ""
    echo "  1. PERFORMANCE_TESTING_README.md  - 测试套件总览（推荐首读）"
    echo "  2. TESTING_GUIDE.md               - 详细测试指南"
    echo "  3. process_analysis.md            - 架构分析与优化方案"
    echo "  4. FILES_SUMMARY.md               - 文件清单"
    echo ""
    echo "查看文档:"
    echo -e "  ${CYAN}cat test/PERFORMANCE_TESTING_README.md${NC}"
    echo -e "  ${CYAN}cat test/TESTING_GUIDE.md${NC}"
    echo ""
    
    read -p "按回车键返回主菜单... " -r
}

# =============================================================================
# 主函数
# =============================================================================
main() {
    # 检查是否在工作空间
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        echo -e "${RED}错误: 不在正确的工作空间目录${NC}"
        echo "当前目录: $WORKSPACE_DIR"
        exit 1
    fi
    
    # 检查系统运行状态
    if check_system_running; then
        echo -e "${GREEN}✓ 检测到导航系统正在运行${NC}"
        echo ""
    else
        show_startup_guide
        
        echo -e "${YELLOW}等待系统启动...${NC}"
        echo ""
        read -p "启动完成后，按回车键继续... " -r
        echo ""
        
        # 再次检查
        if ! check_system_running; then
            echo -e "${RED}错误: 仍未检测到导航系统运行${NC}"
            echo ""
            echo "提示："
            echo "  1. 请确保按照上述步骤启动系统"
            echo "  2. 等待3-5秒让所有节点启动完成"
            echo "  3. 如果问题持续，检查终端的错误日志"
            echo ""
            exit 1
        fi
        
        echo -e "${GREEN}✓ 系统检测成功${NC}"
        echo ""
    fi
    
    # 显示菜单循环
    while true; do
        show_menu
        read -p "请输入选项 [1-5]: " choice
        echo ""
        
        case $choice in
            1)
                run_quick_check
                echo ""
                read -p "按回车键返回主菜单... " -r
                clear
                ;;
            2)
                run_full_test
                echo ""
                read -p "按回车键返回主菜单... " -r
                clear
                ;;
            3)
                run_quick_check
                echo ""
                echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
                echo ""
                read -p "继续执行完整测试？按回车继续，Ctrl+C取消... " -r
                echo ""
                run_full_test
                echo ""
                read -p "按回车键返回主菜单... " -r
                clear
                ;;
            4)
                show_docs
                clear
                ;;
            5)
                echo -e "${GREEN}测试完成，退出${NC}"
                echo ""
                exit 0
                ;;
            *)
                echo -e "${RED}无效选项，请重新选择${NC}"
                echo ""
                sleep 2
                clear
                ;;
        esac
    done
}

# 捕获 Ctrl+C
trap 'echo -e "\n${YELLOW}测试已取消${NC}"; exit 1' INT

# 执行主函数
main "$@"
