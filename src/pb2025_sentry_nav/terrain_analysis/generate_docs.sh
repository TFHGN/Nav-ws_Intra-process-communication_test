#!/bin/bash
# Doxygen 文档生成与验证脚本
# Generate and Validate Doxygen Documentation

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Terrain Analysis 文档生成工具${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 Doxygen 是否安装
echo -e "${YELLOW}[1/5] 检查 Doxygen 安装...${NC}"
if ! command -v doxygen &> /dev/null; then
    echo -e "${RED}❌ Doxygen 未安装${NC}"
    echo -e "${YELLOW}安装命令: sudo apt-get install doxygen graphviz${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Doxygen 已安装: $(doxygen --version)${NC}"
echo ""

# 检查 Graphviz
echo -e "${YELLOW}[2/5] 检查 Graphviz (用于生成图表)...${NC}"
if ! command -v dot &> /dev/null; then
    echo -e "${YELLOW}⚠️  Graphviz 未安装，将无法生成调用图和类图${NC}"
    echo -e "${YELLOW}安装命令: sudo apt-get install graphviz${NC}"
else
    echo -e "${GREEN}✅ Graphviz 已安装${NC}"
fi
echo ""

# 检查 Doxyfile 是否存在
echo -e "${YELLOW}[3/5] 检查配置文件...${NC}"
if [ ! -f "Doxyfile" ]; then
    echo -e "${RED}❌ Doxyfile 不存在${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Doxyfile 找到${NC}"
echo ""

# 生成文档
echo -e "${YELLOW}[4/5] 生成文档...${NC}"
doxygen Doxyfile 2>&1 | tee doxygen_output.log

# 检查是否有错误
if grep -q "error:" doxygen_output.log; then
    echo -e "${RED}❌ 文档生成过程中有错误${NC}"
    echo -e "${RED}查看 doxygen_output.log 了解详情${NC}"
    exit 1
fi
echo -e "${GREEN}✅ 文档生成完成${NC}"
echo ""

# 统计与验证
echo -e "${YELLOW}[5/5] 验证文档...${NC}"

# 检查输出目录
if [ ! -d "docs/html" ]; then
    echo -e "${RED}❌ HTML 文档未生成${NC}"
    exit 1
fi

# 统计文件数量
html_files=$(find docs/html -name "*.html" | wc -l)
echo -e "${GREEN}✅ 生成了 ${html_files} 个 HTML 文件${NC}"

# 检查警告
if [ -f "doxygen_warnings.log" ]; then
    warning_count=$(wc -l < doxygen_warnings.log)
    if [ $warning_count -gt 0 ]; then
        echo -e "${YELLOW}⚠️  有 ${warning_count} 个警告${NC}"
        echo -e "${YELLOW}查看 doxygen_warnings.log 了解详情${NC}"
        echo ""
        echo -e "${YELLOW}警告摘要:${NC}"
        head -n 10 doxygen_warnings.log
        if [ $warning_count -gt 10 ]; then
            echo -e "${YELLOW}... (还有 $((warning_count - 10)) 个警告)${NC}"
        fi
    else
        echo -e "${GREEN}✅ 没有警告！${NC}"
    fi
fi
echo ""

# 生成报告
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  文档生成完成！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}📂 输出目录: docs/html/${NC}"
echo -e "${GREEN}📄 首页: docs/html/index.html${NC}"
echo ""
echo -e "${YELLOW}打开文档:${NC}"
echo -e "  ${BLUE}firefox docs/html/index.html${NC}"
echo -e "  ${BLUE}google-chrome docs/html/index.html${NC}"
echo -e "  ${BLUE}xdg-open docs/html/index.html${NC}"
echo ""

# 代码统计
echo -e "${YELLOW}代码统计:${NC}"
if [ -f "src/terrain_analysis(new).cpp" ]; then
    total_lines=$(wc -l < src/terrain_analysis\(new\).cpp)
    comment_lines=$(grep -c "^\s*/\*\|^\s*\*\|^\s*//\|^\s*///" src/terrain_analysis\(new\).cpp || true)
    code_lines=$((total_lines - comment_lines))
    
    echo -e "  总行数: ${total_lines}"
    echo -e "  注释行: ${comment_lines}"
    echo -e "  代码行: ${code_lines}"
    
    if [ $comment_lines -gt 0 ] && [ $total_lines -gt 0 ]; then
        ratio=$(echo "scale=1; $comment_lines * 100 / $total_lines" | bc)
        echo -e "  注释率: ${ratio}%"
    fi
fi
echo ""

# 提供下一步建议
echo -e "${YELLOW}建议的下一步:${NC}"
echo -e "  1. 在浏览器中查看生成的文档"
echo -e "  2. 检查 doxygen_warnings.log 修复警告"
echo -e "  3. 根据需要调整 Doxyfile 配置"
echo -e "  4. 考虑将文档部署到 GitHub Pages"
echo ""

echo -e "${GREEN}✨ 所有完成！${NC}"
