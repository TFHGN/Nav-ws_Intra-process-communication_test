# Terrain Analysis 文档生成指南

## 📚 Doxygen 注释概览

本项目已为 `terrain_analysis(new).cpp` 添加了完整的 Doxygen 中文注释，包括：

### 注释覆盖范围

#### 1. **文件级注释**
- 文件描述、作者、日期
- 功能概述与架构特性
- 详细的实现说明

#### 2. **类级注释**
- `TerrainAnalysisNode` 类的完整文档
- 数据流程说明
- 处理模式描述
- 坐标系规范

#### 3. **构造函数注释**
- 初始化步骤详解
- 参数说明
- 关键配置项
- 注意事项和警告

#### 4. **回调函数注释**
所有回调函数均包含：
- 功能描述
- 参数说明
- 详细的处理流程
- 使用说明和注意事项

**已注释的回调函数：**
- `odometryHandler()` - 里程计处理
- `laserCloudHandler()` - 点云处理
- `clearingHandler()` - 地图清空
- `timerCallback()` - 主处理循环

#### 5. **成员变量注释**
所有 150+ 个成员变量均包含：
- 简短描述（@brief）
- 单位说明（米、秒、度、弧度等）
- 用途说明
- 数据结构规格

**变量分类：**
- 参数存储（30+ 个配置参数）
- 状态变量（5 个运行时状态）
- 几何计算变量（14 个位姿相关）
- 体素参数（10 个网格配置）
- 体素数据数组（6 个数据容器）
- 点云数据（7 个点云对象）
- ROS2 接口（6 个发布/订阅/定时器）

---

## 🔧 生成文档

### 前置条件

安装 Doxygen 和 Graphviz（用于生成图表）：

```bash
sudo apt-get update
sudo apt-get install doxygen graphviz
```

### 生成步骤

1. **进入包目录**：
```bash
cd /home/tfhgn/nav2_ws/src/pb2025_sentry_nav/terrain_analysis
```

2. **运行 Doxygen**：
```bash
doxygen Doxyfile
```

3. **查看生成的文档**：
```bash
# HTML 文档（推荐）
firefox docs/html/index.html

# 或使用其他浏览器
google-chrome docs/html/index.html
xdg-open docs/html/index.html
```

### 输出结构

```
terrain_analysis/
├── docs/
│   └── html/
│       ├── index.html          # 文档首页
│       ├── annotated.html      # 类列表
│       ├── files.html          # 文件列表
│       ├── functions.html      # 函数索引
│       ├── globals.html        # 全局符号
│       └── ...                 # 其他页面
└── doxygen_warnings.log       # 警告日志
```

---

## 📖 文档使用指南

### 导航结构

生成的文档包含以下主要部分：

1. **首页（Main Page）**
   - 项目概述
   - 快速开始

2. **类列表（Classes）**
   - `TerrainAnalysisNode` 类文档
   - 继承关系图
   - 协作图

3. **文件列表（Files）**
   - `terrain_analysis(new).cpp` 详细文档
   - 包含关系图

4. **命名空间（Namespaces）**
   - `terrain_analysis` 命名空间