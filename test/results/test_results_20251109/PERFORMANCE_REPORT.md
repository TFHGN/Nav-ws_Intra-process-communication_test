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
