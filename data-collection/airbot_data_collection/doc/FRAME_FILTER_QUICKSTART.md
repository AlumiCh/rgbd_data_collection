# 帧过滤功能 - 快速参考

**功能**: 只保存第1帧RGBD图像 + 最后1帧关节角  
**状态**: 可选功能，默认关闭  
**影响**: 不影响其他用户

---

## 🚀 快速启用

在你的配置文件 `config_single_arm_dual_rgbd.yaml` 中：

```yaml
sampler:
  param:
    _target_: airbot_data_collection.airbot.samplers.mcap_sampler.AIRBOTMcapDataSampler
    
    # 添加这一行启用帧过滤
    filter_first_last_frame: true
    
    task_info:
      # ... 其他配置 ...
```

同时调整采样帧数：
```yaml
sample_limit:
  size: 60  # 建议30-100帧
```

---

## ✅ 向后兼容性

### 其他用户无需担心
- ❌ **不添加** `filter_first_last_frame` 配置项 → 功能等同于原版
- ✅ **所有帧都会正常保存**，无任何变化
- ✅ 已有配置文件无需修改

### 代码修改说明
- 所有修改用 `# ========== 可选功能 (2026-01-08) ==========` 标注
- 默认行为：`filter_first_last_frame = False`（不启用）
- 条件执行：只有配置为 `true` 时才会调用过滤函数

---

## 📝 工作原理

```
采集阶段（60帧）:
├── 第1帧: 机械臂远离，记录物体点云
├── 第2-59帧: 机械臂移动过程（会被过滤）
└── 第60帧: 目标抓取位姿，记录关节角

保存到MCAP:
├── RGBD图像: 只保存第1帧
└── 关节角: 只保存第60帧（最后1帧）
```

---

## 📚 详细文档

完整说明请查看：[CUSTOM_MODIFICATION_README.md](./CUSTOM_MODIFICATION_README.md)

---

## 🔍 代码位置

**文件**: `airbot/samplers/mcap_sampler.py`
- 配置参数：第88-95行
- 过滤逻辑：第130-188行（`_filter_first_and_last_frame` 方法）
- 调用位置：第215-221行（`save` 方法开头）

**搜索关键词**: `可选功能` 或 `filter_first_last_frame`
