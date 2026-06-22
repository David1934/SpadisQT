# SpadisQT 开发者文档

[English](README.md) · [项目说明](../README_zh_CN.md) · [移植与开发指南](../documents/SpadisQT_移植与开发指南.md)

本目录是面向**在 SpadisQT 代码内部开发**的工程师的**技术参考**。如果你需要从零开始的
上手式移植/集成步骤（硬件点亮、工具链、部署、调优），请改用
[移植与开发指南](../documents/SpadisQT_移植与开发指南.md)。

## 目录

| 文档 | 内容 |
|------|------|
| [软件架构](architecture.zh_CN.md) | 分层设计、七个核心类、线程模型、编译期特性开关。 |
| [数据流程](data-flow.zh_CN.md) | 端到端的帧处理流水线、depth16 格式、工作模式、单帧控制流。 |
| [API 文档](api-reference.zh_CN.md) | 各类公开 API、关键枚举/结构体、算法库 `DepthMapWrapper*` 接口契约、上位机命令、`/dev/ads6401` ioctl、运行期环境变量开关。 |
| [构建环境](build-environment.zh_CN.md) | 本地依赖部署、aarch64 交叉构建要求、宿主机编译验证方法。 |

## 图表

所有图表均由 [`tools/diagrams/`](../tools/diagrams/) 下的 Graphviz 源文件通过
[`tools/gen_diagrams.sh`](../tools/gen_diagrams.sh) 生成。**请勿手动修改渲染出的图片**——
修改 `.dot` 源文件后重新运行脚本即可：

```bash
sudo apt-get install -y graphviz   # 一次性安装
tools/gen_diagrams.sh              # 渲染 docs/images/*.{png,svg} 并刷新 vx_images/
```

| 图表 | 源文件 | 渲染结果 |
|------|--------|----------|
| 系统架构 | `tools/diagrams/arch_overview.dot` | [`images/arch_overview.svg`](images/arch_overview.svg) |
| 帧数据流 | `tools/diagrams/data_flow.dot` | [`images/data_flow.svg`](images/data_flow.svg) |
| 单帧时序 | `tools/diagrams/capture_sequence.dot` | [`images/capture_sequence.svg`](images/capture_sequence.svg) |
| 线程模型 | `tools/diagrams/threading_model.dot` | [`images/threading_model.svg`](images/threading_model.svg) |
| 部署布局 | `tools/diagrams/deployment.dot` | [`images/deployment.svg`](images/deployment.svg) |

> SVG 在任意缩放下都清晰；PNG 版本用于偏好位图的 Markdown 渲染场景（如 README）。
