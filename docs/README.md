# SpadisQT Developer Documentation

[简体中文](README.zh_CN.md) · [Project README](../README.md) · [Porting Guide](../documents/SpadisQT_Porting_Guide_EN.md)

This folder is the **technical reference** for developers working *inside* the SpadisQT
code base. For a hands-on, step-by-step porting/integration walkthrough (hardware bring-up,
toolchain, deployment, tuning), use the [Porting & Development Guide](../documents/SpadisQT_Porting_Guide_EN.md)
instead.

## Contents

| Document | What it covers |
|----------|----------------|
| [Architecture](architecture.md) | Layered design, the seven core classes, threading model, compile-time feature switches. |
| [Data Flow](data-flow.md) | The frame pipeline end to end, the depth16 format, work modes, and per-frame control flow. |
| [API Reference](api-reference.md) | Public class APIs, key enums/structs, the algo-lib `DepthMapWrapper*` contract, host commands, `/dev/ads6401` ioctls, and runtime env-var toggles. |
| [Build Environment](build-environment.md) | Local dependency setup, the aarch64 cross-build requirement, and how to run a host compile-check. |

## Diagrams

All diagrams are generated from Graphviz sources under [`tools/diagrams/`](../tools/diagrams/)
by [`tools/gen_diagrams.sh`](../tools/gen_diagrams.sh). **Do not edit the rendered images by
hand** — edit the `.dot` source and re-run the script:

```bash
sudo apt-get install -y graphviz   # one-time
tools/gen_diagrams.sh              # renders docs/images/*.{png,svg} and refreshes vx_images/
```

| Diagram | Source | Rendered |
|---------|--------|----------|
| System architecture | `tools/diagrams/arch_overview.dot` | [`images/arch_overview.svg`](images/arch_overview.svg) |
| Frame data flow | `tools/diagrams/data_flow.dot` | [`images/data_flow.svg`](images/data_flow.svg) |
| Single-frame sequence | `tools/diagrams/capture_sequence.dot` | [`images/capture_sequence.svg`](images/capture_sequence.svg) |
| Threading model | `tools/diagrams/threading_model.dot` | [`images/threading_model.svg`](images/threading_model.svg) |
| Deployment layout | `tools/diagrams/deployment.dot` | [`images/deployment.svg`](images/deployment.svg) |

> The SVG files are crisp at any zoom; the PNG variants are used where Markdown renderers
> prefer raster images (e.g. the README).
