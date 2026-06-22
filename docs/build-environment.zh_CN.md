# SpadisQT — 构建环境

[English](build-environment.md) · [文档索引](README.zh_CN.md)

SpadisQT 是一个**嵌入式 Linux Qt 5 应用**。发布二进制为 **aarch64** 且链接两个仅 aarch64
的预编译库，因此真正的目标构建需要 **aarch64 Qt 5 交叉工具链（板端 Qt sysroot）**，或在
板上使用原生 Qt 工具链。由于它使用 Linux V4L2 API，**无法在 Windows 上构建或运行。**

本文还介绍可在 x86_64 工作站上运行的**宿主机编译验证**，用于校验源码可编译——这在 CI 中
以及尚未拿到 sysroot 时很有用。

---

## 1. 依赖

| 需求 | 软件包 / 来源 |
|------|----------------|
| Qt 5.x（Core；GUI 构建另需 Gui/Widgets） | 目标的 aarch64 Qt sysroot；宿主机可用发行版 `qtbase5-dev qtbase5-dev-tools` |
| `qmake`、`moc`、`uic` | `qtbase5-dev-tools`（宿主机）或 sysroot 的 `bin/` |
| OpenSSL + zlib | `libssl-dev zlib1g-dev`（应用链接 `-lssl -lcrypto -lz`） |
| 预编译算法库 | `libadaps_swift_decode.so`、`libAdapsSender.so`（已入库，aarch64） |
| Graphviz（仅文档） | `graphviz` —— 供 `tools/gen_diagrams.sh` 使用 |

---

## 2. 目标构建（aarch64）

将 aarch64 Qt5 交叉工具链置于 `PATH`（其 `qmake` 优先），或直接在板上：

```bash
qmake SpadisQT.pro            # GUI 目标  -> SpadisQT
make -j"$(nproc)"
qmake SpadisQT_console.pro    # 无界面     -> SpadisQT_console
make -j"$(nproc)"
```

`CONFIG(debug, debug|release)` 选择模式：Release 定义 `BUILD_4_RELEASE` 并加上
`-O3 -flto -fno-exceptions`；Debug 保留 `-rdynamic`，使 `main.cpp` 中的 `backtrace()`
崩溃处理能解析函数名。产物在板上的落点见 [部署布局](../README_zh_CN.md#5-部署布局)。

---

## 3. 宿主机编译验证（x86_64）

即便没有 sysroot，你也可以在普通 x86_64 Linux 机器上校验源码能否编译。**链接步骤会失败**，
因为预编译 `.so` 是 aarch64——这是预期内的；目的是捕获编译错误。

```bash
# Ubuntu 24.04 示例
sudo apt-get install -y qtbase5-dev qtbase5-dev-tools libssl-dev zlib1g-dev

# 源外构建，保持仓库干净
mkdir -p /tmp/spadis_build && cd /tmp/spadis_build
qmake CONFIG+=debug /path/to/SpadisQT/SpadisQT.pro
make -j"$(nproc)" -k        # -k：出错继续，使所有目标文件都被编译
```

**预期结果** —— 每个 `.cpp`（及生成的 `moc_*`）都编译为 `.o`，随后最终链接以如下信息中止：

```
/usr/bin/ld: skipping incompatible .../libadaps_swift_decode.so when searching for -ladaps_swift_decode
/usr/bin/ld: cannot find -ladaps_swift_decode
```

该信息意味着**源码是健康的**，仅缺少架构匹配的链接——这正是宿主机侧语法/语义检查所期望的
结果。已在 Ubuntu 24.04 + Qt 5.15.13 上验证：14 个目标文件全部编译通过（仅告警），链接如上
失败。

> ⚠️ 在同一工作区内同时构建 RK356x SDK 的机器上，**请勿**安装 apt 的
> `gcc-aarch64-linux-gnu` / `crossbuild-essential-arm64`——它们会与 buildroot 宿主侧所需的
> `gcc-multilib` 冲突。真正的 aarch64 构建应使用板端 Qt sysroot，而非 apt 的交叉 gcc。

---

## 4. 文档工具链

`docs/images/` 下的图表由 Graphviz 源文件生成：

```bash
sudo apt-get install -y graphviz
tools/gen_diagrams.sh        # 渲染 docs/images/*.{png,svg}，并刷新 vx_images/
```

请修改 `tools/diagrams/` 中的 `.dot` 源文件，切勿改渲染出的图片。参见
[文档索引](README.zh_CN.md#图表)。
