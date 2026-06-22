# SpadisQT — Build Environment

[简体中文](build-environment.zh_CN.md) · [Docs index](README.md)

SpadisQT is an **embedded-Linux Qt 5 application**. The shipping binary is **aarch64** and
links two aarch64-only prebuilt libraries, so a real target build needs an **aarch64 Qt 5
cross-toolchain (the board's Qt sysroot)** or a native Qt toolchain on the board itself.
Because it uses Linux V4L2 APIs, **it cannot build or run on Windows.**

This document also describes a **host compile-check** you can run on an x86_64 workstation to
validate that the sources compile — useful in CI and for catching errors before you have a
sysroot.

---

## 1. Dependencies

| Need | Package / source |
|------|------------------|
| Qt 5.x (Core; +Gui/Widgets for the GUI build) | aarch64 Qt sysroot for the target; on a host, distro `qtbase5-dev qtbase5-dev-tools` |
| `qmake`, `moc`, `uic` | `qtbase5-dev-tools` (host) or the sysroot's `bin/` |
| OpenSSL + zlib | `libssl-dev zlib1g-dev` (the app links `-lssl -lcrypto -lz`) |
| Prebuilt algo libs | `libadaps_swift_decode.so`, `libAdapsSender.so` (checked into the repo, aarch64) |
| Graphviz (docs only) | `graphviz` — for `tools/gen_diagrams.sh` |

---

## 2. Target build (aarch64)

With an aarch64 Qt5 cross-toolchain on `PATH` (its `qmake` first), or on the board:

```bash
qmake SpadisQT.pro            # GUI target  -> SpadisQT
make -j"$(nproc)"
qmake SpadisQT_console.pro    # headless     -> SpadisQT_console
make -j"$(nproc)"
```

`CONFIG(debug, debug|release)` selects the mode: Release defines `BUILD_4_RELEASE` and adds
`-O3 -flto -fno-exceptions`; Debug keeps `-rdynamic` so the `backtrace()` crash handler in
`main.cpp` resolves function names. See [Deployment](../README.md#5-deployment-layout) for
where the artifacts go on the board.

---

## 3. Host compile-check (x86_64)

You can verify the sources compile on a normal x86_64 Linux box even without a sysroot. The
**link step will fail** because the prebuilt `.so` are aarch64 — that is expected; the goal
is to catch compile errors.

```bash
# Ubuntu 24.04 example
sudo apt-get install -y qtbase5-dev qtbase5-dev-tools libssl-dev zlib1g-dev

# out-of-source build so the repo stays clean
mkdir -p /tmp/spadis_build && cd /tmp/spadis_build
qmake CONFIG+=debug /path/to/SpadisQT/SpadisQT.pro
make -j"$(nproc)" -k        # -k: keep going so all objects compile
```

**Expected result** — every `.cpp` (and the generated `moc_*`) compiles to a `.o`, then the
final link aborts with:

```
/usr/bin/ld: skipping incompatible .../libadaps_swift_decode.so when searching for -ladaps_swift_decode
/usr/bin/ld: cannot find -ladaps_swift_decode
```

That message means **the source is healthy** and only the architecture-mismatched link is
missing — exactly what you want from a host-side syntax/semantic check. This was verified
with Qt 5.15.13 on Ubuntu 24.04: all 14 objects compiled (warnings only), link failed as
above.

> ⚠️ Do **not** install apt's `gcc-aarch64-linux-gnu` / `crossbuild-essential-arm64` on a
> machine that also builds the RK356x SDK in this workspace — they conflict with the
> `gcc-multilib` the buildroot host side needs. A real aarch64 build should use the board's
> Qt sysroot, not apt's cross-gcc.

---

## 4. Documentation toolchain

The diagrams under `docs/images/` are generated from Graphviz sources:

```bash
sudo apt-get install -y graphviz
tools/gen_diagrams.sh        # renders docs/images/*.{png,svg}, refreshes vx_images/
```

Edit the `.dot` sources in `tools/diagrams/`, never the rendered images. See the
[docs index](README.md#diagrams).
