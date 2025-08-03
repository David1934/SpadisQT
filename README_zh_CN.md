## **SpadisQT 应用（用于 ADAPS ADS6401 dToF 传感器）**

这是一个基于 ADS6401 dToF 传感器的演示应用，名为 "SpadisQT"，运行于嵌入式 Linux 系统。
通过V4L2框架，采集我司swift dToF芯片的mipi原始数据，经过我司自研发的算法库处理，获得深度或灰度数据，
并转换为RGB色彩来展示不同的深度。

已在 RK3568（Linux 5.10 内核）平台上完成测试。

该应用需使用 QT 5.x 在 Linux 环境下编译。（注意：由于使用了 V4L2 API，无法在 Windows 运行。）

在构建和运行 SpadisQT 前，请确保开发板支持 QT 环境。  
此类基础问题请自行研究，我们无法逐一解答。

ADS6401 芯片支持两种 dToF 模块：  
- **SPOT 模块**  
- **FLOOD 模块**  

当执行 `ADAPS_GET_DTOF_MODULE_STATIC_DATA` ioctl 命令时，Linux 内核驱动（`ads6401`）会返回当前模块类型。

### **部署步骤**  
按以下步骤将 SpadisQT 部署到开发板：

#### **步骤 1：创建必要目录**  
执行以下命令：  
```bash
mkdir -p /vendor/lib64
mkdir -p /vendor/etc/camera
mkdir -p /data/vendor/camera
```

步骤 2A：通过 SSH 复制文件
若开发板支持 SSH，使用以下命令（请替换你的实际构建路径和 IP 地址）：

```
scp /您的/构建/路径/libadaps_swift_decode.so root@[开发板IP]:/vendor/lib64/
scp /您的/构建/路径/libAdapsSender.so root@[开发板IP]:/vendor/lib64/
scp /您的/构建/路径/adapsdepthsettings.xml root@[开发板IP]:/vendor/etc/camera/
scp /您的/构建/路径/SpadisQT root@[开发板IP]:/usr/bin/
scp /您的/构建/路径/SpadisQT_console root@[开发板IP]:/usr/bin/
```

步骤 2B：通过 ADB 复制文件
若使用 ADB，使用以下命令（请替换你的实际构建路径）：

```
adb push /您的/构建/路径/libadaps_swift_decode.so /vendor/lib64/
adb push /您的/构建/路径/libAdapsSender.so /vendor/lib64/
adb push /您的/构建/路径/adapsdepthsettings.xml /vendor/etc/camera/
adb push /您的/构建/路径/SpadisQT /usr/bin/
adb push /您的/构建/路径/SpadisQT_console /usr/bin/
```

步骤 3：设置可执行权限
```
chmod +x /usr/bin/SpadisQT
chmod +x /usr/bin/SpadisQT_console
```

如有问题，请联系：[ADAPS Photonics](https://adapsphotonics.com/)。

许可证
SpadisQT 遵循以下协议：

[GNU LGPLv3](https://opensource.org/licenses/LGPL-3.0)
[QT LGPL](https://doc.qt.io/archives/qt-5.15/lgpl.html)