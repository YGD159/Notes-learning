### vsomeip

##### 版权

Copyright (C) 2015-2017, Bayerische Motoren Werke Aktiengesellschaft (BMW AG)

##### 执照

This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

##### vsomeip 概述

------

- vsomeip 堆栈实现了http://some-ip.com/（基于 IP 的可扩展的面向服务的中间件 (SOME/IP)）协议。堆栈包括：
  - SOME/IP 的共享库 ( `libvsomeip3.so`)
  - SOME/IP 服务发现的第二个共享库 ( `libvsomeip3-sd.so`)，如果启用了服务发现，则在运行时加载。

- ##### Linux 构建说明

  ###### 依赖关系

  - 需要启用 C++11 的编译器，例如 gcc >= 4.8。
  - vsomeip 使用 CMake 作为构建系统。
  - vsomeip 使用 Boost >= 1.55：

Ubuntu 14.04:

```
sudo apt-get install libboost-system1.55-dev libboost-thread1.55-dev libboost-log1.55-dev
```

Ubuntu 12.04: a PPA is necessary to use version 1.54 of Boost: -- URL: https://launchpad.net/~boost-latest/+archive/ubuntu/ppa 

```bash
sudo add-apt-repository ppa:boost-latest/ppa

sudo apt-get install libboost-system1.55-dev libboost-thread1.55-dev libboost-log1.55-dev
```

对于测试，

需要 Google 的测试框架 https://code.google.com/p/googletest/[gtest]  1.7.0 版。

网址：[https](https://googletest.googlecode.com/files/gtest-1.7.0.zip) : [//googletest.googlecode.com/files/gtest-1.7.0.zip](https://googletest.googlecode.com/files/gtest-1.7.0.zip)

要构建文档 asciidoc，需要 source-highlight、doxygen 和 graphviz： 

```bash
sudo apt-get install asciidoc source-highlight doxygen graphviz
```



###### 汇编

对于编译调用：

```
mkdir build
cd build
cmake ..
make
```

要指定安装目录（就像`--prefix=`您习惯使用 autotools 一样）调用 cmake 如下：

```
cmake -DCMAKE_INSTALL_PREFIX:PATH=$YOUR_PATH ..
make
make install
```

###### 使用预定义的单播和/或诊断地址进行编译

要预定义单播地址，请像这样调用 cmake：

```
cmake -DUNICAST_ADDRESS=<IP地址> ..
```

要预定义诊断地址，请调用 cmake，如下所示：

```
cmake -DDIAGNOSIS_ADDRESS=<IP地址> ..
```

诊断地址是单字节值。

###### 使用自定义默认配置文件夹编译

要更改默认配置文件夹，请调用 cmake 如下：

```
cmake -DDEFAULT_CONFIGURATION_FOLDER=<默认配置文件夹> ..
```

默认配置文件夹   /etc/vsomeip.

###### 使用自定义默认配置文件编译

要更改默认配置文件，请调用 cmake，如下所示：

```
cmake -DDEFAULT_CONFIGURATION_FILE=<默认配置文件> ..
```

默认配置文件是 /etc/vsomeip.json。

###### 使用信号处理进行编译

要在启用信号处理（SIGINT/SIGTERM）的情况下编译 vsomeip，请调用 cmake，如下所示：

```
cmake -DENABLE_SIGNAL_HANDLING=1 ..
```

在默认设置中，如果收到这些信号，应用程序必须关闭 vsomeip。

##### Android 构建说明

###### 依赖关系

- vsomeip 使用 Boost >= 1.55。boost 库（系统、线程和日志）必须包含在 Android 源代码树中，并使用适当的 Android.bp 文件集成到构建过程中。

###### 汇编

一般来说，对于构建 Android 源代码树，Android 开源项目 (AOSP) 页面上的说明适用 ( https://source.android.com/setup/build/requirements )。

要将 vsomeip 库集成到构建过程中，必须将源代码与 Android.bp 文件一起插入到 Android 源代码树中（通过简单地复制或使用自定义平台清单获取）。在构建 Android 源代码树时，构建系统会自动找到并考虑 Android.bp 文件。

为了将 vsomeip 库也包含在 Android 映像中，必须将该库添加到设备/目标特定生成文件之一的 PRODUCT_PACKAGES 变量中：

```
PRODUCT_PACKAGES += \
    libvsomeip \
    libvsomeip_cfg \
    libvsomeip_sd
```