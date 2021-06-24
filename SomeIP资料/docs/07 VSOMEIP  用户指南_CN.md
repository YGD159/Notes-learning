# VSOME/IP  用户指南_CN

一、vsomeip
-------

### (一) vsomeip

// 这使一个 TOC 作为侧边栏
**:toc2:**
// 显示目录中的所有标题
**:toclevels: 4**
// 如果使用了 TIP: 或 IMPORTANT 等，则显示图标
**:icons:**
// 设置可以找到默认图标的目录
**:iconsdir: {asciidoc-confdir}/{iconsdir}**
// 编号所有标题
**:numbered:**
// 这会将图像（例如 TIP: $TEXT 的图标）嵌入到 html 文件中
**:data-uri:**



**版权**

+++++++

版权所有 (C) 2015-2019, Bayerische Motoren Werke Aktiengesellschaft (BMW AG)执照

+++++++

本源代码表受 Mozilla Public 的条款约束许可证，第 2.0 版。如果 MPL 的副本未随此分发
文件，您可以在 http://mozilla.org/MPL/2.0/ 获得一个。版本

+++++++

// 将版本设置为我们从 cmake 获得的版本
// 或通过 -a version=$VSOMEIP_VERSION 传递给 asciidoc本文档是为 vsomeip 的 {version} 版本生成的。

### (二) vsomeip 概述

vsomeip 栈实现了 http://some-ip.com/[Scalable service-Oriented Middleware over IP (SOME/IP)] 协议。堆栈包括：

* SOME/IP 的共享库（`libvsomeip.so`）
* 用于 SOME/IP 服务发现的第二个共享库 (`libvsomeip-sd.so`)如果启用了服务发现，则在运行时加载。

二、构建说明
------------------

### (一) 依赖关系


* 需要启用 C++11 的编译器，例如 gcc >= 5.2。
* vsomeip 使用 cmake 作为构建系统。
* vsomeip 使用 Boost >= 1.55：
  ** Ubuntu 14.04：
  ** `sudo apt-get install libboost-system1.55-dev libboost-thread1.55-dev
    libboost-log1.55-dev
  ** Ubuntu 12.04：使用 Boost 1.54 版需要 PPA：
  *** 网址：https://launchpad.net/~boost-latest/+archive/ubuntu/ppa
  *** `sudo add-apt-repository ppa:boost-latest/ppa`
  *** `sudo apt-get install libboost-system1.55-dev libboost-thread1.55-dev
    libboost-log1.55-dev`
* 用于测试 Google 的测试框架
  需要 1.7.0 版中的 https://code.google.com/p/googletest/[gtest]
  ** 网址：https://googletest.googlecode.com/files/gtest-1.7.0.zip[直接链接，
   版本 1.7.0]
* 要构建文档 asciidoc，需要 source-highlight、doxygen 和 graphviz：
  ** `sudo apt-get install asciidoc source-highlight doxygen graphviz`

### (二) 汇编

```bash
mkdir build
cd build
cmake ..
```

要指定安装目录（就像`--prefix=`您习惯使用 autotools 一样）调用 cmake 如下：

```bash
cmake -DCMAKE_INSTALL_PREFIX:PATH=$YOUR_PATH ..
make
```



### (三) 安装

使用预定义的基本路径进行编译
预定义基本路径，用于创建本地套接字的路径， 像这样调用 cmake：

```bash
cmake -DBASE_PATH=<YOUR BASE PATH> ..
```



默认的基本路径是 /tmp。

使用预定义的单播和/或诊断地址进行编译，要预定义单播地址，请像这样调用 cmake：

```bash
cmake -DUNICAST_ADDRESS=<YOUR IP ADDRESS> ..
```



要预定义诊断地址，请调用 cmake，如下所示：

```bash
cmake -DDIAGNOSIS_ADDRESS=<YOUR DIAGNOSIS ADDRESS> ..
```

诊断地址是单字节值。



使用自定义默认配置文件夹编译，要更改默认配置文件夹，请调用 cmake 如下：

```bash
cmake -DDEFAULT_CONFIGURATION_FOLDER=<DEFAULT CONFIGURATION FOLDER> ..
```

默认配置文件夹是 /etc/vsomeip。



使用自定义默认配置文件编译，要更改默认配置文件，请调用 cmake，如下所示：

```bash
cmake -DDEFAULT_CONFIGURATION_FILE=<DEFAULT CONFIGURATION FILE> ..
```

默认配置文件是 /etc/vsomeip.json。



使用信号处理进行编译，要在启用信号处理 (SIGINT/SIGTERM) 的情况下编译 vsomeip，像这样调用 cmake：

```bash
cmake -DENABLE_SIGNAL_HANDLING=1 ..
```

在默认设置中，应用程序必须负责关闭
如果收到这些信号，请关闭 vsomeip。



使用用户定义的“READY”消息编译，使用用户定义的消息信号编译 vsomeip IP 路由
准备发送/接收消息，调用 cmake 如下：

```bash
cmake -DROUTING_READY_MESSAGE=<YOUR MESSAGE> ..
```



使用配置覆盖进行编译
要在启用配置覆盖的情况下编译 vsomeip，请调用 cmake如下：

```bash
cmake -DENABLE_CONFIGURATION_OVERLAYS=1 ..
```



使用 vSomeIP 2 兼容层编译

要编译启用了 vSomeIP 2 兼容层的 vsomeip，请调用cmake 如下：

```bash
cmake -DENABLE_COMPAT=1 ..
```

实例汇编
对于示例调用的编译：

```bash
mkdir buildcd buildcmake ..
```


### (四) 示例

#### 1）**测试汇编**

要编译测试，首先将 gtest 解压缩到您想要的位置。

某些测试需要同一网络上的第二个节点。有两个cmake用于自动使 json 文件适应使用的变量
网络设置：

* `TEST_IP_MASTER`：作为测试接口的IP地址
  掌握。
* `TEST_IP_SLAVE`：第二个节点的接口IP地址
  充当测试从机。

如果未指定此变量之一，则仅使用本地的测试专门的通信将是可运行的。

此外，单元测试需要启用信号处理，可以启用通过`ENABLE_SIGNAL_HANDLING` cmake 变量。

**示例，测试编译：**

```bash
mkdir buildcd buildexport GTEST_ROOT=$PATH_TO_GTEST/gtest-1.7.0/cmake -DENABLE_SIGNAL_HANDLING=1 -DTEST_IP_MASTER=10.0.3.1 -DTEST_IP_SLAVE=10.0.3.125 ..
```

#### 2）**测试检查**

测试的其他 make 目标：

* 调用`make build_tests` 只编译测试
* 在构建目录中调用 `ctest` 来执行测试，而无需冗长输出
* 运行单个测试调用 `ctest --verbose --tests-regex $TESTNAME` 短形式：`ctest -V -R $TESTNAME`
* 要列出所有可用的测试，请运行 `ctest -N`。
* 有关测试的更多信息，请查看`test` 子目录中的 `readme.txt`。

出于开发目的，存在两个 cmake 变量，它们控制json 文件和测试脚本被复制（默认）或符号链接到构建中目录。这些设置在 Windows 上被忽略。

* `TEST_SYMLINK_CONFIG_FILES`：控制是否需要 json 和脚本
  运行测试被复制或符号链接到构建目录中。（默认：
  关闭，在 Windows 上忽略）
* `TEST_SYMLINK_CONFIG_FILES_RELATIVE`：控制是否需要 json 和脚本
  运行测试的符号链接相对于构建目录。
  （默认值：OFF，在 Windows 上忽略）

示例 cmake 调用：

```bash
cmake  -DTEST_SYMLINK_CONFIG_FILES=ON -DTEST_SYMLINK_CONFIG_FILES_RELATIVE=ON ..
```

仅用于编译测试的子集（为了快速功能检查）cmake 变量`TESTS_BAT` 有要设置：

示例 cmake 调用：

```bash
cmake  -DTESTS_BAT=ON ..
```

vsomeip_ctrl 的编译
对于 <<vsomeip_ctrl>> 实用程序调用的编译：

```bash
mkdir buildcd buildcmake ..make vsomeip_ctrl
```



#### 3）**生成文档**

生成文档调用 cmake 如 <<Compilation>> 和然后调用`make doc`。
这将生成：

* README 文件: `$BUILDDIR/documentation/README.html`
* doxygen 文档： `$BUILDDIR/documentation/html/index.html`

三、启动 vsomeip 应用程序/使用的环境变量
----------------------------------------------------------

启动时会读出以下环境变量：

* `VSOMEIP_APPLICATION_NAME`：这个环境变量用于指定应用程序的名称。此名称稍后用于将客户端 ID 映射到
  配置文件中的应用程序。它独立于应用程序的二进制名称。
* `VSOMEIP_CONFIGURATION`：vsomeip 使用默认配置文件`/etc/vsomeip.json`和/或默认配置文件夹`/etc/vsomeip`。这可以被覆盖本地配置文件`./vsomeip.json` 和/或本地配置文件夹`./vsomeip`。如果 `VSOMEIP_CONFIGURATION` 设置为有效的文件或目录路径，则使用它代替的标准配置（因此既不会解析默认文件/文件夹也不会解析本地文件/文件夹）。
* `VSOMEIP_MANDATORY_CONFIGURATION_FILES`：vsomeip 允许指定强制配置文件以加速应用程序启动。虽然强制配置文件被所有人读取应用程序，所有其他配置文件只能由应用程序读取 负责连接外部设备。如果未设置此配置变量，使用默认的强制文件 vsomeip_std.json、vsomeip_app.json 和 vsomeip_plc.json。`
* `VSOMEIP_CLIENTSIDELOGGING`：将此变量设置为空字符串以启用日志记录在充当路由管理器代理的所有应用程序中任何接收到的消息到 DLT。为了例如将以下行添加到应用程序的 systemd 服务文件中：
  `环境=VSOMEIP_CLIENTSIDELOGGING=""`要启用特定于服务的日志，请提供以空格或冒号分隔的 ServiceID 列表（使用4 位十六进制表示法，可选后跟以点分隔的 InstanceID）。例如：
  `Environment=VSOMEIP_CLIENTSIDELOGGING="b003.0001 f013.000a 1001 1002"`
  `Environment=VSOMEIP_CLIENTSIDELOGGING="b003.0001:f013.000a:1001:1002"`

注意：如果`VSOMEIP_CONFIGURATION`配置的文件/文件夹不存在，将使用默认配置位置。

注意：vsomeip 将解析并使用配置文件夹中所有文件的配置但不考虑配置文件夹中的目录。

在以下示例中，启动了应用程序“my_vsomeip_application”。这些设置是从当前工作中的文件 `my_settings.json` 中读取的目录。可以在名称下找到应用程序的客户端 ID
`my_vsomeip_client` 在配置文件中。

```bash
#!/bin/bashexport VSOMEIP_APPLICATION_NAME=my_vsomeip_clientexport VSOMEIP_CONFIGURATION=my_settings.json./my_vsomeip_application
```



四、配置文件结构
----------------------------

vsomeip 的配置文件是 http://www.json.org/[JSON]-Files 并且是由多个键值对和数组组成。

____

* 对象是一组无序的名称/值对。对象以`{开头（左大括号）` 并以`}（右大括号）` 结尾。每个名称后跟`：(冒号)` 和名称/值对由`, (逗号)` 分隔。

* 数组是值的有序集合。数组以`[（左括号）`并以`]（右括号）`结尾。值由`，（逗号）`。

* 值可以是双引号中的 _string_，或 _number_，或 `true` 或 `false`或 `null`，或 _object_ 或 _array_。这些结构可以嵌套。

____

* 配置文件元素说明：


* ```bash
  'unicast'主机系统的 IP 地址。
  ```

  

* ```bahs
  'netmask'指定主机系统子网的网络掩码。
  ```

  

* ```bash
  'device' (optional)如果指定，IP 端点将绑定到此设备。
  ```

  

* ```bash
  'diagnosis' 将用于构建客户端标识符的诊断地址（字节）。这诊断地址分配给所有客户端的最高有效字节如果没有另外指定的标识符（例如通过预定义的客户端ID）。
  ```

  

* ```bash
  'diagnosis_mask'诊断掩码（2 字节）用于控制允许的最大数量ECU 上的并发 vsomeip 客户端和客户端 ID 的起始值+默认值为 `0xFF00` 的意思，客户端 ID 的最高有效字节保留用于诊断地址和客户端 ID 将以指定的诊断地址开头。最大客户端数为 255 作为倒置掩码的汉明权重是 8（2^8 = 256 - 1（对于路由管理器）= 255）。生成的客户端 ID诊断地址为例如 0x45 的范围将是 0x4501 到 0x45ff。+将掩码设置为“0xFE00”会将客户端 ID 范围加倍到 511 个客户端作为倒置掩膜的汉明权大一。带诊断地址0x45 客户端 ID 的起始值为 0x4401，因为 0x4500 中的第 8 位被屏蔽出去。这将产生一个 0x4400 到 0x45ff 的客户端 ID 范围。
  ```

  

* ```bash
  'network'用于在一台主机上支持多个路由管理器的网络标识符。这设置更改`/dev/shm` 中共享内存段的名称和名称`/tmp/` 中的 unix 域套接字。默认为 `vsomeip` 表示共享内存将被命名为 `/dev/shm/vsomeip` 并且 unix 域套接字将被命名为命名为`/tmp/vsomeip-$CLIENTID`
  ```

  

  **日志**

* 'logging'

  ```bash
  ** 'level'指定日志级别（有效值：_trace_, _debug_, _info_, _warning_,_error_, _fatal_).** 'console'指定是否启用通过控制台进行日志记录（有效值：_true、false_）。
  ```

  

* 'file'

  ```bash
   'enable'指定是否应创建日志文件（有效值：_true、false_）。'path'日志文件的绝对路径。'dlt'指定是否启用诊断日志和跟踪 (DLT)（有效值：_true, false_).'version'配置 vsomeip 版本的日志记录'enable' 启用或禁用 vsomeip 版本的循环日志记录，默认为 true（有效值：_true, false_)'interval'以秒为单位配置间隔以记录 vsomeip 版本。默认值为 10。'memory_log_interval'配置路由管理器记录其使用的时间间隔（秒）记忆。设置大于零的值将启用日志记录。'status_log_interval'配置路由管理器记录其内部的时间间隔（以秒为单位）。设置大于零的值将启用日志记录。
  ```

  

  **跟踪**

* anchor:config-tracing[]'tracing' (optional)

  ```bash
   'enable'指定是否启用对 SOME/IP 消息的跟踪 （有效值：_true、false_）。默认值为 _false_。如果启用了跟踪，消息将通过以下方式转发到 DLT <<跟踪连接器，跟踪连接器>> 'sd_enable'指定是否跟踪 SOME/IP 服务发现消息 启用（有效值：_true、false_）。默认值为 _false_。 'channels (array)' (optional)包含通向 DLT 的通道。注意：您可以设置多个通道到 DLT，您可以通过这些通道转发消息。 'name'频道的名称。 'id'频道的ID。 'filters (array)' (optional)包含应用于邮件的过滤器。注意：您可以使用过滤器分别对消息应用过滤器规则 具体标准和表述。所以只转发过滤的消息到 DLT。'channel' (optional)将过滤后的消息转发到 DLT 的通道的 id。如果未指定通道，使用默认通道 (TC)。如果你想使用一个在几个不同的频道中过滤，您可以提供一组频道 ID。注意：如果您使用具有多个通道的正过滤器，则相同的消息将多次转发给 DLT。 'matches' (optional)指定在跟踪中包含/排除消息的标准。您可以指定列表（数组）或匹配元素的范围。+一个列表可能包含单个标识符，这些标识符与来自/发送给所有消息的所有消息相匹配相应服务的实例或由服务组成的元组，实例和方法标识符。'any' 可以用作匹配的通配符所有服务、实例或方法+范围由两个元组“from”和“to”指定，每个元组由 服务、实例和方法标识符。所有带有 service- 的消息，大于或等于“from”的实例和方法标识符 和小于或等于“to”匹配。 'type' (optional)指定过滤器类型（有效值：“positive”、“negative”）。当阳性使用过滤器并且消息与过滤器规则之一匹配，消息将被跟踪/转发到 DLT。使用否定过滤器可以排除消息。所以当一个消息匹配过滤规则之一，消息将不会被跟踪/转发到分布式账本技术。默认值为“positive”。
  ```

  

  **应用**

* 'applications (array)'

  包含使用此配置文件的主机系统的应用程序。

  ```bash
   'name'应用程序的名称。 'id'应用程序的 ID。通常它的高字节等于诊断地址。在这如果低字节必须不为零。因此，如果诊断地址为 0x63，则有效值范围从 0x6301 到 0x63FF。也可以使用高字节的 id 值与诊断地址不同。  'max_dispatchers' (optional)应用于执行应用程序回调的最大线程数。默认值为 10。'max_dispatch_time' (optional)应用程序回调在回调之前可能消耗的最长时间（以毫秒为单位）是被认为是阻塞的（并且使用一个额外的线程来执行挂起的如果 max_dispatchers 配置为大于 0，则回调）。如果未指定，则默认值为 100 毫秒。 'threads' (optional)在应用程序中处理消息和事件的内部线程数。有效值为 1-255。默认值为 2。 'io_thread_nice' (optional)内部线程处理消息和事件的好级别。仅限 POSIX/Linux。有关实际值，请参阅 nice() 文档。 'request_debounce_time' (optional)指定以毫秒为单位的去抖动时间间隔，在该间隔中请求服务消息被发送到路由管理器。如果应用程序在短时间内请求多个服务发送到路由管理器的消息的负载，以及来自路由管理器的回复路由管理器（如果可用，它包含请求服务的路由信息）可以大大减少。如果未指定，则默认值为 10 毫秒。 'plugins' (optional array)包含应加载以扩展 vsomeip 功能的插件。'name'插件的名称。'type'插件类型（有效值：_application_plugin_）。+应用程序插件扩展了应用程序级别的功能。它得到通知通过 vsomeip 覆盖基本的应用程序状态 (INIT/START/STOP) 并且可以基于这些通知，通过运行时访问标准的“应用程序”-API。 'overlay' (optional)包含覆盖特定配置元素的配置的路径（单播、网络掩码、设备、网络、诊断地址和掩码、服务发现） 应用。这允许从单个进程管理不同的网络地址。注意：此功能仅在 vsomeip 使用 ENABLE_CONFIGURATION_OVERLAYS 编译时可用。
  ```

  

* `services` (array)
  包含服务提供者的服务。

```bash
 `service`服务的 ID。 `instance`服务实例的 ID。 `protocol` (optional)用于实现服务实例的协议。默认设置是_someip_。如果提供了不同的设置，vsomeip 不会打开指定的端口（服务器端）或未连接到指定端口（客户端）。因此，此选项可用于让服务发现宣布一个服务外部实施。 `unicast` (optional)承载服务实例的单播。注意：如果要使用外部服务实例，则需要单播地址，但服务发现被禁用。在这种情况下，提供的单播地址用于访问服务实例。  `reliable`分别指定与服务的通信是可靠的TCP协议用于通信。 `port`TCP 端点的端口。`enable-magic-cookies`指定是否启用魔法 cookie（有效值：_true_、_false_）。`unreliable`分别指定与服务的通信不可靠UDP 协议用于通信（有效值：UDP 的 _port_端点）。`events` (array)包含服务的事件。`event`事件的 ID。`is_field`指定事件是否为字段类型。注意：字段是 getter、setter 和通知事件的组合。它至少包含一个 getter、setter 或通知程序。通知器发送事件在更改时传输字段当前值的消息。`is_reliable`分别指定通信是否可靠使用 TCP 协议发送（有效值：_true_,_false_）。+如果值为 _false_，则将使用 UDP 协议。 `eventgroups` (array)事件可以组合到一个事件组中。对于客户来说，它是这样的可以订阅事件组并接收适当的事件组内。`eventgroup`事件组的 ID。`events` (array)包含相应事件的 ID。`multicast`指定用于发布事件组的多播。`address`多播地址。`port`多播端口。`threshold`指定何时使用多播以及何时使用单播发送通知事件。必须设置为非负数。如果设置为零，则事件组的所有事件将通过单播发送。否则，事件将通过单播发送，只要订阅者数量低于阈值，如果数量低于阈值，则通过多播订阅者的数量大于或等于。这意味着，阈值为 1 将导致所有事件通过组播发送。默认值为 _0_。  `debounce-times` (object)用于配置 nPDU 功能。这在<<npdu,vSomeIP nPDU 功能>>。`someip-tp` (object)用于配置 SOME/IP-TP 功能。有一个例子可以在<<someiptp, SOME/IP-TP>>。`service-to-client` (array)包含从节点发送的响应、字段和事件的 ID到远程客户端，如果它们超过了可以通过 SOME/IP-TP 进行分段UDP 通信的最大消息大小。如果此处未列出 ID如果超过最大消息大小，则消息将被丢弃。`client-to-service` (array)包含从节点发送的请求的 ID到可以通过 SOME/IP-TP 分割的远程服务，如果它们超过UDP 通信的最大消息大小。如果此处未列出 ID如果超过最大消息大小，则消息将被丢弃。请注意，单播密钥必须设置为远程 IP 地址提供节点以使此设置生效。
```




* `clients` (array)

  应用于连接到特定服务的客户端端口。对于每个服务，一组端口用于可靠/不可靠可以指定通信。vsomeip 将占用第一个自由端口列表。如果找不到空闲端口，连接将失败。如果要求 vsomeip 连接到没有指定端口的服务实例，该端口将由系统选择。这意味着用户有确保这里配置的端口不与端口重叠由 IP 堆栈自动选择。

```bash
`service``instance`它们一起指定端口配置应应用于的服务实例。`reliable` (array)用于与给定的可靠 (TCP) 通信的客户端端口列表服务实例。  `unreliable` (array)用于与给定的不可靠 (UDP) 通信的客户端端口列表服务实例。 +此外，还可以配置客户端范围之间的映射远程服务端口的端口和范围。（如果为特定服务/实例配置了客户端端口，则忽略端口范围映射） `reliable_remote_ports`指定一系列可靠的远程服务端口 `unreliable_remote_ports`指定一系列不可靠的远程服务端口`reliable_client_ports`指定要映射到reliable_remote_ports 范围的可靠客户端端口范围`unreliable_client_ports`指定要映射到 unreliable_remote_ports 范围的不可靠客户端端口范围`first` 指定端口范围的下限`last`指定端口范围的上限
```



* `payload-sizes` (array)

  用于限制每个 IP 和端口允许的最大负载大小的数组。如果不否则指定允许的有效载荷大小是无限的。中的设置此数组仅影响通过 TCP 的通信。限制本地负载大小可以使用`max-payload-size-local`。

```bash
 `unicast`在客户端：有效载荷大小应为的远程服务的 IP受到限制。在服务端：提供的服务的 IP，其有效载荷大小为接收和发送应该是有限的。`ports` (array)包含端口和负载大小语句对的数组。`port`在客户端：有效负载大小应为的远程服务的端口受到限制。在服务端：所提供服务的端口，其有效载荷大小为接收和发送应该是有限的。`max-payload-size`在客户端：发送到客户端的消息的有效负载大小限制（以字节为单位）远程服务托管在预先指定的 IP 和端口上。在服务端：接收和发送的消息的有效载荷大小限制（以字节为单位）通过先前指定的 IP 和端口上提供的服务。如果多个服务托管在同一个端口上，它们都共享限制指定的。
```



* `max-payload-size-local`

  节点内部通信的最大允许负载大小（以字节为单位）。经过默认节点内部通信的有效负载大小是无限的。有可能通过此设置进行限制。

* `max-payload-size-reliable`

  TCP 通信的最大允许负载大小字节。默认情况下，TCP 通信的有效负载大小为
  无限。可以通过此设置进行限制。

* `max-payload-size-unreliable`

  通过 SOME/IP-TP 进行 UDP 通信的最大允许负载大小字节。默认情况下，通过 SOME/IP-TP 通信的 UDP 有效负载大小为无限。可以通过此设置进行限制。此设置仅适用于某些/IP-TP 启用的方法/事件/字段（否则 UDP 默认值为 1400字节适用）。有关示例配置，请参见 <<someiptp, SOME/IP-TP>>。

* `endpoint-queue-limits` (array)

  用于限制每个缓存的传出消息的最大允许大小（以字节为单位）的数组IP 和端口（每个端点的消息队列大小）。如果没有另外指定允许的队列大小是无限的。此数组中的设置仅影响外部沟通。要限制本地队列大小，`endpoint-queue-limit-local` 可以使用。

```bash
*`unicast`在客户端：发送队列大小的远程服务的 IP要求应该受到限制。在服务端：提供的服务的 IP，其队列大小为发送的响应应该是有限的。因此，这个 IP 地址是与通过“单播”设置指定的 IP 地址相同.json 文件。 `ports` (array)保存成对的端口和队列大小语句的数组。`port`在客户端：发送队列大小的远程服务的端口要求应该受到限制。在服务端：提供的服务的端口，其队列大小为发送响应应该是有限的。 `queue-size-limit`在客户端：发送到客户端的消息的队列大小限制（以字节为单位）远程服务托管在预先指定的 IP 和端口上。在服务端：服务发送的响应的队列大小限制（以字节为单位）在先前指定的 IP 和端口上提供。如果多个服务托管在同一个端口上，它们都共享限制指定的。
```



* `endpoint-queue-limit-external`
  设置以限制缓存的传出消息的最大允许大小（以字节为单位）用于外部通信（每个端点的消息队列大小）。默认情况下外部通信的队列大小是无限的。可以通过这个限制环境。在 `endpoint-queue-limits` 数组中完成的设置会覆盖它环境。

* `endpoint-queue-limit-local`设置以限制缓存的传出消息的最大允许大小（以字节为单位）用于本地通信（每个端点的消息队列大小）。默认队列节点内部通信的大小是无限的。可以通过这个限制环境。

* `buffer-shrink-threshold`

  已处理的消息数量的一半或小于分配的缓冲区用于在缓冲区的内存被分配之前处理它们
  释放并再次开始动态增长。此设置可用于只有少数整体消息大得多的场景
  那么剩下的和分配给处理它们的内存应该在一个及时处理。如果该值设置为零，则不会重置缓冲区大小，并且最大的处理消息一样大。（默认为 5）

  示例：`buffer-shrink-threshold` 设置为 50。500 字节的消息必须被处理并且缓冲区相应地增长。此消息后连续50小于 250 字节的消息必须在缓冲区大小被处理之前减少并再次开始动态增长。

* `tcp-restart-aborts-max`

  由于未完成的 TCP 握手，设置以限制 TCP 客户端端点重启中止的次数。达到限制后，如果连接尝试仍处于挂起状态，则会强制重启 TCP 客户端端点。

* `tcp-connect-time-max`

* 设置以定义 TCP 客户端端点连接尝试完成之前的最长时间。
  如果`tcp-connect-time-max` 已过，则如果连接尝试仍处于挂起状态，则 TCP 客户端端点将被强制重新启动。

* `udp-receive-buffer-size`

* 指定套接字接收缓冲区的大小（`SO_RCVBUF`）用于UDP 客户端和服务器端点（以字节为单位）。（默认：1703936）

* `internal_services` (optional array)

  指定纯内部服务实例的服务/实例范围。vsomeip 使用此信息来避免发送 Find-Service 消息当客户请求不可用的服务时，通过服务发现-实例。它可以在服务/实例级别或服务级别完成只有然后包括从 0x0000-0xffff 的所有实例。

```bash
 `first`内部服务范围的最低条目。`service`内部服务范围的最低条目。`instance` (optional)内部服务实例范围的十六进制最低实例 ID。如果未指定，最低的 Instance-ID 为 0x0000。 `last`内部服务范围的最高入口。 `service`内部服务范围的十六进制最高服务 ID。`instance` (optional)内部服务实例范围的十六进制最高实例 ID。如果未指定，最高 Instance-ID 为 0xFFFF。
```

* `debounce` (optional array)

  外部设备发送的事件/字段将被转发到仅当可配置函数的计算结果为真时才应用程序。这
  函数检查事件/字段有效负载是否已更改以及是否自上次转发以来已经过了指定的时间间隔。

* ```bash
  `service`承载要去抖动的事件的服务 ID。 `instance`承载要去抖动的事件的实例 ID。`events`应根据以下条件去抖动的一系列事件 配置选项。`event`事件 ID。`on_change`指定事件是否只在 paylaod改变与否。（有效值：_true_、_false_）。`ignore`具有给定位掩码的有效载荷索引数组（可选） 在有效载荷变化评估中被忽略。 除了指定索引/位掩码对，只能定义有效载荷索引 在评估中应忽略。`index`要使用给定位掩码检查的负载索引。`mask`1Byte 位掩码应用于给定负载索引的字节。示例掩码：0x0f 忽略给定索引处字节低半字节的负载变化。 `interval`指定是否应根据经过的时间间隔对事件进行去抖动。（有效值：_time in ms_，_never_）。`on_change_resets_interval_` (optional)指定在检测到负载更改时是否重置间隔计时器。（有效值：_false_、_true_）。`routing`负责路由的应用程序的名称。
  ```

* `routing-credentials`

  充当路由管理器的应用程序的 UID/GID。（如果使用 _check_credentials_ 设置为 _true_ 启用凭据检查，则必须指定，以便成功检查通过连接传递的路由管理器凭据）

```bash
 `uid`路由管理器 UID。`gid`路由管理器 GID。
```



* `shutdown_timeout`

配置本地客户端等待确认的时间（以毫秒为单位）他们在关闭期间从路由管理器中注销。默认为5000 毫秒。

* `warn_fill_level`

  路由管理器定期检查发送缓冲区的填充水平到它的客户。该变量定义了最小填充水平百分比，导致正在记录警告。默认为 67。

* `service-discovery`
  包含与主机应用程序的服务发现相关的设置。

```bash
 `enable`
指定是否启用服务发现（有效值：_true_、_flase_）。默认值是true_。

 `multicast`
服务发现的消息将被发送的组播地址。默认值为 _224.0.0.1_。

`port`
服务发现的端口。默认设置为_30490_。

`protocol`
用于发送服务发现消息的协议（有效值：_tcp_、_udp_）。默认设置是_udp_。

`initial_delay_min`
第一个报价消息之前的最小延迟。

`initial_delay_max`
第一个报价消息之前的最大延迟。

`repetitions_base_delay`
在重复阶段发送基本延迟提供消息。

`repetitions_max`
提供服务的最大重复次数 ,重复阶段。

`ttl`
提供的服务以及消费的服务和事件组的条目的生命周期。

`ttl_factor_offers` (optional array)
保存传入远程报价的校正因子的数组。如果一个值为服务实例指定大于 1，则该服务实例的 TTL 字段相应的服务条目将乘以指定的因子。
示例：接收到的服务提供的 TTL 为 3 秒，并且 TTLfactor 设置为 5。远程节点停止提供服务，而无需发送停止提供消息。该服务将在 15 秒后过期（标记为不可用）在收到最后一个报价后。

`service`
服务的 ID。

`instance`
服务实例的 ID。

 `ttl_factor`
TTL校正因子

`ttl_factor_subscriptions` (optional array)
保存传入远程订阅的校正因子的数组。如果一个为服务实例指定大于 1 的值，该服务实例的 TTL 字段相应的事件组条目将乘以指定的因子。

示例：收到对所提供服务的远程订阅，TTL 为 3秒，TTL 因子设置为 5。远程节点停止重新订阅
不发送 StopSubscribeEventgroup 消息的服务。订阅将然后在收到最后一次重新订阅后 15 秒到期。

`service`
服务的 ID。

 `instance`
服务实例的 ID。

`ttl_factor`
TTL校正因子

`cyclic_offer_delay`
 主阶段中的 OfferService 消息的循环。

`request_response_delay`
单播消息到多播消息的最小延迟提供的服务和活动组。

 `offer_debounce_time`
堆栈在进入新服务之前收集新服务的时间重复阶段。这可以用来减少数量在启动期间发送消息。默认设置为_500ms_。
```



**看门狗**

* anchor:config-watchdog[]`watchdog` (optional)

  Watchdog 会定期向所有已知的本地客户端发送 ping。如果客户端在配置的时间/乒乓球数量内没有响应看门狗注销此应用程序/客户端。如果未配置，则不会激活看门狗。

  ```bash
   `enable`
  指定是启用还是禁用看门狗。（有效值：_true、false_），（默认值为 _false_）。
  
  `timeout`
  指定当 ping 时看门狗被激活的超时时间（毫秒）在那段时间内，本地客户没有用乒乓球回答。（有效值：_2 - 2^32_），（默认为 _5000_ 毫秒）。
  
  
  `allowed_missing_pongs`
  指定允许丢失的pongs数量。（有效值：_1 - 2^32_），（默认为 _3_ pongs）。
  ```

  


  **CAPI-选择性广播支持**

* anchor:config-supports_selective_broadcasts[]`supports_selective_broadcasts` (optional array)

  此节点允许添加支持 CAPI-Selective-Broadcasts 功能的 IP 地址列表。如果未指定，则无法使用该功能，并且堆栈的订阅行为与正常事件。

  ```bash
  `address`
  指定支持“选择性”功能的 IP 地址（采用 IPv4 或 IPv6 表示法）。可以配置多个地址。
  ```

  

五、安全
--------

vsomeip 具有基于 UNIX 凭据的安全实现。
如果激活，则在连接期间使用标准 UNIX 凭据传递机制对每个本地连接进行身份验证。
在身份验证期间，客户端将其客户端标识符及其凭据（UID / GID）传输到服务器，然后与配置进行匹配。
如果收到的凭据与策略不匹配，则服务器将立即关闭套接字并记录一条消息。
如果接受，客户端标识符将绑定到接收套接字，因此可用于对传入消息（vsomeip 消息以及内部命令）进行进一步的安全检查。

通常，客户端可以配置为允许/拒绝请求（意味着与之通信）并提供不同的服务实例。
然后根据策略检查每个传入的 vsomeip 消息（请求/响应/通知）以及提供服务请求或本地订阅。
如果传入的 vsomeip 消息或其他操作（例如，提供/订阅）违反了配置的策略，则会跳过它并记录一条消息。

此外，如果应用程序接收有关系统中其他客户端/服务的信息，则必须从经过身份验证的路由管理器接收。 
这是为了避免恶意应用程序伪造路由管理器，从而能够错误地通知其他客户端有关系统上运行的服务。
因此，无论何时指定“安全”标记，路由管理器（例如，routingmanagerd/vsomeipd）必须是具有固定客户端标识符的配置应用程序。 
有关如何配置应用程序以使用特定客户端标识符的信息，请参阅“配置文件结构”一章。

凭证传递只能通过 Unix-Domain-Sockets 进行，因此只能用于本地通信。
但是，如果安全性被激活，从远程客户端到本地服务的方法调用也会被检查，这意味着需要明确允许远程客户端。
除了可以跳过 _credentials_ 标记之外，对于本地客户端，此类策略看起来相同。

安全配置

安全功能的可用配置开关有： 

```bash
安全
 anchor:config-policy[]`security` (optional)
如果指定，则激活凭证传递机制。但是，只要 _check_credentials_ 未设置为 _true_，就不会进行凭据或安全检查，但如果指定了安全标记，则必须配置路由管理器客户端 ID，并且不应将其设置为 0x6300。
  如果 _check_credentials_ 设置为 _true_，则需要使用 _routing-credentials_ 标签指定路由管理器 UID 和 GID。

 `check_credentials` (optional)
指定安全检查是否处于活动状态。这包括对连接的凭据检查以及随后配置的所有策略检查（有效值：_true、false_），（默认值为 _false_）。

 `allow_remote_clients` (optional)
指定是否允许将传入的远程请求/订阅发送到本地代理/客户端。如果未指定，则默认允许接收所有远程请求/订阅。(有效值为 'true' and 'false')

 `policies` (array)
指定安全策略。每个策略至少需要指定 _allow_ 或 _deny_。

`credentials`
指定将应用安全策略的凭据。
如果 _check_credentials_ 设置为 _true_，则需要正确指定本地应用程序的凭据以确保本地套接字身份验证可以成功。

`uid`
将客户端应用程序的 LINUX 用户 ID 指定为十进制数。可以使用通配符“any”。

`gid`
将客户端应用程序的 LINUX 组 ID 指定为十进制数。可以使用通配符“any”。

 `allow` / `deny` (optional)
指定对于该策略是允许还是拒绝 LINUX 用户和组 ID。

 `uid` (array)
指定 LINUX 用户 ID 列表。这些可以指定为十进制数或范围。范围由第一个和最后一个有效 ID 指定（参见下面的示例）。

`gid` (array)
指定 LINUX 组 ID 列表。这些可以指定为十进制数或范围。范围由第一个和最后一个有效 ID 指定（参见下面的示例）。

`allow` / `deny`
此标记指定 _allow_ 或 _deny_，具体取决于是否需要列入白名单或黑名单。因此，不允许在一项策略中指定 _allow_ 和 _deny_ 条目。
使用 _allow_ 可以将允许的内容列入白名单，这意味着空的 _allow_ 标签意味着一切都被拒绝。
使用 _deny_ 可以将允许的内容列入黑名单，这意味着空的 _deny_ 标签意味着所有内容都是允许的。

 `requests` (array)
指定一组服务实例对，允许/拒绝使用上述凭据的上述客户端应用程序与之通信。

.`service`
为 _requests_ 指定服务。Specifies a service for the _requests_.

`instance` (deprecated)
为 _requests_ 指定一个实例可以使用通配符“any”，这意味着从实例 ID 0x01 到 0xFFFF 的范围
这也意味着方法 ID 范围从 0x01 到 0xFFFF。

`instances` (array)
指定一组允许/拒绝与之通信的实例 ID 和方法 ID 范围对。
如果下面的 `ids` 标签不用于在方法 ID 级别指定允许/拒绝的请求，也可以仅指定一组允许/拒绝请求的实例 ID 范围，类似于允许/拒绝“报价”部分。如果未指定方法 ID，则默认情况下允许/拒绝的方法范围为 0x01 到 0xFFFF。

`ids`
指定一组允许/拒绝与之通信的实例 ID 范围。还可以将单个实例 ID 指定为数组元素，而无需给出范围上限/下限。可以使用通配符“any”，这意味着从实例 ID 0x01 到 0xFFFF 的范围。
`first` - The lower bound of the instance range.
`last`  - The upper bound of the instance range.

`methods`
指定一组允许/拒绝与之通信的方法 ID 范围。还可以将单个方法 ID 指定为数组元素，而无需给出范围上限/下限。可以使用通配符“any”，这意味着从方法 ID 0x01 到 0xFFFF 的范围。
`first` - The lower bound of the method range.
`last`  - The upper bound of the method range.

`offers` (array)
使用上述凭据指定客户端应用程序允许/拒绝提供的一组服务实例对。

`service`
Specifies a service for the _offers_.
为 _offers_ 指定服务

`instance` (deprecated)
为 _offers_ 指定一个实例可以使用通配符“any”，这意味着从实例 ID 0x01 到 0xFFFF 的范围。

`instances` (array)
使用上述凭据指定客户端应用程序允许/拒绝提供的一组实例 ID 范围。还可以将单个实例 ID 指定为数组元素，而无需给出范围上限/下限。可以使用通配符“any”，这意味着从实例 ID 0x01 到 0xFFFF 的范围。

`first`
实例范围的下限。

`last`
实例范围的上限。
```



安全配置示例

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~json
"security" :
{
    ...
    "policies" :
    [
        {
            ...
            "credentials" :
            {
                "uid" : "44",
                "gid" : "any"
             },
             "allow" :
             [
                 "requests" :
                 [
                     {
                         "service" : "0x6731",
                         "instance" : "0x0001"
                     }
                 ]
             ]
         },
         {
            "credentials" :
            {
                "deny" :
                [
                    {
                        "uid" : [ "1000", { "first" : "1002", "last" : "max" }],
                        "gid" : [ "0", { "first" : "100", "last" : "243" }, "300"]
                    },
                    {
                        "uid" : ["55"],
                        "gid" : ["55"]
                    }
                 ]
             },
             "allow" :
             [
                 "offers" :
                 [
                     {
                        "service" : "0x6728",
                        "instances" : [ "0x0001", { "first" : "0x0003", "last" : "0x0007" }, "0x0009"]
                     },
                     {
                        "service" : "0x6729",
                        "instances" : ["0x88"]
                     },
                     {
                        "service" : "0x6730",
                        "instance" : "any"
                     }
                 ],
                 "requests" :
                 [
                     {
                         "service" : "0x6732",
                         "instances" :
                         [
                             {
                                 "ids" : [ "0x0001", { "first" : "0x0003", "last" : "0x0007" }],
                                 "methods" : [ "0x0001", "0x0003", { "first" : "0x8001", "last" : "0x8006" } ]
                             },
                             {
                                 "ids" : [ "0x0009" ],
                                 "methods" : "any"
                             }
                         ]
                     },
                     {
                        "service" : "0x6733",
                        "instance" : "0x1"
                     },
                     {
                        "service" : "0x6733",
                        "instances" : [ "0x0002", { "first" : "0x0003", "last" : "0x0007" }, "0x0009"]
                     }
                 ]
             ]
         }
     ]
}
~~~~~~~~~~

config/ 文件夹包含一些额外的 vsomeip 配置文件来运行 vsomeip激活安全检查的示例。此外，在`test/` 子文件夹中有一个安全测试可以使用以供进一步参考。
它们给出了如何使用所描述的安全相关配置标签的基本概述 在本章中在本地运行一个简单的请求/响应或订阅/通知示例或远程。

**审核模式**

vsomeip 的安全实现可以放在所谓的“审核模式”中，其中所有安全违规都将被记录但允许。该模式可用于构建安全配置。

要激活“审核模式”，“安全”对象必须包含在json 文件，但 'check_credentials' 开关必须设置为 false。
例子：

```bash
[...]
"services" :
[
    [...]
],
"security" :
{
    "check_credentials" : "false"
},
"routing" : "service-sample",
[...]
```

----

六、自动配置
-----------------

#### (一) someip 支持自动配置客户端标识符和路由。

第一个开始使用 vsomeip 的应用程序将自动成为路由管理器，如果它未显式配置。客户端标识符
从诊断地址生成，可以通过定义指定编译 vsomeip 时的 DIAGNOSIS_ADDRESS。vsomeip 将使用诊断地址
作为高字节并枚举低字节内的连接应用程序客户端标识符。

客户端标识符的自动配置并不意味着与 vsomeip Security 一起使用。
在激活安全性时，每个本地运行的客户端至少需要配置自己的凭据，以确保凭据检查可以通过。实际上，这意味着如果客户端通过未配置凭据的自动配置请求其标识符（至少事先不知道使用哪个客户端标识符），则该客户端不可能建立到服务器端点的连接。但是，如果所有客户端的凭据都相同，则可以针对整个（DIAGNOSIS_ADDRESS）客户端标识符范围配置它们，以将自动配置与激活的安全性混合在一起。

#### (二) 路由管理器

outingmanagerd 是一个最小的 vsomeip 应用程序，旨在提供路由在一个系统范围的配置文件所在的节点上的管理器功能展示。它可以在示例文件夹中找到。

示例：在系统范围配置为的系统上启动守护程序
存储在`/etc/vsomeip.json`下：

```
VSOMEIP_CONFIGURATION=/etc/vsomeip.json ./routingmanagerd
```

* 使用守护进程时，应确保：

  * 在系统范围的配置文件中，routingmanagerd 被定义为路由管理器，意味着它包含行`"routing" : "routingmanagerd"`。如果默认名称被覆盖，则必须相应地调整条目。
    系统范围的配置文件应包含有关所有系统上提供的其他服务也是如此。
  * 系统上没有使用其他 vsomeip 配置文件，其中包含一个“路由”条目。因为每个系统只能有一个路由管理器。


## 七、vsomeip Hello World

在本段中，一个由客户端和服务组成的 Hello World 程序被开发。客户端向服务发送一条包含字符串的消息。该服务将接收到的字符串附加到字符串 `Hello` 并将其发送回
给客户。在收到来自服务的响应后，客户端打印响应（“Hello World”）。
此示例旨在在同一主机上运行。

此处列出的所有文件都包含在 `examples\hello_world` 子目录中。

构建说明

该示例可以使用自己的 CMakeFile 构建，请编译 vsomeip 堆栈事先如《编译》中所述。然后编译示例开始
从存储库根目录如下：

```bash
cd examples/hello_world
mkdir build
cd build
cmake ..
```


### (一) 构建过程

开始和预期输出

服务的开始和预期输出

```bash
$ VSOMEIP_CONFIGURATION=../helloworld-local.json \
  VSOMEIP_APPLICATION_NAME=hello_world_service \
  ./hello_world_service
2015-04-01 11:31:13.248437 [info] Using configuration file: ../helloworld-local.json
2015-04-01 11:31:13.248766 [debug] Routing endpoint at /tmp/vsomeip-0
2015-04-01 11:31:13.248913 [info] Service Discovery disabled. Using static routing information.
2015-04-01 11:31:13.248979 [debug] Application(hello_world_service, 4444) is initialized.
2015-04-01 11:31:22.705010 [debug] Application/Client 5555 got registered!
```



----

客户端的启动和预期输出

```bash
$ VSOMEIP_CONFIGURATION=../helloworld-local.json \
  VSOMEIP_APPLICATION_NAME=hello_world_client \
  ./hello_world_client
2015-04-01 11:31:22.704166 [info] Using configuration file: ../helloworld-local.json
2015-04-01 11:31:22.704417 [debug] Connecting to [0] at /tmp/vsomeip-0
2015-04-01 11:31:22.704630 [debug] Listening at /tmp/vsomeip-5555
2015-04-01 11:31:22.704680 [debug] Application(hello_world_client, 5555) is initialized.
Sending: World
Received: Hello World
```

### (二) CMakeFile

```bash
# 版权所有 (C) 2015 Bayerische Motoren Werke Aktiengesellschaft (BMW AG) 
# 本源代码表受 Mozilla 公共
# 许可证条款 2.0的约束。如果 MPL 的副本没有随此# 文件一起分发，您可以从http://mozilla.org/MPL/2.0/ 获得一份。

cmake_minimum_required (VERSION 2.8.7)
project (vSomeIPHelloWorld)

＃这将让我们查看在
＃VSOMEIP_INCLUDE_DIRS -包括vSomeIP目录
＃VSOMEIP_LIBRARIES -库链接对

find_package(vsomeip)
if (NOT vsomeip_FOUND)
    message("vsomeip was not found. Please specify vsomeip_DIR")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${VSOMEIP_INCLUDE_DIRS})

add_executable (hello_world_service hello_world_service.cpp)
target_link_libraries(hello_world_service ${VSOMEIP_LIBRARIES})

add_executable (hello_world_client hello_world_client.cpp)
target_link_libraries(hello_world_client ${VSOMEIP_LIBRARIES})
```

### (三) 客户端和服务的配置文件

```json
{
   "unicast" : "134.86.56.94",
   "logging" :
   {
      "level" : "debug",
      "console" : "true"
   },

   "applications" :
   [
      {
         "name" : "hello_world_service",
         "id" : "0x4444"
      },

      {
         "name" : "hello_world_client",
         "id" : "0x5555"
      }
   ],

   "servicegroups" :
   [
      {
         "name" : "default",
         "unicast" : "local",
         "services" :
         [
            {
               "service" : "0x1111",
               "instance" : "0x2222",
               "unreliable" : "30509"
            }
         ]
      }
   ],

   "routing" : "hello_world_service",
   "service-discovery" :
   {
      "enable" : "false"
   }
}
```

### (四) Service

```c++
//版权所有( C )  2015 Bayerische Motoren Werke Aktiengesellschaft ( BMW AG ) 
//本源代码表受 Mozilla Public
 // License , v .  2.0 。如果 MPL 的副本未随此//文件分发，您可以在 http : //mozilla获得一份。组织/MPL / 2.0 /。

#include <vsomeip/vsomeip.hpp>

static vsomeip::service_t service_id = 0x1111;
static vsomeip::instance_t service_instance_id = 0x2222;
static vsomeip::method_t service_method_id = 0x3333;

class hello_world_service {
public:
    //获取 vSomeIP 运行时并
    //通过运行时创建应用程序，我们可以传递应用程序名称
    //否则通过 VSOMEIP_APPLICATION_NAME 提供的名称
    // 使用环境变量
    hello_world_service() :
                    rtm_(vsomeip::runtime::get()),
                    app_(rtm_->create_application())
    {
    }

    void init()
    {
        //初始化应用程序
        app_->init();

      //注册消息处理程序回调为发送到我们的服务消息
        app_->register_message_handler(service_id, service_instance_id,
                service_method_id,
                std::bind(&hello_world_service::on_message_cbk, this,
                        std::placeholders::_1));

        //注册事件处理程序被调用在注册后返回
        //运行时是成功的
        app_->register_event_handler(
                std::bind(&hello_world_service::on_event_cbk, this,
                        std::placeholders::_1));
    }

    void start()
    {
       //启动应用程序并等待对于所述ON_EVENT回调被调用
        //该方法只有当app_->stop（）时被调用
        app_->start();
    }

    void stop()
    {
       //停止提供服务
        app_->stop_offer_service(service_id, service_instance_id);
       //取消注册事件处理程序
        app_->unregister_event_handler();
        //注销消息处理程序
        app_->unregister_message_handler(service_id, service_instance_id,
                service_method_id);
        //关闭应用程序
        app_->stop();
    }

    void on_event_cbk(vsomeip::event_type_e _event)
    {
        if(_event == vsomeip::event_type_e::ET_REGISTERED)
        {
            //我们在运行时注册，可以提供我们的服务
            app_->offer_service(service_id, service_instance_id);
        }
    }

    void on_message_cbk(const std::shared_ptr<vsomeip::message> &_request)
    {
        //根据请求创建响应
        std::shared_ptr<vsomeip::message> resp = rtm_->create_response(_request);

        //构造字符串以发回
        //reinterpret_cast<const char*>为强制转换，参考附录
        std::string str("Hello ");
        str.append(
                reinterpret_cast<const char*>(_request->get_payload()->get_data()),
                0, _request->get_payload()->get_length());

        //创建一个将被发送回客户端的负载
        std::shared_ptr<vsomeip::payload> resp_pl = rtm_->create_payload();
        std::vector<vsomeip::byte_t> pl_data(str.begin(), str.end());
        resp_pl->set_data(pl_data);
        resp->set_payload(resp_pl);

         //将响应发送回
        app_->send(resp, true);
       //我们已经完成了 stop now 
        stop();
    }

private:
    std::shared_ptr<vsomeip::runtime> rtm_;
    std::shared_ptr<vsomeip::application> app_;
};

int main(int argc, char **argv)
{
    hello_world_service hw_srv;
    hw_srv.init();
    hw_srv.start();
    return 0;
}
```

**服务示例导致以下程序执行：**

**Main**

1.***main()*** (line 101-107)

首先，应用程序被初始化（第 104 行）。初始化完成后，应用程序启动（第 105 行）。

**Initialization**

2.***init()*** (line 26-42)

初始化包含消息处理程序和事件处理程序的注册。

消息处理程序为发送到特定服务的消息声明一个回调（*on_message_cbk*）（指定服务 id、服务实例 id 和服务方法 id）。

事件处理程序为发生的事件声明一个回调（*on_event_cbk*）。一个事件可以是在运行时成功注册应用程序。

**Start**

3.***start()*** (line 44-49)

应用程序将启动。此函数仅在应用程序停止时返回。

**Callbacks**

4.***on_event_cbk()*** (line 64-71)

该函数在事件发生时由应用程序调用。如果事件与在运行时成功注册应用程序有关，则提供特定服务。

5.***on_message_cbk()*** (line 73-94)

当收到来自客户端的指定服务的消息/请求时调用此函数。

首先创建基于请求的响应（第 76 行）。之后，字符串*Hello*将与客户端请求的负载连接（第 80-82 行）。之后创建响应的有效负载（第 85 行）。有效载荷数据设置为之前连接的字符串（第 87 行）。最后，响应被发送回客户端（第 91 行）并停止应用程序（第 93 行）。

**Stop**

6.***stop()*** (line 51-62)

该函数停止提供服务（第 54 行），取消注册消息和事件处理程序（第 56-59 行）并关闭应用程序（第 61 行）。

### (五) Client

```c++
//版权所有( C )  2015 Bayerische Motoren Werke Aktiengesellschaft ( BMW AG ) 
//本源代码表受 Mozilla Public
 // License , v .  2.0 。如果 MPL 的副本未随此//文件分发，您可以在 http : //mozilla获得一份。组织/MPL / 2.0 /。

#include <vsomeip/vsomeip.hpp>
#include <iostream>

static vsomeip::service_t service_id = 0x1111;
static vsomeip::instance_t service_instance_id = 0x2222;
static vsomeip::method_t service_method_id = 0x3333;

class hello_world_client {
public:
    //获取 vSomeIP 运行时并
    //通过运行时创建应用程序，我们可以传递应用程序名称
    //否则通过 VSOMEIP_APPLICATION_NAME 提供的名称
    // 使用环境变量
    hello_world_client() :
                    rtm_(vsomeip::runtime::get()),
                    app_(rtm_->create_application())
    {
    }

    void init(){
        //初始化应用程序
        app_->init();

        //注册事件处理程序被调用在注册后回来
        //运行成功
        app_->register_event_handler(
                std::bind(&hello_world_client::on_event_cbk, this,
                        std::placeholders::_1));

        //注册一个回调，它在服务可用时
        app_->register_availability_handler(service_id, service_instance_id,
                std::bind(&hello_world_client::on_availability_cbk, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3));

       //为来自服务的响应注册回调
        app_->register_message_handler(vsomeip::ANY_SERVICE,
                service_instance_id, vsomeip::ANY_METHOD,
                std::bind(&hello_world_client::on_message_cbk, this,
                        std::placeholders::_1));
    }

    void start()
    {
        //启动应用程序并等待对于所述ON_EVENT回调被调用
        //该方法只有当app_->stop（）被调用
        app_->start();
    }

    void on_event_cbk(vsomeip::event_type_e _event)
    {
        if(_event == vsomeip::event_type_e::ET_REGISTERED)
        {
            //我们在运行时注册的，现在我们可以请求服务
            //并等待 对的on_availability回调被称为
            app_->request_service(service_id, service_instance_id);
        }
    }

    void on_availability_cbk(vsomeip::service_t _service,
            vsomeip::instance_t _instance, bool _is_available)
    {
         //检查是否可用的服务是 hello_world 服务
        if(service_id == _service && service_instance_id == _instance
                && _is_available)
        {
            //服务可用然后我们发送请求
            //创建一个新请求
            std::shared_ptr<vsomeip::message> rq = rtm_->create_request();
            //将 hello world 服务设置为请求的目标
            rq->set_service(service_id);
            rq->set_instance(service_instance_id);
            rq->set_method(service_method_id);

            //创建一个将发送到服务的负载
            std::shared_ptr<vsomeip::payload> pl = rtm_->create_payload();
            std::string str("World");
            std::vector<vsomeip::byte_t> pl_data(std::begin(str), std::end(str));

            pl->set_data(pl_data);
            rq->set_payload(pl);
           //向服务发送请求。响应将被传递到
            //注册的消息处理程序
            std::cout << "Sending: " << str << std::endl;
            app_->send(rq, true);
        }
    }

    void on_message_cbk(const std::shared_ptr<vsomeip::message> &_response)
    {
        if(service_id == _response->get_service()
                && service_instance_id == _response->get_instance()
                && vsomeip::message_type_e::MT_RESPONSE
                        == _response->get_message_type()
                && vsomeip::return_code_e::E_OK == _response->get_return_code())
        {
            //获取有效载荷并打印它
            std::shared_ptr<vsomeip::payload> pl = _response->get_payload();
            std::string resp = std::string(
                    reinterpret_cast<const char*>(pl->get_data()), 0,
                    pl->get_length());
            std::cout << "Received: " << resp << std::endl;
            stop();
        }
    }

    void stop()
    {
       //取消注册事件处理程序
        app_->unregister_event_handler();
        //取消注册消息处理程序
        app_->unregister_message_handler(vsomeip::ANY_SERVICE,
                service_instance_id, vsomeip::ANY_METHOD);
       //关闭应用程序
        app_->stop();
    }

private:
    std::shared_ptr<vsomeip::runtime> rtm_;
    std::shared_ptr<vsomeip::application> app_;
};

int main(int argc, char **argv)
{
    hello_world_client hw_cl;
    hw_cl.init();
    hw_cl.start();
    return 0;
}
```

客户端示例导致以下程序执行：

**Main**

1.***main()*** (line 130-136)

首先，应用程序被初始化（第 133 行）。初始化完成后，应用程序启动（第 134 行）。

**Initialization**

2.***init()*** (line 27-48)

初始化包含消息处理程序、事件处理程序和可用性处理程序的注册。

事件处理程序再次为发生的事件声明回调（*on_event_cbk*）。

消息处理程序为从任何服务、任何服务实例和任何方法接收到的消息声明一个回调 ( *on_message_cbk* )。

可用性处理程序声明一个回调（*on_availability_cbk*），当特定服务可用时调用（指定服务 ID 和服务实例 ID）。

**Start**

3.***start()*** (line 50-55)

应用程序将启动。此函数仅在应用程序停止时返回。

**Callbacks**

4.***on_event_cbk()*** (line 57-65)

该函数在事件发生时由应用程序调用。如果事件与在运行时成功注册应用程序有关，则请求特定服务。

5**.*on_availability_cbk()*** (line 67-94)

当请求的服务可用或不再可用时调用此函数。

首先检查可用性的变化是否与 *hello world服务*有关，可用性更改为true。如果检查成功，则创建服务请求并设置适当的服务信息（服务 id、服务实例 id、服务方法 id）（第 76-80 行）。之后创建请求的有效负载（第 83 行）。Payload的数据是*World*，后面会设置（第84-87行）。最后，请求被发送到服务。

6.***on_message_cbk()*** (line 73-94)

当收到消息/响应时调用此函数。如果响应来自请求的服务，类型为 *RESPONSE*并且返回代码为*OK*（第 98-103 行），则打印响应的有效负载（第 105-109 行）。最后应用程序停止。

**Stop**

7.***stop()*** (line 114-123)

此函数取消注册事件和消息处理程序（第 117-120 行）并关闭应用程序（第 122 行）。



八、跟踪连接器
---------------

**概述/先决条件**

跟踪连接器用于转发通过发送的内部消息Unix 域套接字到 DLT。
因此，它需要安装 DLT 并且可以在CMake 的上下文。

**配置**

**静态配置**

跟踪连接器可以通过静态配置 
<<config-tracing,JSON 配置文件>>。

**[float]**

##### **示例 1（最小配置）**

```bash
{
    ...

"tracing" :
{
    "enable" : "true"
},

...
```



----

这是 Trace Connector 的最低配置。这只是使跟踪，所有发送的内部消息都将被跟踪/转发到 DLT。

**[float]**

##### **示例 2（使用过滤器）**

```bash
{
    ...

"tracing" :
{
    "enable" : "true",
    "channels" : 
    [
        {
            "name" : "My channel",
            "id" : "MC"
        }
    ],
    "filters" : [
        {
            "channel" : "MC",
            "matches" : [ { "service" : "0x1234", "instance" : "any", "method" : "0x80e8" } ],
            "type" : "positive"
        }
    ]
},

...
```



----

由于它是一个正过滤器，示例过滤器确保只有消息 表示来自服务“0x1234”实例的方法“0x80e8”将是转发给 DLT。如果它被指定为否定过滤器，则所有消息除了来自服务实例的表示方法“0x80e8”的消息'0x1234' 将被转发到 DLT。

一般过滤规则是：

*  默认过滤器是对所有消息的肯定过滤器。
*  默认过滤器在通道上处于活动状态，只要没有其他正面过滤器被指定。
*  否定过滤器阻止匹配的消息。否定过滤器否决正过滤器。因此，一旦消息与否定过滤器匹配，它不会转发。
*  标识符“0xffff”是匹配任何服务、实例或方法的通配符。关键字“any”可用作“0xffff”的替代。
*  不能在范围过滤器中使用通配符。

**动态配置**

跟踪连接器也可以通过其接口动态配置。您需要包含“<vsomeip/trace.hpp>”以访问其公共接口。

**[float]**

##### **例子：**

```c++
    // 获取跟踪连接器
    std::shared_ptr<vsomeip::trace::connector> its_connector 
	= vsomeip::trace::connector::get();

    // 添加频道
    std::shared_ptr<vsomeip::trace::channel> its_channel
	= its_connector->create_channel("MC", "我的频道");

    // 添加过滤规则
    vsomeip::trace::match_t its_match
        = std::make_tuple(0x1234, 0xffff, 0x80e8);    
    vsomeip::trace::filter_id_t its_filter_id 
	= its_channel->add_filter(its_match, true);

    // 初始化跟踪连接器    
    its_connector->init();

    // 启用跟踪连接器
    its_connector->set_enabled(true);

    // 移除过滤器
    its_channel->remove_filter(its_filter_id);
```

----



(九) vsomeip nPDU feature
------------------

这是 nPDU 功能的附加文档，又名。_Zugverfahren_。nPDU 功能可用于减少网络负载，因为它启用了 vsomeip堆栈将多个 vsomeip 消息合并到一个以太网帧中。

首先是一些关于 nPDU 功能的一般_重要_事情：

* 由于其性质，nPDU 功能以较低的网络负载换取速度。
* 由于 nPDU 功能需要一些未传输的设置通过服务发现，拥有一个 json 已经*不够*了
  客户端没有“服务”部分的文件。
* 由于节点的客户端和服务器端点由路由管理manager（这是在 json 文件中的“路由”处输入的应用程序）nPDU 功能设置*始终*必须在使用的 json 文件中定义 充当路由管理器的应用程序。
* nPDU 功能计时以毫秒为单位定义。
* 通过 UNIX 域套接字的节点内部通信不受nPDU 功能。
* 如果 json 文件中某个方法的 debounce 次数配置丢失或不完整，使用默认值：2ms 去抖动时间和 5ms max保留时间。全局默认值可以通过`npdu-default-timings` json 对象。

#### 配置

有两个特定于 nPDU 功能的参数：

* *去抖动时间*：将消息发送到相同方法之间的最短时间同一连接上的远程服务（src/dst 地址 + src/dst 端口）。
* *最大保留时间*：消息到同一方法的最长时间同一连接上的远程服务（src/dst 地址 + src/dst 端口）是允许在发送方缓冲。

更多信息请参见相应的需求文档。


nPDU 功能特定设置在 json 文件中配置 特殊 _debounce-times_ 部分中有关服务级别的“服务”部分：

```bash
[...]
"services":
[
    {
        "service":"0x1000",
        "instance":"0x0001",
        "unreliable":"30509",
        "debounce-times":
        {
            // nPDU feature configuration for this
            // service here
        }
    }
],

[...]
----
```

此外，可以全局配置 nPDU 默认计时。

全局默认时间可以通过 `npdu-default-timings` 覆盖json 对象。例如，以下配置片段显示了如何设置所有默认时间为零：

```bash
{
    "unicast":"192.168.1.9",
    [...]
    "npdu-default-timings" : {
        "debounce-time-request" : "0",
        "debounce-time-response" : "0",
        "max-retention-time-request" : "0",
        "max-retention-time-response" : "0"
    },
    "routing":"[...]",
    "service-discovery": { [...] }

}
```

##### 示例 1：通过 UDP 提供的一种服务和一种方法

* 该服务托管在 IP：192.168.1.9 上。
* 该服务通过 UDP 在端口 30509 上提供。
* 该服务的 ID 为 0x1000
* 该方法的 ID 为 0x0001
* 客户端从IP访问服务：192.168.1.77

服务端

++++++++++++

响应的去抖时间应该有：

* 去抖动时间为 10 毫秒
* 最大保留时间为 100 毫秒

```bash
{
    "unicast":"192.168.1.9",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unreliable":"30509",
            "debounce-times":
            {
                "responses": {
                    "0x1001" : {
                        "debounce-time":"10",
                        "maximum-retention-time":"100"
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```

客户端

++++++++++++

对 192.168.1.9 上的服务请求的去抖时间应该有：

* 去抖动时间为 20 毫秒
* 最大保留时间为 200 毫秒

```bash
{
    "unicast":"192.168.1.77",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unicast":"192.168.1.9", //需要将服务标记为外部
            "unreliable":"30509",
            "debounce-times":
            {
                "requests": {
                    "0x1001" : {
                        "debounce-time":"20",
                        "maximum-retention-time":"200"
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```

##### 示例 2：通过 UDP 提供具有两种方法的一种服务

* 该服务托管在 IP：192.168.1.9 上。
* 该服务通过 UDP 在端口 30509 上提供。
* 该服务的 ID 为 0x1000
* 该方法的 ID 为 0x0001
* 第二种方法的 ID 为 0x0002
* 客户端从IP访问服务：192.168.1.77

服务端

++++++++++++

响应的去抖时间应该有：

* 方法 0x1001 的去抖动时间为 10 毫秒，方法 0x1002 的去抖动时间为 20
* 方法 0x1001 的最大保留时间为 100 毫秒，0x1002 的最大保留时间为 200

```bash
{
    "unicast":"192.168.1.9",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unreliable":"30509",
            "debounce-times":
            {
                "responses": {
                    "0x1001" : {
                        "debounce-time":"10",
                        "maximum-retention-time":"100"
                    },
                    "0x1002" : {
                        "debounce-time":"20",
                        "maximum-retention-time":"200"
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```

客户端

++++++++++++

对 192.168.1.9 上的服务请求的去抖时间应该有：

* 方法 0x1001 的去抖动时间为 20 毫秒，方法 0x1002 的去抖动时间为 40
* 方法 0x1001 的最大保留时间为 200 毫秒，0x1002 的最大保留时间为 400

```bash
{
    "unicast":"192.168.1.77",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unicast":"192.168.1.9", // required to mark service as external
            "unreliable":"30509",
            "debounce-times":
            {
                "requests": {
                    "0x1001" : {
                        "debounce-time":"20",
                        "maximum-retention-time":"200"
                    },
                    "0x1002" : {
                        "debounce-time":"40",
                        "maximum-retention-time":"400"
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```

通过 UDP 和 TCP 提供一种方法的一种服务

* 该服务托管在 IP：192.168.1.9 上。
* 该服务通过 UDP 在端口 30509 上提供。
* 该服务通过 TCP 在端口 30510 上提供。
* 该服务的 ID 为 0x1000
* 该方法的 ID 为 0x0001
* 客户端从IP访问服务：192.168.1.77

服务端

++++++++++++

响应的去抖时间应该有：

* 去抖动时间为 10 毫秒
* 最大保留时间为 100 毫秒
* TCP 应使用与 UDP 相同的设置

```bash
{
    "unicast":"192.168.1.9",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unreliable":"30509",
            "reliable":
            {
                "port":"30510",
                "enable-magic-cookies":"false"
            },
            "debounce-times":
            {
                "responses": {
                    "0x1001" : {
                        "debounce-time":"10",
                        "maximum-retention-time":"100",
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
----
```

客户端

++++++++++++

对 192.168.1.9 上的服务请求的去抖时间应该有：

* 去抖动时间为 20 毫秒
* 最大保留时间为 200 毫秒
* TCP 应使用与 UDP 相同的设置

```bash
{
    "unicast":"192.168.1.77",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x0001",
            "unicast":"192.168.1.9", // required to mark service as external
            "unreliable":"30509",
            "reliable":
            {
                "port":"30510",
                "enable-magic-cookies":"false"
            },
            "debounce-times":
            {
                "requests": {
                    "0x1001" : {
                        "debounce-time":"20",
                        "maximum-retention-time":"200",
                    }
                }
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```




(十) SOME/IP TP
----------

使用 SOME/IP 传输协议 (TP) 可以传输以下消息：超过 1400 字节的 UDP 负载大小限制。如果启用，消息是分段并以多个 UDP 数据报发送。

示例配置：

* 服务 0x1111(0x1000)/0x1 托管在 192.168.0.1 上的 UDP 端口 40000
* 客户端运行在 192.168.0.100
* 该服务有两个方法，ID 为 0x1 和 0x2，需要大请求
  和大反应。此外，该服务还提供一个 ID 为 0x8001 的字段
  这也需要大量的有效载荷。
* 服务端的最大负载大小应限制为 5000 字节。

配置服务端：
----

```bash
{
    "unicast":"192.168.0.1",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x1",
            "unreliable":"40000",
            "someip-tp": {
                "service-to-client": [
                    "0x1", "0x2", "0x8001"
                ]
            }
        }
    ],
    "max-payload-size-unreliable" : "5000",
    "routing":"[...]",
    "service-discovery": { [...] }

}
----
```

配置客户端：

```bash
{
    "unicast":"192.168.0.100",
    "logging": { [...] },
    "applications": [ [...] ],
    "services":
    [
        {
            "service":"0x1000",
            "instance":"0x1",
            "unicast":"192.168.0.1", // required to mark service as external
            "unreliable":"40000", // required to mark service as external
            "someip-tp": {
                "client-to-service": [
                    "0x1", "0x2"
                ]
            }
        }
    ],
    "routing":"[...]",
    "service-discovery": { [...] }

}
```



(十一)工具
-----

**vsomeip_ctrl**

`vsomeip_ctrl` 是一个小实用程序，可用于发送 SOME/IP 消息
从命令行。如果响应在 5 秒内到达，响应将
被打印。

* 它可以通过`vsomeip_ctrl` make target (`make vsomeip_ctrl`) 构建。
* 目标服务的实例 ID 必须以十六进制传递符号。
* 完整的消息必须以十六进制表示法传递。
* 有关可用选项，请参阅 `--help` 参数。
* 如果`vsomeip_ctrl` 用于向远程服务发送消息并且没有`routingmanagerd` 正在本地机器上运行，确保传递一个 json配置文件，其中 `vsomeip_ctrl` 被设置为路由管理器 环境变量。
* 如果`vsomeip_ctrl` 用于向本地服务发送消息并且没有`routingmanagerd` 在本地机器上运行，确保使用相同的 json 配置文件作为本地服务。

示例：在服务 ID 为 0x1234 的服务上调用方法 ID 为 0x80e8 的方法，
实例 ID 0x5678：

```bash
./vsomeip_ctrl --instance 5678 --message 123480e800000015134300030100000000000009efbbbf576f726c6400
```

示例：向服务发送消息，服务 ID 为 0x1234，实例 ID
0x5678 和方法 ID 0x0bb8 通过 TCP

```bash
./vsomeip_ctrl --tcp --instance 5678 --message 12340bb8000000081344000101010000
```

