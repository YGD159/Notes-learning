# vsomeip

原文链接：https://docs.projects.genivi.org/vSomeIP/1.3.0/html/README.html

##### 版权

版权所有 © 2015, Bayerische Motoren Werke Aktiengesellschaft (BMW AG)

##### 执照

此源代码表单受 Mozilla 公共许可证 v. 2.0 条款的约束。如果 MPL 的副本没有随此文件一起分发，您可以从http://mozilla.org/MPL/2.0/获得一份。

## (一) vsomeip概述

vsomeip 堆栈实现了[基于 IP](http://some-ip.com/)的[可扩展的面向服务的中间件 (SOME/IP)](http://some-ip.com/)协议。堆栈包括：

- SOME/IP 的共享库 ( `libvsomeip.so`)
- SOME/IP 服务发现的第二个共享库 ( `libvsomeip-sd.so`)，如果启用了服务发现，则在运行时加载。

## (二) 构建说明

### 1）依赖关系

- 需要启用 C++11 的编译器，例如 gcc >= 4.8。

- vsomeip 使用 cmake 作为构建系统。

- vsomeip 使用 Boost >= 1.54：

  - Ubuntu 14.04：

    ```bash
    sudo apt-get install libboost-system1.54-dev libboost-thread1.54-dev libboost-log1.54-dev
    ```

  - Ubuntu 12.04：使用 Boost 1.54 版需要 PPA：

    - 网址：[https](https://launchpad.net/~boost-latest/+archive/ubuntu/ppa) : [//launchpad.net/~boost-latest/+archive/ubuntu/ppa](https://launchpad.net/~boost-latest/+archive/ubuntu/ppa)

    - ```bash
      sudo add-apt-repository ppa:boost-latest/ppa
      ```

      

    - ```bash
      sudo apt-get install libboost-system1.54-dev libboost-thread1.54-dev libboost-log1.54-dev
      ```

      

- 对于测试，需要谷歌1.7.0 版 的测试框架 [gtest](https://code.google.com/p/googletest/)

  - URL：[直接链接，版本 1.7.0](https://googletest.googlecode.com/files/gtest-1.7.0.zip)

- 要构建文档 asciidoc，需要 source-highlight、doxygen 和 graphviz：

  - ```bash
    sudo apt-get install asciidoc source-highlight doxygen graphviz
    ```

    

### 2）汇编

对于编译调用：

```bash
mkdir build
cd build
cmake ..
make -j8
```

要指定安装目录（就像`--prefix=`您习惯使用 autotools 一样）调用 cmake 如下：

```bash
cmake -DCMAKE_INSTALL_PREFIX:PATH=$YOUR_PATH ..
make -j8
make install
```

#### 1. 测试汇编

要编译测试，首先将 gtest 解压缩到您想要的位置。然后运行：

```
mkdir build
cd build
export GTEST_ROOT=$PATH_TO_GTEST/gtest-1.7.0/
cmake ..
make check
```

- 测试的其他 make 目标：
  - 调用`make build_tests`只编译测试
  - 调用`ctest`构建目录以在没有详细输出的情况下执行测试
  - 要运行单个测试，请调用`ctest --verbose --tests-regex $TESTNAME`简短格式：`ctest -V -R $TESTNAME`
  - 列出所有可用的测试运行`ctest -N`。
  - 有关测试的更多信息，请看看在 `readme.txt`的`test`子目录。

#### 2. 生成文档

要生成文档，请按照[[编译\] 中](https://docs.projects.genivi.org/vSomeIP/1.3.0/html/README.html#Compilation)所述调用 cmake ，然后调用`make doc`. 这将生成：

- html 中的 README 文件： `$BUILDDIR/documentation/README.html`
- 一个 doxygen 文档在 `$BUILDDIR/documentation/html/index.html`

`

## (三) 启动vsomeip应用程序/使用的环境变量

启动时会读出以下环境变量：

- `VSOMEIP_APPLICATION_NAME`: 这个环境变量用于指定应用程序的名称。此名称稍后用于将客户端 ID 映射到配置文件中的应用程序。它独立于应用程序的二进制名称。
- `VSOMEIP_CONFIGURATION_FILE`：此环境变量可用于指定应用程序要使用的配置文件。如果未定义此变量，`/etc/vsomeip.json`则将使用默认配置文件。

在以下示例中，应用程序`my_vsomeip_application`启动。从`my_settings.json`当前工作目录中的文件中读取设置。应用程序的客户端 ID 可以`my_vsomeip_client`在配置文件的名称下找到 。

```bash
#!/bin/bash
export VSOMEIP_APPLICATION_NAME=my_vsomeip_client
export VSOMEIP_CONFIGURATION_FILE=my_settings.json
./my_vsomeip_application
```

## (四) 配置文件结构

vsomeip 的配置文件是[JSON](http://www.json.org/)文件，由多个键值对和数组组成。

- 对象是一组无序的名称/值对。一个对象以 开始`{ (left brace)`并以 结束`} (right brace)`。每个名称后跟，`: (colon)`名称/值对以 分隔`, (comma)`。
- 数组是值的有序集合。数组以 开始`[ (left bracket)`并以 结束`] (right bracket)`。值由 分隔`, (comma)`。
- 值可以是双引号中的*字符串*、*数字*、或`true`或`false` 或`null`、*对象*或*数组*。这些结构可以嵌套。



配置文件元素说明：

- *unicast*

  主机系统的 IP 地址。

- *netmask*

  指定主机系统子网的网络掩码。

- *logging*

  - *level*

    指定日志级别（有效值：*trace*、*debug*、*info*、*warning*、 *error*、*fatal*）。

  - *console*

    指定是否启用通过控制台进行日志记录（有效值：*true、false*）。

  - *file*

    - *enable*

      指定是否应创建日志文件（有效值：*true 、 false*）。

    - *path*

      日志文件的绝对路径。

  - *dlt*

    指定是否启用诊断日志和跟踪 (DLT)（有效值： *true、false*）。

- *applications (array)*

  包含使用此配置文件的主机系统的应用程序。

  - *name*

    应用程序的名称。

  - *id*

    应用程序的 ID。

  - *num_dispatchers*

    用于执行应用程序回调的线程数。如果*num_dispatchers*设置为*0*，回调将在应用程序线程内执行。如果应用程序想要/必须直接在事件、可用性或消息回调中执行耗时的工作，则*num_dispatchers*应设置为*2*或更高。

- *servicegroups (array)*

  服务可以组合成一个服务组。服务组包含特定服务提供商的服务及其连接信息。

  | ![Note](data:image/png;base64,%0A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) | 还可以定义多个服务组来处理来自不同服务提供商的服务。 |
  | ------------------------------------------------------------ | ---------------------------------------------------- |

  - *name*

    服务组的名称。

  - *unicast*

    服务提供者的 IP 地址（有效值：*local*如果服务提供者是本地主机，否则为远程服务提供者的*有效 IP 地址*）。

  - *delays*

    分别包含与服务发现相关的延迟到服务实例。

    | Service Discovery 的多播消息带有太多消息溢出网络的风险。因此，可以使用合适的消息发送行为来配置服务发现。 |
    | ------------------------------------------------------------ |

    - *initial*

      | 一个服务实例经历不同的阶段。一个阶段称为初始等待阶段。当服务完全可用并等待客户端的服务发现消息时进入此阶段。 |
      | ------------------------------------------------------------ |

      - *minimum*

        指定客户端的服务发现消息将被忽略的最短时间（以毫秒为单位的值）。

      - *maximum*

        指定客户端的服务发现消息将被忽略的最长时间（以毫秒为单位的值）。

    - `repetition-base`

      | 在初始延迟结束的情况下，进入重复阶段。在此阶段，服务提供商的服务发现将通过多播重复提供服务。 |
      | ------------------------------------------------------------ |

      重复基本延迟指定第一个要约开始发送后的时间（以毫秒为单位的值）。

    - `repetition-max`

      指定重复阶段内发送的商品数量。

    - `cyclic-offer`

      | 在特定的重复次数后，进入主要阶段。在主要阶段，服务提供者的服务发现以服务的循环提供开始。 |
      | ------------------------------------------------------------ |

      循环提供指定通过多播提供服务的时间间隔（以毫秒为单位的值）。

    - `cyclic-request`

      指定循环请求延迟。目前未使用。

  - `services` (array)

    包含服务提供者的服务。

    - `service`

      服务的 ID。

    - `instance`

      服务实例的 ID。

    - `reliable`

      指定与服务的通信是可靠的，分别使用 TCP 协议进行通信。

      - `port`

        TCP 端点的端口。

      - `enable-magic-cookies`

        指定是否启用魔法 cookie（有效值：*true*、*false*）。

    - `unreliable`

      指定与服务的通信不可靠，分别使用 UDP 协议进行通信（有效值：UDP 端点的*端口*）。

    - `multicast`

      可以通过多播向特定的客户端组提供服务。

      - `address`

        特定的多播地址。

      - `port`

        具体端口。

    - `events` (array)

      包含服务的事件。

      - `event`

        事件的 ID。

        - `is_field`

          指定事件是否为字段类型。

          | 字段是 getter、setter 和通知事件的组合。它至少包含一个 getter、一个 setter 或一个通知程序。通知器发送一个事件消息，在更改时传输字段的当前值。 |
          | ------------------------------------------------------------ |

        - `is_reliable`

          分别指定事件是否通过 TCP 协议发送（有效值：*true*，*false*），通信是否可靠。

          如果值为*false*，则将使用 UDP 协议。

    - `eventgroups` (array)

      事件可以组合到一个事件组中。因此，对于客户端，可以订阅事件组并在组内接收适当的事件。

      - `eventgroup`

        事件组的 ID。

      - `events` (array)

        包含相应事件的 ID。

      - `is_multicast`

        指定是否应通过多播发送事件（有效值： *true*、*false*）。

      - `multicast`

        事件发送到的多播地址。

- `routing`

  负责路由的应用程序的名称。

- `service-discovery`

  包含与主机应用程序的服务发现相关的设置。

  - `enable`

    指定是否启用服务发现（有效值：*true*、 *false*）。

  - `multicast`

    服务发现的消息将被发送到的多播地址。

  - `port`

    服务发现的端口。

  - `protocol`

    用于发送服务发现消息的协议（有效值：*tcp*、*udp*）

## (五) vsomeip Hello World

在本段中，开发了一个由客户端和服务组成的 Hello World 程序。客户端向服务发送一条包含字符串的消息。服务将接收到的字符串附加到字符串`Hello`并将其发送回客户端。收到来自服务的响应后，客户端打印响应的负载（“Hello World”）。此示例旨在在同一主机上运行。

此处列出的所有文件都包含在`examples\hello_world`子目录中。

### 1）构建说明

该示例可以使用自己的 CMakeFile 构建，请按照[[编译\] 中](https://docs.projects.genivi.org/vSomeIP/1.3.0/html/README.html#Compilation)的说明事先编译 vsomeip 堆栈。然后从存储库根目录开始编译示例，如下所示：

```
cd examples/hello_world
mkdir build
cd build
cmake ..
make
```

### 2）Starting and expected output

#### 1. 服务的开始和预期输出

```bash
$ VSOMEIP_CONFIGURATION_FILE=../helloworld-local.json \
  VSOMEIP_APPLICATION_NAME=hello_world_service \
  ./hello_world_service
2015-04-01 11:31:13.248437 [info] Using configuration file: ../helloworld-local.json
2015-04-01 11:31:13.248766 [debug] Routing endpoint at /tmp/vsomeip-0
2015-04-01 11:31:13.248913 [info] Service Discovery disabled. Using static routing information.
2015-04-01 11:31:13.248979 [debug] Application(hello_world_service, 4444) is initialized.
2015-04-01 11:31:22.705010 [debug] Application/Client 5555 got registered!
```

#### 2. 客户端的启动和预期输出

```bash
$ VSOMEIP_CONFIGURATION_FILE=../helloworld-local.json \
  VSOMEIP_APPLICATION_NAME=hello_world_client \
  ./hello_world_client
2015-04-01 11:31:22.704166 [info] Using configuration file: ../helloworld-local.json
2015-04-01 11:31:22.704417 [debug] Connecting to [0] at /tmp/vsomeip-0
2015-04-01 11:31:22.704630 [debug] Listening at /tmp/vsomeip-5555
2015-04-01 11:31:22.704680 [debug] Application(hello_world_client, 5555) is initialized.
Sending: World
Received: Hello World
```

### 3）CMakeFile

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

### 4）客户端和服务的配置文件

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

### 5） Service

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

        //注册事件处理程序被调用在注册后背部
        //运行时是成功的
        app_->register_event_handler(
                std::bind(&hello_world_service::on_event_cbk, this,
                        std::placeholders::_1));
    }

    void start()
    {
       //启动应用程序并等待对于所述ON_EVENT回调被调用
        //该方法只有当app_-返回>停止（）被调用
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

**服务示例导致以下程序执行：Main**

1.***main()*** (line 101-107)

首先，应用程序被初始化（第 104 行）。初始化完成后，应用程序启动（第 105 行）。Initialization

2.***init()*** (line 26-42)

初始化包含消息处理程序和事件处理程序的注册。

消息处理程序为发送到特定服务的消息声明一个回调（*on_message_cbk*）（指定服务 id、服务实例 id 和服务方法 id）。

事件处理程序为发生的事件声明一个回调（*on_event_cbk*）。一个事件可以是在运行时成功注册应用程序。

#### Start

3.***start()*** (line 44-49)

应用程序将启动。此函数仅在应用程序停止时返回。

#### Callbacks

4.***on_event_cbk()*** (line 64-71)

该函数在事件发生时由应用程序调用。如果事件与在运行时成功注册应用程序有关，则提供特定服务。

5.***on_message_cbk()*** (line 73-94)

当收到来自客户端的指定服务的消息/请求时调用此函数。

首先创建基于请求的响应（第 76 行）。之后，字符串*Hello*将与客户端请求的负载连接（第 80-82 行）。之后创建响应的有效负载（第 85 行）。有效载荷数据设置为之前连接的字符串（第 87 行）。最后，响应被发送回客户端（第 91 行）并停止应用程序（第 93 行）。

#### Stop

6.***stop()*** (line 51-62)

该函数停止提供服务（第 54 行），取消注册消息和事件处理程序（第 56-59 行）并关闭应用程序（第 61 行）。

### 6）Client

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
        //该方法只有当app_-返回>停止（）被调用
        app_->start();
    }

    void on_event_cbk(vsomeip::event_type_e _event)
    {
        if(_event == vsomeip::event_type_e::ET_REGISTERED)
        {
            //
         

        
            我们在运行时注册的，现在我们可以请求服务
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

#### Main

1.***main()*** (line 130-136)

首先，应用程序被初始化（第 133 行）。初始化完成后，应用程序启动（第 134 行）。

#### Initialization

2.***init()*** (line 27-48)

初始化包含消息处理程序、事件处理程序和可用性处理程序的注册。

事件处理程序再次为发生的事件声明回调（*on_event_cbk*）。

消息处理程序为从任何服务、任何服务实例和任何方法接收到的消息声明一个回调 ( *on_message_cbk* )。

可用性处理程序声明一个回调（*on_availability_cbk*），当特定服务可用时调用（指定服务 ID 和服务实例 ID）。

#### Start

3.***start()*** (line 50-55)

应用程序将启动。此函数仅在应用程序停止时返回。

#### Callbacks

4.***on_event_cbk()*** (line 57-65)

该函数在事件发生时由应用程序调用。如果事件与在运行时成功注册应用程序有关，则请求特定服务。

5**.*on_availability_cbk()*** (line 67-94)

当请求的服务可用或不再可用时调用此函数。

首先检查可用性的变化是否与 *hello world服务*有关，可用性更改为true。如果检查成功，则创建服务请求并设置适当的服务信息（服务 id、服务实例 id、服务方法 id）（第 76-80 行）。之后创建请求的有效负载（第 83 行）。Payload的数据是*World*，后面会设置（第84-87行）。最后，请求被发送到服务。

6.***on_message_cbk()*** (line 73-94)

当收到消息/响应时调用此函数。如果响应来自请求的服务，类型为 *RESPONSE*并且返回代码为*OK*（第 98-103 行），则打印响应的有效负载（第 105-109 行）。最后应用程序停止。

#### Stop

7.***stop()*** (line 114-123)

此函数取消注册事件和消息处理程序（第 117-120 行）并关闭应用程序（第 122 行）。



