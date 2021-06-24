## SOME/IP入门

### (一) 总体说明

![20200204140312401](/home/ygd/资料/SomeIp资料/images/20200204140312401.png)

如上图所示为标准的网络七层架构，SOME/IP ( Scalable service-Oriented MiddlewarE over IP)，即“运行于IP之上的可伸缩的面向服务的中间件”。

他在系统中其实就是一个中间件的存在，所谓“Middleware中间件”是一种独立的系统软件或服务程序，分布式应用软件可借助Middleware在不同的技术之间共享资源。

**所谓的分布式应用软件，在这里指的就是“服务”；不同的技术之间，在这里指的就是“不同的平台或操作系统，比如Adaptive AUTOSAR系统等**。

![640](/home/ygd/资料/SomeIp资料/images/640.webp)

SOME/IP协议在OSI七层网络结构中位于应用层，在AUTOSAR中位于BSW的服务层。从功能上讲，SOME/IP是一种将服务接口进行打包或解包的中间件：从应用层发送的数据（就是服务相关的信息以及前文提到的服务接口中的内容），按照SOME/IP的格式打包后，再传递到下层的TCP/IP层，再进行逐层打包和封装，最终通过物理层以比特流的形式进行传输；接收时则按照与打包相反的规则进行解包。



### (二) 服务说明

服务是SOME/IP的最核心概念。在一个服务中，定义了Server和Client两个角色：Server提供服务，Client调用服务。对于同一个服务，只能存在一个Server，但可以同时存在多个Client调用服务。一个Service由0~多个Event/Method/Field组成。与CAN相比，面向服务的通讯方式能够大大降低总线的负载率。s

#### 1）Method

调用或引用一个进程/函数/子程序，通常由Client发起，并由Server答复。

##### 1.Request (最常见的一种Method)，

由Client向Server请求数据；Response是Request的结果，由Server答复Client的Request。

![img](/home/ygd/资料/SomeIp资料/images/v2-b3493d6d4ca3da1cfd29d00fb34bbfa2_720w.jpg)

##### 2.Method Fire & Forget

只Client向Server发起，但Server对该请求不回复。

![img](/home/ygd/资料/SomeIp资料/images/v2-af9c33e3089549548e85ad4823f988a8_720w.jpg)

#### 2）Event

一个单向的数据传输，只能是on change类型，用于Server主动向订阅（Subscribe）了相关服务的Client发布（Publish）信息。

![img](/home/ygd/资料/SomeIp资料/images/v2-641ee90803f26c68d0fe5a50e20f1b5c_720w.jpg)

#### 3）Field

由以下三项内容构成：

Notifier：通知，Server的Client订阅了服务后第一时间主动向其发送数据。

Getter：获取，由Client向Server请求数据。

Setter：设置，由Client修改Server的数据。

![img](/home/ygd/资料/SomeIp资料/images/v2-7e437979775a5857ad88c76f29adc95e_720w.jpg)



### (三) 解析SOME/IP格式

![SOMEIPOnWireFormat](/home/ygd/资料/SomeIp资料/images/SOMEIPOnWireFormat.jpg)

在那里你会看到两个设备（A 和 B）；设备 A 向 B 发送 SOME/IP 消息并返回一条消息。底层传输协议可以是TCP或UDP；对于消息本身，这没有区别。现在我们假设设备 B 正在运行一个服务，该服务提供了一个功能，该功能从设备 A 被此消息调用，返回的消息就是回复。

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FnaW5nTW9vbg==,size_16,color_FFFFFF,t_70)

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FnaW5nTW19vbg==,size_16,color_FFFFFF,t_70)

SOME/IP报文由消息头（Header）和数据段（Payload）组成，消息头是按照固定格式排列的有效信息，这些有效信息包括报文ID、请求ID、协议版本、接口版本以及处理之后（序列化）的服务接口信息等内容。

SOME/IP报文格式如图所示。

报文ID包括Service ID和Method ID，Service ID指的是Service本身的ID值；Method ID是指服务接口中的不同内容的ID，每个服务接口中定义的所有方法、单向方法、事件等都需要设置一个ID值；

Session ID是从1开始增加的循环值，用于使得Request与Response相配对；Message Type主要分为请求、请求无响应、通知、响应、错误几种类型。

- **Service ID**: 每个服务的唯一标识符

- **Method ID**:方法为 0-32767，事件为 32768-65535

- **Length**: 以字节为单位的有效负载长度（还包括接下来的 ID，这意味着 8 个额外的字节）

- **Client ID**: ECU内部调用客户端的唯一标识符；在整个车辆中必须是独一无二的

- **Session ID**: 会话处理的标识符；必须为每次调用递增

- **Protocol Version**: 协议版本一般为 0x01

- **Interface Version**: 服务接口的主要版本

- **Message Type**:  -- REQUEST (0x00) 一个需要响应的请求（甚至无效） 

  ​                            -- REQUEST_NO_RETURN (0x01) 一个即发即弃的请求

  ​                            -- NOTIFICATION (0x02) 一个需要没有响应的通知/事件回调请求

  ​                            -- RESPONSE (0x80)响应消息

- **Return Code**:   -- E_OK (0x00) 未发生错误

  ​                          -- E_NOT_OK (0x01) 发生未指定的错误

  ​                          -- E_WRONG_INTERFACE_VERSION (0x08) 接口版本不匹配

  ​                          -- E_MALFORMED_MESSAGE (0x09) 反序列化错误，因此无法对有效载     荷进行反序列化

  ​                          --E_WRONG_MESSAGE_TYPE (0x0A) 收到了意外的消息类型（例如 RE-QUEST_NO_RETURN 用于定义为 RE-QUEST 的方法）

#### 1）Message Type说明

| **值** | 报文类型          | **说明**                   |
| ------ | ----------------- | -------------------------- |
| 0x00   | REQUEST           | 请求，需要回复             |
| 0x01   | REQUEST_NO_RETURN | 请求，不需要回复           |
| 0x02   | NOTIFICATION      | Notifier/Event，不需要回复 |
| 0x80   | RESPONSE          | 回复                       |
| 0X81   | ERROR             | 带有错误信息的回复         |

#### 2）Payload说明

通常在传输数据时，为了使数据传输更可靠，要把原始数据分批传输，并且在每一批数据的头和尾都加上一定的辅助信息，比如数据量的大小、校验位等，这样就相当于给已经分批的原始数据加一些外套，这些外套起标示作用，使得原始数据不易丢失，一批数据加上“外套”就形成了传输通道的基本传输单元，叫做数据帧或数据包，而其中的原始数据就是payload。



### (四) SOME/IP SD

- Client如何发现服务

- 当服务不可用时，如何通知Client

- Client如何订阅事件

  

  SOME/IP-SD消息通过UDP进行传输，报文格式如下图所示：

  ![img](/home/ygd/资料/SomeIp资料/images/aa72cb7dfc81cf0f862ef372d2bb3217.png)

SD（Service Discovery）是服务的信息清单及管理机制，也是一种服务，主要实现服务寻址及事件订阅两种功能。SD用来对服务进行寻址时，服务提供者（Server端）通过服务发现（SD）通知其他ECU（Client端）某服务可用，并间接地通知该服务的地址（Server端地址）；服务消费者（Client端）了解到某服务状态后，能够调用该服务的相关内容。SD用来事件订阅时，专门针对Event类型的接口，可以通过SD实现对Event所在的Event group进行订阅、停止订阅等操作。

**服务发现的报文格式与一般的SOME/IP报文相同，但是其Message ID固定为0xFFFF8100**。

![图片](/home/ygd/资料/SomeIp资料/images/640)

#### 1）主要功能

定位服务实例
检测服务实例是否在运行（即服务实例的状态）
发布/订阅行为的管理

#### 2）SD报文解析

SOME/IP SD报文也是一种SOME/IP报文，是在SOME/IP报文的基础上进行了扩展，增加了Entry、Option等字段；Entries用于同步服务实例的状态和发布/订阅的管理，Options用于传输Entries的附加信息。

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FnaW5n2TW9vbg==,size_16,color_FFFFFF,t_70)

SOME/IP SD报文也是一种SOME/IP报文，是在SOME/IP报文的基础上进行了扩展，增加了Entry、Option等字段；SOME/IP SD报文的ServiceID（0xFFFF）、MethodID（0x8100）、Request ID（0x0000）、ProtocolVersion（0x01）、Interface Version（0x01）、MessageType（0x02）、ReturnCode（0x00）等属性都是固定值。

Entry字段可以理解为服务实例的“入口”，该入口包含服务实例以及需要订阅的事件组的信息。服务提供者和消费者通过SD中的Entry实现提供服务、发现服务，以及订阅事件组的功能，即服务提供者可以告知其他节点服务的“入口”在哪里，服务消费者也可以通过该“入口”找到自己所需要的服务；也能够实现订阅事件组、取消订阅事件组等功能。

##### 1.Entry

Entry字段可以理解为服务实例的“入口”，该入口包含服务实例以及需要订阅的事件组的信息。主要通过Entry实现提供服务、发现服务，以及订阅事件组的功能。

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0F7naW5nTW9vbg==,size_16,color_FFFFFF,t_70)

供服务用Entries

![img](/home/ygd/资料/SomeIp资料/images/20200204144601360.png)

供EventGroup用Entries

![img](/home/ygd/资料/SomeIp资料/images/20200204144709400.png)

报文中Type内容解释如下：

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L40FnaW5nTW9vbg==,size_16,color_FFFFFF,t_70)

| **类别**   | **Type值** | **Type名称**                                                 |
| ---------- | ---------- | ------------------------------------------------------------ |
| Service    | 0x00       | FindService 用于Client查找服务                               |
|            | 0x01       | Offer/StopOfferService 用于Server向Client提供服务或停止提供服务 |
| Eventgroup | 0x06       | Subscribe/StopSubscribe Client向Server订阅服务或停止订阅     |
|            | 0x07       | SubscribeAck/Nack Server对于订阅的应答，如果订阅成功，Client收到的就是SubscribeAck，否则收到SubscribeNack |

对于Offer/ StopOfferService、Subscribe/ StopSubscribe和SubscribeAck/ Nack，每一组Entries都共用了相同的Type值，但通过TTL字段可以识别究竟是提供服务还是停止提供服务，是订阅事件还是取消订阅，是订阅成功应答还是订阅失败应答：当TTL = 0时，表示报文对应的服务实例不再有效，此时对应的Type类型分别就是停止提供服务、停止订阅事件以及订阅失败应答。

##### 2.Options

![img](/home/ygd/资料/SomeIp资料/images/20200204145035825.png)

每一个Option都是有一个2字节的Length字段、1字节的Type字段和1字节的保留位开始的。Length字段指示的长度是从保留位开始的。

Options的类型如下表所示：

| 类别                                        | Type值 | Type名称          |
| ------------------------------------------- | ------ | ----------------- |
| Configuration  (用于配置服务)               | 0x01   | Configuration     |
| Load  Balancing (用于配置服务实例的优先级 ) | 0x05   | Load Balancing    |
| Endpoint  (发送服务相关的地址和端口)        | 0x04   | IPv4  Endpoint    |
|                                             | 0x06   | IPv6  Endpoint    |
|                                             | 0x24   | IPv4  SD Endpoint |
|                                             | 0x26   | IPv6  SD Endpoint |
| Multicast (声明Multicast地址)               | 0x14   | IPv4  Multicast   |
|                                             | 0x16   | IPv6  Multicast   |



##### 3.SD状态机 


![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Fna1W5nTW9vbg==,size_16,color_FFFFFF,t_70)不管是客户端还是服务端，都有同样的状态机，但是他们的状态机具有不同的行为。 

| 状态       | 服务端行为                                                   | 客户端行为                                                   |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Down       | Service不可用                                                | 服务未被应用请求，则停留在该状态；收到OfferService，启动TTL计时器，此时服务若被应用请求，进入Main； |
| Init       | **进入条件**：当服务准备完毕(Available)后                               **During：**收到Find Service报文，服务端忽略此消息                                                                 **退出条件**：若服务不可用了，将进入Down ；INITIAL_DELAY，当定时器超时后，进入Repetition。 | **进入条件：**服务被请求后，进入此阶段；                                                         **During：**等待INITIAL_DELAY时间；                                                                                           **退出条件：**如果此时收到Offer Service，则取消计时器，直接进入Main ；如果服务请求被释放，进入Down ；计时器超时后，发送第一个Find service，进入Repetition。 |
| Repetition | **作用：**为了让客户端快速找到有哪些Service，                                                                **During**：如果收到某客户端的FindService，延迟一定时间后，单独发送单播OfferService给服务请求端；如果收到SubscribeEventgroup后，发送单播Ack/Nack，启动此订阅Entry的TTL计时器；如果收到StopSubscribeEventgroup后，停止此订阅Entry的TTL计时器；                                                    **退出条件：**如果服务不可用，离开此阶段进入Down ，并发送StopOfferService通知所有客户端。 | **作用：**重复发送Find service；                                                          **退出条件：**收到Offer Service，停止发送计数和计时，立即进入Main 触发发SubscribeEventgroup；如果服务请求被释放，进入Down ，若有订阅，则发送StopSubscribeEventgroup。 |
| Main       | **作用：**此阶段将周期性发送OfferService；                                    **During：**如果收到某客户端的FindService，不影响发送计数，发送单播OfferService给服务请求端；如果收到SubscribeEventgroup后，发送单播Ack/Nack，启动此订阅Entry的TTL计时器；收到StopSubscribeEventgroup后，停止此订阅Entry的TTL计时器；                                     **退出条件：**如果服务不可用，离开此阶段进入Down，并发送StopOfferService。 | **作用：**不再周期发送Find Service，不必要负载；**During：**收到Offer Service，触发发送SubscribeEventgroup；如果收到StopOfferService，则停止所有计时器；                                               **退出条件：**如果服务请求被释放，进入Down Phase；若有订阅，则发送StopSubscribeEventgroup。 |

### (五)SOME/IP SD的通信行为

SD的优点：

**①**上电启动时，车内各ECU的启动电压和启动时间各不相同，ECU通过SD就可以灵活的官宣其Service的可用状态；

**②**车辆变型(可以是同款车型的不同配置，或者是车型沿用)时，可以灵活的适应功能/配置的变化，减少前期的配置工作；

**③**错误处理，当提供的Service出现问题时，可以通过SD即刻了解Service的不可用状态，接收方就可以做出相应的处理了；

**④**能源效率，需要的时候才会提供/订阅服务，减少能量消耗。

总结一下就是：你用与不用，SD一直在那里。简单车载网络可以考虑不使用SD，原因是完全发挥不了SD的优势；而车载网络越复杂，越能体现SD的强大优势。



SD通信主要涉及到3类报文：Find Service、Offer Service和Subscribe报文。

服务端和客户端的通信行为包含以下几个阶段：



![图片](https://mmbiz.qpic.cn/mmbiz_jpg/enicKZWPtXWZn916npljpOuCuOb0VeiaTkjXBzEgYxfKVSgdicR0oiaT957FC3ic5pJC4LlQFIYJjRdeK6iaWLCWEsXA/640?wx_fmt=jpeg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

#### 1） **服务端的通信行为** 

**Down Phase**

- 在这个阶段，Service是不可用的，即服务端无法提供服务。

**Initial Wait Phase**

- 当服务准备完毕(Available)后，进入此阶段；
- 如果此阶段收到Find Service报文，服务端忽略此消息，不做任何处理；
- 如果服务不可用了，将返回进入Down Phase；
- 此阶段需要定义时间参数INITIAL_DELAY_Min和INITIAL_DELAY_Max，初始化时间取其之间的随机值，当定时器超时后，发送第一帧OfferService，标志着进入下一个阶段。


**Repetition Phase**

- 为了让客户端快速找到有哪些Service，此阶段重复发送OfferService，重复次数由REPETITIONS_MAX决定；
- 发送间隔以REPETITIONS_BASE_DELAY为基本时间，每发送一次，间隔是前一间隔的2倍；
- 如果收到某客户端的FindService，不影响当前阶段的发送计数和计时，延迟一定时间(REQUEST_RESPONSE_DELAY)后，单独发送单播OfferService给服务请求端；
- 如果收到SubscribeEventgroup后，发送单播Ack/Nack，启动此订阅Entry的TTL计时器；
- 如果收到StopSubscribeEventgroup后，停止此订阅Entry的TTL计时器；
- 如果服务不可用，离开此阶段进入Down Phase，并发送StopOfferService通知所有客户端。



**Main Phase**

- 此阶段将周期性发送OfferService，周期时间为CYCLIC_OFFER_DELAY；
- 如果收到某客户端的FindService，不影响发送计数，延迟一定时间(REQUEST_RESPONSE_DELAY)后，发送单播OfferService给服务请求端；
- 如果收到SubscribeEventgroup后，发送单播Ack/Nack，启动此订阅Entry的TTL计时器；
- 收到StopSubscribeEventgroup后，停止此订阅Entry的TTL计时器；
- 如果服务不可用，离开此阶段进入Down Phase，并发送StopOfferService。



服务端通信行为：



![图片](https://mmbiz.qpic.cn/mmbiz_png/enicKZWPtXWZ9Mnn28EEicibbQPpHc4YsRj7SEJUjhuslHkNJHWFZnN0XBoXlt0X4dAFkT90S6B6ic77vfzpAauOZw/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

服务端状态机转换图：



![图片](https://mmbiz.qpic.cn/mmbiz_png/enicKZWPtXWZn916npljpOuCuOb0VeiaTk6YsA5GqcmzaMxr6nggMJIiaJnAbGCnexerkn17cHolasia4x6hLxRCbg/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

#### 2）**客户端的通信行为** 

**Down Phase**

- 服务未被应用请求；
- 收到OfferService，存储当前服务实例状态，启动TTL计时器，此时服务若被应用请求，直接进入Main Phase。

**Initial Wait Phase**

- 服务被请求后，进入此阶段；
- 等待INITIAL_DELAY时间（最大和最小值之间的随机值）；
- 如果此时收到Offer Service，则取消计时器，直接进入Main Phase；
- 如果服务请求被释放，进入Down Phase；
- 计时器超时后，发送第一个Find service，进入下一阶段。



客户端通信阶段(在Initial Phase收到offer service)：

![图片](https://mmbiz.qpic.cn/mmbiz_png/enicKZWPtXWZ9Mnn28EEicibbQPpHc4YsRjibdRYASJpfV9VLCTVmgyq9gOMGzuwSDKIJSFz1OlEmmouj5bnz0531Q/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)



**Repetition Phase**

- 重复发送Find service，重复次数由REPETITIONS_MAX决定；
- 发送间隔以REPETITIONS_BASE_DELAY为基时间，每发送一次间隔加倍；
- 收到Offer Service，停止发送计数和计时，立即进入Main Phase；触发发送SubscribeEventgroup(延迟一定时间）；
- 如果服务请求被释放，进入Down Phase；若有订阅，则发送StopSubscribeEventgroup。



客户端通信阶段(在Repetition Phase收到offer service)：

![图片](https://mmbiz.qpic.cn/mmbiz_png/enicKZWPtXWZn916npljpOuCuOb0VeiaTkbZapLvyNSJYHia47lGt3fWDOVsPnC95E1huCbSpPMXQmPdesygAJM0Q/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)



**Main Phase**

- 不再周期发送Find Service；
- 收到Offer Service，触发发送SubscribeEventgroup(延迟一定时间）；
- 如果收到StopOfferService，则停止所有计时器；
- 如果服务请求被释放，进入Down Phase；若有订阅，则发送StopSubscribeEventgroup。



客户端状态机转换图：

![图片](https://mmbiz.qpic.cn/mmbiz_png/enicKZWPtXWZn916npljpOuCuOb0VeiaTkcPHeydeI1PWIh7cNic3kicloLz1CQia9WHucZpU0THU2Eica3er6b1rtCg/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)





### (六) SOME/IP序列化 

#### 1）概念

序列化（Serialization）指的是将数据结构或对象依据事先定义的规则转换成二进制串的过程；反序列化（Deserialization）指的是将二进制串依据相同规则重新构建成数据结构或对象的过程。

![img](/home/ygd/资料/SomeIp资料/images/format,png)

#### 2）说明

在AUTOSAR中是指数据在PDU中的表达形式，可以理解为来自应用层的真实数据转换成固定格式的字节序，以实现数据在网络上的传输。软件组件将数据从应用层传递到RTE层，在RTE层调用SOME/IP Transformer，执行可配置的数据序列化（Serialize）或反序列化（Deserialize）。SOME/IP Serializer将结构体形式的数据序列化为线性结构的数据；SOME/IP Deserializer将线性结构数据再反序列化为结构体形式数据。在服务端，数据经过SOME/IP Serializer序列化后，被传输到服务层的COM模块；在客户端，数据从COM模块传递到SOME/IP Deserializer反序列化后再进入RTE层。如下图参考Autosar Com过程

![img](/home/ygd/资料/SomeIp资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Fn6aW5nTW9vbg==,size_16,color_FFFFFF,t_70)

#### 3）举例

一个unit32类型数据（0x12345678）的序列化。

|                       | Byte0 | Byte1 | Byte2 | Byte3 |
| --------------------- | ----- | ----- | ----- | ----- |
| 大端（Big Endian）    | 12    | 34    | 56    | 78    |
| 小端（Little Endian） | 78    | 56    | 34    | 21    |

### (七) SOME/IP 应用

SOME/IP相关参数的设计是汽车以太网面向服务的架构(SOA)设计中的主要设计内容，在本文中介绍的服务、服务的提供者、消费者、服务接口的各种方法、事件、字段等内容，以及文中未提到的TCP/IP通信中的配置信息，都是以太网SOA设计中的主要内容，因此对SOME/IP中各种参数的理解，对于整个架构和通信设计都非常重要。

随着汽车绿色智能互联的快速发展，对汽车运行时的高灵活性、车辆内部与外部服务的可关联、服务和软件的扩展与升级等需求都提出了更高的要求，此时面向服务架构（Service Oriented Architecture）的优势也变得越来越凸显。SOME/IP作为面向服务架构的通信基础，将会得到越来越多的应用。
