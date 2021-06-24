# ROS开发入门

视频参考教程：https://www.lanqiao.cn/courses/854

ROS (Robot Operating System, 机器人操作系统)是一个适用于机器人的开源操作系统。本课程以 ROS 官网的安装、入门以及初级教程为模版制作，包括安装 ROS、学习并理解相关概念以及技术要点等。结合初级教程，提供每一步详细操作命令，边学边练。此课程已经配置Kinetic和Ardent版本机器人操作系统。

ROS网站链接：http://wiki.ros.org/cn/ROS/Tutorials

ROS问题查询：https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:razor_imu_9dof/page:1/



## 一、从宏观角度看ROS

这里从整体上看一下ROS是个什么东西以及如何工作。

### (一) “分布式”系统

ROS从形式讲是一个分布式的系统，它允许在同一个局域网中的多个计算机(不局限于PC，包括各种能够安装ROS的嵌入式设备)可以在ROS的框架中进行无差别通信。

但是这里的“分布式”是加引号的，原因就是ROS(这里特指ROS1)并不是一个真正的分布式系统。一个分布式系统中各个节点都是平级的，但是ROS系统中，必须有一个Master。从实际操作上讲，即在同一个局域网中的多个计算机中，必须有一个充当ROS Master的角色，其余的计算机必须声明自己的ROS Master为这台计算机，只有这样，真个系统才能够真正的进行互相通信。 通常需要设置的如下环境变量如下：

```bash
export ROS_MASTER_URI=http://ip/of/master/computer:11311
export ROS_HOSTNAME=ip/of/this/computer
```

举个栗子吧，比如现在有X,Y,Z三台电脑，他们的IP分别是：

```bash
X:192.168.1.10
Y:192.168.1.20
Z:192.168.1.30
```

我们要让X作为Master，那么在三台计算机中需要分别配置：

```bash
# X配置
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_HOSTNAME=192.168.1.10

# Y中配置
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_HOSTNAME=192.168.1.20

# Z中配置
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_HOSTNAME=192.168.1.30
```

在整个ROS系统开始工作前，必须先启动ROS Master，一般通过运行`roscore`启动。

### (二) 通信框架

ROS的一般通信是通过Topic的形式进行的，关于Topic后边会进行叙述，这里可以简单理解为，Topic是两个节点进行通信的一个媒介即可，底层实际就是Socket通信。

此外，ROS的通信还有Service和Actionlib的方式，这里简单画一个图。

![img](../images/v2-66fe8d489917fd07b54e56014626e84c_720w.jpg)节点通信示意图

图中，节点A和节点B之间要进行通信，主要有三种方式，topic，service和action。从图上也可以看出，service和action这两种方式都是以client-server的形式进行的。

那么这三种方式如何选择呢？这里从实际经验中给出一些建议。

Service适合那些通信不是很频繁，只需要在有需要的时候请求一次服务并且期望得到一次反馈的情况。其实是接收方(在上图中即节点B)主动请求一次服务，这个服务可以是A中的某种数据，比如A中存着SLAM建的地图，B通过服务申请这个地图数据，也可以是A中存在的一个算法，比如A中提供一个加法服务，B节点拿着两个数去请求A给算一下这两个数的和是多少，A算完以后需会把结果反馈给B。这里B只需要在需要计算的时候请求一次服务即可。

Action的方式常用语那些需要实时给出反馈的以及运行结果的通信。比如A是一个机器人导航功能节点，节点B用于给A发送目标点，但是从导航的逻辑来讲，节点B当然想实时知道A的执行结果，也就是说机器人此时已经到哪里了，到没到终点，路上有没有遇到麻烦，遇到什么样的麻烦了，最终还能不能到达等，这种情况下action就比较方便。

剩下的就都用Topic的形式就可以，这也是用的最多的一种方式。比如传感器的数据要给算法用，那传感器节点就往某个Topic里发数据，而算法节点就从这个Topic数据里获取数据，就是这么简单。

这里需要注意的是，一个topic可能不止一个发布者与订阅者。这个例子中，也可能有节点C,D也往这个topic里发数据，另外有M,N也订阅这个topic，这都是可以的。

### (三) 松耦合的数据依赖

在上边关于传感器依靠topic通信的例子中，两个节点之间只是通过topic进行通信而已，并没有其他的多余依赖。就是说，不管有没有A，B都可以运行起来，反过来也是，不管有没有B，A也都是可以运行起来发送数据的。这二者谁先运行都无所谓，只要通信可以建立就可以。

但是，service和action又有点区别了。前边也说了，二者通过C-S的方式工作的，也就规定了server必须先建立起来，client才能连接上来，那么看起来，A必须在B之前启动。但是实际在写代码的时候会有点小技巧，节点B启动以后会首先创建client并尝试连接server，如果能连上，表示A已经启动了，那就继续运行就好；而如果连接失败了，那表明server端可能还没启动，所以就等一段时间然后再尝试连接。一般情况下会设置一个超时时间，如果超时还连不上，那就不等了，直接退出。这样，A和B也就不用再纠结谁先启动谁后启动了。

### (四) 数据驱动的编程模型

在前边介绍A和B之间通过topic松耦合的依赖关系时提到，B完全可以在A尚未启动的时候就启动。还是前边的例子，假如A是传感器节点，B是算法节点，那么B必须依赖于A产生的数据才能工作，如果没有A,B如何工作呢？

这里就是我想说的数据驱动型编程(我也不知道该怎么描述，姑且这么叫)。就是说在编写B的代码时，首先要等待是否有收到数据，如果收到数据了，那我的算法就依赖该数据开始工作，如果没有收到，那就在那等着，知道有数据收到为止。因此，经常会在收到数据的callback里边做一些数据存储以及设置标志位的工作。

> 这块现在看起来，也不是那么准确，如果数据频率比较快，一般情况下也会只在callback里做存储，然后由算法线程去处理。

## 二、从微观角度看ROS

上文中我们介绍了各种概念比如节点、topic什么的，这里我们再稍微详细的说明一下这都是些什么东西。

### (一) package

package就是一个工程，一个 CMake 工程。里边包括源码与各种配置文件、启动文件等等。一般通过ros提供的工具创建，因此只需要记住怎么创建就可以，别的等创建后再修改相关文件就好了。

### (二) 节点(node)

节点是ROS中最常见的概念，它其实就是一个可执行程序，运行起来就是个进程(可能多线程)。但是这个进程又不是个普通的进程，它一般会用到ROS的通信框架以及其他的一些ROS工具，因此这个进程必须让ROS知道(其实是让ROS Master知道)它是一个节点才行啊，所以节点一般需要调用ROS的一些初始化API(后文再说)。因此，从编程的角度来讲，节点就是一个调用了ROS的一些初始化API，并使用ROS提供的其他功能如通信框架的普通程序。这里再提一下，如果节点要实现一个算法，要注意前文提到的**数据驱动型编程**。

### (三) nodelet

这里还是提一下nodelet, nodelet从本质上讲也是一个node，只不过它是以特殊形式运行起来的node，而且具有某些突出特点。

我们前文提到，节点间的通信实际上是一种Socket通信，既然是Socket通信就免不了一些数据拷贝的开销，想一下，如果数据量非常大比如一帧1080p甚至更大的图象，这种数据拷贝开销是非常可怕的。那么我们有没有可能消除这种开销呢？当然可以了，就是通过nodelet。nodelet在写代码的时候实际上式写一个类继承nodelet`::Nodelet`，然后实现相关中的接口，然后把这个类编译成一个lib，然后通过某种方式把它以线程的方式加载进进程中，这里不展开，可以查阅相关ROS WiKi。

从实现上讲，nodelet之间的topic通信，采用的是共享内存的方式，这也就消除了数据拷贝的开销。一般情况下，产生大量数据的传感器节点，都会有nodelet的实现形式。然后使用这个数据的算法节点也可以实现城nodelet的形式，然后启动一个nodelet manager加载这两个类，他们之间的通信就会舒服很多。

### (四) topic & message

topic是ROS中最常用的通信方式，前边的例子中节点A和节点B通过topic进行通信。那么很多初学者会想一个问题，为什么他们俩就能通信了，谁给他们的权利，B怎么知道收到的数据是个什么东西？

首先简单说一下他们怎么知道他俩可以通信吧。还是ROS Master，前边说了节点会调用ROS的相关API进行初始化，同样的，要发布或者订阅一个topic也是调用相关API。就是在这调用的时候，A就会跟Master说，我要发一个topic，名字叫什么，里边的数据类型是什么，Master就会记载小本本上。当B启动时，也会跟Master说，我要从一个名字叫什么什么的topic中获取数据，数据类型是什么，Master也会记在小本本上，然后Master发现，A不就是你需要的吗，然后就会通知A和B你们就是对方苦苦寻觅的人，开始交往吧，然后这俩就可以开始眉目传情了。

那么数据类型呢？用于topic传递内容叫做message，其实就是个文件实现一坨数据，就像是一个结构体卸载一个文件中。有ROS已经定义好的一些数据类型，比如大多数你能用到传感器数据都有现成的message可以用，如果你的要求比较高，ROS中没有相应的message,你就可以自定义message，当然是要遵守相关的定义规则，不展开，可以自己查wiki。这里需要说明的是，在你自定义message中，可以随便使用ROS已经定义好的message，把它们当做一个普通的数据类型来用就好。

### (五) service & action

service和action的用途前文说了，剩下的需要查阅wiki看看具体的实现方式，action可能还涉及到pluginlib的问题，可以后边再详细看，实际使用的不算很多。可以先看看service怎么用。

### (六) param server

参数服务器。这个是一个很好用的东西，我们在写代码的时候经常需要外部传入一些参数。可以通过命令行参数传入，但是如果参数比较多，一般会写个文件，yaml文件是一种实现方式。

在ROS开发中，参数文件也是yaml格式的文件，但是好处是，你不用自己写解析yaml格式的代码。ROS中实现了一个叫做参数服务器的东西，它会随着ros master一块启动，你可以在launch文件中使用标签导入参数，也可以把参数写进一个yaml文件中，然后在launch文件中用rosparam标签load进来，下边举个栗子。

```xml
<node pkg="hello" type="hello_node" name="hello" output="screen" >
  <param name="param_1" type="string" default="hello world" value="nihao" />
  <rosparam file="path/to/yaml/file" command="load">
</node>
```

上边第一个会在参数服务器中添加一个叫做param_1的string类型参数，然后会加载参数文件中的所有参数到参数服务器。

然后，我们在写代码的时候就可以通过ROS提供的相关api根据名字取参数值。需要注意的是，参数会有一个命名空间，一般像param_1这样的参数，前别没加/,在参数如服务器中存储的key为/hello/param_1。 所以在写代码的时候要注意，下边是个常用的方法：

```cpp
ros::NodeHanler nh_pravate("~");

std::string param;
nh_pravate.param("param_1", param, "hello world");
```

这样默认是在当前包的命名空间下。我在写代码的时候，一般会定义两个nodehandler,其中一个是全局的，用与创建publisher什么，另一个就是如上这种，专门用于取参数。

### (七) 动态参数

程序运行过程中，或许我们需要修改某些参数的值，前边我们介绍的参数服务器上的参数其实可以用rosparam工具修改，问题是，修改了以后，代码不知道啊，所以如果想用新的参数值，可以在代码中搞一个定时器，定时查参数服务器更新相关参数值，但是这样太笨了。

动态参数应运而生。它按照一定的方式实现，具体怎么实现可以去查wiki。当动态参数调整时，代码中会有一个回调函数被调用，在回调函数中，更新相关参数即可，很方便。

### (八) launch file

用于同时启动多个节点，并方便的传入参数的一种文件，很好用。

### (九) TF

ROS中的坐标系的变换，通过TF的broadcast与listen。它提供一些工具，用于计算坐标系的变换关系，它以树的形式存在，一般情况下，在整个ROS系统中，只有一棵TF树。



## (三) ROS教程

### (一) 学习ROS wiki官方教程

原版地址：http://wiki.ros.org/cn/ROS/Tutorials 
创客智造整理版地址：[http://www.ncnynl.com/category ... rial/](http://www.ncnynl.com/category/ros-junior-tutorial/) 
作用：了解ROS机器人操作系统的安装，使用，系统结构，系统命令，包开发等



### (二) 学习来自古月居的ROS探索总结

网址：http://www.ncnynl.com/category/ros-learning/
作用：作者在学习和使用ROS过程中的总结与创新，帮助其他学习者更快了解、熟悉ROS(来自古月居)



### (三) 学习ROS的官方推荐硬件平台Turtlebot系列，目前是turtlebot2

网址：[http://www.ncnynl.com/category ... rial/](http://www.ncnynl.com/category/turtlebot-junior-tutorial/)
网址：[http://www.ncnynl.com/category ... rial/](http://www.ncnynl.com/category/turtlebot-coffee-machine-tutorial/)
网址：[http://www.ncnynl.com/category ... tion/](http://www.ncnynl.com/category/turtlebot-Simulation/)
网址：http://www.ncnynl.com/category/turtlebot-android/
作用：ROS官方的硬件平台，是学习ROS入门的优选平台，通过DEMO的学习，可以快速掌握。

### (四) 学习arduino版的ROS小车

网址：http://www.ncnynl.com/category/ros-car/
网址：http://www.ncnynl.com/category/ros-diego/
作用：了解如何结合Arduino进行ROS开发



### (五) 学习stm32版的ROS小车

网址：http://www.ncnynl.com/category/ros-car-b/
作用：了解如何结合STM32进行ROS开发



### (六) 学习Python语言及ROS开发

网址：http://www.ncnynl.com/category/Python/
网址：http://www.ncnynl.com/category/ros-python/
作用：了解Python语言并如何开发ROS程序

### (七) 学习C++语言及ROS开发

网址：http://www.ncnynl.com/category/cplusplus/
网站：http://www.ncnynl.com/category/roscpp/
作用：了解C++语言并如何开发ROS程序



### (八) 学习SLAM相关

网址：http://www.ncnynl.com/category/ros-rgbd/
网址：http://www.ncnynl.com/category/ros-laser/
网址：http://www.ncnynl.com/category/ros-slam/
网址：http://www.ncnynl.com/category/rgbd-slam/
作用：了解如何结合各种传感器实现SLAM



### (九) 学习无人机相关

网址：http://www.ncnynl.com/category/ros-ardrone/
网址：http://www.ncnynl.com/category/ros-bebop/
作用：了解如何通过ROS控制无人机



### (十) 学习机械臂相关

网址：http://www.ncnynl.com/category/turtlebot-arm/
网址：http://www.ncnynl.com/category/ros-moveit/
作用：了解如何通过ROS控制机械臂