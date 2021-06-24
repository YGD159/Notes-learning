## 机器人操作系统ROS

原文链接:  https://www.jianshu.com/u/143c9c8061ec

github链接  :https://github.com/zhaozhongch/ros_tutorial

资料：ROS_Robot_Programming_CN.pdf



### (一) 发布接收消息

#### 1）ROS常用概念

1: <b>message</b>:　即消息．机器人需要传感器，传感器采集到的信息，即这儿的message. 假如我们的GPS采集到机器人位置消息，温度计采集到的温度等.　任何数据都能作为message.

2: <b>topic</b>: 假设我们有两个传感器，GPS和温度计．在ROS中我们得给采集到的消息取个名字用来区分不同的message，这就是topic了.　至于怎么取名字，后面的程序可见.

3: <b>node</b>: 节点. ROS中，通常来讲你写的c++或者python程序主函数所在的程序称为一个节点(并不准确，暂时这么理解).
4: <b>package</b>: 一个ROS package包含了你要完成的一个项目的所有文件.

5: <b>workSpace</b>: ROS workspace，即ROS的工作空间．你当然需要ROS来完成很多不同的项目．你的ROS工作空间是用来存放很多不同package的．

6: <b>publish, subscribe</b>: ROS很大的一个作用就是传递message．信息的传输在ROS中称为publish. 信息的接收在ROS中称为subscribe.


#### 2）创建一个workspace

创建第一个ROS的工作空间用来放置我们后面会用到的不同的package了．打开一个terminal, 输入

>source /opt/ros/melodic/setup.bash

>mkdir -p ~/catkin_ws/src

>cd ~/catkin_ws/

>catkin_make

第一行 source命令的作用是使电脑进入ROS环境. melodic是ROS的版本名称，如果使用的其他版本就把那儿换成其他版本名称．

第二行命令创建一个名字叫catkin_ws的ROS工作空间，里面包含一个叫src的文件夹用来存放源程序.注意不是所有的ROS工作空间都得叫catkin_ws，那可以是任何名字.ROS官网一般使用catkin_ws这个工作空间名字做教程．但是src文件夹都得有的．

第三行命令进入工作空间文件夹

第四行命令．ROS怎么知道你创建的文件夹是ROS的workspace不是杂七杂八用的文件夹?经过第四行命令之后你会发现你的workspace多了两个文件夹，一个叫build，一个叫devel.　这时你的第一个ROS workspace就已经创建好了

####  3）创建一个package

建立好第一个workspace之后，开始创建你的第一个ROS项目啦．打开一个terminal，键入下面的命令

>cd ~/catkin_ws/src

>catkin_create_pkg pub_sub_test std_msgs rospy roscpp

>cd ..

>catkin_make

第一行进入workspace的src文件夹，所有package都要在这里面创建. 编译的时候ROS才能找得到.

第二行catkin_create_pkg是ROS自带的命令，表示要创建一个package了. pub_sub_test是这个package的名称，当然你也可以取任何名字. ROS的package命名习惯是不出现大写字母．后面的内容表示你这个ROS package的依赖项．(这就好像写c++你需要include,写python你需要import)可以一开始指定你需要包含哪些ROS已有的软件包(也可以后添加)．

备注：std_msgs全称standard_message，即标准消息．如果message是一些比较常见的量比如float, int, bool, array等，就需要包含这个软件包．
rospy表示你需要ROS能识别并使用python文件，
roscpp表示你需要ROS能识别使用并编译c++文件，所以这三个一般我们在创建新的package时都是需要的．

第三行重新回到workspace文件夹，编译命令catkin_make需要在这里执行．catkin_make在前面我们使用过，它集成了cmake,make等一系列命令．执行了这条命令，ROS会自动找到该workspace中的未编译的文件进行编译．
创建了一个新的package之后我们都需要使用catkin_make命令使ROS意识到我又有一个新的package添加到我的workspace中了．

完成上述命令后进入到catkin_ws/src文件夹中会发现多了一个pub_sub_test文件夹，这就是你的新package了．

#### 4）写发布消息的发布器

打开一个terminal键入下面命令

>cd ~/catkin_ws/src/pub_sub_test/src

>touch pub_string.cpp

第一个行命令进入你新package的src文件夹(在你之前catkin_make的时候就自动创建了)，我们一般在这个文件夹写c++的源代码．

第二行命令我们创建一个叫pub_string的c++文件.

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "talker");
    ros::NodeHandle n;
    //初始化节点
    ros::Publisher chatter_pub=n.advertise<std_msgs::String>("chatter",1000);
    //发布话题"chatter"
    ros::Rate loop_rate(10); //睡眠频率，配合loop_rate.sleep()使用
    int count = 0;
    while (ros::ok)
    {
        std_msgs::String msg;
     //定义了std_msgs::String的对象msg       String/Int8
        std::stringstream ss;
        ss << "hello world"<<count;//写入ss 字符串流
        msg.data = ss.str();//赋值msg

        ROS_INFO("%s",msg.data.c_str());
        chatter_pub.publish(msg); //pusblish()
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;    
}

```

1: #include "ros/ros.h" ，你的节点(包含main函数的那个程序)如果要使用ROS都得包含这个头文件

2: #include "std_msgs/String.h"，我们这个程序是用来发布一个String消息的，ROS针对每一个类型消息都有相应的头文件，如果要发布那个类型的消息就必须包含相应的头文件．以后我们发布不同的消息的时候你就会看到不同点．

3:#include<sstream>，sstream是c++自带的头文件，可以实现利用输入输出流的方式往string里写东西，并且还可以拼接string和其他类型的变量．

>std::stringstream ss;

>ss<<"hello world"<<count;

代码中的实现了string hello world和int变量 count的拼接，形成一个新的string．即如果count是１，那么helloworld1会作为string被存放在ss当中．ss.str()就可以调用．

4: ros::init(argc, argv, "talker")，从名字很好理解，在程序初始化ros，这行代码也是你几乎所有节点都需要的．talker就是node(节点)的名字．即你这段程序在ROS当中的名字叫talker．

5: ros::NodeHandle n，官方的解释是nodeHandle是和ROS系统通讯的重要工具。咱们可以暂时这么记着两个东西，4,5都是在程序中初始化ROS节点所必须的。
另外有一点需要提醒的是命名问题，节点名字talker，NodeHandle的对象名n，可以取任何名字，方便以后读懂程序。节点名一般要显示这个节点的作用。NodeHandle的对象名按照习惯一般取为n。

6:ros::Publisher 这一行定义你要publish的信息和信息的名字了。n.advertise通过NodeHandle的对象n告诉ROS系统我要创建一个可以发布信息的对象了。<std_msgs::String>告诉ROS我要发布的是标准信息中的String类型。名字叫chatter。这个chatter就是我们之前提到的topic！(topic名字代表的是你要发布的信息，节点的名字代表你这个程序)。

1000这个数字的意思是要缓冲的信息数量。ROS会把发布的信息都写进缓冲区，先存个1000条，然后接收的程序慢慢从缓冲区里读，只要信息不超过1000条，总归是可以慢慢读完的。超过1000条,最早的信息就直接丢掉了。缓冲区接收最新的信息放到信息序列的最后。即缓冲区的信息的数据结构是queue。第一条来的信息在序列满了的情况下会被第一个丢弃。

7：ros::Rate loop_rate(10) 这个很好理解，我的程序如果在不断地发布信息，那么有时候我会想控制发布的信息的快慢，这行表示你希望你发布信息的速度为10Hz。这个函数要和loop_rate.sleep()配合使用才能达到控制速度目的。

8：while(ros::ok()) 要是ros不OK，程序就退出了。什么时候ROS不OK呢？上面提供的中文链接中说了四种可能。最常见的就是你按下ctrl+c或者在程序遇到ros::shutdown()这行命令。

9: std_msgs::String msg定义了std_msgs::String的对象msg. 这是我们要发布的信息。ROS为了管理方便，把所有可能发布的消息都归类到ROS中了。

10：msg.data = ss.str()这一行把ss.str()赋值给msg.data。这行把我们想要发布的std::string搬运到了std_msgs::String中，这样我们才能发布这条消息。

10: ROS_INFO()这一行就可以理解为ROS里的printf()。

11: chatter_pub.publish(msg) ,定义了一个chatter_pub用来发布信息。在完成了把std::string放到std_msgs::String的msg之后，就可以发布这个信息了。方法就是这个直观的名字pusblish().

12: ros::spinOnce()关于这个函数和将出现的ros::spin()简单来说这个函数是用于接收器的，必须要有spinOnce或者spin，ROS才会检测是不是接收到信息。spinOnce就是检测一次，spin()就是一直检测。

#### 5）写订阅消息的接收器

打开一个terminal，输入下面命令

>cd ~/catkin_ws/src/pub_sub_test/src

>touch sub_string.cpp

通过这两行命令，我们进入和pub_string.cpp相同的文件夹写sub_string.cpp程序

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr&  msg)//msg --接收信息的指针
//回调函数  
//Int 8 类型(const std_msgs::Int8::ConstPtr&  msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    //初始化节点
    ros::Subscriber sub =n.subscribe("chatter", 1000, chatterCallback);
    //订阅chatter话题
    ros::spin();
    return 0;
}

```

1: include的两个头文件在发布器中也包含了的。使用了ROS自然要包含ros/ros.h，同样，作为接收器要接收string类型的消息，你也得包含ROS中的string消息的定义std_msgs/String.h

2: main函数。我们说过节点就是main函数在的那个程序。而节点都得初始化ros和nodeHandle。main函数的头两行，和发布器的头两行一样的。除了节点的名字换成了listener。(如果ROS遇到了相同的节点名字那么他会停止掉旧节点的名字然后使用新节点的那个程序)

3: ros::Subscriber这一行是定义接收器的方法，对比发布器中定义ros::Publisher的方法，发现十分类似。Publisher使用'n.advertise'，这儿使用n.subscriber表示定义接收器, chatter即前面publisher的topic的名字。接收器通过找谁的topic和自己一样就接收谁的信息。chatterCallback称为回调函数，接收器每一次接收到消息，就会调用名字为它的函数。ROS的回调函数返回值只能为空。

4:  (const std_msgs::String::ConstPtr& msg)这个模式还是比较固定的，如果你要接受的是Int8类型的消息，那么参数会变成(const std_msgs::Int8::ConstPtr& msg)。ConstPtr代表一个指针。printf(ROS_INFO)中有msg->data这种调用方式。（在Publisher中我们定义了std_msgs::String对象msg，类包含数据成员data，调用方式为msg.data。如果类的指针叫msg,那么调用该成员的方式是msg->data）。所以现在msg->data就是一个std::string类型的量，假设有string类型的变量st要想print出来,代码就是printf("%s,", st.c_str() )（不能直接print st）。使用ROS_INFO其内部完全一样。

5: ros::spin()会使程序在循环中，一直检测有没有接收到新的消息。其终止方式和使ros::ok()返回false方式一样。

#### 6）编译ROS程序

打开你的terminal，首先进入你的package的CMakeLists所在的文件夹

>cd ~/catkin_ws/src/pub_sub_test/

>gedit CMakeLists.txt

gedit是一个文本编写软件，一般Linux会自带，你要是没有的话就直接进入文件夹，双击CMakeLists.txt打开也可以．(gedit ABC表示用gedit打开ABC，如果没有ABC就会创建一个．)

找到一行写着##Declare a C++ executable，在这一行前面或者后面添加如下内容

```json
add_executable(pub_string src/pub_string.cpp)
target_link_libraries(pub_string ${catkin_LIBRARIES})
add_executable(sub_string src/sub_string.cpp)
target_link_libraries(sub_string ${catkin_LIBRARIES})
```

第一行表示我们要编译add_executable表示我们要添加一个可执行文件，pub_string是这个可执行文件的名字（并不是非得和pub_string.cpp中的pub_string一样，不过建议用一样的），src/pub_string.cpp指定要编译的源文件的位置．

第二行target_link_libraries表示我们要将可执行文件链接到一个库，我们要使用ROS当然是要链接到ROS的库了，括号里pub_string指定要链接可执行文件的名字，后面是指定要链接的库的名字．

三四行类似，作用是编译sub_string.cpp文件．
添加完上述内容后我们保存并退出CMakeLists.txt文件．

编译文件

>cd ~/catkin_ws/

>catkin_make

#### 7）执行ROS程序

打开三个terminal.
第一个terminal中输入

>roscore

roscore是为了让各种节点之间能够沟通用的. 使用rosrun命令之前，如果没有开启roscore则命令会失败．

在第二个terminal中输入下面内容

>cd ~/catkin_ws/

>source devel/setup.bash

>rosrun pub_sub_test sub_string

第二行命令是source是使计算机进入ROS环境．

第三行命令是跑ROS程序的．rosrun代表你要跑一个ROS节点．第二个输入的是节点所在的package的名字，第三个是节点自己的名字了．

在第三个terminal中输入下面内容

>cd ~/catkin_ws/

>source devel/setup.bash

>rosrun pub_sub_test pub_string

即可在发布与订阅终端看到信息

打开新终端，输入以下内容即可看到node和node之间的关系(节点talker通过topicchatter向节点listener发布消息)

>source ~/catkin_ws/devel/setup.bash

>rqt_graph

![r01](../images/r01.webp)



### (二) 发布接收不同类型消息1

#### 1）发布接收int类型消息

打开一个terminal．输入下面内容,创建了用来发布８位整型message的发布程序和接收程序

>cd ~/catkin_ws/src/pub_sub_test/src

>touch pub_int8.cpp

>touch sub_int8.cpp

打开pub_int8.cpp,输入

```cpp
#include "ros/ros.h"
#include "std_msgs/Int8.h" //#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000); //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int8 msg; //std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    msg.data = count;// msg.data = ss.str();


    ROS_INFO("%d", msg.data); //ROS_INFO("%f", msg.data.c_str())

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
```

打开sub_int8.cpp,输入

```cpp
#include "ros/ros.h"
#include "std_msgs/Int8.h" //#include "std_msgs/String.h"

void chatterCallback(const std_msgs::Int8::ConstPtr& msg) //void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data); //ROS_INFO("I heard: [%f]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
```

需要添加到CMakeLists里编译，在terminal中输入

>cd ~/catkin_ws/src/pub_sub_test

>gedit CMakeLists.txt

在CMakeLists.txt里添加了下面的内容

>add_executable(pub_int8 src/pub_int8.cpp)

>target_link_libraries(pub_int8 ${catkin_LIBRARIES})

>add_executable(sub_int8 src/sub_int8.cpp)

>target_link_libraries(sub_int8 ${catkin_LIBRARIES})

保存并退出．接着在terminal中输入下面的内容

>cd ~/catkin_ws

>catkin_make

打开三个terminal.
第一个terminal中输入

>roscore

在第二个terminal中输入下面内容

>cd ~/catkin_ws/

>source devel/setup.bash

>rosrun pub_sub_test sub_int8

在第三个terminal中输入下面内容，运行程序

>cd ~/catkin_ws/

>source devel/setup.bash

>rosrun pub_sub_test pub_int8

#### 2）代码对比

首先是发布器程序
 1.`#include "std_msgs/Int8.h"`代替了`#include "std_msgs/String.h"`．

 每一种不同的消息都有自己的头文件，如果我们要使用不同的消息，就首先要包含它所在的头文件．像`Int8, String`这类都属于C++的标准数据类型，所以这些消息在ROS中也被划分到了`std_msgs`这个名字下．消息当然还有其他大类，比如我们以后要使用类似于pose的消息，它包含在geometry_msgs这个大类里．需要包含的头文件是 `#include "geometry_msgs/Pose.h"`.
 2.`ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000);`　中，`std_msgs::Int8`代替了`std_msgs::String`，定义publisher的时候，它需要发布什么消息是需要指定明确的，之前是ros中的string那么现在就自然换成ros中的int8了
 3.`std_msgs::Int8 msg`　替换了　`std_msgs::String msg`.　同样你要发布的消息的类型替换掉．
 4.发布string类型的消息的代码中连续的三行

```cpp
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
```

直接被替换成了`msg.data = count`．我们知道原来的三行中，被注释的前两行是用来形成一个字符串的．原来的msg是std_msgs::String, msg.data就是string类型，现在msg是std::msgs_Int8，那么大概可以猜到msg.data如今就是int8类型了，考虑到我们的count本来就是一个int型变量，所以这儿直接把count赋值给msg.data了．注意这儿的类型变换是int到int8，问题不大，只是整型的范围缩小到-128到127了而已．
 5.`ROS_INFO`中msg.data本来就是int8型的变量，可以直接print出来，对应需要在ROSINFO中表明数据类型是`%d`(字符串类型是`%f`)．

 接收器程序．

1. `#include "std_msgs/Int8.h"`代替了`#include "std_msgs/String.h"`　不在赘述．

2. `chatterCallback`函数的参数由`std_msgs::String::ConstPtr& msg`变成了`std_msgs::Int8::ConstPtr& msg`

3. `ROS_INFO`中，之前msg->data是stiring类型现在是int8类型，可以直接print出来．
   主函数中没有设计到变量定义的地方，所以不需要更改．
   总结一下就是把涉及到表明数据类型的地方全部换成std_msgs::Int8就可(注意大小写)．

   

#### 3）ROS wiki了解需要数据类型

点进去你会发现里面有std_msgs所包含的所有信息种类．
 http://wiki.ros.org/std_msgs/



**例子:在ROS中使用double类型的变化**

创建pub_arrary.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h" //#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1000); //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  //int count = 0;

 // double testArray[5] = {1,2,3,4,5};  ROS使用double类型需要vector
    
  std::vector<double> testArray = {1,2,3,4,5}
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg; //std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    msg.data = testArray;// msg.data = ss.str();


    ROS_INFO("I have published array data"); //ROS_INFO("%f", msg.data.c_str())

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
   // ++count;
  }
```



即`double testArray[5] = {1,2,3,4,5}`改成`std::vector<double> testArray = {1,2,3,4,5}`. 另外既然要使用vector，那么一般要添加其头文件，添加头文件`#include <vector>`，但是`"std_msgs/Float64MultiArray.h"`，已经包含了<vector>，就不用再重复包含了,`std::vector<double> testArray = {1,2,3,4,5}`这种给向量的赋值方式是C++11之后才有的，所以要添加C++11编译．ROS的CMakeLists.txt的最上面几行有一行是`#add_compile_options(-std=c++11)`，把注释符号#去掉就可以了．保存退出



在CMakeLists.txt中你之前添加编译文件的下方添加

>add_executable(pub_array src/pub_array.cpp)

>target_link_libraries(pub_array ${catkin_LIBRARIES})

在同样的位置创建一个叫sub_array.cpp的文件，内容大同小异．

```cpp
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h" //#include "std_msgs/String.h"

void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) //void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f], [%f]", msg->data[0], msg->data[1]); //ROS_INFO("I heard: [%f]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
```

添加好内容后保存退出，回到catkin_ws目录下使用catkin_make编译．



### (三) 发布接收不同类型消息2

#### 1）发布接收PoseStamped类型消息



![14634719-b5e975d62785423d](../images/14634719-b5e975d62785423d.webp)

１是geometry_msgs/Point类型的`position`，而`position`包含三个float64的变量`x,y,z`，这个很好理解了，怎么定义机器人的位置？三维坐标x,y,z就可以了．
 2是geometry_msgs/Quaternion类型的'oreintation'，而`oreintation`包含四个float64的变量`x,y,z,w`，quaterion中文四元数，是一个用来表示方向的东西．四元数的缺点是不很形象，不熟的人很难直接通过四元数在脑海里构想出机器人目前到底是个什么朝向．我们一般用欧拉角表示方向时，一共有三个数roll,pitch,yaw，比较直观，但是欧拉角表示方向时会遇到一个叫Gimbal Lock（万向锁）的尴尬问题，所以ROS里统一用四元数表示方向．

在`pub_sub_test/src`中创建一个名字叫`pub_poseStamped.cpp`的文件．在文件中写入下面的内容

```cpp
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" //include posestamp head file

#include <cmath>//for sqrt() function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("chatter", 10); //initialize chatter

    ros::Rate loop_rate(10);

    //generate pose by ourselves.
    double positionX, positionY, positionZ;
    double orientationX, orientationY, orientationZ, orientationW;

    //We just make the robot has fixed orientation. Normally quaternion needs to be normalized, which means x^2 + y^2 + z^2 +w^2 = 1
    double fixedOrientation = 0.1;
    orientationX = fixedOrientation ;
    orientationY = fixedOrientation ;
    orientationZ = fixedOrientation ;
    orientationW = sqrt(1.0 - 3.0*fixedOrientation*fixedOrientation); 

    double count = 0.0;
    while (ros::ok())
    {
        //We just make the position x,y,z all the same. The X,Y,Z increase linearly
        positionX = count;
        positionY = count;
        positionZ = count;

        geometry_msgs::PoseStamped msg; 

        //assign value to poseStamped

            //First assign value to "header".
        ros::Time currentTime = ros::Time::now();
        msg.header.stamp = currentTime;

            //Then assign value to "pose", which has member position and orientation
        msg.pose.position.x = positionX;
        msg.pose.position.y = positionY;
        msg.pose.position.z = positionY;

        msg.pose.orientation.x = orientationX;
        msg.pose.orientation.y = orientationY;
        msg.pose.orientation.z = orientationZ;
        msg.pose.orientation.w = orientationW;

        ROS_INFO("we publish the robot's position and orientaion"); 
        ROS_INFO("the position(x,y,z) is %f , %f, %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        ROS_INFO("the time we get the pose is %f",  msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);

        std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


  return 0;
}
```

发布器的代码中`fixedOrientation`的变量，赋值0.1，然后分别赋值给创建的double类型变量`orientationX,Y,Z,W`．在循环中，`orientationX,Y,Z,W`在分别赋值给我们创建的msg的成员变量`msg.pose.orientation.x y z w`．`msg.pose.orientation.x y z w`都是float64类型的变量，赋值了他们几个就可以定义pose的orientation了．orientation是相同的数，那么机器人就没有旋转．

那么pose的position的x y z我们直接赋值了count，count在循环中递增，那么XYZ都同时递增且相同．如果我们画一个三维坐标轴XYZ的话，那么咱么这儿模拟的机器人的运动状态，相当于机器人沿着坐标轴对角线匀速直线行驶．

 同样位置再创建一个`sub_poseStamped.cpp`．写入下面内容．

```cpp
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" 

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
    ROS_INFO("I heard the pose from the robot"); 
    ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);

    ros::spin();

    return 0;
}
```

写完之后，我们同样打开位于`pub_sub_test`目录下的CMakeLists.txt添加编译两个文件的内容．

由于使用了新的依赖,所以添加新的依赖项,需要修改两个文件，一个是package目录下的CMakeLists.txt，另一个是位于同一位置的package.xml

打开CMakeLists.txt，发现就在最前面几行，有下面的内容．



```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
```

括号中的内容正好一一对应我们创建package时添加的依赖项，那么想都不用想啦，肯定要在后面添加geometry_msgs，变成下面的样子，保存退出．



```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
```



添加完成后，保存退出．之后打开位于同一目录下的`package.xml`．（直接双击打开可能不能修改其中内容，还是用gedit什么的打开）．打开之后，在文档下方，你可以看到一下内容



```xml
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

又有`std_msgs, rospy, roscpp`，只不过每个出现了三次，那么同样，不用管什么意思，我们只需要按照这个文档里相同的语法让`geometry_msgs`出现三次就行了．更改之后该文件同样位置变成下面的内容



```xml
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
```



保存退出后使用catkin_make编译（注意catkin_make这个命令要在catkin_ws这个目录下使用的）．



### (四) C++类，命名空间，模版，CMakeLists

#### 1）类(class)

创建一个叫C++Test的文件夹，再创建三个用于测试三种东西的子文件夹．

之后，在classTest文件夹下创建一个叫`classBasic.cpp`的文件和一个叫`CMakeLists.txt`的文件．在classBasic.cpp中输入下面内容.

```cpp
#include <iostream>

class poorPhd{
public:
    /*define constructor*/
    poorPhd(){
        std::cout<<"we create a poor phd class"<<std::endl;
    }

    /*public member variable*/
    int hairNumber = 100;

    /*public member function*/
    int getGirlFriendNumber(){
        return girlFriendNumber;
    }

private:
    /*private member variable*/
    int girlFriendNumber = 0;
};

int main(){
    /*define the object*/
    poorPhd phd;//will use constructor function 
 
    /*call the public memberfunction*/
    std::cout<<"girlFriendnNumber is "<<phd.getGirlFriendNumber()<<std::endl;

    /*change tha value of member variale*/
    phd.hairNumber = 101;

    /*call the member variable*/
    std::cout<<"hairNumber is "<<phd.hairNumber<<std::endl;

    /*define class pointer*/
    poorPhd *phdPointer;

    /*assign the pointer to an object*/
    phdPointer = &phd;

    /*call the member variable*/
    std::cout<<"use pointer, hair number is "<<phdPointer->hairNumber<<std::endl;
}
```

1:#include<>　包含头文件，这样可以使用std::cout<<...std::endl;

2:class poorPhd  定义了一个叫poorPhd的类．类后跟这宗括号{}.宗括号中的内容为类的内容．

3:public 加冒号之后的内容，即为`公有`．公有范围内定义的函数为公有成员函数，变量为公有成员变量．

4:poorPhd().　这个函数称为`构造函数(constructor function)`．在类创建时，会自动调用．构造函数的名字和类的名字必须一样并且没有返回值．

５：int hairNumber = 100.　定义了一个int类型公有成员变量，赋值100.

６：int getGirlFriendNumber().　定义了一个返回值为int的函数，该函数会返回私有成员变量girlFriendNumber.

7:private加冒号之后的内容，即为`私有`．私有范围内定义的函数为私有成员函数，变量为私有成员变量．

8: int girlFriendNumnber=0.　定义了一个int类型的私有成员变量`girlFriendNumber`并赋值为0

main函数中
 9: poorPhd phd 创建了一个类的对象(object)，名字叫`phd`．每一个类，要想实际被使用，都需要创建一个对象．对象会拥有之前我们在类中定义的所有东西．所谓拥有，即是可以调用他们．对象的数量是没有限制的，并且他们之间不会干扰．你还可以用类似方法创建一个名字加`abc`的对象，它也会拥有poorPhd这个类的全部东西．
 对象在创建时，会自动调用构造函数．

10:`std::cout....phd.getGirlFriendNumber()<<std::endl;`
 类对象调用成员函数或者成员变量的方法是`对象名.成员`．**公有成员可以在类的定义外使用这种方式直接调用，私有成员是不可以被直接调用的**．所以如果我们使用`phd.girlFriendNumber`就会报错．因为在类外，不可以直接调用私有成员变量．公有成员函数定义在类中，所以它可以使用私有成员变量，并把变量的值作为返回值，这样我们就得到可私有成员变量的值．

11.phd.hairNumber = 101;
 为公有成员变量赋值101．

12.`std::cout<<...phd.hairNumber...`
 调用公有成员并print出来．

13.`poorPhd *phdPointer` 创建一个类的指针．类的指针被创建时不会调用构造函数．它需要指向一个对象．

14.`phdPointer = &phd`　刚才创建的对象的地址赋值给指针，这个指针就有了`phd`对象的所有内容．

15.`...phdPointer->hairNumber...`　类指针调用类的成员的唯一不同之处就是使用`指针名->成员`调用而不是`对象名.成员`调用．

**和之前写的ROS代码的联系**: 之前我们定义过`std_msgs::Int8 msg`，msg即是类Int8的对象．我们通过查看roswiki http://docs.ros.org/api/std_msgs/html/msg/Int8.html 得知Int8包含类型为int8的成员变量data，所以我们通过`msg.data`使用这个成员．



打开之前建立的CMakeLists.txt文件．输入以下内容．

```bash
project(class_test)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAG} -std=c++11 -Wall")

add_executable(classBasic classBasic.cpp)
```

这基本上算是一个最简单的CMakeLists.txt文件了．CMakeLists.txt是用来编译C++文件的．
 第一行表明了项目名称．
 第二行输入CMake使用的最小版本号，一般是2.8以及以上．
 第三行设定编译器．使用c++11．虽然我们的项目没用到c++11但是考虑到如今c++已经被普遍使用了，所以最好加上．我们在ROS的CMakeLists里注释过这个内容`add_compile_options(-std=c++11)`达到的也是使用c++11编译的效果．
 第四行指定要编译的文件．要编译的文件是classBasic.cpp，编译后的可执行文件名字叫classBasic．
 写完上面的内容后，保存退出．
 在terminal中cd 到`classTest`这个文件夹输入下面的内容

```bash
mkdir build
cd build
cmake ..
make
```

第一二行命令创建一个叫build的文件夹并进入
 第三行命令使用是使用cmake命令并通过`..`表示使用上一个文件夹的CMakeLists.txt．执行这行命令之后我们写的CMakeLists就会产生一系列的文件在build中，其中一个是Makefile．
 执行完上面的命令后，你会看到多了一个叫`classBasic`的文件没有后缀，这就是我们的可执行二进制文件了．使用`./classBasic`执行后得到下面的输出

```kotlin
we create a poor phd class
girlFriendnNumber is 0
hairNumber is 101
use pointer, hair number is 101
```


 在classTest文件夹下再创建一个新的文件叫 classBasic2.cpp．并输入下面的内容．

```cpp
#include <iostream>

class poorPhd{
public:
    /*define constructor*/
    poorPhd(){
        std::cout<<"we create a poor phd class"<<std::endl;
    }

    /*public member variable*/
    int hairNumber = 100;

    /*public member function*/
    int getGirlFriendNumber(){
        return girlFriendNumber;
    }

private:
    /*private member variable*/
    int girlFriendNumber = 0;
};

class master1 {
public:
    /*define constructor*/
    master1(){
        std::cout<<"we create a master class"<<std::endl;
    }
    /*member variable*/
    poorPhd future;
};


int main(){
    /*define the object*/
    master1 mStudent1;

    /*use inheritance*/
    std::cout<<"hairNumber of master student 1 is "<<mStudent1.future.hairNumber<<std::endl;
}
```

`poorPhd`类和上一个文件完全一样，新添加了一个类叫`master1`．master1同样有一个构造函数．另外它有一个成员变量，这个成员变量是poorPhd类型的对象future.那么在main函数中，定义了master1的对象mStudent1．咱们就可以用mStudent1.future调用变量future，再由于future是poorPhd类型的变量，所以可以用future.hairNumber调用hairNumber．连在一起就可以通过定义msater1的对象却最终调用了poorPhd的成员变量了．
 保存退出后，在CMakeLists.txt中添加下面的内容．



```css
add_executable(classBasic2 classBasic2.cpp)
```

terminal中进入classTest/build文件加输入



```go
cmake ..
make
```

这时候就多了一个二进制文件classBasic2，执行该二进制文件你会看到



```kotlin
we create a poor phd class
we create a master class
hairNumber of master student 1 is 100
```

从这个输入可以看出，创建master1的对象mStudent1的时候c++会首先初始化它的成员变量，所以咱得到的是create a poor phd class，之后再调用了构造函数．

#### 2）命名空间(namespace)

命名空间一般是用来避免重命名的．大型的库里面一般定义了很多类，无数的函数．不同的大型的库之间很可能会有函数甚至类的命名重复，这会造成很大的麻烦．
 namespace的命名语法也很简单

```csharp
namespace name{
    //内容
}
```

下面这个程序简单地展示了两个命名空间里定义相同名字的类，并分别使用两个类的简单程序．

```cpp
#include <iostream>

/*define a phd namespace*/
namespace phd {

    /*define a student class in phd namespace*/
    class student{
    public:
        student(){
            std::cout<<"create a student class in phd namespace"<<std::endl;
        }
        int graduateYear = 5;
        int hairNumber   = 100;
    };
}

/*define a master namespace*/
namespace master{

    /*define a student class in master namespace*/
    class student{
    public:
        student(){
            std::cout<<"create a student class in master namespace"<<std::endl;
        }
        int graduateYear = 2;
        int hairNumber   = 10000;
    };
}

int main(){

    /*create an object of student class, in phd namespace*/
    phd::student     phdStudent;

    /*create an object of student class, in master namespace*/
    master::student  masterStudent;

    std::cout<<"phd normally graduate in "<<phdStudent.graduateYear<<" years"<<std::endl;

    std::cout<<"master normally graduate in "<<masterStudent.graduateYear<<" years"<<std::endl;
}
```

上面的这个程序定义了两个命名空间，一个叫`phd`，一个叫`master`，这两个命名空间拥有一个类，类名都叫`student`．
 定义命名空间中的类的对象的方法是`命名空间名::类名　对象名`．`::`被称为作用域符号(scope resolution operator)．在main函数中我们定义了phd命名空间下的student类的对象phdStudent和master命名空间下的类student的对象masterStudenrt.　后面的两行各自输出了成员变量`graduateYear`．
 在我们之前的ros程序中，遇到了两个命名空间，一个是`std_msgs`，另一个是`geometry_msgs`．`Int8, Float64`等都是std_msgs这个命名空间下的类，`PoseStamped`等是geometry_msgs这个命名空间下的类．
 回到上面的程序我们在定义完phd这个命名空间后，可以使用`using namespace phd`，这样在main函数中我们可以不使用`phd::`来定义一个phd下的student类的对象,直接`student phdStudent`即可．同样，如果我们添加`using namespace master`，我们也可以直接使用`student masterStudent`来定义msater命名空间下student类的对象．
 但是如果在程序中同时添加了

```csharp
using namespace phd;
using namespace master;
```

这时候你在main函数中写`student object_name`就肯定会报错．因为电无法知道你要使用的student类是属于哪个命名空间的．所以一般为了图方便，在确定没有类名会重复时，我们添加`using namespace ...`这一行在定义完头文件之后，这样我们就可以省去在定义类时一直使用`namespace_name::类名`这种格式命名．但是有些时候如果两个库很有可能有相同的类名，就不要使用`using namespace ...`，不然很有可能造成程序的误解．



#### 3）模版(Template)

```cpp
#include <iostream>

int square(int a){
    return a*a;
}

int main(){
   double x = 5.3;
   std::cout<<"the square of "<<x <<" is "<<square(x)<<std::endl;
}
```

这个程序有个很明显的缺点，编写函数或者使用变量时，都必须先指定类型，由于c++函数形参类型和返回值已经指定为int类型了，你只能传int类型进去，如果传double类型的变量进去，变量会被强制转换截断为int类型．而且只能return整型的变量．所以你只能得到25．
 基本的解决方法是函数的重载，即我可以命名相同的函数但是变量类型或者个数不同以实现对不同输入的处理．类似于下面这样

```cpp
#include <iostream>

int square(int a){
    return a*a;
}

double suqare(double a ){
    return a*a;
}

int main(){
   double x = 5.3;
   std::cout<<"the square of "<<x <<" is "<<square(x)<<std::endl;
}
```

这样调用square(x)时会自动匹配形参相同的函数．我们可以得到5.3的平方．但是可以想象，如果我有很多不同类型的变量要传入，我就得写好多不同的除了变量类型不同，其他的一模一样的函数了！
 模版应运而生．模版的定义方式是

```cpp
template <typename T>
```

或者

```cpp
template <class T>
```

定义完之后后面紧跟要实现的函数或者是类．这个class不是我们之前理解的那种class了．这儿的class和typename作用完全一样，表示定义了一个新的类型T．这个新的类型具体是什么不知道，要等我们具体使用时程序根据传入的类型自行判断．
实现数字平方相同的功能.

```cpp
#include <iostream>

template <typename T>
T square(T a){
    return a*a;
}

int main(){
    double x = 5.3;
    std::cout<<"square of "<<x<<" is "<<square(x)<<std::endl;
}
```

现在你无论传什么类型的数据进去，都会得到它的平方．sqaure指定的函数形参和返回值类型都为T．可以这样理解，现在当我们传入一个double类型的变量时，T就会自动变成double，传入int时，T就自动变为int．
我们定义向量时要指定向量元素的类型．比如`std::vector<int> a`，`std::vector<double> b`等．和上一个例子一样，为了避免传入重载函数，我们使用模版．代码如下

```cpp
#include <iostream>
#include <vector>

template <typename T, typename U>
U addVector(T vec1, U vec2){
    
    U result;

    if(vec1.size()!=vec2.size()){
        std::cout<<"cannot add two vector, they must be the same length. Return a null vector"<<std::endl;
        return result;
    }

    for(int i = 0; i<vec1.size(); i++){
        result.push_back(vec1[i]+vec2[i]);
    }
    return result;
}

int main(){
    std::vector<int> vec1 = {1,2,3};
    std::vector<double> vec2 = {4.0,5.0,6.0};

    auto addVec = addVector(vec1,vec2);

    for(auto i:addVec)
        std::cout<<i<<",";

    std::cout<<std::endl;
}
```

我们的tempalte定义了两个类型，一个叫U，一个叫T．为什么要定义两个呢？因为前面说过模板定义的具体类型在使用时确定的，在主函数中我们要加两个vector,一个是int类型的，作为第一个参数传入addVector，那么T就会是`std::vector<int>`，而第二个参数是double类型的向量，作为第二个参数传入函数后U就会相当于`std::vector<double>`，函数返回的类型也是U.
 程序主函数第三行使用了`auto`这个关键字．使用c++11编译才可使用auto．这个是很有用的关键字．auto会自动分配被它定义的对象的类型，根据赋值的变量的类型．addVector返回的是U，在这个程序里也就是std::vector<double>了．那么auto会自动让addVec称为dpuble类型的vector．
 主函数第四行的for循环采用的是有别于我们常用的for循环的形式．

```cpp
for(auto i:addVec)
```

其中`i:addVec`的作用是把addVec中的元素依次赋值给i，这就要求i的类型得和addVec中的元素的类型相同，不过有auto的帮助，我们也就不用管这么多了，把i的类型定义为auto，那么程序会自动让i成为addVec中要赋值给i的元素的类型，这儿也就是double了．

 我们来建立一个简单的sqaure类．

```cpp
#include <iostream>

template <typename T>
class square{
public:
    T a;
    /*constructor function will store _a*_a as public member a*/
    square(T _a){
        a = _a*_a;
    }
};


int main(){
    double x = 5.5;
    square<double> test(x);
    std::cout<<"the square of "<<x<<" is "<<test.a<<std::endl;
}
```

在声明了模版之后紧接着我们声明了一个类，类的公有成员函数是一个类型为T的值a．主函数中，在我们声明模版下定义的类的对象时，我们需要在`<>`之中表明T的类型．再这之后才能定义对象．即普通的类的对象的定义格式如下

```undefined
类名　对象名(构造函数参数)
```

模版下的类的对象定义的格式就是

```xml
类名<模版变量类型> 对象名(构造函数参数)
```

main函数第二行的这种定义方法，就类似于我们`std::vector<int> ABC`这种定义方法了，后者多的不过是在命名空间下定义了模版．然后再在模版下定义类．



### (五) 在类中发布和接收消息

#### 1）例一

创建pub_int8_class.cpp和sub_int8_class.cpp．在pub_int8_class.cpp中输入以下内容．

```cpp
#include "ros/ros.h"
#include "std_msgs/Int8.h"

class pubInt8{
public:
   pubInt8(ros::NodeHandle& nh){
       chatter_pub = nh.advertise<std_msgs::Int8>("chatter", 1000); //assign value to chatter_pub
   };

   void pub(){
       int count = 0;
       ros::Rate loop_rate(10);
       while (ros::ok())
       {
           std_msgs::Int8 msg; 

           msg.data = count;// msg.data = ss.str();

           ROS_INFO("%d", msg.data);

           chatter_pub.publish(msg);

           ros::spinOnce();

           loop_rate.sleep();
           ++count;
       }
   }

private:
   ros::Publisher chatter_pub;//set the publisher as a member
};

int main(int argc, char **argv)
{
 ros::init(argc, argv, "talker");

 ros::NodeHandle n;

 pubInt8 p8(n);

 p8.pub();

 return 0;
}
```

对比我们在第二节中的pub_int8.cpp中的内容, 总的来说大同小异．
 我们把ros::Publisher chatter_pub作为类的一个私有成员成员变量定义在了类中．这时候并没有对它赋值．然后在构造函数中才对他赋值的．
 对publisher赋值需要rosNodeHandle，所以我们在创建对象时把rosNodeHandle作为一个参数传入了构造函数，这样我们就可以在构造函数中为chatter_pub赋值．
 之后在主函数中我们使用类的pub函数实现信息的发布，发布的部分的代码和pub_int8.cpp中的一模一样．

下面来看看sub_int8_class.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/Int8.h" 

class subInt8{
public:
    subInt8(){};
    void chatterCallback(const std_msgs::Int8::ConstPtr& msg){
        ROS_INFO("I heard: [%d]", msg->data);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  subInt8 s8;

  ros::Subscriber sub = n.subscribe("chatter", 1000, &subInt8::chatterCallback, &s8);

  ros::spin();

  return 0;
}
```

同样来看看和sub_int8.cpp不同的地方．
 callback函数内容一模一样，只是放到了类中．
 在定义subscriber时和不使用类时有很大的不同．
 不使用类时，我们是这么写的
 `ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);`
 使用类时，我们是这么写的
 `ros::Subscriber sub = n.subscribe("chatter", 1000, &subInt8::chatterCallback, &s8);`

即原来我们只需要把回调函数名作为参数写在n.subscribe中，现在我们需要用`&类名::函数名`代替原来那个参数，之后还需要增加一个参数，即在定义subscriber之前定义好的类的对象名`&对象名`．

把这两个程序写入CMakeLists中，catkin_make后使用rosrun跑起来．会得到第二讲pub_int8程序一样的效果．
 (注意不要死板，使用要灵活．比如说你写了一个在类中发布消息的程序，你不仅需要在构造函数当中使用rosNodeHandle，还需要在其他函数中使用，怎么做？你可以把ros::NodeHandle也作为一个私有成员（或者公有成员）写在类中，使用构造函数把外来这个nodeHandle传值给私有成员nodeHandle．或者甚至不在构造函数传值，专门写一个register　nodehandle的函数)

```cpp
void registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh; //_nh is a member in class, _nh is the nodehandle you define, for example, in main function.
}
```

这样nh属于你的类，你就可以在类的其他函数进行任何有关它的操作了．
 在类外，你只需要定义一个该类的对象，通过 `对象.registerNodeHandle(主函数中(或者其他位置)定义的NodeHandle)`把nodehandle传入类中．

#### 2）例二

首先写一个叫pubSub_class_example2A.cpp的程序，代码如下

```cpp
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <unistd.h>

class tl1{
public:
    tl1();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void iniPub();
    void fCallback(const std_msgs::Float64::ConstPtr& msg);
private:
    ros::Publisher pub_f64;
    ros::Subscriber sub_f64;
    ros::NodeHandle nh;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "talker_listener1");

    ros::NodeHandle nh;

    tl1 pubSub1;

    pubSub1.registerNodeHandle(nh);
    pubSub1.registerPubSub();

    pubSub1.iniPub();

    ros::spin();
}

tl1::tl1(){};

void tl1::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};

void tl1::registerPubSub(){
    pub_f64 = nh.advertise<std_msgs::Float64>("chatter1to2",1000);
    sub_f64 = nh.subscribe("chatter2to1",1000,&tl1::fCallback, this); //this pointer here means the class itself
};

//When you receive some data, it is highly possible you do some operations on that data and then publish something in/after the call back function
void tl1::fCallback(const std_msgs::Float64::ConstPtr& msg){
    std::cout<<"talker_listener1 heard "<<msg->data<<std::endl;

    std_msgs::Float64 pubData;

    pubData.data = msg->data - 1;

    ros::Rate sr(10);
    pub_f64.publish(pubData);
    sr.sleep();
};

void tl1::iniPub(){
    std::cout<<"publish the first data "<<std::endl;

    std_msgs::Float64 pubData;

    pubData.data = 1;

    usleep(500000);//wait for connection
    pub_f64.publish(pubData);
}
```

接下来写一个pubSub_class_example2B.cpp的程序．

```cpp
#include "ros/ros.h"
#include "std_msgs/Float64.h"

class tl1{
public:
    tl1();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void fCallback(const std_msgs::Float64::ConstPtr& msg);
private:
    ros::Publisher pub_f64;
    ros::Subscriber sub_f64;
    ros::NodeHandle nh;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "talker_listener2");

    ros::NodeHandle nh;

    tl1 pubSub1;

    pubSub1.registerNodeHandle(nh);
    pubSub1.registerPubSub();

    ros::spin();
}

tl1::tl1(){};

void tl1::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};

void tl1::registerPubSub(){
    pub_f64 = nh.advertise<std_msgs::Float64>("chatter2to1",1000);
    sub_f64 = nh.subscribe("chatter1to2",1000,&tl1::fCallback, this); //this pointer here means the class itself
};

//When you receive some data, it is highly possible you do some operations on that data and then publish something in/after the call back function
void tl1::fCallback(const std_msgs::Float64::ConstPtr& msg){
    std::cout<<"talker_listener2 heard "<<msg->data<<std::endl;

    std_msgs::Float64 pubData;

    pubData.data = msg->data + 1;

    ros::Rate sr(10);
    pub_f64.publish(pubData);
    sr.sleep();
};
```

两个文件代码基本一样，
 不同的地方是
 1:...B中没有iniPub这个函数．
 2:两个程序的node名字和topic名字有些差异，方便建立连接，自行观察，一会儿用rqt_graph就可以更明了地看到．
 针对pubSub_class_example2A把代码中比较重要的点说明一下
 1:在tl1的类中我们把ros NodeHandle也作为类的成员了，专门谢了一个registerNodeHandle的函数把nodeHandle传递到类中．另外类成员不仅有了publisher还有subscriber．我们也专门写了一个registerPubSub函数来给publisher和subscriber赋值．这样写虽然我们麻烦了一些，但是代码读起来就更加明了．
 2:这里注意一下registerPubSub中给sub_f64赋值的方法，最后一个参数是this指针．我们的第一个例子在类外定义了subscriber，最后一个参数是类的对象，我们要在类中定义subscriber，不可能给一个类的对象了，毕竟这时候我们都还不知道对象叫啥．**最流行的说法是this指针指针指向类本身．当我们在类中需要指定类对象的时候，就可以用this指针代替那个我们不知道的对象**．
 3:回调函数fCallback中我做了一个很无聊的事儿，就是把接收到的数据减１而已．这代表了对你接收到的数据以操作．实际运用中你可能有很复杂的操作．这一系列操作完成后你需要将结果发布出去．我们再以每秒１０次的频率发布出去．
 ４：iniPub函数用来初始化发布一个数据，这样2B程序的subcriber接收到后经过一系列操作(我其实就是加了１==)，发布出来再被我们这个程序的subcriber接收，即fCallback接收，"一系列操作"(其实就是减１...)再发布出去，如此循环．
 这里注意到有一个usleep(500000)代表休眠500ms，至关重要．两个节点的通过topic建立起来联系是需要时间的．我们这个程序，如果没有休眠一下，可以说程序一跑起来瞬间消息就发布出去了．但是我们接下来要写的另一个节点和这个节点还没连接起来，说白了就是ROS没反应过来．那这个消息就漏掉了．有些程序漏掉一两个消息没关系，我们这个程序第一个消息漏掉了就没有然后了．因为B程序没有接收到消息就没办法发布，A程序就没法接收，也没法继续发布了．
 **当你一则消息都不想漏掉时让程序睡一会儿，等待节点连接起来再发布消息**．
 把两个代码写进CMakeLists跑起来，注意先跑B程序，如果先跑A程序，因为只休眠了500ms，你A程序iniPub的消息在你慢悠悠地在另一个terminal中跑B程序的时间之前也已经发布出去了并被漏掉了．跑起来后Ｂ可以看到一个程序一直接收１，另一个一直接收０．这时候再开一个terminal，像第一讲一样输入rqt_graph，可以看到下图.

<img src="../images/14634719-5f2a801b2429996e.webp" alt="14634719-5f2a801b2429996e" style="zoom:67%;" />

程序Ａ，即节点talker_listener1中通过topic chatter1to2把消息发送给节点talker_listener2，talker_listener2把消息接收到后，在callback函数中一系列操作再通过topic chatter2to1把消息发布回talker_listener1．可以回看代码中两个程序的subcriber和publisher赋值时他们对应的topic名字与这个图联系起来．




### (六) 使用roslaunch

#### 1）利用roslaunch同时启动数个节点

1:在我们最开始建立的pub_sub_test这个文件夹下建立一个新的文件夹，名字叫`launch`
 2:在launch中建立的一个文件，名字可以随意，后缀必须是launch．我们起名为`pub_int8.launch`
 3:用gedit或者你自己的IDE打开launch文件，输入下面的内容

```xml
<launch>
    <node name="pub_int8" pkg = "pub_sub_test" type = "pub_int8">
    </node>
</launch>
```

roslaunch使用的是xml语言，不过这种语言不需要专门学习了，大家就看这个例子就懂了．launch文件的内容是跑一个node最简单的形式．`<launch>`,`<\launch>`表示launch文件的开始和结束．要开始一个节点，那么内容很简单，第二行，<node ....>表示接下来输入node相关的内容，比如说首先输入的是node的名字，这个东西一般和type后面输入的内容一样，`type`需要被赋值为节点对应的可执行文件的名字，`name`则是节点的名字．具体区别是你在CMakeLists.txt文件里编译文件的命令

```css
add_executable(abc ABC.cpp)
```

中abc就是你需要填写在`type`后的内容，而填写在name后的内容，就是你的节点名字

```cpp
ros::init(argc, argv, "abcde");
```

ros程序中的这一行的`abcde`也是节点名,如果你程序中起的节点名字和launch文件中name后面对应的名字不同，ros会采用name后面的名字作为节点名。当你使用rqt_graph观察时，你看到的代表此程序的名字就是abcde了。一般我们把name和type后都接一样的名字，比如这儿都用的`pub_int8`。但是如果我们有时候需要把一个可执行文件同时作为数个节点运行，我们就需要给他们不同的节点名，即相同的type不同的name。
 即我们在cmakelists中add_executable(A a.cpp b.cpp)的A.　这里就是`pub_int8`，pkg参数被赋值为节点存在于哪个package，我们这儿自然是`pub_sub_test`．`</node>`表示要输入node相关的信息结束．
 其实总的来说，roslauch的最简形式，和我们使用rosrun差不多，指定了那个package和哪个node．接下来就是跑程序了．
 4:打开一个terminal,进入到我们创建的workspace，即之前创建的`catkin_ws`文件夹，和跑rosrun之前一样，我们先source.

```bash
cd catkin_ws
source devel/setup.bash
```

和跑rosrun之前不一样的是，如果我们没有开一个terminal跑`roscore`,运行roslaunch文件后rosmaster会自动启动．当然你关闭了roslaucn之后rosmaster也会关闭．
 5:运行roslaunch．接着在你的terminal中输入

```css
roslaunch pub_sub_test pub_int8.launch
```

格式和使用rosrun时相似，`roslaunch package_name launch_file_name`
 这时候节点就会跑起来开始发布消息了．我们可以用rosrun跑sub_int8的程序来检查一下．
 打开另外一个terminal

```bash
cd catkin_ws
source devel/setup.bash
rosrun pub_sub_test sub_int8
```

这时候可以看到在sub_int8节点中接收到信息．
使用roslaunch时，它默认把来自节点print出来的信息存放到一个log文件里面二不直接print在屏幕上．那么我们如何显示出来呢？output赋值为screen．模仿name, pkg这几个东西的赋值方式，我们只需要把roslaunch文件修改成下面的样子就可以了

```xml
<launch>
    <node name="pub_int8" pkg = "pub_sub_test" type = "pub_int8" output = "screen">
    </node>
</launch>
```

这时候再跑这个launch文件，我们就可以看到terminal中显示了我们发布的消息了．
接下来我们同时启动两个节点．我们同时启动以前写过的pub_int8和pub_string. 其实非常简单,　再相同的位置再写一个叫`double_pub.launch`的文件，在里面输入下面的内容．

```xml
<launch>
    <node name="pub_string" pkg = "pub_sub_test" type = "pub_string" output = "screen">
    </node>
    <node name="pub_int8" pkg = "pub_sub_test" type = "pub_int8" output = "screen">
    </node>
</launch>
```

我们就是把定义pub_int8的launch文件中多加了行类似的pub_string的东西，大家进行简单的比对就能看懂．以同样的方式跑这个launch文件，我们就可以看到两个publisher在同时发布消息．

### 

#### 2）利用roslaunch传递参数

建立一个新的pakcage

```bash
cd catkin_ws/src
catkin_create_pkg read_param_test roscpp rospy std_msgs
cd ..
catkin_make
```

这就建立好一个新的pakcage了．在这个pakcage的src文件夹中我们建立一个名叫show_param.cpp的文件．在文件中输入下面的内容

```cpp
#include "ros/ros.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "show_param");

    ros::NodeHandle nh;

    double noise;
    nh.getParam("noise", noise);

    ROS_INFO("noise parameter is................... %f", noise);
};
```

保存退出后，进入该package的CMakeLists.txt，编译show_param.cpp，在CMakeLists中加入

```bash
add_executable(read_param src/show_param.cpp)
target_link_libraries(read_param ${catkin_LIBRARIES})
```

保存退出后使用catkin_make编译．
 之后在package中创建一个launch文件夹，在launch文件夹中创建一个名叫read_param.launch的文件，写入如下内容。

```xml
<launch>
    <param name = "noise" type = "double" value = "10.0" />
    <node name="read_param" pkg = "read_param_test" type = "read_param" output = "screen">
    </node>
</launch>
```

我们把show_param.cpp和这个launch文件结合起来读一下。
 在show_param.cpp中，到double noise的部分大家应该已经熟悉了，之后的一行是
 `nh.getParam("noise", noise)`nh是之前定义的nodehandle了，getParam为获取参数的函数，函数的参数，第一个是`"noise"`这个noise对应的是你在launch文件里为要传递的参数取的名字，即read_param.launch中`param name`后面跟的那个"noise"；getParam的第二个参数是你在程序中定义的变量的名字，即定义的`double noise`。

可以看出param中的name和程序中的变量名不需要一样，但是在实际使用中，我们为了不让自己搞混，通常param里给变量什么名字在launch文件里就给变量什么名字。程序中定义的变量类型和launch文件中type参数所赋的值保持，都是double. launch文件中通过给value一个double类型的数值给变量赋值。经过上面的操作，launch文件中的10就会传给程序中的noise。

下面我们来跑一下程序,打开terminal，输入

```bash
cd catkin_ws
source devel/setup.bash
roslaunch read_param_test read_param.launch
```

注意一下，通常我们source 之后，输入roslaunch或者rosrun，按空格之后输入package的前几个字母按tap建可以获取完整的package的名字，但是如果这是一个新建的package我们第一次使用时需要手动把整个名字输进去才行。
 程序跑起来后我们就可以看到terminal中有一行

```css
[ INFO] [1550108193.348086703]: noise parameter is................... 10.000000
```

证明参数已经读取成功了。

再传递一下srting类型，向量类型的参数。
 完全一样的方法，我们如下修改launch file

```xml
<launch>
    <param name = "noise" type = "double" value = "10.0" />
    <param name = "string_var" type = "string" value = "abc" />
    <node name="read_param" pkg = "read_param_test" type = "read_param" output = "screen">
    </node>
</launch>
```

可以看到只是添加了一行代码，定义变量名字叫`string_var`，值为`abc`，类型为`string`。自然要在程序中读取这个参数你就需要在代码中定义一个string类型的变量，用nh.getParam读取，这儿不再赘述。

如果我们想传递一个数组呢？其实是不能直接通过<param...>这种形式直接添加一个数组的。我们可以参考官网
 [http://wiki.ros.org/roslaunch/XML/param](https://links.jianshu.com/go?to=http%3A%2F%2Fwiki.ros.org%2Froslaunch%2FXML%2Fparam)
 在Attributes, type那一行可以看到type一共有

```swift
"str|int|double|bool|yaml"(optional)
```

五种形式，其中并不包含数组。
 要从外部传递数组的话，需要使用rosparam，简单来讲，把launch添加一行

```xml
<launch>
    <param name = "noise" type = "double" value = "10.0" />
    <param name = "string_var" type = "string" value = "abc" />
    <rosparam param="a_list" >[1, 2, 3, 4]</rosparam>
    <node name="read_param" pkg = "read_param_test" type = "read_param" output = "screen">
    </node>
</launch>
```

在程序中添加两行

```cpp
std::vector<int> a_list;
nh.getParam("a_list",a_list)
```

即可把[1,2,3,4]这个vector传递到程序中的a_list中。当我们遇到参数很多的程序时，我们需要使用yaml文件。

什么是yaml文件呢？yaml文件是使用一种特定结构来储存外部参数的文件。先不说我们要传递数组什么的，设想你有一个大型的程序，需要从外部读取数十个上百个参数，你把这些参数全部用<param...>形式写到launch文件中会让launch文件显得特别冗长，使用起来并不友好。而有了yaml文件的帮助，我们可以把所有要读取的参数全部写到yaml文件中去，然后launch文件中只需要一行代码读取yaml文件即可。

yaml文件内容的固定格式是

```undefined
变量名: 变量值
```

在`read_param_test`这个pakcage中创建一个叫config的文件夹，在该文件夹里面创建一个叫`read_param_test.yaml`的文件，打开并输入下面的内容。

```css
noise: 10.0
string_var: abc
vector_var: [1,2,3]
```

前两个变量是我们之前在launch文件中通过写<param...>传递的参数，现在试着通过yaml文件传递一下参数。至于第三个一看就是个向量(数组)了。
 注意在yaml文件中我们不需要指定变量类型什么的了，`变量名: 变量值`是它的固定格式，变量值包含非数字的量它会自动认为这是string类型变量，纯数字的值，如果不包含小数则会认为是int类型的变量，包含小数则是double类型的变量。
 接着写一个新的launch文件来读取这个yaml文件，进而能把yaml文件的值传递到程序中去。我们在之前建立的launch文件夹中新建一个launch文件，叫`read_param_from_yaml.launch`。写入下面的内容

```xml
<launch>
    <rosparam file="$(find read_param_test)/config/read_param_test.yaml" command="load" />  
    <node name="read_param" pkg = "read_param_test" type = "read_param" output = "screen">
    </node>
</launch>
```

可以看到launch文件的开头和结尾，<node...>那一行和之前写的读取参数的launch文件相比没有变化，变化的是我们有了一行<rosparam...>来读取yaml文件。
 rosparam后面接的第一个参数`file`用来指定yaml文件的地址，其中`$(find read_param_test)`表示read_param_test这个package的路径，之后建立的config文件夹，再接刚刚创建的read_param_test.yaml。第二个参数表示需要加载即读取yaml文件中的参数。
 接下来就是我们的读取程序了。其实如果不需要读取那个向量的话其实程序不需要修改，不过这儿要读取向量我还是把程序写出来吧。

```cpp
#include "ros/ros.h"
#include <string>
#include <vector>

int main(int argc, char **argv){

    ros::init(argc, argv, "show_param");

    ros::NodeHandle nh;

    double noise;
    if(nh.getParam("noise", noise))
        ROS_INFO("noise is %f", noise);
    else
        ROS_WARN("didn't find parameter noise");

    std::string string_var;
    if (nh.getParam("string_var", string_var))
        ROS_INFO("string_var: %s", string_var.c_str());
    else
        ROS_WARN("No string_var name message");

    std::vector<int> a_list;
    if (nh.getParam("a_list",a_list))
        ROS_INFO("get a_list");
    else
        ROS_WARN("didn't find a_list");
    
    std::vector<int> vector_var;
    if (nh.getParam("vector_var",vector_var))
        ROS_INFO("got vector");
    else
        ROS_WARN("didn't find vector");
};
```

把寻找参数的代码稍微修改了一下。人们在写yaml文件或者程序中的变量时常常手误，二者名字匹配不上还觉得自己写对了呀。这时候来个类似于

```bash
    if(nh.getParam("noise", noise))
        ROS_INFO("noise is %f", noise);
    else
        ROS_WARN("didn't find parameter noise");
```

这几行代码会让人很愉快。nh.getParam是有返回值的，如果找到了参数，则返回true，没有找到则返回false，基于这一点，我们可以在找不到时利用ROS_WARN输出信息，如果找到了变量，则输入它的值之类的，让我们能更方便地debug。
 好了yaml文件,launch文件，程序都弄好了，接下来使用catkin_make编译好程序后。使用

```css
roslaunch read_param_test read_param_from_yaml.launch
```

跑程序，你就应该能看到想要的结果了。




### (七) 使用rosbag

#### 1）使用rosbag采集数据

**rosbag在采集数据时自身相当于一个subscriber程序**

然后在terminal中，写入下面内容

```undefined
rosbag record /chatter -O record_poseStamped 
```

这行命令第一个是表示我们要使用rosbag相关命令了，第二个record表示我们想要采集(记录)数据。
 第三个，由于我们之前说了，rosbag在采集数据时自身相当于一个subscirber程序，既然要subscribe，那么我们肯定需要指定subscriber哪个topic了，观察我们在pub_poseStamped.cpp中，定义topic那一行我们定义的topic的名字是chatter，于是我们就在第三个参数写入`/chatter`表示记录这个topic发布出来的信息。rosbag可以同时记录多个topic的信息，你只需要在record后接`/topicA /topicB /topicC....`即可。
 命令行中-O及其后面的参数，是用来给bag文件命名的。我们在记录结束之后，rosbag的名字就会是`record_poseStamped.bag`，(如果参数是-o，那么记录名字会自动加上年月日时间，比如上面的例子如果用`rosbag record /chatter -o record_poseStamped`，输出会是`record_poseStamped-2019-09-08-15-24-05.bag`)。按下确认键后，你会看到下面的消息

```csharp
[ INFO] [1551144307.272157111]: Subscribing to /chatter
[ INFO] [1551144307.274766644]: Recording to record_poseStamped.bag.
```

表示已经在开始记录chatter发布的消息了，消息会存储到record_poseStamped.bag里。如果topic没有消息发布，那么自然不会有任何东西会储存到这个rosbag文件里，这个文件会一直处于检测是否有消息发布的状态。你如果去看dataset文件夹里的图标，会发现一个叫record_poseStamped.bag.active的文件，表示这个bag文件目前正在处于记录的状态。

打开另一个terminal，输入下面的命令

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun pub_sub_test pub_poseStamped
```

把pub_poseStamped跑起来。你应该看到print出很多信息，这时候你观察跑rosbag那个terminal，什么也没有输出。 你可能会担心，到底是不是在记录发布出来的消息。rosbag里的topic名字和你publisher的名字是在对应上的，就一定在记录。极少数情况rosbag会拒绝记录新的消息，比如你的磁盘空间小于一个G了，rosbag会自动停止记录，如果你的磁盘空间小于5个G了，会出现warning但是不影响记录。
有些时候写程序的人并不会把publisher发布的消息print出来，你如果有成千上万条消息发布都print出来确实很烦。这时候你publisher的terminal中没什么信息print出来，rosbag的terminal中没什么信息print出来，你可能慌得一匹，不知道消息有没有被发布出来。你可以用下面的方式检查一下你的topic有没有发布消息。
打开另一个terminal，输入

```bash
rostopic echo /chatter
```

这行命令会把你chatter这个topic发布的命令print到terminal中。按下确认之后你会安心地看到下面的内容。

<img src="../images/14634719-5cafb764b54e38de.webp" alt="14634719-5cafb764b54e38de" style="zoom:67%;" />

上面那行命令其实也相当于你跑了一个subscriber并且把收到的消息print出来。并且格式固定。可以看到每一则消息由`---`分开。然后显示出你消息的header。同时print出来了pose的内容，即position和orientation。
 当你觉得记录到足够的数据之后，点击rosbag在运行的那个terminal并按下`ctrl+c`，rosbag就会停止记录并被保存下来了。每次记录完数据，我们都要马上检查一下是否记录成功。这时候我们在terminal中使用下面的命令可以看到rosbag的信息

```css
rosbag info record_poseStamped.bag
```

运行这行命令，我们可以看到下面的信息。

![14634719-9f30811492103552](../images/14634719-9f30811492103552.webp)

其中我们可以看到这个bag文件记录了多少消息，记录的是哪个topic的消息，消息的类型是什么等。读者自行研究了。
至此利用rosbag采集(记录)数据工作就完成了。

#### 2）重新使用储存在rosbag内的数据

当你采集完成数据之后，要拿回去分析，这时你就需要重新把储存在bag里的消息发布出来。
 首先确保roscore已经运行，cd到你rosbag所在的文件夹，然后执行下面的命令



```css
rosbag play record_poseStamped.bag
```

这时你会看到下面的内容

<img src="../images/14634719-60639e1de30b39bb.webp" alt="14634719-60639e1de30b39bb" style="zoom:67%;" />

**当你重新使用rosbag的数据(即使用rosbag play bag_name.bag)的时候，你的rosbag自身相当于一个publisher程序**。
 你的消息会在rosbag记录的topic的名字下重新发布出来。如果你打开一个terminal，重新运行我们之前的sub_poseStamped.cpp对用的那个node，即在新的terminal中source之后运行

```undefined
rosrun pub_sub_test sub_poseStamped
```

你应该会看到运行pub和sub程序时一样的内容。只不过这时候我们的rosbag取代了那个pub程序.



### (八) 使用rviz进行可视化

#### 1）rviz 安装

终端运行:

rosdep update

rosdep install rviz

rosmake rviz


运行rviz：

在一个控制台中运行

roscore

在另一个控制台中运行

rosrun rviz rviz

注意:如果不事先运行roscore的话就会出现could not contact ROS master的错误


#### 2）rviz操作

```css
osbag play record_poseStamped.bag
```

有了publisher，我们自然就需要一个subscriber。虽然我们现在还不知道rviz怎么用，但是我们大概能猜到在rviz中我们需要定义一个subsriber，来接收那个rosbag发布出的消息。接下来我们打开rviz。
 首先跑roscore，之后打开另一个terminal，在里面输入

```undefined
rosrun rviz rviz
```



<img src="../images/14634719-c847caab458e3c15.webp" alt="14634719-c847caab458e3c15" style="zoom:67%;" />

1：点击界面左下方的`Add`
 出现下面窗口

<img src="../images/14634719-0e1545c9c61b1204.webp" alt="14634719-0e1545c9c61b1204" style="zoom:67%;" />


 2：点击里面的`Marker`，然后选择OK。这时候我们会发现在rivz界面左边的Displays那个大框框中，Grid下出现了一个新的东西，名字就是我们刚刚选择的Marker。

![14634719-28ed59885abe9f2e](../images/14634719-28ed59885abe9f2e.webp)


 Marker旁边有一个小三角，是Marker包含的下拉列表，点一下，就会出来和我这张图一样的东西了。在下拉列表里我们看到了一个很熟悉的东西`... Topic`。此时，**这个Marker就已经是一个subscriber了，并且正在试图接收消息**，它目前接收的是来自`visualization_marker`这个topic的消息。下面还有Queue_Size，也是我们以前定义一个subscriber时所熟悉的内容。Namespace可以暂且不管。
 那么marker到底是什么呢？和它的名字一样，它就是一个用来标记的东西。如果我们给定marker一个position和orientation（位置和姿态），那么rviz就会在中间画图区域的指定位置生成一个指定方向的marker，这个marker可以是立方体，箭头等我们可以在程序中自行选择。如何给marker一个orientation和position那自然就是我们发布消息，marker接收消息，消息里面包含了marker的位置和姿态。
下面我们就可以试试写程序，把我们之前的rosbag中记录的position和orientation发布给marker。这样rviz就能在画出每一时刻指定的position和orientation看起来什么样。我们写一个接收rosbag里posetamped的程序，并把posestamp转化为marker能接收的消息种类再发布出来，让marker接收。
 我们创建一个新的package来学习rviz

```bash
cd ~/catkin_ws/src
catkin_create_pkg learn_rviz_tf roscpp rospy std_msgs geometry_msgs visualization_msgs tf
cd ..
catkin_make
```

创建的package名称叫`learn_rviz_tf`，因为下一讲我们会结合rviz一起学习tf，所以就干脆在一个package里写程序了。 他的依赖项除了我们之前常用的外多了visualization_msgs，这里面会包含marker类型的消息;多了tf，tf是我们下一讲的内容。
 接下来在learn_rviz_tf的src文件夹创建一个pub_marker_msgs.cpp的文件并在里面输入下面的代码。

```cpp
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

class MarkerPublisher{
public:
    MarkerPublisher(ros::NodeHandle& nh){
        pub_marker_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);//initialize marker publisher
        set_marker_fixed_property();
    };
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        count_++;
        marker_.ns = "my_namespace";
        marker_.id = count_;
        
        marker_.header.stamp = ros::Time();
        marker_.pose = msg->pose;

        pub_marker_.publish(marker_);
    };
    void set_marker_fixed_property(){
        /*decide from which view we can see the marker*/
        marker_.header.frame_id = "my_frame";
        /*set marker type*/
        marker_.type = visualization_msgs::Marker::SPHERE;
        /*decide if the marker will be enlarge*/
        marker_.scale.x = 1;
        marker_.scale.y = 0.1;
        marker_.scale.z = 0.1;
        /*decide the color of the marker*/
        marker_.color.a = 1.0; // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;
        /*set marker action*/
        marker_.action = visualization_msgs::Marker::ADD;
    };

private:
    ros::Publisher  pub_marker_;
    visualization_msgs::Marker marker_;
    int count_ = 0;
};




int main(int argc, char **argv){

    ros::init(argc, argv, "marker_worker");

    ros::NodeHandle n;

    MarkerPublisher mp(n);

    ros::Subscriber sub_pose   = n.subscribe("chatter", 100, &MarkerPublisher::PoseCallback, &mp);

    ros::spin();
}
```

主函数的前面两行大家都很熟悉了。
 第三行我们定义了MarkerPublisher的对象mp并把nodehandle传入，MarkerPublisher这个类是我们建立来发布Marker这种类型的消息的。
 第四行我们定义了接收chatter这个topic的接收器，由于callback函数定义在了MarkerPublisher这个类里，所以注意一下定义的方式。这个接收器以及接收函数是用来接收我们rosbag里topic名字为chatter，消息类型为posestamped的消息，所以注意topic的名字和callback函数里参数要和rosbag里一一对应起来。
 在类中发布消息的方式我们在第五讲讲过，不熟悉可以再去回顾一下。
 spin函数检测是否有消息发布。

接下来看MarkerPublisher这个类。
 首先我们已经将`visualization_msgs::Marker`类型的消息`marker`定义为私有成员了。这样我们不用每次发布消息时都重新定义一个marker并且为它的各种性质重新赋值。
 类的构造函数中我们定义了pub_marker的具体内容，它将要发布topic名为`visualization_marker`，消息类型为`visualization_msgs::Marker`的消息。首先topic的名字，是必须和rviz中我们设置的marker的topic名字对应(上一张图里我们可以看到 Marker Topic的名字是visualization_marker)。这个消息类型具体怎么赋值呢？我们第三讲讲过查自己需要的消息类型的方法，我们在搜索引擎中输入类似于visualization_msgs, marker, ros这类的东西，就可以看到下面的网页
 http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html
 点进去你会看到这个消息包含的成员变量太多了，头都大了，怎么用不是很清楚。还好对于这种稍微复杂类型的消息，ROS一般有例子可寻找，其实你稍微花点儿功夫就可以找到下面的网页
 http://wiki.ros.org/rviz/DisplayTypes/Marker
 在网页中exmaple_usage里我们可以看到它有示例代码。我上面的代码也是根据它的内容来的。下面回到我的代码，在构造函数的第二行，我们创建了一个叫`set_marker_fixed_property()`的函数，这个函数我们设置了一些要发布的marker的一些我们不想改变的性质。比如该函数中的

```bash
marker_.header.frame_id = "my_frame"
```

这行的作用是指定我们在rviz中从哪个坐标系去观看我们的marker。其实道理很简单，大家知道机器人一般有很多参考坐标系，什么世界坐标系，相机坐标系或者其他传感器坐标系，世界坐标系一般固定不动，其他的可能随机器人移动。我们上面定义这个`my_frame`，表示我们希望从`my_frame`这个坐标系里去观察marker，而之后，可以想象，我们需要在rviz中把世界坐标系的名字设置为`my_frame`。二者能对应上，我们就能从一个固定不变的坐标系中观察我们的marker了。
 接下来是

```php
 marker_.type = visualization_msgs::Marker::SPHERE;
```

从字面意思不难理解是把我们marker的样子定义为球形。能定义成哪些形状大家可以看上面参考网页中的资料
 这个球得有多大呢，得设置它的scale。所以接下来是

```undefined
marker_.scale.x = 1;
marker_.scale.y = 0.1;
marker_.scale.z = 0.1;
```

设置marker的scale，很好理解，marker自身在x,y,z方向上的缩放系数。假设我们把marker的样子设置为一个球，x方向scalr为1，其他为0.1，那么我们应该看到的是一个椭球。
 接下来是定义marker这个球的颜色

```php
marker_.color.a = 1.0; // Don't forget to set the alpha!
marker_.color.r = 0.0;
marker_.color.g = 1.0;
marker_.color.b = 0.0;
```

第一个`marker_.color.a`要设置为1,marker才看得见(真是奇怪的性质hhhh)，后面三个分别是red,green,blue，颜色由0到1由浅变深，不必细说，如上设置的话marker就是绿色。
 接下来是

```php
marker_.action = visualization_msgs::Marker::ADD;
```

这行表示我们接收到marker的信息了之后是增加相应的marker，你也可以定义删除相应的marker。

上面这些性质我并不想每次更新marker的位置时都设置，所以放到一个函数里了。
 接下来我们看callback函数`void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)`
 函数的第一行是`count_++`，其中count_是一个私有成员，可以看到count_在第三行赋值给了`frame_id`，这是什么意思呢？每一个rviz中出现的marker都有自己特定的id和namespace(即我们第二行给marker_ns赋值的内容)，不同的id或者namespace都代表着不同的marker。第三行代码意味着每一次接收函数接收到一条新的消息，marker的id都会改变。那么每一次接收到新的消息都会有一个新的marker诞生。
 可以想象，因为每一次接收到消息都会给marker赋值新的来自于posestamp消息的pose，如果这些pose的id都不同，我们就会看到那个椭球形的marker在不断增加，如果id和namespace都相同，那么我们会只看到一个marker,那个marker仿佛在移动。
 callback函数第四行给maker自身包含的timestamp赋值。在这个例子里不是很重要。
 第五行就是关键了，我们接收到的pose赋值给marker。为什么能这么直接赋值呢？打开我们的第一个网页链接我们可以看到marker是包含了类型为`geometry_msgs/Pose`的成员`pose`的。而posetamped类型的消息包含的是`pose`和`header`，所以我们可以直接进行pose的赋值，这样也就定义了marker的位置和方向。
 接下来我们只要发布marker_这个消息，rviz就应该能接收了。
 等一下，貌似忘了什么。刚才说了我们需要把rviz中世界坐标系的名字和marker的frame_id对应起来。这样我们才能从一个不动的上帝视角看marker。具体的做法是你的rviz中，找到Global Frame下面的`Fixed Frame`，这个就是世界坐标系。目前它的名字叫`map`，我们点击那个map，把它改成`my_frame`。现在就万事俱备了。

把cpp文件写到CMakeLists里

```bash
add_executable(pub_marker_msgs src/pub_marker_msgs.cpp)
target_link_libraries(pub_marker_msgs ${catkin_LIBRARIES})
```

编译好后打开一个terminal，source之后输入

```undefined
rosrun learn_rviz_tf pub_marker_msgs
```

开始接收来自topic`chatter`的`geometry_msgs::PoseStamped`类型的消息。接收到后发布类型为`visualization_msgs::Marker`，topic为`visualization_marker`的消息。
 之后再打开一个terminal,cd到rosbag的文件夹跑我们的rosbag，发布topic为`chatter`的消息

```css
rosbag play record_poseStamped.bag
```

我们就应该能在rviz中看到类似下面的东西。

![14634719-31c72a7d89bf5abd](../images/14634719-31c72a7d89bf5abd.webp)




### (九) 使用tf追踪不同坐标系

#### 1）tf2基础例子

ROS官网例子链接如下
 [http://wiki.ros.org/tf2/Tutorials](https://links.jianshu.com/go?to=http%3A%2F%2Fwiki.ros.org%2Ftf2%2FTutorials)
 我们使用第二个 [Writing a tf2 broadcaster (C++)](https://links.jianshu.com/go?to=http%3A%2F%2Fwiki.ros.org%2Ftf2%2FTutorials%2FWriting%20a%20tf2%20broadcaster%20%28C%2B%2B%29)来讲解。
 进入链接后我们直接从2.1，the code部分开始，在我们的learn_rviz_tf/src中创建`turtle_tf2_broadcaster.cpp`并把代码复制进去。这儿为了方便讲解我也复制过来了

```cpp
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("turtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
  }
  else
  {
    private_node.getParam("turtle", turtle_name);
  }
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
```

官网的代码解读不甚详细，我赘述一遍并加点东西。
 首先坐标系之间转换的发布不再是定义一个publisher然后用publisher.publish()函数来发布了，而是在`tf2_ros::TransformBroadcaster`这个类下定义一个专门的发布坐标转换的对象，使用`对象.sendTransform(msg_type)`来发布坐标之间的关系。发布的msg_type是特定的消息类型`geometry_msgs/TransformStamped.h`。基于这一点我们来看代码。
 1:
 头文件部分包含了`#include <tf2/LinearMath/Quaternion.h>`，代码有一行定义了tf::quaternion的对象`tf::Quaternion q`需要通过包含这个头文件来实现。
 回想我们在使用geometry_msgs/PoseStamped时曾需要定义消息的pose，而pose包含position和orientation，其中orientation的消息类型是geometry/Quaternion，也就是说我们也可以通过`geometry::Quaternion q`定义一个四元数对象q，这也是四元数。这两种四元数的定义的对象是有很大的区别的。后者只能通过q.w, q.x, q.y, q.z调用四元数包含的数据。

我们上面的代码中使用了这个函数，`q.setRPY(0, 0, msg->theta)`，这个函数完成了从roll pitch yaw到四元数的转换功能，通过设定roll pitch yaw来设定四元数的具体数据。tf::Quaternion定义的对象q获取四元数的具体方法是q.x(),q.y()...有别于geometry::Quaternion定义的q通过q.x,q.y...的获取方式。x(),y()..这几个函数我们并没有在上面的链接看到，原因是tf::Quaternion继承了QuadWord这个类的，在后者的定义中我们能看到能通过x()，y()这类型的函数来获取quaternion的具体数据。
 这意味着，利用tf定义的四元数是重载了计算符号的，即你如果有如下定义

```php
tf::Quaternion q_AB //A坐标到B坐标的的旋转变换
tf::Quaternion q_BC //B坐标到C坐标的旋转变换
```

那么你可以通过

```undefined
q_AB * q_BC
```

来获取到A坐标到C坐标的旋转变换。geometry_msgs定义的四元数是不具备上面的任何功能的，仅仅是储存数据并用来发布或者接收。
 这里啰嗦了这么多，主要是希望大家通过例子能学会自己找到相关资料，这是最重要的。比如以后使用了tf::Transform定义了对象tf1，那么tf1是什么东西，能包含哪些功能，有哪些函数可以调用？你在google上搜寻tf::Transform进入 [ tf: tf::Transform Class Reference](https://links.jianshu.com/go?to=http%3A%2F%2Fdocs.ros.org%2Fjade%2Fapi%2Ftf%2Fhtml%2Fc%2B%2B%2Fclasstf_1_1Transform.html)就自然可以查到。
 2：
 头文件包含了`<tf2_ros/transform_broadcaster.h>`。包含了这个头文件，就可以用代码中的`tf2_ros::TransformBroadcaster br`来定义一个发布坐标转换的对象了。
 3：
 头文件包含了`<geometry_msgs/TransformStamped.h>`，即我们之前说的br发布的内容需要是geometry_msgs命名空间下，TransformStamped类型的消息。可以看到要发布的消息类型在代码中这么定义的
 `geometry_msgs::TransformStamped transformStamped;`
 另外在代码中有一行`br.sendTransform(transformStamped)`就是用来发布坐标转换的消息的代码。
 总的来说，这和我们一般定义publisher并发布消息的过程很像

```php
ros::Publisher pub_abc = .... //定义publisher
std_msgs::Float64 abc; //定义要发布的消息类型
abc.data = .....; //给要发布的消息赋值 
pub_abc.publish(abc);//发布消息
```

只不过到发布坐标转换这儿变成了

```cpp
tf2_ros::TransformBroadcaster br; //定义坐标转换的publisher
geometry_msgs::TransformStamped transformStamped; //定义坐标转换要发布的消息类型(只能是此类型)
...//赋值，赋值部分是最上面代码中位于poseCallback()函数中transformedStamed.header.stamp = ...以及后面的是来行的内容，稍后再讲
br.sendTransform(transformStamped);//发布坐标转换
```

可以看到发布坐标变换的整体过程和发布普通消息是类似的，只不过使用的函数，消息名字有些变化而已。
 4：
 头文件包含了`#include <turtlesim/Pose.h>`. ROS真是很喜欢用turtlesim这个package的内容来讲解了，具体里面包含的内容我也没去看过。不过相信大家在自己学习ROS官网的教程时都使用过这个package，打开过一个窗口看到两个可爱的小乌龟了。
 我们的poseCallback函数的形参是这样的`(const turtlesim::PoseConstPtr& msg)`，对比我们写过很多的callback函数的形参比如接收flaot类型消息的`(const std_msgs::Float64::ConstPtr& msg)`
 我们就能猜到`#include <turtlesim/Pose.h>`是包含了消息类型Pose，这个Pose专门用来定义小乌龟的位置的。(定义ConstPtr时前面居然没有作用域符号::，我也吃了一惊，不过试了一下确实有没有都可以Em...)。
 5: `std::string turtle_name` 用来接收后面要写的launch文件传递进来乌龟的名字。如何使用launch文件传递参数参考使用roslaunch那一节。
 6: void poseCallback(const turtlesim::PoseConstPtr& msg), 这个回调函数对应的subscriber我们在main函数中可以看到。`ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);`也就是说它subscribe的topic是turtle_name+"/pose".一会儿我们运行程序我们就能看到小乌龟，小乌龟的位置会时刻发布出来，这个subscriber就会接收
 回调函数的内容，一二行我们已经讲过，第一行用了static定义br，主要是避免重复定义。就像定义publisher我们肯定也只定义一次。遇到这种情况其实我们最好把回调函数写在类中，把br定义为类成员(见在类中publish和subscribr那一讲)。
 回调函数的第三行开始给我们要发布的坐标转换赋值。关于transformStamped包含哪些成员，怎么使用，赋值等，和我们之前在第三讲过的poseStamped类似。
 `transformStamped.header.stamp = ros::Time::now();`坐标之间的关系可能变化，那么自然我们在定义坐标之间的关系时，自然要给它赋值在什么时刻，坐标之间的关系如何
 `transformStamped.header.frame_id = "world"` 和 `transformStamped.child_frame_id = turtle_name;`两行，既然涉及到两个坐标之间的关系，我们肯定需要知道两个坐标系的名字，所以world那一行定义了parent坐标系的名字，turtle_name那一行定义了child坐标系的名字。
 那么具体两个坐标之间的关系是怎么样的呢？自然就涉及到给坐标系之间的rotation和translation赋值。我们说回调函数接收到的是turtle的位置和方向，那么msg中的位置和方向我们就需要赋值transformStamped。即下面几行。

```php
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
...
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
```

中间省略的两行

```css
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
```

主要是换个方式表达rotation。后面我们可以看到由于乌龟是二维运动，所以msg涉及到的只有x,y和theta. theta即三维坐标中的yaw角度，这就是为什么利用`q.setRPY(0, 0, msg->theta);`定义了quaternion具体的内容。之后把tf的quaternon的值赋值给我们要发布的消息类型transformStamped所包含的quaternion的值就可以了。
 当transformed的内容都设置好了后`br.sendTransform(transformStamped);`发布坐标转换。

5：主函数中，`ros::NodeHandle private_node("~")`和我们平时定义node的方式不是很一样。平时我们定义node都是直接`ros::NodeHandle nh`。这里多了一个参数`～`。它表示此时这个nodehandle是读取局部参数的。具体可见这篇文章[https://blog.csdn.net/shijinqiao/article/details/50450844](https://links.jianshu.com/go?to=https%3A%2F%2Fblog.csdn.net%2Fshijinqiao%2Farticle%2Fdetails%2F50450844)。总体作用和使用方式和普通node没有大的区别。:
 6：main函数的if else语句就是要从launch文件中读取turtle_name那个参数。
 再之后定义获取乌龟pose的subscriber。为什么有两个nodehandle呢，我们在launch文件中解释。
 把这个文件添加到CMakeLists.txt编译。
 之后我们在learn_rviz_tf目录下添加一个launch文件夹，创建一个start_demo.launch。写入下面的内容。

```xml
  <launch>
     <!-- Turtlesim Node-->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    <!-- Axes -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <node pkg="learn_rviz_tf" type="turtle_tf2_broadcaster"
          args="/turtle1" name="turtle1_tf2_broadcaster" />
    <node pkg="learn_rviz_tf" type="turtle_tf2_broadcaster"
          args="/turtle2" name="turtle2_tf2_broadcaster" />

  </launch>
```

注意这儿和原tutorial的launch文件有些小出入。原launch文件第三个和第四个node的后面pkg的名字是`learning_tf2`，因为他们创建了一个叫learningtf2的package来做tf的tutorial，而我们的现在代码是在learning_rviz_tf这个pakcage里，所以package名字改一下，不是大问题。
 写好roslauch编译好文件后我们先来运行一下程序看看什么效果。

```css
roslaunch learn_rviz_tf start_demo.launch
```

看到下图一个小乌龟出来了。

![14634719-a43adab19cee2c99](../images/14634719-a43adab19cee2c99.webp)

重新点击跑roslaunch的终端，你的小乌龟会移动。打开另一个terminal，在其中输入

```undefined
rosrun tf tf_echo /world /turtle1
```

这时候你会看到类似于下面的内容

<img src="../images/14634719-1b1f992c8f4d5511.webp" alt="14634719-1b1f992c8f4d5511" style="zoom:67%;" />

每一个

```undefined
At time ...
- Transolation ..
- Rotation ...
```

是一组tf，显示的是在某个时间点,world坐标系和turtle坐标系之间的关系。当你移动小乌龟时，translation和rotation会发生改变。
 下面解释launch文件中的内容
 1：

```bash
<node pkg="turtlesim" type="turtlesim_node" name="sim"/>
```

这个节点是显示乌龟这个GUI的，不用深究。
 2：

```bash
<node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
```

这是为了使你的的键盘能控制乌龟的移动。(从key那个名字就可以看出)不用深究。
 3：
 param两行的参数目前在我们程序中没有使用。
 4：



```go
<node ... args = `/turtle1`...>
```

即运行刚刚写的程序
 5：
 下面一个node暂时没有使用。官网关于tf2的tutorial的另一个例子会使用。
 好像并没有解释什么...
 最后回到程序中关于praivate_node的问题，为什么多用一个nodeHandle?其实我们在launch文件中可以发现



```bash
    <node pkg="learn_rviz_tf" type="turtle_tf2_broadcaster"
          args="/turtle1" name="turtle1_tf2_broadcaster" />
    <node pkg="learn_rviz_tf" type="turtle_tf2_broadcaster"
          args="/turtle2" name="turtle2_tf2_broadcaster" />
```

这代表这两个node其实使用的是一个可执行文件，即`turtle_tf2_broadcast`，为了区分他们，给了他们不同的名字`turtle1_tf2_broadcaster`，`turtle2_tf2_broadcaster`，即同样的执行文件不同的**节点
 **名。同时这两个节点的传入args不同。
 当好几个节点都来自于一个可执行文件时，我们难免遇到同一个问题。如果他们都需要传入某个名叫A的参数，但是A的值需要不一样，该怎么办呢？这时候我们通常要把要读取的参数分开写到节点内。如下



```xml
<param name="A" value="11" />
  <node name="you" pkg="ABC" type="abc" output="screen">
    <param name="A" value="10" />
  </node>
  <node name="me" pkg="ABC" type="abc" output="screen">
    <param name="A" value="9" />
  </node>
```

**在<node>...</node>内部定义的参数称为局部参数，必须定义一个私有nodehandle, 即`ros::NodeHandle abc("~")`并使用代码abc.getParam(...)才能读取**
 例如上面为了让节点you和me分别读取值为10的A和值为9的A，我们需要在程序内定义带参数的"~"的nodeHandle。表示读取自己内部的参数，同时不要读取值为11的全局参数A。
 但其实上面的launch文件没有重复名字的参数(args不算)，理论上可以不用局部的nodehandle。我把("~")去掉了也一切正常。不过大家以后要在一个launch文件中使用相同名字但值不同的参数时，一定要考虑这个问题。
 所以总的来说，我们原来的程序只定义一个node并且不添加参数("~")也没有问题，由于没用args为turtle2的节点，所以把args = "turtle2"那一个节点删掉了也没有问题，本来可以大大做简化，但是考虑到很多同学还是按照官网的例子学习，所以就冗杂地解释了一番。
 在程序运行时，我们在使用rviz来可视化tf。打开一个新的terminal，在其中输入



```undefined
rosrun rviz rivz
```

上一章节我们为了可视化pose通过ADD按钮添加了marker，现在类似，rviz打开后点击左下方的ADD按钮
 ，在显示出的对话框中选择`TF`，由于我们发布的TF是关于world和turtle1的，我们希望`world`坐标系是固定的，所以在rviz中把`Fixed Frame`改成`world`，这时候你刚才roslaunch跑的程序所发布的坐标系之间的转换就正在被rviz接收了。你会看到类似下图的东西

![14634719-d46c2f507816ad05](../images/14634719-d46c2f507816ad05.webp)


 rviz很清晰地显示了两个坐标系之间的联系。在你跑roslaunch的terminal中移动小乌龟，你会看到turtle1这个坐标系也在移动。
 同样类似于你可以用rqt_graph命令来观察publisher和subscriber的关系，你可以在terminal中用下面的命令观察坐标系之间的关系





```undefined
rosrun rqt_tf_tree rqt_tf_tree
```

输入该命令后会出现类似于下面的图像

![14634719-21a2f8ad106a4ba8](../images/14634719-21a2f8ad106a4ba8.webp)

这表示world是母坐标系,turtle1是子坐标系。

#### 2）tf2多个坐标系追踪

讲完了基础例子，我们就可以来实现我第一张动图moving_robot.gif。在那个动图中，我们有三个坐标系，camera和gps相对静止，和我们的机器人(方块marker)一块儿相对于世界坐标系world移动。官网的例子很不错，不过turtlesim之类的东西内部是咋样的我们毕竟不是那么了解，还有局部nodeHandle，几个例子里还使用了service之类，虽然都不是复杂东西，但和在一起总会有人不清楚其中的部分。我还是倾向于大家能懂tutorial的例子的每一行代码，代码的内容我们在之前都有所接触。
 我们在learn_rviz_tf/src中创建一个叫`moving_coordinate_system.cpp`的文件。写入下面内容

```php
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>

class MovingObject{
public:
    MovingObject(ros::NodeHandle& nh);
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void set_marker_fixed_property();
private:
    ros::Publisher pub_object_;
    visualization_msgs::Marker mk_;
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "tf2_broadcaster");

    ros::NodeHandle nh;

    MovingObject mo(nh);

    ros::Subscriber sub_gps = nh.subscribe("/chatter", 10, &MovingObject::PoseCallback, &mo);
  
    ros::spin();
}

MovingObject::MovingObject(ros::NodeHandle& nh){
    pub_object_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    set_marker_fixed_property();
}

void MovingObject::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("get gps position, %f, %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "gps";
    transformStamped.transform.translation.x = msg->pose.position.x - 0.1;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;
    transformStamped.transform.rotation.w = msg->pose.orientation.w;
    transformStamped.transform.rotation.x = msg->pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.orientation.z;
    br_.sendTransform(transformStamped);

    mk_.header.stamp = ros::Time();
    mk_.pose = msg->pose;
    pub_object_.publish(mk_);
}

void MovingObject::set_marker_fixed_property(){
    mk_.ns = "my_namespace";
    mk_.id = 0;
    mk_.header.frame_id = "world";
    mk_.type = visualization_msgs::Marker::CUBE;

    mk_.scale.x = 0.5;
    mk_.scale.y = 0.5;
    mk_.scale.z = 0.5;

    mk_.color.a = 0.3; 
    mk_.color.r = 0.0;
    mk_.color.g = 1.0;
    mk_.color.b = 0.0;

    mk_.action = visualization_msgs::Marker::ADD;
}
```

头文件除了第一个基础例子的以外，增加了

```cpp
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
```

我们依次讲解程序内容
 1:我们在上一章可视化marker的时候写了一个subscriber文件，用来接收rosbag发出的机器人pose的信息，并转换为marker的pose使之能在rviz中显示出来。而我们现在的例子同样显示marker，marker的pose后面可以看到同样来自于rosbag发出的消息。所以包含了这两个上一章节使用过的头文件，一个用来使用PoseStamped消息，一个用来使用Marker消息。并不意外。
 2:定义了MovingObject类。
 成员函数：
 类的构造函数传入了nodehandle，用来初始化一些性质并定义publisher的内容。
 PoseCallback函数用来接收来自于rosbag的机器人位置的消息并转化为marker类型的消息发布出去。
 set_marker_fixed_property()函数用来设置marker一些我们不想改变的性质，和上一章类似。
 成员变量：
 数据成员pub_object_用来发布marker的pose使rviz接收
 mk_就是maker了
 br_即用来发布坐标系之间的关系的。
 3：主函数中sub_gps就是用来接收来自rosbag发布的poseStamped类型消息。消息在PoseCallback函数中处理。
 4：主函数之后，是MovingObject类的构造函数，在构造函数中，我们首先给pub_object赋值，负责发布marker类型的消息，pub_object发布的内容会用来产生我们动图1中的小方块。构造函数调用了set_marker_fixed_property()函数，在上一讲我们用这个函数来设置一些我们不想要改变的marker性质。除了颜色和大小不想改变外，我们并不想像上次那样，每接收一个pose就产生一个新的marker，我们希望当前的marker接收到一个新的pose后就移动到那个位置，所以marker只能有一个。上一讲我们说过marker的id和namespace两个量共同定义一个marker，所以在set_marker_fixed_property中把这两个量设置为常亮就可以了。即

```bash
    mk_.ns = "my_namespace";
    mk_.id = 0;
```

5：最后是PoseCallback函数。
 我们要发布坐标系之间的tf，和第一个例子一样，需要定义是哪两个坐标系，即frame_id和child_frame_id，分别是world和gps。需要定义transform的平移和旋转。pose和transform本就是一个东西，都需要定义平移和旋转，他们在transformStamped和PoseStamped的中的名字不一样。所以我们挨个把pose的position赋值给transform的translation，把pose的orinetation赋值给transform的rotation就可以了。
 代码中有一行

```php
mk_.pose = msg->pose;
```

我们直接把接收到的消息的pose直接赋值给了marker的pose。如果gps到world的transform也用接收到的消息的pose，那么marker的中心就会和gps坐标系的中心重合。所以代码中我把`pose.position.x`减了0.1，只是人为地把为了gps的坐标系原点和marker中心做点区别，不是非得加。
 定义好了tranformStamped之后就可以用br_.sendTranform把它发布出去，定义好了marker的pose之后，由于它的其他性质我们已经在set_marker_fixed_property中定义，那么就可以用pub_object_把marker发布出去了。这两个东西同时发布出去，用的同一个pose(除了x方向有0.1的固定差别)，如果我们像第一个例子那样编译并跑程序，设置好rviz，跑rosbag文件发布pose，理论上就可以在rviz当中看到gps这个坐标系随着我们的机器人(marker一起移动了)。
 等一下！还有一个坐标系呢？？动图里还有个camera坐标系，可是仔细看我们的代码里，没有任何关于`camera`的东西。**原来当两个坐标系相对静止的时候我们，我们最好直接把他们之间的tf直接设置到launch文件里**。
 首先我们把源文件写到CMakelists里进行编译。

```bash
add_executable(moving_coordinate_system src/moving_coordinate_system.cpp)
target_link_libraries(moving_coordinate_system ${catkin_LIBRARIES})
```

然后在launch文件夹里创建一个叫`run_mcs.launch`的launch文件。并把下面的内容写到launch文件中

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0.3 0 0 0 0 0 1 gps camera" />
    <node type="moving_coordinate_system" pkg="learn_rviz_tf" name="moving_coordinate_system" output="screen">
    </node>
</launch>
```

可以看到`<node pkg = "tf2_ros"...>`那一行，我们使用了名叫`tf2_ros`这个pakcage里名叫`static_transform_publisher`这个可执行文件，节点名我们命名为`link1_broadcaster`，args即我们要传入的transform了，前三个数字是transform的平移，后四个是四元数x,y,z,w(如果只有三个数字默认roll pitch yaw)。`gps camera`表示前面那组平移旋转就是这两个坐标系的关系。接着如果我们运行我们刚刚编译好的`moving_coordinate_system`，就可以看到一个新的坐标系出现在rviz里了。
 我们先打开rviz

```undefined
rosrun rviz rviz
```

如果你保存了之前rviz的信息，现在你rviz可能直接能接收marker和tf的信息，但我们假设是空白的。现在由于我们的程序既要发布marker的消息，又要发布tf的消息，自然我们需要加两个接收器。如上一章一样，点击rviz中ADD按钮，选择`Marker`点击OK。再次点击ADD按钮，选择`TF`，点击OK。把`Global Options`下面的Fixed_Frame设置为`world`。这时候你的rviz界面应该是这样的(注意左边栏包含了TF和Marker)。

![14634719-2bc0066ad57c5aed](../images/14634719-2bc0066ad57c5aed.webp)


 下面我们运行roslaunch

```css
roslaunch learn_tf_rviz run_mcs.launch
```

这时候我们注意到rviz中有些变化，如下图

![14634719-695f6f866563e872](../images/14634719-695f6f866563e872.webp)
 可以看到，rviz左边的Global Statue有刚刚的黄色警告变成了红色错误,TF也有个地方变成黄色警告。点击拥有红色错误标识的Fixed Frame，它旁边写着`Fixed Frame [world] does not exist`。是world frame还没有定义。
 这是因为我们的程序中world是定义在PoseCallback函数里的，也就是需要接收到消息后，world才有定义，如果没有接收到消息，那么world 就一直没有定义。所以我们只要让程序开始接收消息，错误就会消失。下面跑我们的rosbag。这里的rosbag本质上和我们在rosbag那一章创造的rosbag没有不同，使用的同样是第三讲那个程序，不过我把每两个pose之间的时间缩短了，这样我们的方块儿移动起来看起来就更流畅。新的rosbag的名字叫`robot_pose.bag`，你用我们之前的bag文件跑也是可以的，只是看起来没那么流畅。

```css
rosbag play robot_pose.bag
```

这行命令一执行，rviz里的错误会消失.

![14634719-22779723efea2b3b](../images/14634719-22779723efea2b3b.webp)



### (十) 创建自己的消息类型

#### 1）定制基本消息类型

当ROS提供的消息类型不满足你的需求时，你就需要考虑制作自己的消息类型了。比如你想要发布一则消息，这则消息包含一个double类型的向量，一个整数，一个字符串，他们都是基本的消息类型(属于std_msgs里的)。
 下面我们来看步骤
 1：在pub_sub_test这个package下建立一个新的文件夹，文件夹的名字叫msg
 2：在msg里创建一个新的文件，名字叫MyBasicMessage.msg。
 3：打开MyBasicMessage.msg，并在其中输入下面内容

```go
string message_id
int64 message_data1
float64[] message_data2
```

从这个页面我们知道Float64这个消息包含一个消息类型是float64名字叫data的成员。于是我们在pub_float64.cpp代码里类似如下使用





```cpp
...
double abc = 123.456;
std_msgs::Float64 msg; //定义Float64对象msg
msg.data = abc;//为类成员data赋值，赋值类型为double，即float64
...
```

当我们要自定义消息时，我们要做的就是模仿就行了。在.msg文件夹里，模仿上面的定义方式即可



```undefined
成员类型 自定义成员名字
```

那么我们怎么知道成员类型是什么呢？比如字符串是string，这个到简单，double变成float64，double类型的向量竟然是float64[]。其实这个我们同样在第一二讲讲了，ROS把这些基本类型重新定义了一番，具体可见[http://wiki.ros.org/msg](https://links.jianshu.com/go?to=http%3A%2F%2Fwiki.ros.org%2Fmsg)。在**Built-in types**下就写明了ROS自身的基本消息类型和C++,python中消息类型的对应关系，比如C++里的double在ROS中对应的是float64。所以我们会在msg文件里使用`float64 name`这种方式定义一个数据成员。同样在该页面中**Array handling**部分我们可以看到ROS对某种数据类型的数组的定义方式就是在基本类型后面加了个`[]`符号。比如`bool[]`对应c++的`std::vector<uint8_t>`.
 现在我们知道如何定义以及为什么如此定义基本类型了，在msg文件写好后，我们需要让我们的package知道我们新定义了消息类型，接着上面如下做
 4：打开pub_sub_test的CMakeLists.txt
 找到



```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
```

添加一行



```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
```

依赖项 message_generation是要自定义message所必须添加的。

找到



```bash
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```

改为



```css
add_message_files(
  FILES
  TestBasicMessage.msg
)
```

这相当于让这个package知道我们定义了新的message了。

找到



```bash
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#  
# )
```

改为



```undefined
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

这是让package知道我们定义的消息是依赖于std_msgs的。因为我们上面定义的消息类型都属于基础消息类型。我们得指明这一点。
 关闭保存CMakeLists.txt。打开pub_sub_test的package.xml。
 找到



```xml
<!--   <build_depend>message_generation</build_depend> -->
...
<!--   <build_export_depend>message_generation</build_export_depend> -->
...
<!--   <exec_depend>message_runtime</exec_depend> -->
```

把注释都去掉，变为



```xml
<build_depend>message_generation</build_depend> 
  ...
<build_export_depend>message_generation</build_export_depend>
...
<exec_depend>message_runtime</exec_depend> 
```

保存并关闭package.xml。这些都是死步骤，需要自定义message这么做即可。
 之后使用catkin_make编译。你自定义的消息类型就已经产生了。如何使用呢？和一般消息类型使用没有差别。
 我们在pub_sub_test/src里新创建一个cpp文件，名字叫`pub_my_basic_message.cpp`，把pub_string.cpp里或者之前写的其他基础pub程序赋值进去，改成下面的样子.



```cpp
#include "ros/ros.h"
#include "pub_sub_test/MyBasicMessage.h" //#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<pub_sub_test::MyBasicMessage>("chatter", 1000); //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  double data2 = 1;
  while (ros::ok())
  {
    pub_sub_test::MyBasicMessage msg; //std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    msg.message_id = "1";
    msg.message_data1 = count;
    msg.message_data2.push_back(data2);


    ROS_INFO("%ld", msg.message_data1); //ROS_INFO("%f", msg.data.c_str())

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
```

变化
 1：
 `#include "std_msgs/String.h"`变成了`#include "pub_sub_test/MyBasicMessage.h"`，即包含package名/msg文件名.h
 2：在advertise函数使用的地方
 <std_msgs::String>变成了<pub_sub_test::MyBasicMessage>
 3：std_msgs::String msg 变成了pub_sub_test::MyBasicMessage msg;
 4：为我们定义的消息类型赋值
 在MyBasicMessage.msg里我们定义了类型为string名字叫message_id的成员，所以我们使用`msg.message_id`，并赋值为字符串“1”。定义了float64(std::vector<double>)类型的成员message_data2，使用vector的函数push_back传入一个float64的变量。ROS_INFO就print出mesage_data1大家感受一下就是了。
 其余没变。总的来说程序中pub你自定义的消息和你想pub任何ROS自带的消息的步骤一样。
 你把这个cpp写入CMakeLists里编译即可。同样使用rosrun可以跑这个程序。sub文件就不再写了，很类似地改。不再赘述。

#### 2）定制高级消息

所谓高级消息，即是想PoseStamped那样的东西。其实步骤也一模一样，我们在msg文件夹再创建一个`MyAdvancedMessage.msg`。并在其中写入下面内容。



```undefined
geometry_msgs/Inertia SiHuan
geometry_msgs/Pose WuHuan
std_msgs/Header LiuHuan
```

定义了geometry_msgs/Inertia类型的成员名字叫四环，geometry_msgs/Pose类型的成员名字叫五环，std_msgs/Header类型的成员名字叫六环。

注意Compact Message Definition下面变量的定义方式，模仿就是了。你可以自己组建任何类型的消息。所以我把上面三种消息类型组合在了一起。其实没有什么大的意义hhh。至于Inertia类型的消息是什么怎么用，我想如果你看了前三讲这应该不是问题。
 之后我们需要进一步修改pub_sub_test的CMakeLists。打开CMakeLists.txt，在



```css
add_message_files(
  FILES
  MyBasicMessage.msg
)
```

中添加一行



```css
add_message_files(
  FILES
  MyBasicMessage.msg
  MyAdvancedMessage.msg
)
```

即我们刚刚新建立的message的名字。

在



```undefined
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

中添加一行



```undefined
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

由于我们新建立的消息类型不仅有来自于std_msgs的，还有来自于geometry_msgs的，所以我们需要把这个包添加到消息的dependency里。保存退出，使用catkin_make编译即可。
 之后你如果想发布这个类型的消息，写一个pub_my_advanced_message.cpp之类的文件，包含`#include "pub_sub_test/MyAdvancedMessage.h"`，定义变量，赋值等，和前面一样。至于如何为Pose等类型的成员Wuhuan之类的赋值，我想看过第三讲应该不会有问题。

#### 3）在packageA中使用packageB的中定义的消息类型

如果我们在pub_sub_test里定义的MyBasicMessage想在另一个包，比如我们的在讲如何使用roslaunch时建立的read_param_test这个package里，使用，应该怎么办呢？
 我们可以先试一下，在read_param_test/src中我们写了一个show_param.cpp文件。我们可以先试一下添加头文件看成功否。打开show_param.cpp文件，在头文件那几行添加一行`#include "pub_sub_test/MyBasicMessage.h"`，保存退出，使用catkin_make编译。编译不成功，显示找不到头文件。

![14634719-fbdb4b1158ac983d](../images/14634719-fbdb4b1158ac983d.webp)



那么如何找到它呢？如下
 1：打开read_param_test里的CMakeLists.txt，在





```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
```

中添加一行



```undefined
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pub_sub_test
)
```

是的，我们自己定义的包pub_sub_test也成了依赖项了。
 之后在我们定义的可执行文件



```bash
add_executable(read_param src/show_param.cpp)
target_link_libraries(read_param ${catkin_LIBRARIES})
```

后添加一行



```bash
add_executable(read_param src/show_param.cpp)
target_link_libraries(read_param ${catkin_LIBRARIES})
add_dependencies(read_param ${catkin_EXPORTED_TARGETS})
```

即添加`add_dependencies(可执行文件名 ${catkin_EXPORTED_TARGETS})`。关闭保存CMakeLists.txt。打开read_param_test的package.xml，在`<build_depend>roscpp</build_depend>`周围的位置添加



```xml
  <build_depend>pub_sub_test</build_depend>
  <exec_depend>pub_sub_test</exec_depend>
```

其实和任何在find_package里添加的依赖项一样，我们也需要在xml中指明添加的依赖项。保存xml，使用catkin_make编译。现在就不会报错了。现在你成功包含头文件，要定义发布MyBasicMessage消息之类的事情就可以完成了.

#### 4）自定义的消息类型制成package/在不同workspace使用自定义的消息

现在我们能在同一个workspace(catkin_ws)的不同package里使用我们在pub_sub_test中定义的消息了，但是如果以后我们建立了一个其他的workspace我们还想用自定义的消息MyBasicMessage类型怎么办呢？
 方法1：把pub_sub_test这个package直接复制到新的workspace（比如名字叫my_ws）my_ws/src里，使用catkin_make编译之后，pub_sub_test就在你的新的workspace里了，这时候你可以根据上面在**packageA中使用pakcageB中定义的消息类型**中的内容在新的workspace的不同package里使用自定义的消息了。
 **而且这个方法有一个bug，是ROS自己的问题**。比如当我们把pub_sub_test移动到另一个新建的test_ws/src之后，使用catkin_make会编译失败，提示找不到`MyBasicMessage.h`。

![14634719-c1ca2193fc56aa00](../images/14634719-c1ca2193fc56aa00.webp)

因为**首先得编译成功新的消息类型，才可以使用**。现在这个workspace里并没有记载有这个类型的消息，然而我们的cpp文件

```bash
add_executable(pub_my_basic_message src/pub_my_basic_message.cpp)
target_link_libraries(pub_my_basic_message ${catkin_LIBRARIES})
```

已经包含了头文件并且试图编译使用了。这时候我们需要先把这两行文件注释，就能编译成功，编译成功后，新的消息类型在workspace里有了记录，在去掉那两行的注释，再编译，就能成功了= =....

方法2：方法1的弊端是我们只是想使用自定义的消息，却把整个package都复制过去了，那个package里所有内容(cpp文件什么的)都用不到呢，非常'划不来'。**当你意识到你自定义的消息类型需要被很多不同的workspace里的很多不同package使用时，把它单独制成一个package**。这个package里没有任何的cpp文件或者python文件，只有msg的定义。这样你把这个package复制到各个不同workspace，将不会有任何多余的累赘复制过去，其实本质上和方法1是一样的。我们可以简单地试一下，cd 到catkin_ws/src，建立一个新的package，假设我们现在自定义的消息同样只包含std_msgs中的内容



```undefined
catkin_create_package my_custom_message std_msgs message_generation message_runtime
```

由于我们并不会写任何执行文件，所以连roscpp和rospy这两个元老都省了。
 这时候你新创建了一个pakcage，在pakcage中新建一个叫msg的文件夹。在文件夹中新建一个`MyNewMsg.msg`，在其中随便写点内容，如下



```go
float64 data
string id
```

之后打开该pakcage的CMakeLists.txt，和前面的内容类似了



```bash
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```

去掉注释改为



```css
add_message_files(
  FILES
  MyNewMsg.msg
)
```

另外



```bash
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```

去掉注释，改为



```undefined
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

由于我们一开始就添加了依赖项message_generation和message_runtime所以我们不需要早find_packge()中做修改了，也不需要修改package.xml了。保存CMakeLists.txt关闭。之后CMakeLists.txt编译。编译成功之后你就拥有了一个只为定义消息而生的pakcage，这时候结合**在packageA中使用packageB的中定义的消息类型**和方法1，你可以把它复制到任何workspace给任何package使用了。同时复制到新的workspace也编译也不会像方法1出错，因为这个pakcage里没有任何可执行文件使用了自定义的消息。



### (十一) 附录

#### 1）c++头文件实现函数的问题

头文件即.h文件一般是用来写函数或者类的定义的，而函数的实现一般在cpp文件中完成．但是我也看到过一些文件就在头文件中同时定义函数和实现．打个比方我现在有个头文件名叫`header.h`,内容如下



```cpp
#include <iostream>

void printInt(int number){
    std::cout<<"print an int number "<<number<<std::endl;
}
```

这个头文件同时定义与实现了这个函数．似乎不像是头文件的功能，但是这是允许的．那么这会不会造成什么问题呢？会的．如果你有两个文件同时包含了这个头文件并一起编译，就会有error产生．
 比如，你有一个cpp文件叫main.cpp，一个cpp文件叫test.cpp，两个文件都包含头文件header.h，里面实现了某个内容．编译时(这里使用CMakeLists.txt的语法作为例子)我们一起编译这两个文件



```css
add_executable(main main.cpp test.cpp)
```

就会出现问题．给出的错误是`...multiple definition...`．这个原因是test.cpp和main.cpp实现并定义了函数printInt，所以有重复定义．但这个问题很有意思的是如果你把函数写到类中，如下面



```cpp
#include <iostream>

class Print{
    void printInt(int number){
        std::cout<<"print an int number "<<number<<std::endl;
    }
};
```

则同样被两个文件包含，同样的编译，却不会出现error能正常编译．
 更有意思的是，我们经常喜欢在类外实现函数定义，即头文件改成如下面的这个样子



```cpp
#include <iostream>

class Print{
    void printInt(int number);
};

void Print::printInt(int number){
        std::cout<<"print an int number "<<number<<std::endl;
}
```

再重复上面的步骤编译，又会出现同样的multi definition的error了．

在头文件的实现中，有两种语法，即使实现了函数且被重复包含，也是不会出错的．一个是模版．模版需要定义和实现必须写在一起，无论是写在头文件还是cpp文件中．第二种是内联inline函数．即利用下面语句定义的函数．



```cpp
inline function_type function_name(){...}
```

个人想法是，除非用hpp文件(个人其实不经常使用)或者写模版内联函数，函数或者类的定义和实现还是分开在头文件和cpp源文件中写更好了．第一是代码清晰好读，第二是就算是函数写在类中实现没有大的问题，有经验的程序猿知道我们debug时头文件中函数实现部分的修改会导致包含头文件的所有文件重新编译，编译会耗时更长

#### 2）动态库和静态库

动态库是dynamic library 或者 shared library，在linux中以.so的后续名结尾．
 静态库是static library，　在linux中以.a方式结尾．
 具体见
 https://medium.com/@StueyGK/static-libraries-vs-dynamic-libraries-af78f0b5f1e4
 简单来讲，静态库会在可执行文件编译时一并被编译到文件中去．而动态库则不会，相当于是链接到可执行文件的．
 你如果修改了静态库，并重新编译了静态库，那么你使用了该库的执行文件也需要重新编译才能再次把静态库囊括进去。你如果修改了动态库，则只需要重新编译动态库本身不需要重新编译可执行文件。

#### 3）cmake module链接库

首先我有一个CMakeLists.txt文件，里面的可执行程序需要用到一个叫`libbayesopt.a`的静态库，库的位置位于`/usr/local/lib`.
 我们使用一些有名的库的时候，他好像自己就能找到，不需要多费精神，比如OpenCv，我们只需要在CMakeLists.txt里添加



```undefined
find_package(OpenCV REQUIRED)
```

并把某可执行文件链接到${OpenCv_LIBS}就可以了



```bash
add_executable(abc abc.cpp)
target_link_libraries(abc ${OpenCv_LIBS})
```

可是一些不知名的库，你直接使用find_package(<lib_name> REQUIRED)是找不到库的。因为其实都不知道那么lib_name是什么。比如我那么libbayesopt.a，我在CMakeLists.txt里应该叫它Bayesopt呢还是bayesopt呢？反正试了诸多名字，都是不能直接找到的。比如说我填写的是



```undefined
find_pakcage(Bayesopt REQUIRED)
```

在编译CMakeLists.txt时就会出现下面的错误:



```kotlin
CMake Error at bayesopt_compass/CMakeLists.txt:20 (find_package):
  By not providing "FindBayesopt.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "Bayesopt",
  but CMake did not find one.

  Could not find a package configuration file provided by "Bayesopt" with any
  of the following names:

    BayesoptConfig.cmake
    bayesopt-config.cmake

  Add the installation prefix of "Bayesopt" to CMAKE_PREFIX_PATH or set
  "Bayesopt_DIR" to a directory containing one of the above files.  If
  "Bayesopt" provides a separate development package or SDK, be sure it has
  been installed.
```

其实不用仔细看，大概就是程序找不到Bayesopt是个什么玩意儿，想让你写一个叫`BayesoptConfig.cmake`或者`bayesopt-config.cmake`的文件来找到这个Bayesopt。那么接下来怎么做呢？
 1:在和CMakeLists.txt相同位置建立一个叫cmake_modules的文件夹，在里面创建一个叫`BayesoptConfig.cmake`的文件。注意根据上面error的信息我们只有两种命名选择，这也增加了规范性。
 2:在`BayesoptConfig.cmake`中写入下面内容。



```bash
# Try to find bayesopt library
find_library(Bayesopt_LIBRARIES NAMES bayesopt)
find_path(Bayesopt_INCLUDE_PATH NAMES bayesopt/bayesopt.h)
```

第一行是注释。
 第二行要求寻找一个叫(NAMES)libbayesopt.a或者.so的库。并在CMakeLists中命名为`Bayesopt_LIBRARIES`. 可以看到我们只是指定了bayesopt，并不是指定找libbayesopt.a。这是程序默认的。任何库的名字都是lib开头。你只需要指定lib后的那串字符就可以了。
 第三行指定该库对应的头文件的位置，并在CMakeLists中命名为`Bayesopt_INCLUDE_PATH`.
 这两个find函数会在`/usr/local/lib`和`/usr/local/include`里找相应的库，应该(注意我这儿说的应该，因为我还没自己尝试过找/usr/lib里的库)也会在`/usr/lib`这类文件夹里找相应的库。
 注意有些同学在编译时命名库在`usr/local/lib`里电脑却找不到，这时候你得考虑一下自己添加寻找目录。打开terminal输入



```ruby
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

即可。或者要永久有效的话



```undefined
gedit ~/.bashrc
```

打开一个叫bashrc的文件，在里面最后面添加上面的那行代码。之后保存退出再



```bash
source ~/.bashrc
```

即可。你可以这样指定电脑去任何目录寻找库文件
 3：在你的CMakeLists.txt中，定义好最小cmake版本需求和项目名称后就可以输入我们cmake_modules的位置



```bash
cmake_minimum_required(VERSION 2.8.3)
project(bayesopt_compass)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")
find_package(Bayesopt REQUIRED)
...
add_executable(abc abc.cpp)
target_link_libraries(abc ${Bayesopt_LIBRARIES})
```

list那一行告诉cmake在哪儿去找我们的modules。之后输入我们要找的package名称。这里正如前面所说的，我们需要把Bayesopt这个名字和FindXXX.cmake中的XXX对应起来。最后在可执行文件添加库链接的时候，那个库的名字和`FindBayesopt.cmake`中的find_library中定义的名字一样。(其实个人试了一下，链接到${Bayesopt_LIBRARY}也是可以的，好像不区分单复数)。可以参考一下https://blog.csdn.net/dbzhang800/article/details/6329314里的内容
 保存CMakeLists.txt之后就能正常编译了。

#####   暴力链接库

如果你实在不想写.cmake文件，但是又要链接到一个你不知道find_package(<name> REQUIRED)中name是何物的库，你可以用一种很暴力的方法。
 比如你知道你要链接的库的位置就是在`/usr/local/lib/libabc.a`，你可以在target_link_libraries中直接添加这个路径



```ruby
cmake_minimum_required(VERSION 2.8.3)
project(your_project)
...
add_executable(def def.cpp)
target_link_libraries(def /usr/local/lib/libabc.a)
```

这种方法也是work的！不仅不用写cmake，而且连find_package这个语句都省略了，怎么样是不是简单又暴力！不过这样做的话看起来也就没那么专业了hhhh，而且以后别人如果使用你的程序来扩展也会比较麻烦，如果你写的程序只想自己或少数几个人弄懂，这也不失为一个办法。

