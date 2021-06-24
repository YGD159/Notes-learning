## ROS安装

### (一) 版本说明

Ubuntu版本：Ubuntu18.04 LTS

Ros版本：Melodic

**注意：终端“无法找到软件包”一般为ROS与Ubuntu系统版本不一致，更换即可**



### (二) 配置系统软件源

打开“软件和更新”，进入到“Ubuntu软件”界面，允许universe、restricted、multiverse三项，即勾选这三项，如下图所示。安装完Ubuntu系统后这三项是默认勾选的，但还是检查一下为好，同时将源改为清华源或阿里源（下载速度快）。

<img src="../images/2021-06-07 13-39-47屏幕截图.png" alt="2021-06-07 13-39-47屏幕截图" style="zoom:67%;" />



### (三) 添加Ros软件源

* 在终端输入如下指令，设置sources.list

>sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'



### (四) 添加密钥

* 在终端输入如下指令，添加Ros密钥

>sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654



### (五) 安装ROS

* 在终端输入如下指令，确保软件索引是最新的

>sudo apt update

* 在终端输入如下指令，安装桌面完整版Ros

>sudo apt install ros-melodic-desktop-full



### (六) 设置环境变量

* 在终端输入如下指令，对环境变量进行配置

>echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
>source ~/.bashrc



### (七) 初始化rosdep

* 在终端输入如下指令，对rosdep进行初始化

>sudo rosdep init

 >rosdep update

注意：在初始化的时候可能会发生一些无法下载或者连接失败的错误，原因是https://raw.githubusercontent.com 这个网站被DSN污染，国内无法访问。(不会影响ROS运行，只是依赖不容易查看)

解决方案：https://zhuanlan.zhihu.com/p/370785600



### (八) 安装rosinstall

* 在终端输入如下指令，安装rosinstall

>sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential



### (九) 运行Ros

* 在终端输入如下指令，运行Ros，查看安装好的ROS的发行版名称为melodic和版本号1.14.11

>roscore



<img src="../images/2021-06-07 13-59-03屏幕截图.png" alt="2021-06-07 13-59-03屏幕截图" style="zoom:67%;" />