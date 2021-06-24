# Vsomeip环境搭建

vsomeip是一个开源C++库，它实现了SOME/IP协议栈。

## 一、安装Boost (vsomeip依赖)

boost中，用到了别的函数库，所以为了使用boost中相应的功能，需要先安装系统中可能缺失的库。

### (一) 安装依赖库

```bash
sudo apt-get install mpi-default-dev　　#安装mpi库  
sudo apt-get install libicu-dev　　　　　#支持正则表达式的UNICODE字符集   
sudo apt-get install python-dev　　　　　#需要python的话  
sudo apt-get install libbz2-dev　　　　　#如果编译出现错误：bzlib.h: No such file ordirectory
```


注意：复制粘贴的时候，不要复制命令后面的空格，这样会导致一个无法定位软件包的错误。

安装失败统一的办法：

```bash
sudo apt-get update  
```

### (二) 下载boost

http://sourceforge.net/projects/boost/files/latest/download?source=dlp

(如果下载速度很慢，重新下载，不用一直等)

下载好了以后，解压 .bz2 文件

```bash
tar -jxvf xx.tar.bz2
```

解压之后，进入解压目录，执行：

```bash
./bootstrap.sh
sudo ./b2
sudo ./b2 install
```

PS : boost的安装时间还是很长的，单核的虚拟机上面 30 min 左右。

### (三) 测试

在test文件夹中建立test.cpp

```c++
#include<iostream>
#include<boost/bind.hpp>
using namespace std;
using namespace boost;
int fun(int x,int y){return x+y;}
int main(){
    int m=1;int n=2;
    cout<<boost::bind(fun,_1,_2)(m,n)<<endl;
    return 0;
}
```

编译：

```bash
g++ test.cpp -o test  
```

最后执行的结果是 3

![2021-06-09 16-49-27屏幕截图](/home/ygd/资料/SomeIp资料/images/2021-06-09 16-49-27屏幕截图.png)

参考：http://valleylord.github.io/post/201601-boost-install/

linux查看boost版本

```
dpkg -S /usr/include/boost/version.hpp
```



## 二、Vsomeip安装

- **vsomeip 概述**

  vsomeip 堆栈实现了http://some-ip.com/（基于 IP 的可扩展的面向服务的中间件 (SOME/IP)）协议。堆栈包括：

  - SOME/IP 的共享库 ( `libvsomeip3.so`)
  - SOME/IP 服务发现的第二个共享库 ( `libvsomeip3-sd.so`)，如果启用了服务发现，则在运行时加载。

- ##### Linux 构建说明

  ###### 依赖关系

  - 需要启用 C++11 的编译器，例如 gcc >= 4.8。
  - vsomeip 使用 CMake 作为构建系统。
  - vsomeip 使用 Boost >= 1.55：

### (一) 安装Vsomeip

打开终端，执行安装语句

```bash
git clone https://github.com/GENIVI/vsomeip.git
cd vsomeip
mkdir build
cd build
cmake -DENABLE_SIGNAL_HANDLING=1 -DDIAGNOSIS_ADDRESS=0x10 ..
make
sudo make install
```

(如果git失败，多试几次就行,或者直接下载https://github.com/GENIVI/vsomeip)



### (二) 测试

#### 1）构建 hello_world

```bash
cmake --build . --target hello_world
cd ./examples/hello_world/
make
```

#### 2）运行 hello_world

```bash
vim ~/.bashrc
```

添加环境变量内容：

```bash
export VSOMEIP_CONFIGURATION=../examples/hello_world/helloworld-local.json
export VSOMEIP_APPLICATION_NAME=hello_world_service
```

```bash
export VSOMEIP_CONFIGURATION=../examples/hello_world/helloworld-local.json
export VSOMEIP_APPLICATION_NAME=hello_world_client
```

编辑生效：source ~/.bashrc

启动服务端与客户端：

```bash
./hello_world_service

./hello_world_client
```

#### 3）运行结果

预期服务端输出：

![2021-06-10 15-10-47屏幕截图](/home/ygd/资料/SomeIp资料/images/2021-06-10 15-10-47屏幕截图.png)

预期客户端输出：

![2021-06-10 15-12-51屏幕截图](/home/ygd/资料/SomeIp资料/images/2021-06-10 15-12-51屏幕截图.png)



## 