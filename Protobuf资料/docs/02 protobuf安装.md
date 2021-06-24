## 一、protobuf安装

```bash
// 下载 protoBuf：
$ git clone https://github.com/protocolbuffers/protobuf.git
//  安装依赖库
$ sudo apt-get install autoconf  automake  libtool curl make  g++  unzip libffi-dev -y
// 进入目录
$ cd protobuf/ 
// 自动生成configure配置文件：
$ ./autogen.sh 
// 配置环境：
$ ./configure
// 编译源代码(有的慢，要耐心等待)：
$ make 
// 安装
$ sudo make install
// 刷新共享库 （很重要的一步啊）
$ sudo ldconfig 
// 成功后需要使用命令测试（成功则会显示protobuf版本）
$ protoc -version
```

## 二、测试示例

### 1.编写 .proto 文件

 首先我们需要编写一个 proto 文件，定义我们程序中需要处理的结构化数据，在 protobuf 的术语中，结构化数据被称为 Message。proto 文件非常类似 java 或者 C 语言的数据定义。

**proto文件：**

```protobuf
syntax = "proto3";//声明protobuf版本

 package lm; 
 message helloworld 
 { 
     int32     id = 1;  // ID 
 	 string    str = 2;  // str 
 	 int32     opt = 3;  //optional field
 }
```


 一个比较好的习惯是认真对待 proto 文件的文件名。比如将命名规则定于如下：

 **packageName.MessageName.proto**

 在上例中，package 名字叫做 lm，定义了一个消息 helloworld，该消息有三个成员，类型为 int32 的 id，另一个为类型为 string 的成员 str。opt 是一个可选的成员，即消息中可以不包含该成员。

**注意：**

**1.proto3中，在第一行非空白非注释行，必须写：syntax = "proto3";**

**2、消息定义时，移除了 “required”、 “optional” ：**

**3、移除了 default 选项：**

**4、枚举类型的第一个字段必须为 0 ：**

**4、编译 .proto 文件**

**5、增加maps结构：**



### 2.Protobuf 编译器将文件编译成目标语言

 本例中我们将使用 C++。

```bash
// $SRC_DIR: .proto所在的源目录
// --cpp_out: 生成C++代码
// $DST_DIR: 生成代码的目标目录
// xxx.proto: 要针对哪个proto文件生成接口代码

protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/xxx.proto

```

进入proto文件目录，可以使用如下命令编译：

```bash
# protoc （proto文件名） --cpp_out=（生成文件的目标文件夹）
protoc ./Mymessage.proto --cpp_out=./test
```

 命令将生成两个文件：

 `Mymessage.pb.h` ， 定义了 C++ 类的头文件

`Mymessage.pb.cc` ， C++ 类的实现文件

 在生成的头文件中，定义了一个 C++ 类 helloworld，后面的 Writer 和 Reader 将使用这个类来对消息进行操作。诸如对消息的成员进行赋值，将消息序列化等等都有相应的方法。

### 3.编写 writer 和 Reader

 使用 Protobuf，Writer 的工作很简单，需要处理的结构化数据由 .proto 文件描述，经过的编译过程后，该数据化结构对应了一个 C++ 的类，并定义在 `Mymessage.pb.h` 中。对于本例，类名为 Mymessage。

 Writer 需要 include 该头文件，然后便可以使用这个类了。

 现在，在 Writer 代码中，将要存入磁盘的结构化数据由一个 lm::helloworld 类的对象表示，它提供了一系列的 get/set 函数用来修改和读取结构化数据中的数据成员，或者叫 field。

#### 1） Writer 的主要代码:

```c++
#include <iostream>
#include <fstream>
#include "Mymessage.pb.h"

   int main()

   {

         Im::Content msg1;

        msg1.set_id(10);
        msg1.set_str("hello world");

          std::fstream output("./log", std::ios::out|std::ios::trunc|std::ios::binary);

         if(!msg1.SerializeToOstream(&output))

         {

             std::cerr << "Failed to write msg." << std::endl;

                 return -1;

   }

        return 0;

 }
```

 Msg1 是一个Mymessage 类的对象，set_id() 用来设置 id 的值。SerializeToOstream 将对象序列化后写入一个 fstream 流。

执行Writer文件，生成log文件。

对于Reader，只需从log文件中读取，反序列化后就能获得结构化的数据。利用Im::Cotent对象的ParseFromIstream方法从一个fstream流中读取信息并反序列化，此后，ListMsg 中采用 get 方法读取消息的内部信息，并进行打印输出操作。


#### 2）Reader 的主要代码

```c++
  #include <iostream>

  #include <fstream>

  #include "Mymessage.pb.h"



   void ListMsg(const Im::Content& msg)

   {

           std::cout << msg.id() << std::endl;

          std::cout << msg.str() << std::endl;

   }

  int main()

  {

         Im::Content msg1;

         std::fstream input("./log", std::ios::in|std::ios::binary);

        if(!msg1.ParseFromIstream(&input))

          {

                 std::cerr << "Failed to parse address book." << std::endl;

                 return -1;

          }

       ListMsg(msg1);

         return 0;

  }
```

Reader 声明类Mymessage的对象 msg1，然后利用 ParseFromIstream 从一个 fstream 流中读取信息并反序列化。此后，ListMsg 中采用 get 方法读取消息的内部信息，并进行打印输出操作。

## 3）CMakeList.txt文件

**注意：未知错误导致vscode无法编译采用办法**使用cmake编译

```bash
cmake  ..

make
```

**CMakeList,txt主要代码**

```bash
cmake_minimum_required(VERSION 3.5)

PROJECT(bin)

# Find required protobuf package

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories("/usr/include/")

add_compile_options( -std=c++11 -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines )

add_executable (proto_r Reader.cpp Mymessage.pb.cc)
target_link_libraries(proto_r  ${PROTOBUF_LIBRARIES} )

add_executable (proto_w Writer.cpp Mymessage.pb.cc)
target_link_libraries(proto_w  ${PROTOBUF_LIBRARIES} )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
```

**运行结果:**

![sendpix0](/home/ygd/资料/03 Protobuf资料/images/sendpix0.jpg)

### 4.序列化接口API

```c++
class MessageLite {
public:
	//序列化：
	bool SerializeToOstream(ostream* output) const;
	bool SerializeToArray(void *data, int size) const;
	bool SerializeToString(string* output) const;
	

	//反序列化：
	bool ParseFromIstream(istream* input);
	bool ParseFromArray(const void* data, int size);
	bool ParseFromString(const string& data);

};
```

三种序列化的方法没有本质上的区别，只是序列化后输出的格式不同，可以供不同的应用场景使用。
序列化的API函数均为const成员函数，因为序列化不会改变类对象的内容， 而是将序列化的结果保存到函数入参指定的地址中。