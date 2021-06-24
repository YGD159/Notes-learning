# protobuf学习

**1、Protobuf 的优点**
 Protobuf 有如 XML，不过它更小、更快、也更简单。我们可以定义自己的数据结构，然后使用代码生成器生成的代码来读写这个数据结构。甚至可以在无需重新部署程序的情况下更新数据结构。只需使用 Protobuf 对数据结构进行一次描述，即可利用各种不同语言或从各种不同数据流中对结构化数据轻松读写。

 它有一个非常棒的特性，即“向后”兼容性好，人们不必破坏已部署的、依靠“老”数据格式的程序就可以对数据结构进行升级。这样我们的程序就可以不必担心因为消息结构的改变而造成的大规模的代码重构或者迁移的问题。因为添加新的消息中的 field 并不会引起已经发布的程序的任何改变。

 Protobuf 语义更清晰，无需类似 XML 解析器的东西（因为 Protobuf 编译器会将 .proto 文件编译生成对应的数据访问类以对 Protobuf 数据进行序列化、反序列化操作）。

 使用 Protobuf 无需学习复杂的文档对象模型，Protobuf 的编程模式比较友好，简单易学，同时它拥有良好的文档和示例，对于喜欢简单事物的人们而言，Protobuf 比其他的技术更加有吸引力。

**2、Protobuf 的不足**
 Protbuf 与 XML 相比也有不足之处。它功能简单，无法用来表示复杂的概念。

 XML 已经成为多种行业标准的编写工具，Protobuf 只是 Google 公司内部使用的工具，在通用性上还差很多。

 由于文本并不适合用来描述数据结构，所以 Protobuf 也不适合用来对基于文本的标记文档（如 HTML）建模。另外，由于 XML 具有某种程度上的自解释性，它可以被人直接读取编辑，在这一点上 Protobuf 不行，它以二进制的方式存储，除非你有 .proto 定义，否则你没法直接读出 Protobuf 的任何内容。

**3、Protobuf示例**

github链接：https://github.com/protocolbuffers/protobuf

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

![sendpix0](../images/sendpix0.jpg)

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

## 三、Protobuf语法

### 1.包（package）

为.proto文件添加package声明符，可以防止不同 .proto项目间消息类型的命名发生冲突。

```protobuf
package foo.bar;
message Open { ... }
```

```protobuf
message Foo {
  ...
  foo.bar.Open open = 1;
  ...
}
```

protobuf包类型的解析和C++类似，都是由内而外进行解析。对于C++，产生的类会被包装在C++的命名空间中，如上例中的Open会被封装在 foo::bar空间中。

### 2.选项（option）

**option会影响特定环境下的处理方式，但是不会改变整个文件声明的含义**。

```protobuf
option optimize_for = CODE_SIZE;
```

![在这里插入图片描述](../images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2RhYWlrdWFpY12h1YW4=,size_16,color_FFFFFF,t_70)

### 3.消息类型（message）

  **message用于定义结构数据，可以包含多种类型字段（field），每个字段声明以分号结尾。message经过protoc编译后会生成对应的class类，field则会生成对应的方法**

```protobuf
syntax = "proto3"; // 表示使用的protobuf版本是proto3

message SearchRequest {
  string query = 1;
  int32 page_number = 2;
  int32 result_per_page = 3;
}
```

#### 1）常规消息类型

##### 1、字段修饰符

  在proto3中，去掉了required和optional，对于原始数据类型字段不再提供 hasxxx()方法，只有单个字段或者重复字段：

**单个字段**：表示字段可以出现0次或者1次。

**repeated**：表示字段可以重复任意次。

##### 2、字段类型

###### ① 标量类型

protobuf标量数据类型与各平台的数据类型对应如下表：
![在这里插入图片描述](../images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2RhYWlrdWFpY2h1Y2W4=,size_16,color_FFFFFF,t_70)

###### ②枚举类型

  **protobuf中的enum类型和C++中的枚举类型相似，表示字段取值的集合**。

```protobuf
message SearchRequest {
  string query = 1;
  int32 page_number = 2;
  int32 result_per_page = 3;
  enum Corpus {
    UNIVERSAL = 0;
    WEB = 1;
    IMAGES = 2;
    LOCAL = 3;
    NEWS = 4;
    PRODUCTS = 5;
    VIDEO = 6;
  }
  Corpus corpus = 4;
}
```

  **枚举类型中第一个元素的值必须从0开始，而且proto3中删除了default标记，默认值为第一个元素**。

  当枚举类型是在某一个消息内部定义，但是**希望在另一个消息中使用时**，需要采用**MessageType.EnumType**的语法格式。



###### ③Any类型

  protobuf中的Any类型与C++中的泛型概念类似，可以定义为任意的类型。在序列化的时候可以通过PackFrom()方法将任意的数据类型打包为Any类型，反序列化的时候通过UnpackTo()把Any类型还原为原始类型。

```protobuf
// 要使用Any类型必须导入该proto文件
import "google/protobuf/any.proto";

message ErrorStatus {
  string message = 1;
  repeated google.protobuf.Any details = 2;
}
```

```protobuf
// Storing an arbitrary message type in Any.
NetworkErrorDetails details = ...;
ErrorStatus status;
status.add_details()->PackFrom(details);

// Reading an arbitrary message from Any.
ErrorStatus status = ...;
for (const Any& detail : status.details()) {
  if (detail.Is<NetworkErrorDetails>()) {
    NetworkErrorDetails network_error;
    detail.UnpackTo(&network_error);
    ... processing network_error ...
  }
}
```



###### ④oneof类型

  protobuf中的oneof类似与C++中的联合体类型相似，所有的字段共享内存，最多只能同时设置一个字段，设置oneof的任何字段会自动清除所有其他字段，可以使用case()或WhichOneof()方法检查oneof中使用的是哪个字段。

```protobuf
message SampleMessage {
  oneof test_oneof {
    string name = 4;
    SubMessage sub_message = 9;
  }
}
```


【oneof特性】：

设置oneof会自动清除其它oneof字段的值：

```protobuf
SampleMessage message;
message.set_name("name");
CHECK(message.has_name());
message.mutable_sub_message();   // Will clear name field.
CHECK(!message.has_name());
```

oneof不能声明为repeated类型。
注意不要出现内存崩溃问题：

```protobuf
SampleMessage message;
SubMessage* sub_message = message.mutable_sub_message();
message.set_name("name");      // Will delete sub_message
sub_message->set_...            // Crashes here
```

可以在oneof内部添加和删除field，但是删除和添加oneof要小心

###### ⑤map类型

  protobuf中的map类似与STL中的关联型容器相似，map是key-value类型，key可以是int或者string，value可以是自定义message。

```protobuf
map<key_type, value_type> map_field = N;

// 与上述定义等价
message MapFieldEntry {
  key_type key = 1;
  value_type value = 2;
}
repeated MapFieldEntry map_field = N;
```


【map特性】：

map不能定义为repeated类型。

当为.proto文件产生生成文本格式的时候，map会按照key 的顺序排序，数值化的key会按照数值排序。

从序列化中解析时，如果有重复的key，只会使用第一个key。

##### 3、默认值说明

string类型，默认值是空字符串。

bytes类型，默认值是空bytes。

bool类型，默认值是false。

数字类型，默认值是0。

枚举类型，默认值是第一个枚举值，即0。

repeated修饰的属性，默认值是空。

##### 4、标识号

  在消息类型中，每一个字段都有一个唯一的标识符（Tag），不应该随意改动。

  [1-15]内的标识号在编码时只占用一个字节，包含标识符和字段类型，[16-2047]之间的标识符占用2个字节。建议为频繁出现的字段使用[1-15]间的标识符。

  如果考虑到以后可能扩展元素，可以预留一些标识符或者字段。注意不能在一个reserved声明中混合使用字段名和标识符。

```protobuf
message Foo {
  reserved 2, 15, 9 to 11;
  reserved "foo", "bar";
}
```


  最小的标识符可以从1开始，最大到2^29 - 1，或536,870,911。不可以使用[19000－19999]之间的标识符， Protobuf协议实现中预留了这些标识符。在.proto文件中使用这些预留标识号，编译时就会报错。

#### 2）多个消息类型

  一个.proto文件中可以定义多个消息类型：

```protobuf
syntax = "proto3";

// SearchRequest 搜索请求
message SearchRequest {
    string query = 1;           // 查询字符串
    int32  page_number = 2;     // 页码
    int32  result_per_page = 3; // 每页条数
}

// SearchResponse 搜索响应
message SearchResponse {
    ...
}
```

#### 3）嵌套消息类型

  在protobuf中message之前可以嵌套使用：

```protobuf
message SearchResponse {
    message Result {
        string url = 1;
        string title = 2;
        repeated string snippets = 3;
    }
    repeated Result results = 1;
}
```


  内部声明的消息message名称只可在内部直接使用，在外部使用需要添加父级message名称

```protobuf
（Parent.Type）：

message SomeOtherMessage {
    SearchResponse.Result result = 1;
}
```

  支持多层嵌套：

```protobuf
message Outer {                // Level 0
    message MiddleAA {         // Level 1
        message Inner {        // Level 2
            int64 ival = 1;
            bool  booly = 2;
        }
    }
    message MiddleBB {         // Level 1
        message Inner {        // Level 2
            int32 ival = 1;
            bool  booly = 2;
        }
    }
}
```



#### 4）更新消息类型

  如果一个已有的消息类型已无法满足新的需求，比如需要添加一个额外的字段，但是同时旧版本写的代码仍然可用。在更新消息类型需要遵循以下规则：

1、不要更改任何已有字段的标识号。

2、int32、uint32、int64、uint64,和bool是全部兼容的，这意味着可以将这些类型中的一个转换为另外一个，而不会破坏向前、 向后的兼容性。

3、sint32和sint64是互相兼容的，但是它们与其他整数类型不兼容。

4、string和bytes是兼容的，只要bytes是有效的UTF-8编码。

5、嵌套消息与bytes是兼容的，只要bytes包含该消息的一个编码过的版本。

6、fixed32与sfixed32是兼容的，fixed64与sfixed64是兼容的。

### 4.RPC服务（service）

  如果想要将消息类型用在远程方法调用（RPC）系统中，可以在.proto文件中定义一个RPC服务接口。

```protobuf
service UserService {
    //  包含方法名、方法参数和返回值，
    // 接收SearchRequest并返回一个SearchResponse
    rpc GetUser(Request) returns (Response); 
}
```


gRPC在使用protobuf时非常有效，如果使用特殊的protobuf插件可以直接从.proto文件中产生相关的RPC代码。

### 5.其他

导入proto文件（import）
  如果希望在当前proto文件中引用其他的proto文件中的内容，可以使用import：

```protobuf
import "other_project/other_protos.proto";
```

详细见：[06 Protobuf、序列化、反序列化](docs/06 Protobuf、序列化、反序列化)

## 四、Protobuf语法风格

### 1.代码风格

每一行的代码长度不要超过80。

使用两个空格进行缩进。

### 2.文件格式

  文件命名应该采用蛇形命名法（即用下划线连接），如：lower_snake_case.proto。所有文件应以下列方式排列：

```
License header (if applicable)
File overview
Syntax
Package
Imports (sorted)
File options
Everything else
```

#### 3.包

  包名应该是小写的，并且应该对应于目录层次结构。例如，如果一个文件位于my/Package/中，那么包名应该是my.Package。

#### 4.消息类型和字段

  消息名使用驼峰命名法，例如：SongServerRequest，字段名和扩展名使用小写的下划线分隔式，例如：song_name。

```protobuf
message SongServerRequest {
  required string song_name = 1;
}
```

```protobuf
const string& song_name() { ... }
void set_song_name(const string& x) { ... }
```


如果字段名包含数字，则该数字应出现在字母之后，而不是下划线之后。例如：song_name1。

#### 5.repeated字段

  repeated字段使用复数命名：

```protobuf
repeated string keys = 1;
repeated MyMessage accounts = 17;
```



#### 6.枚举类型

  枚举名使用使用驼峰命名法，成员使用大写的下划线分隔式：

```protobuf
enum Foo {
  FOO_UNSPECIFIED = 0;
  FOO_FIRST_VALUE = 1;
  FOO_SECOND_VALUE = 2;
}
```

#### 7.服务

  服务名称和任何RPC方法名称均使用驼峰命名法：

```protobuf
service FooService {
  rpc GetSomething(FooRequest) returns (FooResponse);
}
```

## 五、Protobuf使用实例

### 1.ProtoBuf使用的一般步骤

知道了ProtoBuf的作用与支持的数据类型。我么需要知道ProtoBuf使用的一般步骤，下面以C++中使用ProtoBuf为例来描述使用的一般步骤。

**第一步：**定义proto文件，文件的内容就是定义我们需要存储或者传输的数据结构，也就是定义我们自己的数据存储或者传输的协议。

**第二步：**编译安装protocol buffer编译器来编译自定义的.proto文件，用于生成.pb.h文件（proto文件中自定义类的头文件）和 .pb.cc（proto文件中自定义类的实现文件）。

**第三步：** 使用protoco buffer的C++ API来读写消息。

### 2实例

#### 1）实例1

##### 1、定义proto文件

```protobuf
syntax = "proto3";

enum Messagetype
{
	REQUEST_RESPONSE_NONE = 0;            
	REQUEST_HEARTBEAT_SIGNAL = 1;          
	RESPONSE_HEARTBEAT_RESULT = 2;      
}

message MsgResult
{
	bool result =1;  
	bytes error_code = 2; 
}

message TopMessage
{
	Messagetype message_type = 1; 		//message type
	MsgResult msg_result = 2;

}

```

`syntax` 指定protobuf的版本,必须是文件的第一行

`message` 代表这是一个消息,TopMessage消息中指定了2个字段,每一个字段对应于要包含在这种类型的消息中的数据,每一个字段都有一个名称和类型

`enum` 代表枚举

等号后面的不是默认值,可以认为是该字段的身份证,不可重复,这些数字用于以二进制格式标识您的字段,一旦消息类型被使用就不应该被更改

**注意:** 需要注意的是1到15范围内的字段编号需要一个字节来编码,而16到2047范围内的字段编号需要两个字节,所以经常使用的应该为其编码为1到15之间,这些数字的范围是[1, 536870911],其中19000～19999是为协议缓冲区实现而保留的,不可以使用,否则报错



##### 2、编译.proto文件

使用一下命令

```bash
protoc proto文件路径 --cpp_out=C++代码文件导出目录
```

##### 3.Protobuf数据写入和读取

```c++
#include "example_message.pb.h"		//解析出来的.h文件
#include "stdio.h"

void sendHeart();
void receHeart(TopMessage* topMessage);
void receHeartResp(TopMessage* topMessage);

//  Method      :  sendHeart
 // Description :  数据写入

void sendHeart(){
    
	TopMessage message;
	message.set_message_type(REQUEST_HEARTBEAT_SIGNAL);
	printf("sendHeart %d\n",message.message_type());
	receHeart(&message);

}

//Method      :  receHeart
//Description :  数据读取然后写入

void receHeart(TopMessage* topMessage){

	if (topMessage->message_type() == REQUEST_HEARTBEAT_SIGNAL)
    {
		printf("request_heartbeat_signal\n");
		TopMessage topMessageResp;

		MsgResult mesResult;
		mesResult.set_result(true);

		mesResult.set_error_code("error");

		topMessageResp.set_message_type(RESPONSE_HEARTBEAT_RESULT);

		*topMessageResp.mutable_msg_result() = mesResult;

		receHeartResp(&topMessageResp);
	}

}

//Method      :  receHeartResp
//Description :  数据读取

void receHeartResp(TopMessage* topMessage){

	if (topMessage->message_type() == RESPONSE_HEARTBEAT_RESULT)
	{
		printf("response_heartbeat_result\n");

		printf("%s\n",topMessage->msg_result().error_code().c_str());

	}

}
int main()
{
	sendHeart();
	google::protobuf::ShutdownProtobufLibrary();
}
```

##### 4.编译执行

**cmake编译方法：**

进入文件目录

```bash
cmake ..

make
```

执行生成的test文件

```bash
./proto_rw
```

输出如下：

![sendpix1](/home/ygd/资料/03 Protobuf资料/images/sendpix1.jpg)

#### 2）实例2

##### 1、定义proto文件

定义proto文件就是定义自己的数据存储或者传输的协议格式。

要想序列化Student对象进行网络传输，那么我们需要从编写一个.proto文件开始。

.proto文件的定义是比较简单的：为每一个你需要序列化的数据结构添加一个消息（message），然后为消息（message）中的每一个字段（field）指定一个名字、类型和修饰符以及唯一标识（tag）。每一个消息对应到C++中就是一个类，嵌套消息对应的就是嵌套类，当然一个.proto文件中可以定义多个消息，就像一个头文件中可以定义多个类一样。

**student.proto**(对应protobuf2版本)

```protobuf
syntax = "proto2";

package tutorial;

message Student{
    required uint64 id = 1;
    required string name =2;
    optional string email = 3;

    enum PhoneType {
        MOBILE = 0;
        HOME = 1;
    }

    message PhoneNumber { 
        required string number = 1;
        optional PhoneType type = 2 [default = HOME];
    }
    repeated PhoneNumber phone = 4;
}
```

正如你所看到的一样，该语法类似于C++或Java的语法。让我们依次来看看文件的每一部分的作用。

**关于package声明。**  .proto文件以一个package声明开始。这个声明是为了防止不同项目之间的命名冲突。对应到C++中去，你用这个.proto文件生成的类将被放置在一个与package名相同的命名空间中。

**关于字段类型。**  再往下看，就是若干消息（message）定义了。一个消息就是某些类型的字段的集合。许多标准的、简单的数据类型都可以用作字段类型，包括bool，int32，float，double以及string。你也可以使用其他的消息（message）类型来作为你的字段类型——在上面的例子中，消息PhoneNumber 就是一个被用作字段类型的例子。

**关于修饰符。** 

 每一个字段都必须用以下之一的修饰符来修饰：

**required：**必须提供字段值，否则对应的消息就会被认为是“未初始化的”。如果libprotobuf是以debug模式编译的，序列化一个未初始化的消息（message）将会导致一个断言错误。在优化过的编译情况下（译者注：例如release），该检查会被跳过，消息会被写入。然而，解析一个未初始化的消息仍然会失败（解析函数会返回false）。除此之外，一个required的字段与一个optional的字段就没有区别了。

**optional：**字段值指定与否都可以。如果没有指定一个optional的字段值，它就会使用默认值。对简单类型来说，你可以指定你自己的默认值，就像我们在上面的例子中对phone number的type字段所做的一样。如果你不指定默认值，就会使用系统默认值：数据类型的默认值为0，string的默认值为空字符串，bool的默认值为false。对嵌套消息（message）来说，其默认值总是消息的“默认实例”或“原型”，即：没有任何一个字段是指定了值的。调用访问类来取一个未显式指定其值的optional（或者required）的字段的值，总是会返回字段的默认值。

**repeated：**字段会重复N次（N可以为0）。重复的值的顺序将被保存在protocol buffer中。你只要将重复的字段视为动态大小的数组就可以了。

**注意：** required是永久性的：在把一个字段标识为required的时候，你应该特别小心。如果在某些情况下你不想写入或者发送一个required的字段，那么将该字段更改为optional可能会遇到问题——旧版本的读者（译者注：即读取、解析消息的一方）会认为不含该字段的消息（message）是不完整的，从而有可能会拒绝解析。在这种情况下，你应该考虑编写特别针对于应用程序的、自定义的消息校验函数。Google的一些工程师得出了一个结论：使用required弊多于利；他们更愿意使用optional和repeated而不是required。当然，这个观点并不具有普遍性。

**关于标识。**  在每一项后面的、类似于“= 1”，“= 2”的标志指出了该字段在二进制编码中使用的唯一“标识（tag）”。标识号1~15编码所需的字节数比更大的标识号使用的字节数要少1个，所以，如果你想寻求优化，可以为经常使用或者重复的项采用1~15的标识（tag），其他经常使用的optional项采用≥16的标识（tag）。在重复的字段中，每一项都要求重编码标识号（tag number），所以重复的字段特别适用于这种优化情况。

你可以在[Language Guide (proto3)](https://developers.google.com/protocol-buffers/docs/proto3)一文中找到编写.proto文件的完整指南（包括所有可能的字段类型）。但是，不要想在里面找到与类继承相似的特性，因为protocol buffers不是拿来做这个的。

##### 2、编译.proto文件

有了Protocol Buffers的编译器protoc，我们就可以来编译我们自定义的.proto文件来产生对应的消息类，生成一个头文件 ( 定义.proto文件中的消息类 )，和一个源文件（实现.proto文件中的消息类）。

编译方法。指定源目录（即你的应用程序源代码所在的目录——如果不指定的话，就使用当前目录）、目标目录（即生成的代码放置的目录，通常与$SRC_DIR是一样的），以及你的.proto文件所在的目录。命令如下：

```javascript
protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/addressbook.proto
```

因为需要生成的是C++类，所以使用了–cpp_out选项参数——protocol buffers也为其他支持的语言提供了类似的选项参数，如`--java_out=OUT_DIR`，指定java源文件生成目录。

以上面自定义的student.proto为例，来编译产生我们的student消息类。运行如下命令：

```javascript
protoc student.proto --cpp_out=./
```

这样就可以在我指定的当前目录下生成如下文件：

```javascript
student.pb.h：声明你生成的类的头文件。
student.pb.cc：你生成的类的实现文件。
```

![sendpix3](../images/sendpix3.jpg)

##### 3、了解Protocol Buffer API

让我们看一下生成的代码，了解一下编译器为你创建了什么样的类和函数。如果你看了编译器protoc为我们生成的student.pb.h文件，就会发现你得到了一个类，它对应于student.proto文件中写的每一个消息（message）。更深入一步，看看Student类：编译器为每一个字段生成了读写函数。例如，对name，id，email以及phone字段，分别有如下函数：

```javascript
  // required uint64 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::uint64 id() const;
  inline void set_id(::google::protobuf::uint64 value);

  // required string name = 2;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 2;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // optional string email = 3;
  inline bool has_email() const;
  inline void clear_email();
  static const int kEmailFieldNumber = 3;
  inline const ::std::string& email() const;
  inline void set_email(const ::std::string& value);
  inline void set_email(const char* value);
  inline void set_email(const char* value, size_t size);
  inline ::std::string* mutable_email();
  inline ::std::string* release_email();
  inline void set_allocated_email(::std::string* email);

  // repeated .tutorial.Student.PhoneNumber phone = 4;
  inline int phone_size() const;
  inline void clear_phone();
  static const int kPhoneFieldNumber = 4;
  inline const ::tutorial::Student_PhoneNumber& phone(int index) const;
  inline ::tutorial::Student_PhoneNumber* mutable_phone(int index);
  inline ::tutorial::Student_PhoneNumber* add_phone();
  inline const ::google::protobuf::RepeatedPtrField< ::tutorial::Student_PhoneNumber >& phone() const;
  inline ::google::protobuf::RepeatedPtrField< ::tutorial::Student_PhoneNumber >* mutable_phone();
```

正如你所看到的，getter函数具有与字段名一模一样的名字，并且是小写的，而setter函数都是以set_前缀开头。此外，还有has_前缀的函数，对每一个单一的（required或optional的）字段来说，如果字段被置（set）了值，该函数会返回true。最后，每一个字段还有一个clear_前缀的函数，用来将字段重置（un-set）到空状态（empty state）。

然而，数值类型的字段id就只有如上所述的基本读写函数，name和email字段则有一些额外的函数，因为它们是string——前缀为mutable_的函数返回string的直接指针（direct pointer）。除此之外，还有一个额外的setter函数。注意：你甚至可以在email还没有被置（set）值的时候就调用mutable_email()，它会被自动初始化为一个空字符串。在此例中，如果有一个单一消息字段，那么它也会有一个mutable_ 前缀的函数，但是没有一个set_ 前缀的函数。

重复的字段也有一些特殊的函数——如果你看一下重复字段phone 的那些函数，就会发现你可以：  （1）得到重复字段的_size（换句话说，这个Person关联了多少个电话号码）。

（2）通过索引（index）来获取一个指定的电话号码。

（3）通过指定的索引（index）来更新一个已经存在的电话号码。

（3）向消息（message）中添加另一个电话号码，然后你可以编辑它（重复的标量类型有一个add_前缀的函数，允许你传新值进去）。

关于编译器如何生成特殊字段的更多信息，请查看文章[C++ generated code reference](https://developers.google.com/protocol-buffers/docs/reference/cpp-generated)。

**关于枚举和嵌套类（Enums and Nested Classes）。**  生成的代码中包含了一个PhoneType 枚举，它对应于.proto文件中的那个枚举。你可以把这个类型当作Student::PhoneType，其值为Student::MOBILE和Student::HOME（实现的细节稍微复杂了点，但是没关系，不理解它也不会影响你使用该枚举）。

编译器还生成了一个名为Student::PhoneNumber的嵌套类。如果你看看代码，就会发现“真实的”类实际上是叫做Student_PhoneNumber，只不过Student内部的一个typedef允许你像一个嵌套类一样来对待它。这一点所造成的唯一的一个区别就是：如果你想在另一个文件中对类进行前向声明（forward-declare）的话，你就不能在C++中对嵌套类型进行前向声明了，但是你可以对Student_PhoneNumber进行前向声明。

**关于标准消息函数（Standard Message Methods）。**  每一个消息（message）还包含了其他一系列函数，用来检查或管理整个消息，包括：

```javascript
bool IsInitialized() const; //检查是否全部的required字段都被置（set）了值。

void CopyFrom(const Person& from); //用外部消息的值，覆写调用者消息内部的值。

void Clear();   //将所有项复位到空状态（empty state）。

int ByteSize() const;   //消息字节大小
```

**关于Debug的API。**

```javascript
string DebugString() const; //将消息内容以可读的方式输出

string ShortDebugString() const; //功能类似于，DebugString(),输出时会有较少的空白

string Utf8DebugString() const; //Like DebugString(), but do not escape UTF-8 byte sequences.

void PrintDebugString() const;  //Convenience function useful in GDB. Prints DebugString() to stdout.
```

这些函数以及后面章节将要提到的I/O函数实现了Message 的接口，它们被所有C++ protocol buffer类共享。更多信息，请查看文章[complete API documentation for Message](https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.message#Message)。

**关于解析&序列化(Parsing and Serialization)。**

最后，每一个protocol buffer类都有读写你所选择的消息类型的函数。它们包括：

```javascript
bool SerializeToString(string* output) const; //将消息序列化并储存在指定的string中。注意里面的内容是二进制的，而不是文本；我们只是使用string作为一个很方便的容器。

bool ParseFromString(const string& data); //从给定的string解析消息。

bool SerializeToArray(void * data, int size) const  //将消息序列化至数组

bool ParseFromArray(const void * data, int size)    //从数组解析消息

bool SerializeToOstream(ostream* output) const; //将消息写入到给定的C++ ostream中。

bool ParseFromIstream(istream* input); //从给定的C++ istream解析消息。
```

这些函数只是用于解析和序列化的几个函数罢了。请再次参考[Message API reference](https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.message#Message)以查看完整的函数列表。

**注意：**  protocol buffers和面向对象的设计 protocol buffer类通常只是纯粹的数据存储器（就像C++中的结构体一样）；它们在对象模型中并不是一等公民。如果你想向生成的类中添加更丰富的行为，最好的方法就是在应用程序中对它进行封装。如果你无权控制.proto文件的设计的话，封装protocol buffers也是一个好主意（例如，你从另一个项目中重用一个.proto文件）。在那种情况下，你可以用封装类来设计接口，以更好地适应你的应用程序的特定环境：隐藏一些数据和方法，暴露一些便于使用的函数，等等。但是你绝对不要通过继承生成的类来添加行为。这样做的话，会破坏其内部机制，并且不是一个好的面向对象的实践。

##### 4、使用Protocol Buffer来读写消息

下面让我们尝试使用protobuf为我们产生的消息类来进行序列化和反序列的操作。你想让你的Student程序完成的第一件事情就是向Student消息类对象进行 赋值，并且进行序列化操作。然后在从序列化结果进行反序列话操作，解析我们需要的字段信息。具体参考如下示例代码：

```javascript
//student.cpp

#include <iostream>
#include <string>
#include "student.pb.h"
using namespace std;

int main(int argc, char* argv[]){
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    tutorial::Student student;

    //给消息类Student对象student赋值
    student.set_id(201421031059);
    *student.mutable_name()="dablelv";
    student.set_email("dablelv@tencent.com");
    //增加一个号码对象
    tutorial::Student::PhoneNumber* phone_number = student.add_phone();
    phone_number->set_number("15813354925");
    phone_number->set_type(tutorial::Student::MOBILE);

    //再增加一个号码对象
    tutorial::Student::PhoneNumber* phone_number1 = student.add_phone();
    phone_number1->set_number("0564-4762652");
    phone_number1->set_type(tutorial::Student::HOME);

    //对消息对象student序列化到string容器
    string serializedStr;
    student.SerializeToString(&serializedStr);
    cout<<"serialization result:"<<serializedStr<<endl; //序列化后的字符串内容是二进制内容，非可打印字符，预计输出乱码
    cout<<endl<<"debugString:"<<student.DebugString();

/*----------------上面是序列化，下面是反序列化-----------------------*/
    //解析序列化后的消息对象，即反序列化
    tutorial::Student deserializedStudent;
    if(!deserializedStudent.ParseFromString(serializedStr)){
      cerr << "Failed to parse student." << endl;
      return -1;
    }

    cout<<"-------------上面是序列化，下面是反序列化---------------"<<endl;
    //打印解析后的student消息对象 
    cout<<"deserializedStudent debugString:"<<deserializedStudent.DebugString();
    cout <<endl<<"Student ID: " << deserializedStudent.id() << endl;
    cout <<"Name: " << deserializedStudent.name() << endl;
    if (deserializedStudent.has_email()){
        cout << "E-mail address: " << deserializedStudent.email() << endl;
    }
    for (int j = 0; j < deserializedStudent.phone_size(); j++){
        const tutorial::Student::PhoneNumber& phone_number = deserializedStudent.phone(j);

        switch (phone_number.type()) {
            case tutorial::Student::MOBILE:
            cout << "Mobile phone #: ";
            break;
            case tutorial::Student::HOME:
                cout << "Home phone #: ";
            break;
        }
        cout <<phone_number.number()<<endl;
    }

    google::protobuf::ShutdownProtobufLibrary();
}
```

编译上面的测试程序，可使用如下命令：

```javascript
g++ -o protobufTest.out -lprotobuf test.cpp student.pb.cc
```

编译成功后，运行protobufTest.out程序，可能会报如下错误：

```javascript
error while loading shared libraries: libprotobuf.so.9: cannot open shared object file: No such file or directory
```

原因是protobuf的连接库默认安装路径是/usr/local/lib，而/usr/local/lib 不在常见Linux系统的LD_LIBRARY_PATH链接库路径这个环境变量里，所以就找不到该lib。LD_LIBRARY_PATH是Linux环境变量名，该环境变量主要用于指定查找共享库（动态链接库）。所以，解决办法就是修改环境变量LD_LIBRARY_PATH的值。  **方法一：**  使用export命令临时修改LD_LIBRARY_PATH，只对当前shell会话有效：

```javascript
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

**方法二：**  或者永久修改，在~/目录下打开.bash_profile文件，设置环境变量如下：

```javascript
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  
export LD_LIBRARY_PATH  
```

注意Linux下点号开头的文件都是隐藏文件，使用`ls -a` 可查看指定目录下的所有文件，包括隐藏文件。

**方法三：**  永久有效的话，可以创建protobuf的动态连接库配置文件/etc/ld.so.conf.d/libprotobuf.conf并包含如下内容：

```javascript
/usr/local/lib 
```

然后运行动态链接库的管理命令ldconfig。

```javascript
sudo ldconfig
```

ldconfig通常在系统启动时运行,而当用户安装了一个新的动态链接库时,就需要手工运行这个命令。

测试程序输出结果：  

![sendpix4](../images/sendpix4.jpg)

##### 5、扩展一个protocol buffer（Extending a Protocol Buffer）

无论或早或晚，在你放出你那使用protocol buffer的代码之后，你必定会想“改进“protocol buffer的定义，即我们自定义定义消息的proto文件。如果你想让你的新buffer向后兼容（backwards-compatible），并且旧的buffer能够向前兼容（forward-compatible），你一定希望如此，那么你在新的protocol buffer中就要遵守其他的一些规则了：  （1）对已存在的任何字段，你都不能更改其标识（tag）号。

（2）你绝对不能添加或删除任何required的字段。

（3）你可以添加新的optional或repeated的字段，但是你必须使用新的标识（tag）号（例如，在这个protocol buffer中从未使用过的标识号——甚至于已经被删除过的字段使用过的标识号也不行）。

（有一些例外情况，但是它们很少使用。）

如果你遵守这些规则，老的代码将能很好地解析新的消息（message），并忽略掉任何新的字段。对老代码来说，已经被删除的optional字段将被赋予默认值，已被删除的repeated字段将是空的。新的代码也能够透明地读取旧的消息。但是，请牢记心中：新的optional字段将不会出现在旧的消息中，所以你要么需要显式地检查它们是否由has_前缀的函数置（set）了值，要么在你的.proto文件中，在标识（tag）号的后面用[default = value]提供一个合理的默认值。如果没有为一个optional项指定默认值，那么就会使用与特定类型相关的默认值：对string来说，默认值是空字符串。对boolean来说，默认值是false。对数值类型来说，默认值是0。还要注意：如果你添加了一个新的repeated字段，你的新代码将无法告诉你它是否被留空了（被新代码），或者是否从未被置（set）值（被旧代码），这是因为它没有has_标志。

##### 6、优化小技巧（Optimization Tips）

Protocol Buffer 的C++库已经做了极度优化。但是，正确的使用方法仍然会提高很多性能。下面是一些小技巧，用来提升protocol buffer库的最后一丝速度能力：  （1）如果有可能，重复利用消息（message）对象。即使被清除掉，消息（message）对象也会尽量保存所有被分配来重用的内存。这样的话，如果你正在处理很多类型相同的消息以及一系列相似的结构，有一个好办法就是重复使用同一个消息（message）对象，从而使内存分配的压力减小一些。然而，随着时间的流逝，对象占用的内存也有可能变得越来越大，尤其是当你的消息尺寸（译者注：各消息内容不同，有些消息内容多一些，有些消息内容少一些）不同的时候，或者你偶尔创建了一个比平常大很多的消息（message）的时候。你应该自己监测消息（message）对象的大小——通过调用SpaceUsed函数——并在它太大的时候删除它。

（2）在多线程中分配大量小对象的内存的时候，你的操作系统的内存分配器可能优化得不够好。在这种情况下，你可以尝试用一下Google’s tcmalloc。

##### 7、高级使用（Advanced Usage）

Protocol Buffers的作用绝不仅仅是简单的数据存取以及序列化。请阅读[C++ API reference](https://developers.google.com/protocol-buffers/docs/reference/cpp/)全文来看看你还能用它来做什么。

protocol消息类所提供的一个关键特性就是反射。你不需要编写针对一个特殊的消息（message）类型的代码，就可以遍历一个消息的字段，并操纵它们的值，就像XML和JSON一样。“反射”的一个更高级的用法可能就是可以找出两个相同类型的消息之间的区别，或者开发某种“协议消息的正则表达式”，利用正则表达式，你可以对某种消息内容进行匹配。只要你发挥你的想像力，就有可能将Protocol Buffers应用到一个更广泛的、你可能一开始就期望解决的问题范围上。

“反射”是由Message::Reflection interface提供的。

# 参考文献

1、protobuf官网(https://developers.google.com/protocol-buffers/) 

2、protobuf github源码(https://github.com/google/protobuf/)  

3、序列化和反序列化(http://kb.cnblogs.com/page/515982/) 

4、最常用的两种C++序列化方案的使用心得（protobuf和boost serialization）(http://www.cnblogs.com/lanxuezaipiao/p/3703988.html)  

5、Protocol Buffer Basics: C++中文翻译（Google Protocol Buffers中文教程）(http://www.codelast.com/) 

 6、Protobuf C++英文教程(https://developers.google.com/protocol-buffers/docs/cpptutorial) 

 7、LD_LIBRARY_PATH环境变量的设置(http://james23dier.iteye.com/blog/763274)  

8、LD_LIBRARY_PATH.百度百科(http://baike.baidu.com/link?url=VslKYj-RxAKt8-dVQOaaulO-pTBdge5NiIjgnNb_9OFKFX_yu940dnWFWPasNgYEPD_gXwkGz4wG3mGG0W_OFa) 

 9、ubuntu下编译protobuf(http://blog.csdn.net/xocoder/article/details/9155901)

