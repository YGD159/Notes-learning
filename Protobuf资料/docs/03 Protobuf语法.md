## Protobuf语法

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

![在这里插入图片描述](/home/ygd/资料/03 Protobuf资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2RhYWlrdWFpY12h1YW4=,size_16,color_FFFFFF,t_70)

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
![在这里插入图片描述](/home/ygd/资料/03 Protobuf资料/images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2RhYWlrdWFpY2h1Y2W4=,size_16,color_FFFFFF,t_70)

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

