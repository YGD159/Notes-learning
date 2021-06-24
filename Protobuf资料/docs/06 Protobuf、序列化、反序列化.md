# Protobuf、序列化、反序列化

## 一、编译.proto文件

  **protoc**：protobuf自带的编译工具，将.proto文件生成指定的类  

  **–cpp_out**：将生成的C++代码文件放到等号后面指定的目录，这里也指定当前目录

```javascript
    通过protoc工具编译.proto文件时，编译器将生成所选择语言的代码，这些代码可以操作在.proto文件中定义的消息类型，包括获取、设置字段值，将消息序列化到一个输出流中，以及从一个输入流中解析消息。对C++来说，编译器会为每个.proto文件生成一个.h文件和一个.cc文件，.proto文件中的每一个消息有一个对应的类。
```

## 二、编译代码

```javascript
g++ *.cpp *.c *.cc `pkg-config --cflags --libs protobuf`
```

 反引号(` )：反引号的作用就是将反引号内的linux命令执行  

 pkg-config 是通过库提供的一个.pc文件获得库的各种必要信息的，包括版本信息、编译和连接需要的参数等。  

 pkg-config –cflags protobuf：列出指定共享库的预处理和编译flags 

 pkg-config –libs protobuf：列出指定共享库的链接flags

## 三、在.proto文件中定义消息格式

消息由至少一个字段组合而成，类似于C语言中的结构体，每个字段都有一定的格式：

数据类型 字段名称 = 唯一的编号标签值;

```javascript
syntax = "proto3"; //指定版本信息，不指定会报错

message Person  //message为关键字，作用为定义一种消息类型
{
    string name = 1;    //姓名
    int32 id = 2;       //id
    string email = 3;   //邮件
}
```

### 1.数据类型

| .proto类型 | C++类型         | 备注                                                         |
| :--------- | :-------------- | :----------------------------------------------------------- |
| double     | double          | 64位浮点数                                                   |
| float      | float           | 32位浮点数                                                   |
| int32      | int32           | 32位整数                                                     |
| int64      | int64           | 64位整数                                                     |
| uint32     | uint32          | 32位无符号整数                                               |
| uint64     | uint64          | 64位无符号整数                                               |
| sint32     | int32           | 32位整数，处理负数效率比int32更高                            |
| sint32     | sint64          | 64位整数，处理负数效率比int64更高                            |
| fixed32    | uint32          | 总是4个字节。如果数值总是比总是比228大的话，这个类型会比uint32高效。 |
| fixed64    | uint64          | 总是8个字节。如果数值总是比总是比256大的话，这个类型会比uint64高效。 |
| sfixed32   | int32           | 总是4个字节                                                  |
| sfixed64   | int64           | 总是8个字节                                                  |
| bool       | bool            | 布尔类型                                                     |
| string     | string          | 一个字符串必须是UTF-8编码或者7-bit ASCII编码的文本           |
| bytes      | string          | 处理多字节的语言字符、如中文                                 |
| enum       | enum            | 枚举                                                         |
| message    | object of class | 自定义的消息类型                                             |

​    proto文件即消息协议原型定义文件，在该文件中我们可以通过使用描述性语言，来良好的定义我们程序中需要用到数据格式。通过查看头文件，可以发现针对每个字段都会大致生成如下几种函数，以name为例。可以看出，对于每个字段会生成一个clear清除函数(clear_name)、set函数(set_name)、get函数(name和mutable_name)。

```javascript
void clear_name();
void set_name(const ::std::string& value);
void set_name(const char* value);
void set_name(const char* value, size_t size);
const ::std::string& name() const;
::std::string* mutable_name();
```

### 2.C数组的序列化和反序列化

```javascript
#include <iostream>
#include "person.pb.h"

using namespace std;

int main()
{
    char buf[1024];
    int len;

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    Person obj;
    obj.set_name("gongluck");
    obj.set_id(1);
    *obj.mutable_email() = "http://blog.csdn.net/gongluck93";
    len = obj.ByteSize();
    cout << "len = " << len << endl;
    obj.SerializeToArray(buf, len);

    Person obj2;
    obj2.ParseFromArray(buf, len);
    cout << "name = " << obj2.name() << endl;
    cout << "id = " << obj2.id() << endl;
    cout << "email = " << obj2.email() << endl;

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
```

### 3.C++ String的序列化和反序列化

```javascript
#include <iostream>
#include "person.pb.h"

using namespace std;

int main()
{
    string str;

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    Person obj;
    obj.set_name("gongluck");
    obj.set_id(1);
    *obj.mutable_email() = "http://blog.csdn.net/gongluck93";
    obj.SerializeToString(&str);

    Person obj2;
    obj2.ParseFromString(str);
    cout << "name = " << obj2.name() << endl;
    cout << "id = " << obj2.id() << endl;
    cout << "email = " << obj2.email() << endl;

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
```

### 4.文件描述符序列化和反序列化

```javascript
#include <unistd.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "person.pb.h"

using namespace std;

int main()
{
    int fd = open("./testFileDesc.xxx", O_CREAT|O_TRUNC|O_RDWR, 0664);

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    Person obj;
    obj.set_name("gongluck");
    obj.set_id(1);
    *obj.mutable_email() = "http://blog.csdn.net/gongluck93";
    obj.SerializeToFileDescriptor(fd);
    fsync(fd);
    lseek(fd, 0, SEEK_SET);

    Person obj2;
    obj2.ParseFromFileDescriptor(fd);
    cout << "name = " << obj2.name() << endl;
    cout << "id = " << obj2.id() << endl;
    cout << "email = " << obj2.email() << endl;

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
```

### 5.C++ stream 序列化和反序列化

```javascript
#include <iostream>
#include <fstream>
#include "person.pb.h"

using namespace std;

int main()
{
    fstream file("testStream.xxx", ios::in|ios::out|ios::trunc|ios::binary);

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    Person obj;
    obj.set_name("gongluck");
    obj.set_id(1);
    *obj.mutable_email() = "http://blog.csdn.net/gongluck93";
    obj.SerializeToOstream(&file);
    file.flush();
    file.seekg(0, ios::beg);

    Person obj2;
    obj2.ParseFromIstream(&file);
    cout << "name = " << obj2.name() << endl;
    cout << "id = " << obj2.id() << endl;
    cout << "email = " << obj2.email() << endl;

    google::protobuf::ShutdownProtobufLibrary();

    file.close();

    return 0;
}
```

### 6.repeated限定修饰符

repeated 代表可重复，我们可以理解为数组。

```javascript
syntax = "proto3";

message Person
{
    string name = 1;
    int32 id = 2;
    string email = 3;
}

message AddressBook
{
    repeated Person people = 1;
}
```

而对于字段修饰符为repeated的字段生成的函数，则稍微有一些不同，如people字段，则编译器会为其产生如下的代码：

```javascript
int people_size() const;
void clear_people();
const ::Person& people(int index) const;
::Person* mutable_people(int index);
::Person* add_people();
::google::protobuf::RepeatedPtrField< ::Person >* mutable_people();
const ::google::protobuf::RepeatedPtrField< ::Person >& people() const;
```

**·例子**

```javascript
#include <iostream>
#include <fstream>
#include "person.pb.h"

using namespace std;

int main()
{
    fstream file("testStream.xxx", ios::in|ios::out|ios::trunc|ios::binary);

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    AddressBook obj;

    Person* p1 = obj.add_people();
    p1->set_name("gongluck");
    p1->set_id(1);
    *(p1->mutable_email()) = "http://blog.csdn.net/gongluck93";

    Person* p2 = obj.add_people();
    p2->set_name("panzhikun");
    p2->set_id(2);
    *(p2->mutable_email()) = "panzhikun@gg.com";

    obj.SerializeToOstream(&file);
    file.flush();
    file.seekg(0, ios::beg);

    AddressBook obj2;
    obj2.ParseFromIstream(&file);
    for(int i= 0; i< obj.people_size(); ++i)
    {
        Person per = obj2.people(i);
        cout << "name = " << per.name() << endl;
        cout << "id = " << per.id() << endl;
        cout << "email = " << per.email() << endl;
    }

    google::protobuf::ShutdownProtobufLibrary();

    file.close();

    return 0;
}
```

### 7.枚举

```javascript
syntax = "proto3";

message Person
{
    string name = 1;
    int32 id = 2;
    string email = 3;

    enum PhoneType
    {
        MOBLIE = 0;//首成员必须为0
        HOME = 1;
        WORK = 2;
    }
    message PhoneNumber
    {
        string number = 1;
        PhoneType type = 2;
    }
    repeated PhoneNumber phones = 4;
}

message AddressBook
{
    repeated Person people = 1;
}
```

**·例子**

```javascript
#include <iostream>
#include <fstream>
#include "person.pb.h"

using namespace std;

int main()
{
    fstream file("testStream.xxx", ios::in|ios::out|ios::trunc|ios::binary);

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    AddressBook obj;

    Person* p1 = obj.add_people();
    p1->set_name("gongluck");
    p1->set_id(1);
    *(p1->mutable_email()) = "http://blog.csdn.net/gongluck93";

    Person::PhoneNumber* phone1 = p1->add_phones();
    phone1->set_number("110");
    phone1->set_type(Person::MOBLIE);
    Person::PhoneNumber* phone2 = p1->add_phones();
    phone2->set_number("120");
    phone2->set_type(Person::WORK);

    obj.SerializeToOstream(&file);
    file.flush();
    file.seekg(0, ios::beg);

    AddressBook obj2;
    obj2.ParseFromIstream(&file);
    for(int i= 0; i< obj.people_size(); ++i)
    {
        Person per = obj2.people(i);
        cout << "name = " << per.name() << endl;
        cout << "id = " << per.id() << endl;
        cout << "email = " << per.email() << endl;
        for(int j=0; j< per.phones_size(); ++j)
        {
            Person::PhoneNumber phonenum= per.phones(j);
            switch(phonenum.type())
            {
            case Person::MOBLIE:
                cout << "mobile : " ;
                break;
            case Person::WORK:
                cout << "work : ";
                break;
            case Person::HOME:
                cout << "home : ";
                break;
            default:
                cout << "Not Know : ";
                break;
            }
            cout << phonenum.number() << endl;
        }
    }

    google::protobuf::ShutdownProtobufLibrary();

    file.close();

    return 0;
}
```

### 8.包

 .proto文件新增一个可选的package声明符，用来防止不同的消息类型有命名冲突。包的声明符会根据使用语言的不同影响生成的代码。对于C++，产生的类会被包装在C++的命名空间中。

```javascript
syntax = "proto3";

package Test;//package

message Person
{
    string name = 1;
    int32 id = 2;
    string email = 3;

    enum PhoneType
    {
        MOBLIE = 0;//首成员必须为0
        HOME = 1;
        WORK = 2;
    }
    message PhoneNumber
    {
        string number = 1;
        PhoneType type = 2;
    }
    repeated PhoneNumber phones = 4;
}

message AddressBook
{
    repeated Person people = 1;
}
```

**·例子**

```javascript
using namespace Test;
#include <iostream>
#include <fstream>
#include "person.pb.h"

using namespace std;
using namespace Test;

int main()
{
    fstream file("testStream.xxx", ios::in|ios::out|ios::trunc|ios::binary);

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    AddressBook obj;

    Person* p1 = obj.add_people();
    p1->set_name("gongluck");
    p1->set_id(1);
    *(p1->mutable_email()) = "http://blog.csdn.net/gongluck93";

    Person::PhoneNumber* phone1 = p1->add_phones();
    phone1->set_number("110");
    phone1->set_type(Person::MOBLIE);
    Person::PhoneNumber* phone2 = p1->add_phones();
    phone2->set_number("120");
    phone2->set_type(Person::WORK);

    obj.SerializeToOstream(&file);
    file.flush();
    file.seekg(0, ios::beg);

    AddressBook obj2;
    obj2.ParseFromIstream(&file);
    for(int i= 0; i< obj.people_size(); ++i)
    {
        Person per = obj2.people(i);
        cout << "name = " << per.name() << endl;
        cout << "id = " << per.id() << endl;
        cout << "email = " << per.email() << endl;
        for(int j=0; j< per.phones_size(); ++j)
        {
            Person::PhoneNumber phonenum= per.phones(j);
            switch(phonenum.type())
            {
            case Person::MOBLIE:
                cout << "mobile : " ;
                break;
            case Person::WORK:
                cout << "work : ";
                break;
            case Person::HOME:
                cout << "home : ";
                break;
            default:
                cout << "Not Know : ";
                break;
            }
            cout << phonenum.number() << endl;
        }
    }

    google::protobuf::ShutdownProtobufLibrary();

    file.close();

    return 0;
}
```

### 9.导入定义

```javascript
syntax = "proto3";//指定版本信息，不指定会报错

import "info.proto"; //导入定义

package tutorial; //package声明符

message Person //message为关键字，作用为定义一种消息类型
{
    string name = 1;    //姓名
    int32 id = 2;       //id
    string email = 3; //邮件

    enum PhoneType //枚举消息类型
    {
        MOBILE = 0; //proto3版本中，首成员必须为0，成员不应有相同的值
        HOME = 1;
        WORK = 2;
    }

    message PhoneNumber
    {
        string number = 1;
        PhoneType type = 2;
    }

    repeated PhoneNumber phones = 4; //phones为数组

    //info定义在"info.proto"
    //类型格式：包名.信息名
    infopack.info tmp = 5;
}

message AddressBook
{
    repeated Person people = 1;
}
```

github地址：https://github.com/gongluck/CodeBase/tree/master/notes/protobuf-notes

