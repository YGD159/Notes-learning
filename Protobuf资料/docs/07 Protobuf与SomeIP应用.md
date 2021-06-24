# Protobuf与SomeIP应用

## 一、Protobuf序列化

### 1.protobuf的定义

  protobuf是一种用于序列化结构数据的工具，实现数据的存储与交换，与编程语言和开发平台无关。

  **序列化：**将结构数据或者对象转换成能够用于存储和传输的格式。

  **反序列化：**在其他的计算环境中，将序列化后的数据还原为结构数据和对象。

  定义数据的结构，然后使用protoc编译器生成源代码，在各种数据流中使用各种语言进行编写和读取结构数据。甚至可以更新数据结构，而不破坏由旧数据结构编译的已部署程序。

### 2.protobuf的优缺点

**2.1、优点**
性能高效：与XML相比，protobuf更小（3 ~ 10倍）、更快（20 ~ 100倍）、更为简单。

语言无关、平台无关：protobuf支持Java、C++、Python 等多种语言，支持多个平台。

扩展性、兼容性强：只需要使用protobuf对结构数据进行一次描述，即可从各种数据流中读取结构数据，更新数据结构时不会破坏原有的程序。

- **压缩效率高**：服务器间的海量数据传输与通信，可以节省磁盘和带宽，protobuf适合处理大数据集中的单个小消息，但并不适合处理单个的大消息。
- **解析速度快**：可以提高服务器的吞吐能力

**2.2、缺点**
不适合用来对基于文本的标记文档（如 HTML）建模。

自解释性较差，数据存储格式为二进制，需要通过proto文件才能了解到内部的数据结构。

![在这里插入图片描述](../images/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2RhYWlrdWFpY2h1YW4=,size_16,color_FFFFFF,t_70)

 XML、JSON、protobuf都具有数据结构化和数据序列化的能力。

 XML、JSON更注重数据结构化，关注可读性和语义表达能力；protobuf 更注重数据序列化，关注效率、空间、速度，可读性较差，语义表达能力不足。

 protobuf的应用场景更为明确，XML、JSON的应用场景更为丰富。


### 3.示例

#### 1）**message.proto:**

```protobuf
syntax = "proto3"; //指定版本信息，不指定会报错
message Person  //message为关键字，作用为定义一种消息类型
{
   string name = 1;    //姓名
   int32 id = 2;       //id
   string email = 3;   //邮件
}
```

#### 2）**protoc编译器**

  使用proto文件定义好结构数据后，**可以使用protoc编译器生成结构数据的源代码**，这些源代码提供了读写结构数据的接口，从而能够构造、初始化、读取、序列化、反序列化结构数据。使用以下命令生成相应的接口代码：

```bash
// $SRC_DIR: .proto所在的源目录
// --cpp_out: 生成C++代码
// $DST_DIR: 生成代码的目标目录
// xxx.proto: 要针对哪个proto文件生成接口代码

protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/xxx.proto
```

编译完成后将会生成一个xxx.pb.h和xxx.pb.cpp文件，会提供类似SerializeToOstream()、set_name()、name()等方法。

#### 3）**调用接口进行序列化、反序列化**

```c++
/*
下面的代码即为protoc编译器生成的原结构数据的接口，
提供了构造函数、初始化、序列化、反序列化和读取数据的方法，
因此可以调用这些接口进行序列化与反序列化。
*/

// 构造函数
Person person;
// 初始化
person.set_name("John Doe");
person.set_id(1234);
person.set_email("jdoe@example.com");
fstream output("myfile", ios::out | ios::binary);
// 序列化结构数据到文件中
person.SerializeToOstream(&output);

fstream input("myfile", ios::in | ios::binary);
Person person;
// 从文件中反序列化出结构数据
person.ParseFromIstream(&input);
// 读取结构数据
cout << "Name: " << person.name() << endl;
cout << "E-mail: " << person.email() << endl;
```



## 二、序列化与反序列化API

提供的序列化和反序列化的API接口函数：

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

## 三、vsomeip（protobuf）

### 1.原理

**发送**

在服务端或客户端进行message的序列化，将序列化数据放入负载payload中，发送二进制数据。

**接收**

在服务端或者客户端接收到负载携带的二进制数据，先将数据通过对应API反序列化，进行数据操作。

### 2.示例

**（.proto文件如二中所示）**

**文件目录**

├── _bin |   

​            ├── test1

​            └── test2

├── _CMakeList.txt|   

├── _message.proto|   

├── _src |   

​            ├── test1.cpp

​            └── test2.cpp

​            ├── message.pb.cc

​            └── message.pb.h

#### 1）test1.cpp（客户端）

```c++
#include <iostream>
#include "message.pb.h"
//using namespace std;

#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
#include <csignal>
#endif
#include <vsomeip/vsomeip.hpp>
#if defined ANDROID || defined __ANDROID__
#else
#include <cstdio>
#define LOG_INF(...) fprintf(stdout, __VA_ARGS__), fprintf(stdout, "\n")
#define LOG_ERR(...) fprintf(stderr, __VA_ARGS__), fprintf(stderr, "\n")
#endif

static vsomeip::service_t service_id = 0x1111;
static vsomeip::instance_t service_instance_id = 0x2222;
static vsomeip::method_t service_method_id = 0x3333;

class request_client {
public:
    //获取 vSomeIP 运行时并
    //通过运行时创建应用程序，我们可以传递应用程序名称
    //否则通过 VSOMEIP_APPLICATION_NAME 提供的名称
    // 使用环境变量
    request_client() :
                    rtm_(vsomeip::runtime::get()),
                    app_(rtm_->create_application())
    {
    }

    bool init(){
        //初始化应用程序
        if (!app_->init()) {
            LOG_ERR ("Couldn't initialize application");
            return false;
        }
    
        //注册事件处理程序被调用在注册后回来
        //运行成功
        app_->register_state_handler(
                std::bind(&request_client::on_state_cbk, this,
                        std::placeholders::_1));
    
       //注册一个回调，它在服务可用时
        app_->register_message_handler(vsomeip::ANY_SERVICE,
                service_instance_id, vsomeip::ANY_METHOD,
                std::bind(&request_client::on_message_cbk, this,
                        std::placeholders::_1));
    
        //为来自服务的响应注册回调
        app_->register_availability_handler(service_id, service_instance_id,
                std::bind(&request_client::on_availability_cbk, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3));
        return true;
    }
    
    void start()
    {
       //启动应用程序并等待对于所述ON_EVENT回调被调用
        //该方法只有当app_->stop（）被调用
        app_->start();
    }
    
    void on_state_cbk(vsomeip::state_type_e _state)
    {
        if(_state == vsomeip::state_type_e::ST_REGISTERED)
        {
             //我们在运行时注册的，现在我们可以请求服务
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
    
            std::string str;
                GOOGLE_PROTOBUF_VERIFY_VERSION;
            //序列化
            Person obj;
            obj.set_name("ygd");
            obj.set_id(1);
            *obj.mutable_email() = "123@qq.com";
            obj.SerializeToString(&str);
            //创建一个将发送到服务的负载
            //发送需要回复
            std::shared_ptr<vsomeip::payload> pl = rtm_->create_payload();
           //std::string str("");
            std::vector<vsomeip::byte_t> pl_data(std::begin(str), std::end(str));
            pl->set_data(pl_data);
            rq->set_payload(pl);
    
            //向服务发送请求。响应将被传递到
            //注册的消息处理程序
            LOG_INF("Sending: %s", str.c_str());
            app_->send(rq);
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
            LOG_INF("Received: %s", resp.c_str());
            stop();
        }
    }
    
    void stop()
    {
        //取消注册事件处理程序
        app_->unregister_state_handler();
        //取消注册消息处理程序
        app_->unregister_message_handler(vsomeip::ANY_SERVICE,
                service_instance_id, vsomeip::ANY_METHOD);
        // alternatively unregister all registered handlers at once
        app_->clear_all_handler();
        // release the service
        app_->release_service(service_id, service_instance_id);
        //关闭应用程序
        app_->stop();
    }

private:
    std::shared_ptr<vsomeip::runtime> rtm_;
    std::shared_ptr<vsomeip::application> app_;
};


#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
request_client *hw_cl_ptr(nullptr);
    void handle_signal(int _signal) {
        if (hw_cl_ptr != nullptr &&
                (_signal == SIGINT || _signal == SIGTERM))
            hw_cl_ptr->stop();
    }
#endif

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

  request_client hw_cl;
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
    hw_cl_ptr = &hw_cl;
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
#endif
    if (hw_cl.init()) {
        hw_cl.start();
        return 0;
    } else {
        return 1;
    }
     google::protobuf::ShutdownProtobufLibrary();
}
```

#### 2）test2.cpp（服务端）

```c++
#include <iostream>
#include "message.pb.h"
//using namespace std;
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
#include <csignal>
#endif
#include <vsomeip/vsomeip.hpp>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
//互斥锁，同步机制来确保每次只有一个线程使用资源
#if defined ANDROID || defined __ANDROID__

#else
#include <cstdio>
#define LOG_INF(...) fprintf(stdout, __VA_ARGS__), fprintf(stdout, "\n")
#define LOG_ERR(...) fprintf(stderr, __VA_ARGS__), fprintf(stderr, "\n")
#endif



static vsomeip::service_t service_id = 0x1111;
static vsomeip::instance_t service_instance_id = 0x2222;
static vsomeip::method_t service_method_id = 0x3333;

class request_service {
public:
    //获取 vSomeIP 运行时并
    //通过运行时创建应用程序，我们可以传递应用程序名称
    //否则通过 VSOMEIP_APPLICATION_NAME 提供的名称
    // 使用环境变量
    request_service() :
                    rtm_(vsomeip::runtime::get()),
                    app_(rtm_->create_application()),
                    stop_(false),
                    stop_thread_(std::bind(&request_service::stop, this))
    {
    }

    ~request_service()
    {
        stop_thread_.join();
    }

    bool init()
    {
       //初始化应用程序n
        if (!app_->init()) {
            LOG_ERR("Couldn't initialize application");
            return false;
        }

        //注册消息处理程序回调为发送到我们的服务消息e
        app_->register_message_handler(service_id, service_instance_id,
                service_method_id,
                std::bind(&request_service::on_message_cbk, this,
                        std::placeholders::_1));

       //注册事件处理程序被调用在注册后返回
        //运行时是成功的l
        app_->register_state_handler(
                std::bind(&request_service::on_state_cbk, this,
                        std::placeholders::_1));
        return true;
    }

    void start()
    {
        //启动应用程序并等待对于所述ON_EVENT回调被调用
        //该方法只有当app_->stop（）时被调用d
        app_->start();
    }

    void stop()
    {
        std::unique_lock<std::mutex> its_lock(mutex_);
        while(!stop_) {
            condition_.wait(its_lock);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
         //停止提供服务
        app_->stop_offer_service(service_id, service_instance_id);
        //取消注册事件处理程序
        app_->unregister_state_handler();
        //注销消息处理程序
        app_->unregister_message_handler(service_id, service_instance_id,
                service_method_id);
        //关闭应用程序
        app_->stop();
    }

    void terminate() {
        std::lock_guard<std::mutex> its_lock(mutex_);
        stop_ = true;
        condition_.notify_one();
    }

    void on_state_cbk(vsomeip::state_type_e _state)
    {
        if(_state == vsomeip::state_type_e::ST_REGISTERED)
        {
           //我们在运行时注册，可以提供我们的服务
            app_->offer_service(service_id, service_instance_id);
        }
    }

    void on_message_cbk(const std::shared_ptr<vsomeip::message> &_request)
    {
         //根据请求创建响应
        std::shared_ptr<vsomeip::message> resp = rtm_->create_response(_request);
          GOOGLE_PROTOBUF_VERIFY_VERSION;
        //反序列化
       std::string str1=reinterpret_cast<const char*>(_request->get_payload()->get_data());
        Person obj2;
        obj2.ParseFromString(str1);
        std::cout << "name = " << obj2.name() << std::endl;
        std::cout << "id = " << obj2.id() << std::endl;
        std::cout << "email = " << obj2.email() << std::endl;

       //构造字符串以发回
        //reinterpret_cast<const char*>为强制转换，参考附录
        std::string str("success");
        //str.append(
        //       reinterpret_cast<const char*>(_request->get_payload()->get_data()),
        //     0, _request->get_payload()->get_length());

         //创建一个将被发送回客户端的负载
        std::shared_ptr<vsomeip::payload> resp_pl = rtm_->create_payload();
        std::vector<vsomeip::byte_t> pl_data(str.begin(), str.end());
        resp_pl->set_data(pl_data);
        resp->set_payload(resp_pl);

         //将响应发送回
        app_->send(resp);
         //我们已经完成了 stop now 
        terminate();
    }

private:
    std::shared_ptr<vsomeip::runtime> rtm_;
    std::shared_ptr<vsomeip::application> app_;
    bool stop_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread stop_thread_;
};


#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
request_service *hw_srv_ptr(nullptr);
    void handle_signal(int _signal) {
        if (hw_srv_ptr != nullptr &&
                (_signal == SIGINT || _signal == SIGTERM))
            hw_srv_ptr->terminate();
    }
#endif

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    request_service hw_srv;
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
    hw_srv_ptr = &hw_srv;
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
#endif
    if (hw_srv.init()) {
        hw_srv.start();
        return 0;
    } else {
        return 1;
    }
     google::protobuf::ShutdownProtobufLibrary();

}

```

#### 3）CMakeList.txt

```bash
cmake_minimum_required(VERSION 3.5)

project(vSomeIPHelloWorld)
find_package(Threads REQUIRED)

include_directories("/usr/local/include/vsomeip")
link_directories("/usr/local/lib")
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

add_executable (test1 test1.cpp message.pb.cc)
target_link_libraries(test1  vsomeip3 pthread ${PROTOBUF_LIBRARIES} )
add_executable (test2 test2.cpp message.pb.cc)
target_link_libraries(test2  vsomeip3 pthread ${PROTOBUF_LIBRARIES} )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

```

### 3.序列化信息与负载

#### 1）客户端

```c++
      std::string str;
                GOOGLE_PROTOBUF_VERIFY_VERSION;
            //序列化
            Person obj;
            obj.set_name("ygd");
            obj.set_id(1);
            *obj.mutable_email() = "123@qq.com";
            obj.SerializeToString(&str);
            //创建一个将发送到服务的负载
            //发送需要回复
            std::shared_ptr<vsomeip::payload> pl = rtm_->create_payload();
           //std::string str("");
            std::vector<vsomeip::byte_t> pl_data(std::begin(str), std::end(str));
            pl->set_data(pl_data);
            rq->set_payload(pl);
```

`obj.SerializeToString(&str)`将序列化信息生成二进制字符串流 

`std::vector<vsomeip::byte_t> pl_data(std::begin(str), std::end(str))`将字符串流放入负载

#### 2）服务端

```c++
        //根据请求创建响应
        std::shared_ptr<vsomeip::message> resp = rtm_->create_response(_request);
          GOOGLE_PROTOBUF_VERIFY_VERSION;
        //反序列化
       std::string str1=reinterpret_cast<const char*>(_request->get_payload()->get_data());
        Person obj2;
        obj2.ParseFromString(str1);
        std::cout << "name = " << obj2.name() << std::endl;
        std::cout << "id = " << obj2.id() << std::endl;
        std::cout << "email = " << obj2.email() << std::endl;

```

`std::string str1=reinterpret_cast<const char*>(_request->get_payload()->get_data())`将接受负载消息转化为字符串流

 `obj2.ParseFromString(str1)`将序列化字符串流反序列化

