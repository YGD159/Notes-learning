#include <iomanip>
#include <iostream>
#include <sstream>
 
#include <condition_variable>
#include <thread>
 
#include <vsomeip/vsomeip.hpp>

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENTGROUP_ID 0x001
#define SAMPLE_EVENT_ID 0x002

std::shared_ptr< vsomeip::application > app;
std::mutex mutex;
std::condition_variable condition;

void run() {
    std::unique_lock<std::mutex> its_lock(mutex);
    condition.wait(its_lock);
   // 服务可用后，进行一系列常规操作：
    // 订阅事件组SAMPLE_EVENTGROUP_ID和事件SAMPLE_EVENT_ID

    std::set<vsomeip::eventgroup_t> its_groups;
    its_groups.insert(SAMPLE_EVENTGROUP_ID);
    app->request_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups);
    app->subscribe(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENTGROUP_ID);

    // 发送SAMPLE_METHOD_ID请求
    std::shared_ptr< vsomeip::message > request;
    request = vsomeip::runtime::get()->create_request();
    request->set_service(SAMPLE_SERVICE_ID);
    request->set_instance(SAMPLE_INSTANCE_ID);
    request->set_method(SAMPLE_METHOD_ID);

    std::shared_ptr< vsomeip::payload > its_payload = vsomeip::runtime::get()->create_payload();
    std::vector< vsomeip::byte_t > its_payload_data;
    for (vsomeip::byte_t i=0; i<10; i++) {
    its_payload_data.push_back(i % 256);
    }
    its_payload->set_data(its_payload_data);
    request->set_payload(its_payload);
    app->send(request);
}
//接收所有消息的回调函数：
void on_message(const std::shared_ptr<vsomeip::message> &_response) {
    std::stringstream its_message;
    its_message << "CLIENT: received a notification for event ["
            << std::setw(4) << std::setfill('0') << std::hex
            << _response->get_service() << "."
            << std::setw(4) << std::setfill('0') << std::hex
            << _response->get_instance() << "."
            << std::setw(4) << std::setfill('0') << std::hex
            << _response->get_method() << "] to Client/Session ["
            << std::setw(4) << std::setfill('0') << std::hex
            << _response->get_client() << "/"
            << std::setw(4) << std::setfill('0') << std::hex
            << _response->get_session()
            << "] = ";
    std::shared_ptr<vsomeip::payload> its_payload = _response->get_payload();
    its_message << "(" << std::dec << its_payload->get_length() << ") ";
    for (uint32_t i = 0; i < its_payload->get_length(); ++i)
        its_message << std::hex << std::setw(2) << std::setfill('0')
            << (int) its_payload->get_data()[i] << " ";
    std::cout << its_message.str() << std::endl;

    // std::shared_ptr<vsomeip::payload> its_payload = _response->get_payload();
    // vsomeip::length_t l = its_payload->get_length();

    // // Get payload
    // std::stringstream ss;
    // for (vsomeip::length_t i=0; i<l; i++) {
    //     ss << std::setw(2) << std::setfill('0') << std::hex
    //         << (int)*(its_payload->get_data()+i) << " ";
    // }

    // std::cout << "CLIENT: Received message with Client/Session ["
    //           << std::setw(4) << std::setfill('0') << std::hex << _response->get_client() << "/"
    //           << std::setw(4) << std::setfill('0') << std::hex << _response->get_session() << "] "
    //           << ss.str() << std::endl;
}

//监听服务是否可用的回调函数：
void on_availability(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
    std::cout << "CLIENT: Service ["
              << std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
              << "] is "
              << (_is_available ? "available." : "NOT available.")
              << std::endl;
    // 服务可用了，可以去发送请求啦:)
    if (_is_available) { condition.notify_one(); }
}

int main() {

// 同样地，创建应用对象
app = vsomeip::runtime::get()->create_application("Hello");
// 初始化应用
app->init();
// 注册服务是否可用的回调
app->register_availability_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, on_availability);
// 请求服务
app->request_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
// 注册接收所有消息的回调
app->register_message_handler(vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD, on_message);
// 发送请求线程
std::thread sender(run);
// 启动应用
app->start();
}
