#include <iomanip>
#include <iostream>
#include <sstream>

#include <compat/vsomeip/vsomeip.hpp>

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENTGROUP_ID 0x001
#define SAMPLE_EVENT_ID 0x002

std::shared_ptr< vsomeip::application > app;

void on_message(const std::shared_ptr<vsomeip::message> &_request) {

    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();

     // 获取Payload
    std::stringstream ss;
    for (vsomeip::length_t i=0; i<l; i++) {
        ss << std::setw(2) << std::setfill('0') << std::hex
            << (int)*(its_payload->get_data()+i) << " "; 
            }

    std::cout << "SERVICE: Received message with Client/Session ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "] "
        << ss.str() << std::endl;

    // 发送SAMPLE_METHOD_ID响应
    std::shared_ptr<vsomeip::message> its_response = vsomeip::runtime::get()->create_response(_request);
    its_payload = vsomeip::runtime::get()->create_payload();
    std::vector<vsomeip::byte_t> its_payload_data;
    for (int i=9; i>=0; i--) {
        its_payload_data.push_back(i % 256); }
    its_payload->set_data(its_payload_data);
    its_response->set_payload(its_payload);
    app->send(its_response, true);

    // 触发SAMPLE_EVENT_ID事件，因为Demo没有什么触发机制，所以就偷个懒，直接在发送响应之后触发事件啦:)
    const vsomeip::byte_t its_data[] = { 0x10 };
    its_payload = vsomeip::runtime::get()->create_payload();
    its_payload->set_data(its_data, sizeof(its_data));
    app->notify(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_payload);
}

int main() {

// 创建应用对象
app = vsomeip::runtime::get()->create_application("World");
// 创建事件组，并添加事件组SAMPLE_EVENTGROUP_ID
std::set<vsomeip::eventgroup_t> its_groups;
its_groups.insert(SAMPLE_EVENTGROUP_ID);
// 初始化应用
app->init();
// 注册消息SAMPLE_METHOD_ID的回调
app->register_message_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID, on_message);
// 提供服务
app->offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
// 提供事件SAMPLE_EVENT_ID，最后一个参数为true表示这是一个Field
app->offer_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups, true);
// 启动应用
app->start();
}