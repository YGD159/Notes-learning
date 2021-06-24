// Copyright (C) 2015-2017 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#include <vsomeip/vsomeip.hpp>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
//互斥锁，同步机制来确保每次只有一个线程使用资源
#if defined ANDROID || defined __ANDROID__
#include "android/log.h"
#define LOG_TAG "request_service"
#define LOG_INF(...) fprintf(stdout, __VA_ARGS__), fprintf(stdout, "\n"), (void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
#define LOG_ERR(...) fprintf(stderr, __VA_ARGS__), fprintf(stderr, "\n"), (void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)
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

       //构造字符串以发回
        //reinterpret_cast<const char*>为强制转换，参考附录
        std::string str("method-response");
        str.append(
                reinterpret_cast<const char*>(_request->get_payload()->get_data()),
                0, _request->get_payload()->get_length());

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
