// Copyright (C) 2015-2017 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#include <vsomeip/vsomeip.hpp>

#if defined ANDROID || defined __ANDROID__
#include "android/log.h"
#define LOG_TAG "request_client"
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


            //创建一个将发送到服务的负载
            //发送需要回复
            std::shared_ptr<vsomeip::payload> pl = rtm_->create_payload();
            std::string str("method-request");
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
