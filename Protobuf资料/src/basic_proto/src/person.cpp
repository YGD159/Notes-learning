#include <iostream>
#include "message.pb.h"

using namespace std;

int main()
{
    string str;

    GOOGLE_PROTOBUF_VERIFY_VERSION;
//序列化
    Person obj;
    obj.set_name("ygd");
    obj.set_id(1);
    *obj.mutable_email() = "123@qq.com";
    obj.SerializeToString(&str);
//反序列化
    Person obj2;
    obj2.ParseFromString(str);
    cout << "name = " << obj2.name() << endl;
    cout << "id = " << obj2.id() << endl;
    cout << "email = " << obj2.email() << endl;

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}