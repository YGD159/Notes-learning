# C++深入

## 1.什么是C++？

C++是C语言的升级版，在C的基础上增加了很多功能，是一种高级语言。

## 2.什么是面向对象，什么又是面向过程？

举例：a+b
直接计算a+b就是面向过程。
面向对象就是给a+b穿上了一件衣服。而不是直接计算a+b。

## 3.c++的类就是c++的灵魂

	类大家可以把他看看做c语言结构体的升级版，类的成员不仅可以是变量，也可以是函数。


## 4.如何定义一个类？

```c++
class student
{
public:
    char name[64];
    int age;
};
```

## 5.什么是对象？

对象是类的实例化。

## 6.如何定义一个对象？

直接定义：

```c++
#include <iostream>
using namespace std;
class student
{
public:
    char name[64];
    int age;
};
int main()
{
    student my;//student就是类，my就是对象
    return 0;
}
```


在堆里面定义：

```c++
student *my=new student;
```


删除对象（释放内存）：delete my;这种只能删除在堆里面定义的对象，不能删除直接定义的对象。

## 7.怎么访问类里面的数据？

```c++
#include <iostream>
using namespace std;
class student
{
public:
    char name[64];
    int age;
};
int main()
{
    student my;//student就是类，my就是对象

    student *m=new student;
    my.age=18;
    m->age=19;
    cout<<my.age<<endl;
    cout<<m->age<<endl;

  //  delete m;

    return 0;

}
```


和c语言一样，普通用.指针用->。

## 8.类的函数成员

因为类里面的成员不仅可以是变量，也可以是函数。
第一步：在类里面声明

```c++
#include <iostream>
using namespace std;
class student
{
public:
    char name[64];
    int age;
    void test();
   {
        cout<<1024<<endl;
    }
};
```

在外面调用函数就可以输出数据了；
第二步：在类的外面

```c++
#include <iostream>
using namespace std;
class student
{
public:
    char name[64];
    int age;
    void test();
};

void student::test()//表示属于类的函数，不加的话就会被识别成普通函数。
{

    cout<<2048<<endl;

};
int main()
{
    student my;//student就是类，my就是对象

    student *m=new student;
    my.age=18;
    m->age=19;
    cout<<my.age<<endl;
    cout<<m->age<<endl;
    my.test();
    my.test();
    
    return 0;

}
```




访问函数和访问变量是一样的哦。

## 9.类的访问修饰符

类的访问修饰符就是对类的成员进行权限管理

public：表示函数和变量是公开的，任何人都可以访问
private：表示函数和变量只能在自己的类里面访问自己，不能通过对象来访问。

在类里面定义一个private

```c++
class student
{
public:
    char name[64];
    int age;
    void test();
private:
    int haha;
};
```

不能访问。但可以强行访问，怎么强行访问呢？
可以用间接的方式访问：

```c++
class student
{
public:
    char name[64];
    int age;
    void test();
private:
    int haha;
};

void student::test()//表示属于类的函数，不加的话就会被识别成普通函数。
{
     haha=1000;
    cout<<haha<<endl;

};
```

在test函数里边对haha修改值并输出，在主函数里面直接饮用test就可以了。

```c++
int main()
{
    student my;//student就是类，my就是对象

    student *m=new student;
    my.age=18;
    m->age=19;

  //  my.haha
    my.test();
    cout<<my.age<<endl;
    cout<<m->age<<endl;
    my.test();
    my.test();

    return 0;

}
```

protected：表示函数和变量只能在自己的类里面访问自己，但是可以被派生类来访问的。

## 10.类函数的重载特性

什么是承载特性：
我们可以在类里定义同名的函数，但是参数类型不能相同。

```c++
class student
{
public:
    char *name;
    int age;
    void test()
    {
        cout<<1234<<endl;
    }
    void test(int a);
private:
    int num;
};
```


这样是不会报错的。重载函数在调用的时候回根据参数的类型，然后去调用相应的函数；
比如这样：

```c++
#include <iostream>

using namespace std;

class student
{
public:
    char *name;
    int age;
    void test();
    void test(int a);
private:
    int num;
};
void student::test()
{
    cout<<"this is test()"<<endl;
}
void student::test(int a)
{
    cout<<"this is test(int a)"<<endl;
}
int main()
{
    student my;
    int a=10;
    student *mm=new student;


    my.test();
    my.test(a);


    cout << "Hello World!" << endl;
    return 0;

}
```



## 11.构造函数和析构函数

析构函数：假如我们定义了析构函数，当对象被删除或者生命周期结束后就会触发触发析构函数。
构造函数：假如我们定义了构造函数就会触发这个构造函数。
我们要怎么定义析构函数和构造函数呢？

1.析构造函数和构造函数名字和类名一模一样。
2.析构函数要在前面添加一个’~’。

```c++
#include <iostream>

using namespace std;

class student
{
public:
    student();
    ~student();
    char *name;
    int age;
    void test();
    void test(int a);
private:
    int num;
};
void student::test()
{
    cout<<"this is test()"<<endl;
}
void student::test(int a)
{
    cout<<"this is test(int a)"<<endl;
}
student::student()
{

    cout<<"student"<<endl;

}
student::~ student()
{

    cout<<"bye"<<endl;

}
int main()
{
    student my;
    int a=10;
    student *mm=new student;


    my.test();
    my.test(a);


    cout << "Hello World!" << endl;
    delete mm;
    return 0;

}


```

这两个函数根据自己要求决定用还是不用。

构造函数是可以被重载的。
析构函数不能重载。

## 12.类的继承

什么是类的继承？
类的继承允许我们在新的类里面继承父类的public还有protected部分，private不能继承额。当我们觉得这个类不好的时候就可以使用继承。

格式：

```c++
class 儿子: public 爸爸()
{
	public :
			......
	protected:
			.......
}
```

在子类访问父类的成员也是用过.和->访问的。


```
#include <iostream>

using namespace std;

class student
{
public:
    student();
    ~student();
    char *name;
    int age;
    void test();
    void test(int a);
private:
    int num;
};

class mystudent:public student
{
public:
    int grade;
private:
    int haha;
};

void student::test()
{
    cout<<"this is test()"<<endl;
}
void student::test(int a)
{
    cout<<"this is test(int a)"<<endl;
}
student::student()
{

    cout<<"student"<<endl;

}
student::~ student()
{

    cout<<"bye"<<endl;

}
int main()
{
    student my;
    mystudent newa;
    int a=10;
    student *mm=new student;

    newa.grade=99;
    newa.age=33;
    my.test();
    my.test(a);
    
    cout<<newa.age<<endl;
    cout<<newa.grade<<endl;
    cout << "Hello World!" << endl;
    delete mm;
    return 0;

}
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210609145654574.png)

## 13.虚函数和纯虚函数

虚函数：有实际定义的，允许派生类对他进行覆盖式的替换，virtual来修饰。
纯虚函数：没有实际定义的虚函数就是纯虚函数。
怎么定义一个虚函数呢？

用virtual来修饰的，虚函数是用在类的继承上的。

上面的为虚函数，因为刚才我们test函数里面是是有代码的、有实际定义的，二下面的就是纯虚函数，没有代码啥的。

虚函数的优点：

可以预留接口实现分工合作。
