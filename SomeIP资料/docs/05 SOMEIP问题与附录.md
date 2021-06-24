## 一、SOME/IP问题

EPT为众多客户提供过非常多的协议一致性测试服务，下面我们就简单罗列一些在SOME/IP测试中经常遇到的一些问题：

### (一) UTF格式数据传输

遇到问题：UTF存在定长测试，例如64byte的定长UTF8传输，其BOM与结束符是否应该包含于64byte之内？

解决办法：UTF格式的定长传输，应将BOM和结束符包含在定长长度之中。

### (二) TCP服务订阅

遇到问题：通过TCP传输的服务，客户端直接订阅该服务会被服务器NACK。

解决办法：SOME/IP的客户端与服务器的TCP连接必须由客户端建立，因此客户端订阅TCP服务之前必须主动建立TCP连接。如果TCP链接未建立而直接去订阅，则会收到服务器的NACK。

### (三)  服务器休眠问题

遇到问题：SOME/IP服务器在有休眠需求时会主动断开TCP连接。

解决方法：SOME/IP协议文档规定，TCP连接应由客户端建立，且服务器不应主动断开TCP连接。从网络管理的角度出发，客户端若未断开TCP连接，意味着对服务尚有需求，此时服务器不应该主动断开TCP连接进行休眠。若服务器满足休眠状态，有休眠需求时，应发出stop offer service报文，客户端收到该消息后，需要主动发起断开连接请求，在连接全部断开后，服务器方可进入休眠状态。

### (四) Session ID的处理机制问题

遇到问题：SOME/IP协议栈对Session ID字段的处理逻辑与规范要求不符。

解决方法：SOME/IP协议标准对Session ID有标准定义：Session ID是client用来标记每次call的独特标记；Session ID区分‘通信关系’，如单播和组播；Session ID区分发送和接收方的关系；错误的Session ID会导致有效的SOME/IP报文被丢弃。总结来说，Session ID应该区分不同的sockets分别计数。



## 二、附录

### (一) C++类型转换

强制类型转换是有一定风险的，有的转换并不一定安全，如把整型数值转换成[指针](http://c.biancheng.net/c/80/)，把基类指针转换成派生类指针，把一种函数指针转换成另一种函数指针，把常量指针转换成非常量指针等。C++ 引入新的强制类型转换机制，主要是为了克服C语言强制类型转换的以下三个缺点。

**1) 没有从形式上体现转换功能和风险的不同。**

**2) 将多态基类指针转换成派生类指针时不检查安全性，即无法判断转换后的指针是否确实指向一个派生类对象。**

**3) 难以在程序中寻找到底什么地方进行了强制类型转换。**

C++是兼容C的，因此C语言中的强制类型转换在C++中同样适用，具体使用方法可以参照下面的代码示例：

```cpp
float valueA = 3.0f;
int valueB = (int) valueA;
```

可以看到，C语言中强制类型转换的一般格式为：

#### 1）类型说明符）表达式

实现的功能就是把表达式的值强制转换为类型说明符表示的类型。除了这种强制类型转换方法外，C++还提供了四种类型转换方法，分别为

- **static_cast<类型说明符>(表达式）**
- **dynamic_cast<类型说明符>(表达式）**
- **const_cast<类型说明符>(表达式）**
- **reinterpret_cast<类型说明符>(表达式）**

下面在比较它们的异同时，按照适用范围从窄到宽的顺序介绍，先从使用频率比较低的reinterpret_cast开始，然后依次是const_cast，dynamic_cast，最后介绍static_cast。

#### 1）static_cast

static_cast 用于进行比较“自然”和低风险的转换，如整型和浮点型、字符型之间的互相转换。另外，如果对象所属的类重载了强制类型转换运算符 T（如 T 是 int、int* 或其他类型名），则 static_cast 也能用来进行对象到 T 类型的转换。

static_cast 不能用于在不同类型的指针之间互相转换，也不能用于整型和指针之间的互相转换，当然也不能用于不同类型的引用之间的转换。因为这些属于风险比较高的转换。

static_cast 用法示例如下：

```c++
#include <iostream>
using namespace std;
class A{
    public:    
    operator int() { return 1; }   
    operator char*() { return NULL; }
};
int main()
{   
    A a;   
    int n;   
    char* p = "New Dragon Inn";    
    n = static_cast <int> (3.14);  // n 的值变为 3   
    n = static_cast <int> (a);  //调用 a.operator int，n 的值变为 1    
    p = static_cast <char*> (a);  //调用 a.operator char*，p 的值变为 NULL    
    n = static_cast <int> (p);  //编译错误，static_cast不能将指针转换成整型    
    p = static_cast <char*> (n);  //编译错误，static_cast 不能将整型转换成指针 
    return 0;
}
```

#### 2）reinterpret_cast

##### 1.reinterpret_cast概念

首先从英文字面的意思理解，interpret是“解释，诠释”的意思，加上前缀“re”，就是“重新诠释”的意思；cast在这里可以翻译成“转型”，这样整个词顺下来就是“重新诠释的转型”。我们知道变量在内存中是以“…0101…”二进制格式存储的，一个int型变量一般占用32个位（bit)，参考下面的代码

```cpp
#include <iostream>
using namespace std;
int main(int argc, char** argv)
{
	int num = 0x00636261;//用16进制表示32位int，0x61是字符'a'的ASCII码
	int * pnum = &num;
	char * pstr = reinterpret_cast<char *>(pnum);
	cout<<"pnum指针的值: "<<pnum<<endl;
	cout<<"pstr指针的值: "<<static_cast<void *>(pstr)<<endl;//直接输出pstr会输出其指向的字符串，这里的类型转换是为了保证输出pstr的值
	cout<<"pnum指向的内容: "<<hex<<*pnum<<endl;
	cout<<"pstr指向的内容: "<<pstr<<endl;
	return 0;
}
```

在Ubuntu 14.04 LTS系统下，采用g++ 4.8.4版本编译器编译该源文件并执行，得到的输出结果如下：

![img](/home/ygd/资料/SomeIp资料/images/v2-30703f6f7169c536065765f41092c2b4_720w.jpg)

第6行定义了一个整型变量num，并初始化为0x00636261（十六进制表示），然后取num的地址用来初始化整型指针变量pnum。接着到了关键的地方，使用reinterpret_cast运算符把pnum从int*转变成char*类型并用于初始化pstr。

将pnum和pstr两个指针的值输出，对比发现，两个指针的值是完全相同的，这是因为**“reinterpret_cast 运算符并不会改变括号中运算对象的值，而是对该对象从位模式上进行重新解释”**。如何理解位模式上的重新解释呢？通过推敲代码11行和12行的输出内容，就可见一斑。

很显然，按照十六进制输出pnum指向的内容，得到636261；但是输出pstr指向的内容，为什么会得到”abc”呢？

在回答这个问题之前，先套用《深度探索C++对象模型》中的一段话，“一个指向字符串的指针是如何地与一个指向整数的指针或一个指向其他自定义类型对象的指针有所不同呢？从内存需求的观点来说，没有什么不同！它们三个都需要足够的内存（并且是相同大小的内存）来放置一个机器地址。指向不同类型之各指针间的差异，既不在其指针表示法不同，也不在其内容（代表一个地址）不同，而是在其所寻址出来的对象类型不同。也就是说，指针类型会教导编译器如何解释某个特定地址中的内存内容及其大小。”参考这段话和下面的内存示意图，答案已经呼之欲出了。

<img src="/home/ygd/资料/SomeIp资料/images/v2-b86b1bc33810aa5981fea8b2e5ba0f0e_720w.jpg" alt="img" style="zoom:67%;" />

使用reinterpret_cast运算符把pnum从int*转变成char*类型并用于初始化pstr后，pstr也指向num的内存区域，但是由于pstr是char类型的，通过pstr读写num内存区域将不再按照整型变量的规则，而是按照char型变量规则。一个char型变量占用一个Byte，对pstr解引用得到的将是一个字符，也就是’a’。而在使用输出流输出pstr时，将输出pstr指向的内存区域的字符，那pstr指向的是一个的字符，那为什么输出三个字符呢？这是由于在输出char指针时，输出流会把它当做输出一个字符串来处理，直至遇到’\0’才表示字符串结束。对代码稍做改动，就会得到不一样的输出结果，例如将num的值改为0x63006261,输出的字符串就变为”ab”。

上面的例子融合了一些巧妙的设计，我们在pstr指向的内存区域中故意地设置了结束符’\0’。假如将num的值改为0x64636261，运行结果会是怎样的呢？

![img](/home/ygd/资料/SomeIp资料/images/v2-699e6efadbf8975046378480d21a7fe1_720w.jpg)

上面是我测试的截图，大家可以思考一下为什么在输出”abcd”之后又输出了6个字符才结束呢（提示：参考上面的内存示意图）？

但是在有些情况下，就不会这么幸运了，迎接我们的很可能是运行崩溃。例如我们直接将num（而不是pnum）转型为char*，再运行程序的截图如下

![img](/home/ygd/资料/SomeIp资料/images/v2-f769af70ab0f0a5ad27de14f0eb34a1c_720w.jpg)

可以分析出，程序在输出pstr时崩溃了，这是为什么呢？pstr指向的内存区域的地址是0x64636261，而这片内存区域很有可能并不在操作系统为当前进程分配的虚拟内存空间中，从而导致段错误。

##### 2.reinterpret_cast转换

reinterpret_cast 用于进行各种不同类型的指针之间、不同类型的引用之间以及指针和能容纳指针的整数类型之间的转换。转换时，执行的是逐个比特复制的操作。

这种转换提供了很强的灵活性，但转换的安全性只能由程序员的细心来保证了。例如，程序员执意要把一个 int* 指针、函数指针或其他类型的指针转换成 string* 类型的指针也是可以的，至于以后用转换后的指针调用 string 类的成员函数引发错误，程序员也只能自行承担查找错误的烦琐工作：（C++ 标准不允许将函数指针转换成对象指针，但有些编译器，如 Visual Studio 2010，则支持这种转换）。

reinterpret_cast 用法示例如下：

```c++
#include <iostream>
using namespace std;
class A{
    public:   
    int i;    int j;   
        A(int n):i(n),j(n) { }
};
int main(){    
           A a(100);    
           int &r = reinterpret_cast<int&>(a); //强行让 r 引用 a   
           r = 200;  //把 a.i 变成了 200    
           cout << a.i << "," << a.j << endl;  // 输出 200,100    
           int n = 300;    A *pa = reinterpret_cast<A*> ( & n); //强行让 pa 指向 n    
           pa->i = 400;  // n 变成 400    
           pa->j = 500;  //此条语句不安全，很可能导致程序崩溃    
           cout << n << endl;  // 输出 400    
           long long la = 0x12345678abcdLL;    
           pa = reinterpret_cast<A*>(la); //la太长，只取低32位0x5678abcd拷贝给pa    
           unsigned int u = reinterpret_cast<unsigned int>(pa);//pa逐个比特拷贝到u    
           cout << hex << u << endl;  //输出 5678abcd    
           typedef void (* PF1) (int);    
           typedef int (* PF2) (int,char *);   
           PF1 pf1;  PF2 pf2;    
           pf2 = reinterpret_cast<PF2>(pf1); //两个不同类型的函数指针之间可以互相转换
          }
```

程序的输出结果是：

```c++
200, 100
400
5678abed
```

第 19 行的代码不安全，因为在编译器看来，pa->j 的存放位置就是 n 后面的 4 个字节。 本条语句会向这 4 个字节中写入 500。但这 4 个字节不知道是用来存放什么的，贸然向其中写入可能会导致程序错误甚至崩溃。

上面程序中的各种转换都没有实际意义，只是为了演示 reinteipret_cast 的用法而已。在编写黑客程序、病毒或反病毒程序时，也许会用到这样怪异的转换。

reinterpret_cast体现了 C++ 语言的设计思想：**用户可以做任何操作，但要为自己的行为负责。**

#### 3）const_cast

const_cast 运算符仅用于进行去除 const 属性的转换，它也是四个强制类型转换运算符中唯一能够去除 const 属性的运算符。

将 const 引用转换为同类型的非 const 引用，将 const 指针转换为同类型的非 const 指针时可以使用 const_cast 运算符。例如：

```c++
const string s = "Inception";
string& p = const_cast <string&> (s);
string* ps = const_cast <string*> (&s);  // &s 的类型是 const string*
```

#### 4）dynamic_cast

用 reinterpret_cast 可以将多态基类（包含虚函数的基类）的指针强制转换为派生类的指针，但是这种转换不检查安全性，即不检查转换后的指针是否确实指向一个派生类对象。dynamic_cast专门用于将多态基类的指针或引用强制转换为派生类的指针或引用，而且能够检查转换的安全性。对于不安全的指针转换，转换结果返回 NULL 指针。

dynamic_cast 是通过“运行时类型检查”来保证安全性的。dynamic_cast 不能用于将非多态基类的指针或引用强制转换为派生类的指针或引用——这种转换没法保证安全性，只好用 reinterpret_cast 来完成。

dynamic_cast 示例程序如下：

```c++
#include <iostream>
#include <string>
using namespace std;
class Base{  //有虚函数，因此是多态基类
    public:    
    virtual ~Base() {}
};
class Derived : public Base { };
int main(){    
    Base b;    
    Derived d;    
    Derived* pd;    
    pd = reinterpret_cast <Derived*> (&b);    
    if (pd == NULL)        
        //此处pd不会为 NULL。reinterpret_cast不检查安全性，总是进行转换        
        cout << "unsafe reinterpret_cast" << endl; //不会执行    
    pd = dynamic_cast <Derived*> (&b);   
    if (pd == NULL)  //结果会是NULL，因为 &b 不指向派生类对象，此转换不安全        
        cout << "unsafe dynamic_cast1" << endl;  //会执行   
    pd = dynamic_cast <Derived*> (&d);  //安全的转换    
    if (pd == NULL)  //此处 pd 不会为 NULL        
        cout << "unsafe dynamic_cast2" << endl;  //不会执行    
    return 0;
}
```

程序的输出结果是：

```c++
unsafe dynamic_cast1
```

第 20 行，通过判断 pd 的值是否为 NULL，就能知道第 19 行进行的转换是否是安全的。第 23 行同理。

如果上面的程序中出现了下面的语句：

```c++
Derived & r = dynamic_cast <Derived &> (b);
```

那该如何判断该转换是否安全呢？不存在空引用，因此不能通过返回值来判断转换是否安全。C++ 的解决办法是：dynamic_cast 在进行引用的强制转换时，如果发现转换不安全，就会拋出一个异常，通过处理异常，就能发现不安全的转换。

