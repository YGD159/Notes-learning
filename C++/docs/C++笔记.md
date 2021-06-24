# C++笔记

## 函数

```
setw(n)
```

n 表示宽度，用数字表示。

setw() 函数只对紧接着的输出产生作用。

**当后面紧跟着的输出字段长度小于 n 的时候，在该字段前面用空格补齐，当输出字段长度大于 n 时，全部整体输出。**

![img](../images/cpp-setw-20200922-RUNOOB.svg)

```cpp
setfill (char_type c);
```

- `c` − 流的新填充字符。`char_type`是流使用的字符类型（即，它的第一个类模板参数`charT`）。



```cpp
#include <iostream>
#include <iomanip>

int main () {
   std::cout << std::setfill ('y') << std::setw (10);
   std::cout << 77 << std::endl;
   return 0;
}
```

编译和运行上面的程序，将产生以下结果 -

```cpp
yyyyyyyy77
```



```
std::dec, std::hex, std::oct #进制输出
```

```
#include <iostream>
#include <sstream>
#include <bitset>
int main()
{
    std::cout << "The number 42 in octal:   " << std::oct << 42 << '\n'
              << "The number 42 in decimal: " << std::dec << 42 << '\n'
              << "The number 42 in hex:     " << std::hex << 42 << '\n';
    int n;
    std::istringstream("2A") >> std::hex >> n;
    std::cout << std::dec << "Parsing \"2A\" as hex gives " << n << '\n';
    // the output base is sticky until changed
    std::cout << std::hex << "42 as hex gives " << 42
        << " and 21 as hex gives " << 21 << '\n';
 
    // Note: there is no I/O manipulator that sets up a stream to print out
    // numbers in binary format (e.g. bin). If binary output is necessary
    // the std::bitset trick can be used:
    std::cout << "The number 42 in binary:  " << std::bitset<8>{42} << '\n';
}
```

Output:

```
The number 42 in octal:   52
The number 42 in decimal: 42
The number 42 in hex:     2a
Parsing "2A" as hex gives 42
42 as hex gives 2a and 21 as hex gives 15
The number 42 in binary:  00101010
```



C++11标准库提供的这两种智能指针的区别在于管理底层指针的方式：shared_ptr允许多个指针指向同一个对象；unique_ptr则"独占"所指向的对象