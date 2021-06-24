## Protobuf语法风格

### 1.代码风格

每一行的代码长度不要超过80。

使用两个空格进行缩进。

### 2.文件格式

  文件命名应该采用蛇形命名法（即用下划线连接），如：lower_snake_case.proto。所有文件应以下列方式排列：

```
License header (if applicable)
File overview
Syntax
Package
Imports (sorted)
File options
Everything else
```

#### 3.包

  包名应该是小写的，并且应该对应于目录层次结构。例如，如果一个文件位于my/Package/中，那么包名应该是my.Package。

#### 4.消息类型和字段

  消息名使用驼峰命名法，例如：SongServerRequest，字段名和扩展名使用小写的下划线分隔式，例如：song_name。

```protobuf
message SongServerRequest {
  required string song_name = 1;
}
```

```protobuf
const string& song_name() { ... }
void set_song_name(const string& x) { ... }
```



如果字段名包含数字，则该数字应出现在字母之后，而不是下划线之后。例如：song_name1。

#### 5.repeated字段

  repeated字段使用复数命名：

```protobuf
repeated string keys = 1;
repeated MyMessage accounts = 17;
```



#### 6.枚举类型

  枚举名使用使用驼峰命名法，成员使用大写的下划线分隔式：

```protobuf
enum Foo {
  FOO_UNSPECIFIED = 0;
  FOO_FIRST_VALUE = 1;
  FOO_SECOND_VALUE = 2;
}
```

#### 7.服务

  服务名称和任何RPC方法名称均使用驼峰命名法：

```protobuf
service FooService {
  rpc GetSomething(FooRequest) returns (FooResponse);
}
```

