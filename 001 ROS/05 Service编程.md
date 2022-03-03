## 一、Server - Client模型

双向同步通信机制，ROS中称其为“服务”，传输请求/应答数据，是一个request-reply模型。

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161136775.png" alt="image-20220216113623694" style="zoom: 40%;" />

特点：

*   **“双向”**：这种机制不仅可以发送消息，还存在反馈
*   **“同步”**：在Client发送请求后，它会在原地等待反馈，只有当Server接收处理完请求并完成response反馈，Client才会继续执行。Client等待过程是处于阻塞状态的通信。
*   与话题不同的是，ROS中**只允许有一个节点提供指定命名的服务**。

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161140739.png" style="zoom: 33%;" />

服务通讯建立过程：

*   **Talker注册**。启动Talker，通过1234端口使用RPC向ROS Master注册发布者的信息，包括所提供的服务名。ROS Master会将节点的注册信息加入注册列表中
*   **Listener注册**。启动Listener，同样使用RPC向ROS Master注册订阅者的信息，包括所需要查找的服务名
*   **ROS Master进行信息匹配**。Master根据Listener的订阅信息从注册列表中进行查找，如果没有找到匹配的服务提供者，则等待该服务的提供者加入；如果找到匹配的服务提供者信息，则通过RPC向Listener发送Talker的TCP地址信息
*   **Listener与Talker建立网络连接**。Listener接收到确认信息后，使用TCP尝试与Talker建立网络连接，并且发送服务的请求数据
*   **Talker向Listener发布服务应答数据**。Talker接收到服务请求和参数后，开始执行服务功能，执行完后，向Listener发送应答数据

话题与服务的区别：

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161143319.png" alt="image-20220216114305252" style="zoom:40%;" />

## 二、Hello World — 简单的服务器-客户端实例

### 2.1 服务器实现

1.  步骤：
    *   初始化ROS节点
    *   创建Server实例
    *   循环等待服务请求，进入回调函数
    *   在回调函数中完成服务功能的处理，并反馈应答数据
2.  创建服务器程序文件，编写服务器程序

```shell
cd ~/catkin_ws/src/learning_communication/src
touch string_server.cpp
```

```c++
# include <ros/ros.h>
# include <std_srvs/SetBool.h>

// 服务器的回调函数，输入参数req，输出参数res
bool print(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    // 打印字符串
    if(req.data){
        ROS_INFO("Hello World");
        res.success = true;
        res.message = "Print Successfully";
    }else{
        res.success = false;
        res.message = "Print Failed";
    }

    return true;
}

int main(int argc, char **argv){
    // 节点初始化
    ros::init(argc, argv, "string_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为print_string的服务器，注册回调函数print()
    ros::ServiceServer service = n.advertiseService("print_string", print);

    // 循环等待回调函数
    ROS_INFO("Ready to print hello string.");
    ros::spin();

    return 0;

}

```

### 2.2 客户端实现

1.  步骤：
    *   初始化ROS节点
    *   创建一个客户端实例
    *   发布服务请求数据
    *   等待Server处理之后的应答结果
2.  创建客户端程序文件，编写客户端程序

```shell
cd ~/catkin_ws/src/learning_communication/src
touch string_client.cpp
```

```c++
/**
 * 该例程将请求print_string服务，std_srvs::SetBool
 */

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

int main(int argc, char **argv){
    // ros节点初始化
    ros::init(argc, argv, "string_client");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个客户端，服务消息类型是std_srvs::SetBool
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("print_string");

    // 创建std_srvs::SetBool类型的service消息
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 发布service请求，等待应答结果
    if(client.call(srv)){
        ROS_INFO("Response : [%s] %s", srv.response.success?"True":"False", srv.response.message.c_str());

    }else{
        ROS_ERROR("Failed to call service print_string");
        return 1;
    }

    return 0;
}
```

### 2.3 配置CMakeLists.txt

1.  设置需要编译的代码和生成的可执行文件
2.  设置链接库

```txt
add_executable(string_server src/string_server.cpp)
add_executable(string_client src/string_client.cpp)

target_link_libraries(string_server ${catkin_LIBRARIES})
target_link_libraries(string_client ${catkin_LIBRARIES})
```

### 2.4 编译

```shell
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```

运行：

```shell
roscore 
```

```shell
rosrun learning_communication string_server
```

```shell
rosrun learning_communication string_client
```

![image-20220217103331417](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171033475.png)

## 三、自定义服务数据

### 3.1 自定义服务数据

1.  定义srv文件

    *   在代码目录下创建srv文件夹，在srv文件夹下创建PersonSrv.srv文件

    ```shell
    cd ~/catkin_ws/src/learning_communication
    mkdir srv
    cd srv
    touch PersonSrv.msg
    ```

    *   在PersonSrv.srv自定义服务数据

    ![image-20220217104012087](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171040116.png)

2.  在package.xml中添加功能包依赖

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

3.  **在CMakeList.txt添加编译选项**

    *    **find_package( ……  message_generation)**

         <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162244474.png" alt="image-20220216224414448" style="zoom:80%;" />

    *   **add_service_files(FILES PersonSrv.srv)**

        ![image-20220217104627955](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171046986.png)

    *   **generate_messages(DEPENDENCIES std_msgs)**

        <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162246212.png" alt="image-20220216224608181" style="zoom:80%;" />

    *    **catkin_package(…… message_runtime)**

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162251128.png" alt="image-20220216225117969" style="zoom:67%;" />

4.  回到根目录编译，并source环境变量
    *   可用`rossrv show 消息类型名`查看自定义的PersonSrv消息类型

![image-20220217104848204](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171048230.png)

### 3.2 Service通信中使用自定义服务数据

1.  服务器程序person_server.cpp

```c++
/**
 * 该例程将执行/show_person服务，服务数据类型learning_communication::PersonSrv
 */

#include <ros/ros.h>
#include "learning_communication/PersonSrv.h"

// service回调函数，输入参数req，输出参数res
bool personCallback(learning_communication::PersonSrv::Request  &req,
         			learning_communication::PersonSrv::Response &res)
{
    // 显示请求数据
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age, req.sex);

	// 设置反馈数据
	res.result = "OK";

    return true;
}

int main(int argc, char **argv){
    // ROS节点初始化
    ros::init(argc, argv, "person_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为/show_person的server，注册回调函数personCallback
    ros::ServiceServer person_service = n.advertiseService("/show_person", personCallback);

    // 循环等待回调函数
    ROS_INFO("Ready to show person informtion.");
    ros::spin();

    return 0;
}
```

2.  客户端程序person_client.cpp

```c++
/**
 * 该例程将请求/show_person服务，服务数据类型learning_communication::PersonSrv
 */

#include <ros/ros.h>
#include "learning_communication/PersonSrv.h"

int main(int argc, char **argv){
    // 初始化ROS节点
    ros::init(argc, argv, "person_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/show_person服务后，创建一个服务客户端，连接名为/show_person的service
    ros::service::waitForService("/show_person");
    ros::ServiceClient person_client = node.serviceClient<learning_communication::PersonSrv>("/show_person");

    // 初始化learning_communication::Person的请求数据
    learning_communication::PersonSrv srv;
    srv.request.name = "Tom";
    srv.request.age = 20;
    srv.request.sex = learning_communication::PersonSrv::Request::male;

    // 请求服务调用
    ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

    // 显示服务调用结果
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
}
```

3.  配置CMakeLists.txt中的编译规则

    *   设置需要编译的代码和生成的可执行文件

        ![image-20220217110623186](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171106214.png)

    *   设置链接库

        ![image-20220217110647643](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171106669.png)

    *   添加依赖项

        ![image-20220217110710930](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171107970.png)

4.  回到根目录下编译，并source环境变量

5.  执行效果：

![image-20220217110854349](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171108378.png)