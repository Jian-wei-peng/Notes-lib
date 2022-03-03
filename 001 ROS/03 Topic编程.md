## 一、Publish-Subscribe模型

Topic通信是ROS中最重要也是最常用的一种**单向异步通信机制**，传输消息Message。该机制中，消息以**发布/订阅**的方式进行传递，因为每个**Topic的消息类型都是强类型**，所以发布到其上的消息都必须与Topic的消息类型匹配，而且节点只能接收类型匹配的消息。

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202160906852.png" alt="image-20220216090652785" style="zoom: 40%;" />

特点：

* **单向**：数据只能从发布者传输到订阅者，如果订阅者需要传输数据则需要另外开辟一个Topic进行数据传输 
* **异步**：
  * 对接收者来讲，其订阅Topic，只要Message从Topic过来就接收并进行处理，不管是谁发布的
  * 对于发布者而言，只管发布Message到Topic，不管有没有接收者接收Message，也不需要等待接收者的处理反馈
* 发布节点和订阅节点不知道彼此的存在，甚至还可以订阅一个没有任何发布者的话题。简而言之，**信息的生产和消费是分离的**
* **每个话题都有一个唯一的名称，任何节点都可以访问这个话题并通过它发送数据，只要他们有正确的消息类型**
* **系统中可能同时有多个节点发布或者订阅同一个话题的消息**

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202140959150.png" alt="image-20220214095951096" style="zoom:67%;" />

话题通讯建立过程：

1.  **Talker注册**。启动Talker，在启动时，Talker通过端口将其信息注册到Master端，其中包括Talker所发布消息的话题名、节点地址信息等。Master将这些信息加入到一个注册列表中；
2.  **Listener注册**。启动Listener，Listener向Master端注册，注册其所需订阅的话题以及Listener自己的地址信息；
3.  **ROS Master进行信息匹配**。Master根据Listener的订阅信息从注册列表中进行查找，如果没有找到匹配的发布者，则等待发布者的加入。如果找到匹配的发布者信息，则把Talker的地址发送给Listener；
4.  **Listener发送连接请求**。Listener接收到Master发送的Talker地址后，向Talker发送连接请求，同时将Listener要订阅的话题名、消息类型和通信协议（TCP/UDP）全发给Talker；
5.  **Talker发送连接请求**。Talker接收到Listener请求后，返还一个确认连接的信息，包括其自身的TCP地址；
6.  **Listener尝试与Talker建立网络连接**。Listener接收到Talker的TCP地址后，通过TCP与Talker建立网络连接；
7.  **Talker向Listener发布数据**。Talker通过建立的TCP网络连接将话题消息数据发送给Listener。

## 二、Hello World — 编写简单的发布器与订阅器

>   **talker**节点发布信息在话题**chatter**上，消息类型为std_msgs::String，**listener**节点订阅话题**chatter**，接收信息并打印信息

### 2.1 发布者实现

1.  实现步骤：
    *   初始化ROS节点
    *   向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型
    *   创建消息数据
    *   按照一定频率循环发布消息

2.  创建发布者程序文件，编写发布者代码

```shell
cd ~/catkin_ws/src/learning_communication/src
touch string_publisher.cpp
```

```c++
# include "ros/ros.h"
# include "std_msgs/String.h"   // ros内置字符串数据类型
# include <sstream>

int main(int argc, char **argv){
    // 节点初始化
    ros::init(argc, argv, "string_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个发布者，发布到话题chatter上，消息类型为std_msgs::String
    // NodeHandle::advertise()返回一个ros::Publisher对象
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()){
        // 初始化std_msgs::String类型的消息
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello World" << count;
        msg.data = ss.str();

        // 发布消息
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        // 按照循环频率延时
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
```

### 2.2 订阅者实现

1.  实现步骤：
    *   初始化ROS节点
    *   订阅需要的话题
    *   循环等待话题消息，接收到消息后进入回调函数
    *   在回调函数中完成消息处理
2.  创建订阅者程序文件，编写订阅者代码

```shell
cd ~/catkin_ws/src/learning_communication/src
touch string_subscriber.cpp
```

```c++
# include "ros/ros.h"
# include "std_msgs/String.h"

// 接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg){
    // 将接收到的消息打印出来
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
    // 初始化ROS节点
    ros::init(argc, argv, "string_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅节点chatter，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // 循环等待回调函数
    // spin()是死循环，依靠ros::ok()退出
    ros::spin();

    return 0;
}
```

### 2.3 配置CMakeLists.txt

1.  设置需要编译的代码和生成的可执行文件
2.  设置链接库

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162159666.png" alt="image-20220216215959606" style="zoom:67%;" />

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
rosrun learning_communication string_publisher
```

```shell
rosrun learning_communication string_subscriber
```

![image-20220216172817931](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161728959.png)

![image-20220216172748003](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161727042.png)

## 三、自定义话题消息

>   节点person_publisher发布一个person_info的话题，其消息类型为自定义的learning_communication::PersonMsg，节点person_subscriber订阅话题person_info，将信息打印出来

### 3.1 自定义话题消息

1.  定义msg文件

    *   在代码目录下创建msg文件夹，在msg文件夹下创建PersonMsg.msg文件

    ```shell
    cd ~/catkin_ws/src/learning_communication
    mkdir msg
    cd msg
    touch PersonMsg.msg
    ```

    ![image-20220216223347317](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162233360.png)

    *   在PersonMsg.msg中自定义消息类型：姓名、性别、年龄等

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162234395.png" alt="image-20220216223429368" style="zoom:67%;" />

2.  **在package.xml中添加功能包依赖**

    ```xml
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ```

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162253390.png" alt="image-20220216225323355" style="zoom:70%;" />

3.  **在CMakeList.txt添加编译选项**

    *    **find_package( ……  message_generation)**

        <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162244474.png" alt="image-20220216224414448" style="zoom:80%;" />

    *   **add_message_files(FILES PersonMsg.msg)**

        <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162245372.png" alt="image-20220216224541344" style="zoom:80%;" />

    *   **generate_messages(DEPENDENCIES std_msgs)**

        <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162246212.png" alt="image-20220216224608181" style="zoom:80%;" />

    *    **catkin_package(…… message_runtime)**

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162251128.png" alt="image-20220216225117969" style="zoom:67%;" />

4.  回到根目录编译，并source环境变量
    *   可用`rosmsg show 消息类型名`查看自定义的PersonMsg消息类型

![image-20220216230210511](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162302541.png)

### 3.2 Topic通信中使用自定义话题消息类型

1.  发布者程序person_publisher.cpp

```c++
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_communication::PersonMsg，队列长度10
    ros::Publisher person_info_pub = n.advertise<learning_communication::PersonMsg>("/person_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_communication::Person类型的消息
    	learning_communication::PersonMsg person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_communication::PersonMsg::male;

        // 发布消息
		person_info_pub.publish(person_msg);

       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```

2.  订阅者程序person_subscriber.cpp

```C++
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h"

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_communication::PersonMsg::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
```

3.  配置CMakeLists.txt中的编译规则

    *   设置需要编译的代码和生成的可执行文件

        ![image-20220216231356261](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162313287.png)

    *   设置链接库

        ![image-20220216231437910](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162314935.png)

    *   添加依赖项

        ![](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162314679.png)

4.  回到根目录下编译，并source环境变量
5.  执行效果：

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202162311097.png" alt="image-20220216231118053" style="zoom: 80%;" />

