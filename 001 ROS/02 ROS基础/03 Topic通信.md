## 一、Publish-Subscribe模型

​		Topic通信是ROS中最重要也是使用频率最高的一种通信机制，是一种**单向异步通信机制**，传输消息Message。该机制中，消息以**发布/订阅**的方式进行传递，因为每个**Topic的消息类型都是强类型**，所以发布到其上的消息都必须与Topic的消息类型匹配，而且节点只能接收类型匹配的消息。

​		**常用于不断更新的、少逻辑处理的数据传输场景**

- 导航中激光雷达信息的采集处理：
	- 需要一个节点实时发布当前雷达采集到的数据
	- 导航模块中会有另一个节点订阅并解析雷达数据
- 机器人运动控制指令：
	- 导航模块会根据传感器采集的数据实时的计算出运动控制信息并发布给底盘
	- 底盘可以有一个节点订阅运动信息并最终转换成控制电机的脉冲信号

特点：

* **单向**：数据只能从发布者传输到订阅者，如果订阅者需要传输数据则需要另外开辟一个Topic进行数据传输 
* **异步**：
	* 对接收者来讲，其订阅Topic，只要Message从Topic过来就接收并进行处理，不管是谁发布的
	* 对于发布者而言，只管发布Message到Topic，不管有没有接收者接收Message，也不需要等待接收者的处理反馈
* 发布节点和订阅节点不知道彼此的存在，甚至还可以订阅一个没有任何发布者的话题。简而言之，**信息的生产和消费是分离的**
* **每个话题都有一个唯一的名称，任何节点都可以访问这个话题并通过它发送数据，只要他们有正确的消息类型**
* **系统中可能同时有多个节点发布或者订阅同一个话题的消息**

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205280055370.png" alt="image-20220528005526297" style="zoom: 40%;" />

**话题通讯建立过程**：

1.  **Talker注册**。启动Talker，在启动时，Talker通过端口将其信息注册到Master端，其中包括Talker所发布消息的话题名、节点地址信息等。Master将这些信息加入到一个注册列表中；
2.  **Listener注册**。启动Listener，Listener向Master端注册，注册其所需订阅的话题以及Listener自己的地址信息；
3.  **ROS Master进行信息匹配**。Master根据Listener的订阅信息从注册列表中进行查找，如果没有找到匹配的发布者，则等待发布者的加入。如果找到匹配的发布者信息，则把Talker的地址发送给Listener；
4.  **Listener发送连接请求**。Listener接收到Master发送的Talker地址后，向Talker发送连接请求，同时将Listener要订阅的话题名、消息类型和通信协议（TCP/UDP）全发给Talker；
5.  **Talker发送连接请求**。Talker接收到Listener请求后，返还一个确认连接的信息，包括其自身的TCP地址；
6.  **Listener尝试与Talker建立网络连接**。Listener接收到Talker的TCP地址后，通过TCP与Talker建立网络连接；
7.  **Talker向Listener发布数据**。Talker通过建立的TCP网络连接将话题消息数据发送给Listener。

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205280059463.jpeg" alt="img" style="zoom:50%;" />

**注意：**

- **Talker和Listener的启动无先后顺序要求**
- **Talker和Listener都可以有多个**
- **Talker和Listener连接建立后，不再需要ROS Master。即关闭ROS Master，Talker与Listern照常通信**

## 二、Hello World（简单的Topic通信）

> **talker**节点以10Hz的频率发布信息在话题**chatter**上，消息类型为std_msgs::String，**listener**节点订阅话题**chatter**，接收信息并打印信息

### 2.1  创建工作空间以及功能包

1. 创建工作空间

```shell
mkdir -p ~/catkin_ws/src
cd ~/carkin_ws/src
catkin_init_workspace
```

2. 编译工作空间

```shell
cd ~/catkin_ws/
catkin_make
```

3. 设置环境变量

```shell
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. 创建功能包

```shell
cd ~/catkin_ws/src
catkin_create_pkg learning_communication rospy roscpp std_msgs std_srvs
```

5. 编译功能包

```shell
cd ~/catkin_ws/
catkin_make
source ~/.bashrc
```

### 2.2 publisher实现

主要步骤：

1. **引用最基础的头文件**：`#include <ros/ros.h>`

2. **初始化节点**：

```c++
ros::init(argc, argv, "节点名称");
// 参数1和参数2 后期为节点传值会使用
// 参数3 是节点的名称，必须是唯一的，且名称必须是基本名称（不可包含任何斜杠/）
// 该语句使得ROS可以通过命令行进行名称重映射
```

3. **实例化ros句柄**：

```c++
ros::NodeHandle 句柄名;
// 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源
```

4. **实例化 发布者 对象**：

```c++
ros::Publisher 发布者名 = 句柄名.advertise<话题类型>("话题名称", 发布队列大小);
// 告诉主节点将要在 “话题名称” 话题上发布一个类型为 <话题类型> 的消息。这会让主节点告诉任何正在监听 “话题名称” 的节点，我们将在这一话题上发布数据
```

5. **组织被发布的数据，并编写逻辑发布数据**

```c++
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>

int main(int argc, char  **argv)
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用；参数3 是节点名称，是一个标识符，需要保证运行后，在ROS网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄。该类封装了 ROS 中的一些常用功能
    ros::NodeHandle nh;

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型；参数1: 要发布到的话题；参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    std_msgs::String msg;
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(1);

    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
    }
    return 0;
}
```

### 2.3 subscriber实现

主要步骤：

1. **引用最基础的头文件**：`#include <ros/ros.h>`

2. **初始化节点**：

```c++
ros::init(argc, argv, "节点名称");
// 参数1和参数2 后期为节点传值会使用
// 参数3 是节点的名称，必须是唯一的，且名称必须是基本名称（不可包含任何斜杠/）
// 该语句使得ROS可以通过命令行进行名称重映射
```

3. **实例化ros句柄**：

```c++
ros::NodeHandle 句柄名;
// 创建的第一个NodeHandle实际上将执行节点的初始化，而最后一个被销毁的NodeHandle将清除节点所使用的任何资源
```

4. **实例化 订阅者 对象，定义回调函数，用于处理订阅的消息**：

```c++
ros::Subscriber 发布者名 = 句柄名.subscribe<话题类型>("话题名称", 发布队列大小, 回调函数名称);
// 告诉主节点将要在 “话题名称” 话题上发布一个类型为 <话题类型> 的消息。这会让主节点告诉任何正在监听 “话题名称” 的节点，我们将在这一话题上发布数据
```

5. **设置循环调用回调函数**：

```c++
ros::spin();
//循环读取接收的数据，并调用回调函数处理
```

```c++
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h"

void doMsg(const std_msgs::String::ConstPtr& msg_p){
    ROS_INFO("我听见:%s",msg_p->data.c_str());
}

int main(int argc, char  **argv)
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"listener");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter",10,doMsg);
    //5.处理订阅的消息(回调函数)

    //6.设置循环调用回调函数
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}
```

### 2.4 修改CMakeList.txt文件

```txt
add_executable(hello_publisher
  src/hello_publisher.cpp
)
add_executable(hello_subscriber
  src/hello_subscriber.cpp
)

target_link_libraries(hello_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(hello_subscriber
  ${catkin_LIBRARIES}
)
```

### 2.5 编译

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
rosrun learning_communication hello_publisher
```

```shell
rosrun learning_communication hello_subscriber
```

## 三、自定义msg消息

​		ROS 中通过 std_msgs 封装了一些原生的数据类型,比如:String、Int32、Int64、Char、Bool、Empty.... 但是，这些数据一般只包含一个 data 字段，结构的单一意味着功能上的局限性，当传输一些复杂的数据，比如: 激光雷达的信息... std_msgs 由于描述性较差而显得力不从心，这种场景下可以使用自定义的消息类型。

实例：

- 需求：创建自定义消息，该消息包含人的信息:姓名、身高、年龄等
- 流程：
	1. 按照固定格式创建 msg 文件
	2. 编辑配置文件
	3. 编译生成可以被 Python 或 C++ 调用的中间文件

### 3.1 自定义msg

**步骤一、创建msg文件**

- 目录路径：～/catkin_ws/src/learning_communication/
- 在learning_communication下建立一个msg文件夹，并且新建一个Person.msg文件
	- ～/catkin_ws/src/learning_communication/msg/Person.msg

```msg
string name
uint16 age
float64 height
```

**步骤二、编辑配置文件**

1. **package.xml中添加编译依赖与执行依赖**
   
   ```xml
   <build_depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```
   
2. **CMakeLists.txt编辑msg相关配置**
   
    ```
    # 需要加入 message_generation,必须有 std_msgs
    find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
       message_generation
    )
    ```
    
    ```
    # 配置 msg 源文件
    add_message_files(
      FILES
      Person.msg
    )
    ```
    
    ```
    # 生成消息时依赖于 std_msgs
    generate_messages(
      DEPENDENCIES
      std_msgs
    )
    ```
    
    ```
    # 执行时依赖
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES demo02_talker_listener
    
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    
    #  DEPENDS system_lib
    )
    ```

**步骤三、编译**

- 回到根目录使用catkin_make编译，并source环境变量
- 生成中间文件：~/catkin_ws/devel/include/learning_communication/Person.h

### 3.2 使用自定义msg

引入头文件：

```c++
#include "learning_communication/Person.h"
```

发布者：

```c++
/*
    需求: 循环发布人的信息
*/
#include <ros/ros.h>
#include "learning_communication/Person.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"talker_person");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<learning_communication::Person>("chatter_person",1000);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    learning_communication::Person p;
    p.name = "sunwukong";
    p.age = 2000;
    p.height = 1.45;

    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(p);
        p.age += 1;
        ROS_INFO("我叫:%s,今年%d岁,高%.2f米", p.name.c_str(), p.age, p.height);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

```

订阅者：

```c++
/*
    需求: 订阅人的信息
*/
#include "ros/ros.h"
#include "learning_communication/Person.h"

void doPerson(const learning_communication::Person::ConstPtr& person_p){
    ROS_INFO("订阅的人信息:%s, %d, %.2f", person_p->name.c_str(), person_p->age, person_p->height);
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"listener_person");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<learning_communication::Person>("chatter_person",10,doPerson);

    //4.回调函数中处理 person

    //5.ros::spin();
    ros::spin();    
    return 0;
}
```





































