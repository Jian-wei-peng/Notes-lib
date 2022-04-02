


## 一、ROS的定义与组成

*    **ROS（Robot Operating System）--- 机器人操作系统**，它是一个用于编写机器人软件的灵活框架，它提供了类似操作系统所提供的功能，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理，同时也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。可以极大简化繁杂多样的机器人平台下的复杂任务创建与稳定行为控制
*   ROS被设计为一种**分布式结构**，使得框架中的每个功能模块都可以被单独设计、编译，并且在运行时以**松散耦合**的方式结合在一起。而且ROS中的功能模块都封装于独立的功能包（package）或者元功能包（meta package）中，便于在社区中共享和分发
*   ROS的设计目标是***提高机器人研发中的软件复用率***，可以避免在机器人技术研发中重复“造轮子”，以达到简化、高效、直接的效果

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161451084.png" alt="image-20220216145144900" style="zoom:50%;" />

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161452330.png" alt="image-20220216145235292" style="zoom:70%;" />

## 二、ROS核心概念

1.  **节点（Node）**

    *   执行单元。执行具体任务的进程、独立运行的可执行文件
        *   可用指令：`ps -ef|grep 进程名`，查看系统进程

            ![](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161500210.png)

        *   不同节点可使用不同的编程语言，可分布式运行在不同主机

    *   一个系统一般由多个节点组成，通过端对端的方式相互通信，构成**节点关系图**

        *   查看节点关系图：`rosrun rqt_graph rqt_graph`
        
        *   查看当前运行的节点：`rosnode list`
        
            <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161512190.png" alt="image-20220216151232164" style="zoom:70%;" />
        
    * 启动节点：`rosrun package_name executable`
    
        * package_name：功能包名称
        * executable：可执行程序名称
        * **rosrun本质是一个shell脚本，位于/opt/ros/melodic/bin/rosrun**
    
2.  **节点管理器（ROS Master）**

    *   控制中心
    *   为节点提供命名和注册服务
    *   跟踪和记录话题/服务通信，辅助节点相互查找、建立连接
    *   提供参数服务器，节点使用此服务器存储和检索运行时的参数

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161514772.png" alt="image-20220216151424732" style="zoom:50%;" />

3.  **话题（Topic）**

    *   节点间用来**传输数据**的重要**总线**

    *   单向异步通信机制，使用**发布/订阅**模型，数据由发布者传输到订阅者

        <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161519745.png" alt="image-20220216151917705" style="zoom:60%;" />

        *   **单向**：数据只能从发布者传输到订阅者，如果订阅者需要传输数据则需要另外开辟一个Topic进行数据传输 
        *   **异步**：
            *   对接收者来讲，其订阅Topic，只要Message从Topic过来就接收并进行处理，不管是谁发布的
            *   对于发布者而言，只管发布Message到Topic，不管有没有接收者接收Message，也不需要等待接收者的处理反馈

    *   同一话题的发布者或订阅者可以不唯一

4.  **消息（Message）**
    *   话题数据，具有一定的类型和数据结构，包括ROS提供的标准类型和用户自定义类型
    *   使用与编程语言无关的**.msg**文件定义，编译过程中生成对应的代码文件

5.  **服务（Service）**
    *   **双向同步通信机制**，使用**客户端/服务器模型**，客户端发送**请求数据**，服务器完成处理后返回**应答数据**
        *   **“双向”**：这种机制不仅可以发送消息，还存在反馈
        *   **“同步”**：在Client发送请求后，它会在原地等待反馈，只有当Server接收处理完请求并完成response反馈，Client才会继续执行。Client等待过程是处于阻塞状态的通信
    *   使用编程语言无关的**.srv**文件定义请求和应答数据结构，编译过程中生成对应的代码文件

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161526796.png" alt="image-20220216152614752" style="zoom:50%;" />

6.  话题与服务的区别

    *   topic举例：激光雷达、里程计发布数据
    *   service举例：开关传感器、拍照、逆解计算

    <img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161526383.png" alt="image-20220216152654327" style="zoom:40%;" />

7.  **参数（Parameter）**
    *   全局共享字典，可通过网络访问的共享、多变量字典
    *   节点使用此服务器来存储和检索运行时的参数
    *   适合存储静态、非二进制的配置参数，不适合存储动态配置的数据

8.  **功能包（Package）**

    *   ROS软件中的基本单元，包含节点源码、配置文件、数据定义等

9.  **功能包清单（Package manifest）**

    *   记录功能包的基本信息，包含作者信息、许可依赖、依赖选项、编译标志等

10.  **元功能包（Meta Package）**

     *   组织多个用于同一目的的功能包

<img src="https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202161532954.png" alt="image-20220216153254917" style="zoom:50%;" />

## 三、常用指令

1.  rosnode
    *   查看当前系统运行的节点：`rosnode list`
    *   查看节点信息：`rosnode info 节点名`
        *   提供信息：节点名，订阅的话题，发布的话题，提供的服务
    *   测试节点是否正常：`rosnode ping 节点名`
        *   `rosnode ping --all`：测试系统所有节点运行是否正常
    *   关闭节点：`rosnode kill 节点名`
        *   `rosnode kill --all`：关闭所有节点
2.  rostopic
    *   查看当前系统中存在的话题：`rostopic list`
    *   查看话题信息：`rostopic info 话题名`
        *   输出话题属性：话题的消息类型，订阅者和发布者
    *   查看话题内容：`rostopic echo 话题名`
    *   查看话题通信频率：
        *   平均频率值：`rostopic hz 话题名`
        *   通信带宽：`rostopic bw 话题名`
    *   发布话题：`rostopic pub 话题名`，然后点两次tab进行命令补全
    *   查看话题消息类型：`rostopic type 话题名`
3.  rosservice
    *   查看当前系统中存在的服务：`rostopic list`
    *   查看服务类型：`rosservice type 服务名`
    *   查看服务信息：`rosservice info 服务名`

















