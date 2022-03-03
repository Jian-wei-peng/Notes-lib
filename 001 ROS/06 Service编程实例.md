## 一、创建服务客户端，生成多只乌龟

>   在turtlesim仿真环境中，写一个C++程序，创建客户端，请求/spawn服务，生成多只乌龟

1.  启动ros，打开turtlesim仿真环境

2.  查看当前的服务：`rosservice list`

    ![image-20220217112229183](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171122227.png)

    *   其中**/spawn**可用于调用生成新的乌龟
        *   输入`rosservice call /spawn`，然后点击两下tab进行补全
        *   输入`rosservice type /spawn`，可以查看/spawn的消息格式为**turtlesim/Spawn**
        *   可输入新生成乌龟的初始位置以及名字

![image-20220217112418575](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171124602.png)

![image-20220217112634696](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171126726.png)

3.  turtle_spawn_client.cpp代码：

```c++
/**
 * 
 * 服务通信：在turtlesim仿真环境下，请求/spawn服务，服务数据类型turtlesim::Spawn
 */

# include <ros/ros.h>
# include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "turtle_spawn");
    ros::NodeHandle n;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle_client = n.serviceClient<turtlesim::Spawn>("/spawn");

    // 初始化turtlesim::Spawn的请求数据
    turtlesim::Spawn srv;
    srv.request.x = 2.0;
    srv.request.y = 2.0;
    srv.request.name = "turtle2";

    // 请求服务调用
    ROS_INFO("Call service to spawn turtle[x:%0.6f, y:%0.6f, name:%s]", srv.request.x, srv.request.y, srv.request.name.c_str());

    add_turtle_client.call(srv);

    // 显示服务调用结果
    ROS_INFO("Spawn turtle successfully [name:%s]", srv.response.name.c_str());

    return 0;
    
}
```

4.  配置CMakeLists.txt

    ```txt
    add_executable(turtle_spawn_client src/turtle_spawn_client.cpp)
    
    target_link_libraries(turtle_spawn_client ${catkin_LIBRARIES})
    ```

5.  回到根目录下编译，source环境变量

![image-20220217114322144](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171143171.png)

![image-20220217114336862](https://gitee.com/jianweipeng/typora-notes-gitee/raw/master/typora-notes-gitee/202202171143886.png)

## 二、触发小车控制

```c++
/*
 * 服务通信：该例程在F1TENTH仿真环境下，执行/car_command服务，服务数据类型std_srvs/Trigger
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

ros::Publisher pub_drive;
bool pubCommand = false;
ackermann_msgs::AckermannDriveStamped vel_drive;

// service回调函数，输入参数req，输出参数res
bool commandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	  pubCommand = !pubCommand;
    // 显示请求数据
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");
	  // 设置反馈数据
  	res.success = true;
  	res.message = "Change turtle command state!";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_command_server");
    ros::NodeHandle n;

    // 创建一个名为/car_command的server，注册回调函数commandCallback
    ros::ServiceServer command_server = n.advertiseService("/car_command", commandCallback);

    // 创建一个Publisher，发布名为/drive的topic，阿克曼速度消息类型，队列长度10
    pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

    // 循环等待回调函数
    ROS_INFO("Ready to receive car command.");

    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok())
	{
	  // 查看一次回调函数队列
     ros::spinOnce();

		// 如果为true，则发布速度指令, 否则停止
	  	if(pubCommand)
		{
			vel_drive.drive.speed = 1.0;	
			pub_drive.publish(vel_drive);
		}
  	else
		{
			vel_drive.drive.speed = 0.0;
			pub_drive.publish(vel_drive);
		}
      loop_rate.sleep();
	}

    return 0;
}
```

