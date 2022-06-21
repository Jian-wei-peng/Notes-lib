## 一、控制小乌龟画圆

### 1.1 方式一：命令行发布速度指令

1.  启动ROS
2.  打开小乌龟节点：`rosrun turtlesim turtlesim_node`

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201140687.png" alt="image-20220620114029644"  />

-   查看当前的话题信息
    
    - 其中**/turtle1/cmd_vel**是小乌龟的速度话题
    
    <img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201141585.png" alt="image-20220620114151545" style="zoom:33%;" />

3.  查看**/turtle1/cmd_vel**的消息类型：`rostopic type /turtle1/cmd_vel`
4.  使用`rostopic pub`向小乌龟发布一个速度指令，用tab进行指令补全即可，修改xyz的值，回车运行即可

	1.  linear指线速度，有x，y，z三个方向；angular指角速度，有x，y，z三个方向
	2.  将线速度的x修改为1，角速度z修改为1，回车执行后小乌龟就会开始做圆周运动

![image-20220620114238325](https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201142370.png)



### 1.2 方式二：发布小乌龟速度话题

1.  启动ROS，打开小乌龟仿真环境

2.  小乌龟的速度话题：**/turtle1/cmd_vel**，其消息类型为：**geometry_msgs/Twist**

3.  在工作空间目录`~/catkin_ws/src`下创建一个新的功能包tutorial并编译，然后在tutorial路径下创建一个scripts文件夹存放python脚本：`touch scripts`

4.  在scripts中创建发布者代码文件pub_vel.py：

    ```shell
    mkdir pub_vel.py
    chmod +x pub_vel.py # 赋予可执行权限
    ```

5.  编写发布者代码

- 参考ros wiki上的topic通信编程中的发布者代码模板进行修改

原代码：

```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

-  **发布者节点名称为talker，发布的话题为chatter，消息类型是String**
- 在终端运行该文件（20版本的ros自带python3）：

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201152048.png" alt="image-20220620115219925" style="zoom:50%;" />

- 修改上述代码，向小乌龟发布速度命令

- 步骤：

	1. 查看小乌龟的速度话题名：`rostopic list`

	2. 查看小乌龟的速度话题类型：`rostopic type /turtle1/cmd_vel`

		<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201200833.png" alt="image-20220620120008791" style="zoom:50%;" />

	3. 修改发布的话题名和话题类型

		```python
		# 发布的话题名是：/turtle1/cmd_vel ； 话题类型：Twist
		# Twist是来自geometry_msgs.msg
					
		from geometry_msgs.msg import Twist	
		
		pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		```

	4. 数据处理

		- 可以在终端查看Twist的内容：`rosmsg info geometry_msgs/Twist`

			<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201546380.png" alt="image-20220620154603346" style="zoom:50%;" />

		- 在vscode中将光标放到Twist()会提示该类的属性

			<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201453251.png" alt="image-20220620145300204" style="zoom:50%;" />

```python
	while not rospy.is_shutdown():
	    # 实例化一个Twist对象
	    vel_cmd = Twist()
	    
	    # 给对象属性赋值（定义待发布消息）
	    vel_cmd.linear.x = 0.5		# 设置乌龟x方向线速度
	    vel_cmd.angular.z = 1.0		# 设置乌龟z方向角速度
	    
	    # 发布消息
	    pub.publish(vel_cmd)		# 发布消息到vel_cmd话题
	    rate.sleep()
```

完整代码：

```python
# !/usr/bin/env python					# 指定通过python解释代码，必须放在首行
# -*- coding:utf-8 -*					# 加上这句才可以在文件中添加中文注释

# 需求：修改代码发布速度命令，控制乌龟
# 思路：
# 		1、查看话题并修改
# 		2、话题的消息类型修改
# 		3、 修改发布内容
# 围绕通信方式、通信内容、通信消息类型做修改

import rospy						  # 导入rospy包，rospy是ROS的python客户端
from geometry_msgs.msg import Twist		# 导入Twist数据类型，小乌龟的速度消息类型为geometry_msgs/Twist

def talker():
    # 声明一个节点（发布者），该节点使用Twist消息类型发布消息到/turtle1/cmd_vel话题
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # 节点初始化，节点名称为talker
    # anonymous=True，要求每个节点都有唯一的名称，避免冲突，这样就可以运行多个发布者节点
    rospy.init_node('talker', anonymous=True)
    
    # 定义发布的频率，单位是赫兹，每秒钟发布次数
    # 在方法sleep()的帮助下，提供一个可以以一定速率循环的简便方法
    rate = rospy.Rate(10)   # 10Hz
    
    # rospy.is_shutdown()检查节点是否关闭，如果没有则循环执行函数体内指令
    while not rospy.is_shutdown():
        vel_cmd = Twist()   	 # 实例化对象
        # 定义待发布消息
        vel_cmd.linear.x = 0.5	 # 设置乌龟x方向线速度
        vel_cmd.angular.z = 1.0  # 设置乌龟z方向角速度

        pub.publish(vel_cmd)	 # 发布消息到vel_cmd话题
        rate.sleep()

# 主程序
if __name__ == '__main__':
    try:
        talker()
    except rospy.rospy.ROSInitException:
        pass
```

6.  在scripts文件夹下打开终端，运行pub_vel.py，即可看到小乌龟在做圆周运动


<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201458400.png" alt="image-20220620145840370" style="zoom:50%;" />



### 1.3 订阅乌龟的位姿信息

1.  在当前话题中**/turtle1/pose**指小乌龟的位姿话题
2.  打印查看pose话题的具体内容：`rostopic echo /turtle1/pose`
	- 包含x，y方向位置，方向角theta以及线速度和角速度

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201505845.png" alt="image-20220620150519790" style="zoom: 40%;" />

3. **编写一个订阅者程序，查看乌龟的位姿信息**

 4. 在scripts文件夹下创建sub_odom.py

	```shell
	cd ~/catkin_ws/src/tutorial_ws/scripts
	mkdir sub_odom.py
	chmod +x sub_odom.py
	```

	- 查看乌龟的位姿话题的消息类型：`rostopic type /turtle1/pose`，类型为：**turtlesim/Pose**

	- 编写订阅者程序。参考ros wiki上的topic通信编程中的订阅者代码模板进行修改

	原代码：

	```python
	#!/usr/bin/env python
	import rospy
	from std_msgs.msg import String
	
	def callback(data):
	    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	    
	def listener():
	    rospy.init_node('listener', anonymous=True)
	    rospy.Subscriber("chatter", String, callback)
	
	    # spin() simply keeps python from exiting until this node is stopped
	    rospy.spin()
	
	if __name__ == '__main__':
	    listener()
	```

	1. 小乌龟位姿话题的名称为`/turtle1/pose`，话题的类型为`turtlesim/Pose`

	2. 导入小乌龟位姿话题的消息类型

	```python
	from turtlesim.msg import Pose
	```

	3. 修改订阅者对象的定义

	```python
	rospy.Subscriber("/turtle1/pose", Pose, callback)
	```

	完整代码：

	```python
	# !/usr/bin/env python
	# -*- coding:utf-8 -*
	
	import rospy
	from turtlesim.msg import Pose	# 导入乌龟位姿话题的消息类型
	
	# 回调函数
	# 将接收到的消息作为参数进行处理
	def callback(data):
	    rospy.loginfo(data.x)	# 在终端打印消息
	
	def listener():
	    # 定义名为listener的节点（订阅者）
	    # 初始化节点
	    rospy.init_node('listener', anonymous=True)
	    
	    # 订阅函数，订阅/turtle1/pose话题，消息类型是Pose
	    # 当有新的消息时，调用callback函数处理。将接收到的消息作为参数传递给callback
	    rospy.Subscriber("/turtle1/pose", Pose, callback)
	    rospy.spin()
	
	if __name__ == '__main__':
	    listener()
	```

	4. 打开小乌龟的键盘控制节点：`rosrun turtlesim turtle_teleop_key`

	5. 运行订阅者程序，用键盘移动小乌龟，可以在终端看到小乌龟的位姿数据在变化（回调函数只打印了x轴的数据）

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201540313.png" alt="image-20220620154038240" style="zoom:50%;" />



## 二、控制小车移动

### 2.1 配置tianbot仿真环境

1.  安装目录：`~/catkin_ws/src`
2.  git下载tianbot功能包以及tianbot的仿真文件

```shell
# tianbot功能包
git clone https://github.com/tianbot/tianbot_mini.git

# tianbot的仿真文件
git clone https://github.com/tianbot/tianbot_mini_gazebo.git
git clone https://github.com/tianbot/tianbot_mini_description.git
```

4.  回到根目录下编译，source环境变量
5.  进入tianbot仿真启动文件目录

```shell
cd ~/catkin_ws/src/tianbot_mini_gazebo/launch
```

6.  在该目录下打开终端，运行launch文件

```shell
roslaunch simulation.launch
```

7. 查看tianbot的话题

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201731108.png" alt="image-20220620173128053" style="zoom:50%;" />

### 2.2 订阅tianbot的里程计传感器数据

1. tianbot的里程计传感器数据话题：**/tianbot_mini/odom**，其消息类型为：**nav_msgs/Odometry**

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201732026.png" alt="image-20220620173222002" style="zoom:50%;" />

2. 查看tianbot的里程计消息结构：`rosmsg show nav_msgs/Odometry`
	- 调用`w`：`data.pose.pose.orientation.w`

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201732787.png" alt="image-20220620173259725" style="zoom:33%;" />

3. 可以直接在小乌龟的订阅程序基础上进行修改，以实现对小车里程计数据的订阅

```python
# !/usr/bin/env python
# -*- coding:utf-8 -*

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo(data.pose.pose.orientation.w)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tianbot_mini/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

4.  执行订阅程序，可在终端看到里程计数据

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202206201734126.png" alt="image-20220620173409062" style="zoom:50%;" />

### 2.3  控制小车转圈前进：订阅+发布

>订阅并发布话题实现机器人转圈前进
>机器人先原地旋转，当转到一定角度后开始前进

- 要完成机器人转圈前进，则需要**先订阅机器人的里程数据（Odometry）**，**然后发布速度指令给小车**

1. 订阅小车里程数据（前一节已有）

```python
def callback(data):
    rospy.loginfo(data.pose.pose.orientation.w)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tianbot_mini/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tianbot_mini/odom", Odometry, callback)
    rospy.spin()
```

2. 发布速度指令
	- 在回调函数中定义发布者，依据订阅的里程数据进行判断

```python
def callback(data):
    rospy.loginfo(data.pose.pose.orientation.w)
    car_pub = rospy.Publisher('/tianbot_mini/cmd_vel', Twist, queue_size=10)
    
    car_cmd = Twist()
    car_cmd.angular.z = 1.0
    car_pub.publish(car_cmd)
    if data.pose.pose.orientation.w > 0.8:	# 旋转角度超过0.8则前进
        car_cmd.linear.x = 0.3
        car_pub.publish(car_cmd)
```

完整代码：

```python
# !/usr/bin/env python
# -*- coding:utf-8 -*

# 需求：订阅里程信息，然后发布速度消息给小车
# 思路：1、导入里程与速度
#      2、添加并修改话题
#      3、修改订阅后发布的内容
#围绕通信方式、通信内容、通信消息类型做修改

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(data.pose.pose.orientation.w)
    car_pub = rospy.Publisher('/tianbot_mini/cmd_vel', Twist, queue_size=10)

    car_cmd = Twist()
    car_cmd.angular.z = 1.0
    car_pub.publish(car_cmd)
    if data.pose.pose.orientation.w > 0.8:
        car_cmd.linear.x = 0.3
        car_pub.publish(car_cmd)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tianbot_mini/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```







