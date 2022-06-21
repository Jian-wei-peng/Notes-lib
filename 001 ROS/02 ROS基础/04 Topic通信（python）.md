## 一、发布方

### 1.1 导入ros包

```python
#!/usr/bin/env python #确保脚本作为python脚本运行
import rospy
```

### 1.2 初始化节点

```python
rospy.init_node("发布节点名称", anonymous=True)

# anonymous=True会让名称末尾添加随机数，来确保节点具有唯一的名称
```

### 1.3 实例化发布者对象

```python
pub = rospy.Publisher("话题名称", 消息类型, 消息队列大小)

rate = rospy.Rate(10) # 10hz
# 创建一个Rate对象rate。借助其方法sleep()，它提供了一种方便的方法，来以你想要的速率循环。它的参数是10，即表示希望它每秒循环10次（只要我们的处理时间不超过十分之一秒）！
```

完整代码：

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



## 二、订阅方

### 2.1 导包、初始化节点和实例化订阅者对象

```python
import rospy

rospy.init_node("订阅节点名称", anonymous=True)

sub = rospy.Subscriber("话题名称", 消息类型,回调函数, 消息队列大小)
```

### 2.2 设置循环调用回调函数

```python
rospy.spin()
```

完整代码：

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



## 三、调用自定义msg

### 3.1 自定义msg（Person.msg）

1. 定义msg文件

	```python
	# 功能包下新建 msg 目录，添加文件 Person.msg
	string name
	uint16 age
	float64 height
	```

2. 编辑配置文件

	1. **package.xml**中添加编译依赖与执行依赖

		```python
		<build_depend>message_generation</build_depend>
		<exec_depend>message_runtime</exec_depend>
		```

	2. **CMakeLists.txt**编辑 msg 相关配置

		```python
		find_package(catkin REQUIRED COMPONENTS
		  roscpp
		  rospy
		  std_msgs
		  message_generation
		)
		# 需要加入 message_generation,必须有 std_msgs
		
		## 配置 msg 源文件
		add_message_files(
		  FILES
		  Person.msg
		)
		
		# 生成消息时依赖于 std_msgs
		generate_messages(
		  DEPENDENCIES
		  std_msgs
		)
		
		#执行时依赖
		catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES demo02_talker_listener
		  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
		#  DEPENDS system_lib
		)
		```

3. 编译

	- C++ 需要调用的中间文件(**.../工作空间/devel/include/包名/xxx.h**)
	- Python 需要调用的中间文件(**.../工作空间/devel/lib/python3/dist-packages/包名/msg**)

### 3.2 发布方（person_talker.py）

1. 导入自定义的msg

	```python
	from demo02_talker_listener.msg import Person
	```

完整代码：

```python
#! /usr/bin/env python
"""
    发布方:
        循环发送消息

"""
import rospy
from demo02_talker_listener.msg import Person

if __name__ == "__main__":
    #1.初始化 ROS 节点
    rospy.init_node("talker_person_p")
    #2.创建发布者对象
    pub = rospy.Publisher("chatter_person",Person,queue_size=10)
    #3.组织消息
    p = Person()
    p.name = "葫芦瓦"
    p.age = 18
    p.height = 0.75

    #4.编写消息发布逻辑
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(p)  #发布消息
        rate.sleep()  #休眠
        rospy.loginfo("姓名:%s, 年龄:%d, 身高:%.2f",p.name, p.age, p.height)
```

### 3.3 订阅者（person_listener.py）

```python
#! /usr/bin/env python
"""
    订阅方:
        订阅消息
"""
import rospy
from demo02_talker_listener.msg import Person

#3.定义回调函数
def doPerson(p):
    rospy.loginfo("接收到的人的信息:%s, %d, %.2f",p.name, p.age, p.height)

if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("listener_person_p")
    #2.创建订阅者对象
    sub = rospy.Subscriber("chatter_person",Person,doPerson,queue_size=10)
    rospy.spin() #4.循环

```

### 3.4 权限设置

终端下进入 scripts 执行:`chmod +x *.py`

### 3.5 配置CMakeList.txt

```txt
catkin_install_python(PROGRAMS
  scripts/person_talker.py
  scripts/person_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```





















































