## 一、自定义srv

### 1.1 定义srv文件

- 数据分成两部分，请求与响应，在 srv 文件中请求和响应使用`---`分割

```python
# 客户端请求时发送的两个数字
int32 num1
int32 num2
---
# 服务器响应发送的数据
int32 sum

```

### 1.2 编辑配置文件

- **package.xml**中添加编译依赖与执行依赖

	```python
	  <build_depend>message_generation</build_depend>
	  <exec_depend>message_runtime</exec_depend>
	```

- **CMakeLists.txt**编辑 srv 相关配置

	```python
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  rospy
	  std_msgs
	  message_generation
	)
	# 需要加入 message_generation,必须有 std_msgs
	
	add_service_files(
	  FILES
	  AddInts.srv
	)
	
	generate_messages(
	  DEPENDENCIES
	  std_msgs
	)
	```

### 1.3 编译

- C++ 需要调用的中间文件(**.../工作空间/devel/include/包名/xxx.h**)
- Python 需要调用的中间文件(**.../工作空间/devel/lib/python3/dist-packages/包名/srv**)

## 二、服务器端（AddInts_Server_p.py）

### 2.1 导包

```python
import rospy
from demo03_server_client.srv import AddInts, AddIntsRequest, AddIntsResponse
```

### 2.2 初始化节点

```python
rospy.init_node("服务器端名称")
```

### 2.3 创建服务端对象

```python
server = rospy.Service("服务名", 服务类型, 回调函数名)
```

完整代码：

```python
#! /usr/bin/env python

# 1.导包
import rospy
from demo03_server_client.srv import AddInts,AddIntsRequest,AddIntsResponse

# 回调函数的参数是请求对象，返回值是响应对象
def doReq(req):
    # 解析提交的数据
    sum = req.num1 + req.num2
    rospy.loginfo("提交的数据:num1 = %d, num2 = %d, sum = %d",req.num1, req.num2, sum)

    # 创建响应对象，赋值并返回
    resp = AddIntsResponse(sum)
    return resp

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("addints_server_p")
    # 3.创建服务对象
    server = rospy.Service("AddInts", AddInts, doReq)
    # 4.回调函数处理请求并产生响应
    # 5.spin 函数
    rospy.spin()

```

## 三、客户端（AddInts_Client_p.py）

### 3.1 导包

```python
import rospy
from demo03_server_client.srv import *
```

### 3.2 初始化节点和创建客户端对象

```python
rospy.init_node("客户端节点名称")

client = rospy.ServiceProxy("服务名", 服务类型)

```

完整代码：

```python
#! /usr/bin/env python

#1.导包
import rospy
from demo03_server_client.srv import *
import sys

if __name__ == "__main__":

    #优化实现
    if len(sys.argv) != 3:
        rospy.logerr("请正确提交参数")
        sys.exit(1)

    # 2.初始化 ROS 节点
    rospy.init_node("AddInts_Client_p")
    # 3.创建请求对象
    client = rospy.ServiceProxy("AddInts",AddInts)
    # 请求前，等待服务已经就绪
    # 方式1:
    # rospy.wait_for_service("AddInts")
    # 方式2
    client.wait_for_service()
    # 4.发送请求,接收并处理响应
    # 方式1
    # resp = client(3,4)
    # 方式2
    # resp = client(AddIntsRequest(1,5))
    # 方式3
    req = AddIntsRequest()
    # req.num1 = 100
    # req.num2 = 200 

    #优化
    req.num1 = int(sys.argv[1])
    req.num2 = int(sys.argv[2]) 

    resp = client.call(req)
    rospy.loginfo("响应结果:%d",resp.sum)

```

### 3.3 权限设置

终端下进入 scripts 执行:`chmod +x *.py`

### 3.4 配置CMakeList.txt

```python
catkin_install_python(PROGRAMS
  scripts/AddInts_Server_p.py 
  scripts/AddInts_Client_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

















