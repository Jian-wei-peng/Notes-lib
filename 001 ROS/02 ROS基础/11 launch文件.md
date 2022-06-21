## 一、元功能包

例如在机器人导航模块下，包含有地图、定位、路径规划…等不同的子级功能包。那么安装该模块时如果逐一安装功能包那么会使得效率非常低下，ROS提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该包又称之为元功能包（metapackage）

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

- 实现：

	- 新建一个功能包

	- 修改package.xml：

		```xml
		 <exec_depend>被集成的功能包</exec_depend>
		 .....
		 <export>
		   <metapackage />
		 </export>
		```

	- 修改CMakeLists.txt：

		```c++
		cmake_minimum_required(VERSION 3.0.2)
		project(demo)
		find_package(catkin REQUIRED)
		catkin_metapackage()
		
		```

## 二、launch文件

> 一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。如果每次都调用 rosrun 逐一启动，显然效率低下，如何优化?

- 采用的优化策略便是使用roslaunch 命令集合 launch 文件启动管理节点，并且在后续教程中，也多次使用到了 launch 文件
- **概念**：launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数
- **作用**：简化节点的配置与启动，提高ROS程序的启动效率

在功能包下添加 launch目录, 目录下新建 xxxx.launch 文件，编辑 launch 文件：

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node"     name="myTurtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key"  name="myTurtleContro" output="screen" />
</launch>

```

调用launch文件：

```shell
roslaunch 包名 xxx.launch
```

- **roslaunch 命令执行launch文件时，首先会判断是否启动了 roscore,如果启动了，则不再启动，否则，会自动调用 roscore**

## 三、launch文件常用标签

### 3.1 根标签launch

- XML文件必须包含一个根元素，launch文件的**根元素**采用`<launch>`标签，**它的唯一目的是作为其他元素的容器（文件中的其他内容都必须包含在这个标签中）**

```xml
<launch>
    ...
    ...
    ...
</launch>
```

### 3.2 `<node>`

- 启动文件的核心是启动ROS节点，采用<node>标签定义。语法如下：

```xml
<node pkg="package_name" type="executable_name" name="node_name"/>
```

- **pkg：定义节点所在的功能包名称**
- **type：定义节点的可执行文件名称**
- **name：定义节点运行的名称**，将覆盖节点中`init()`赋予节点的名称

其他属性：

- **output = “screen”：将节点的标准输出打印到终端屏幕，默认输出日志文档**
- **resapwn = “true”：复位属性，该节点停止时，会自动重启，默认为false**
- **required=“true”：必要节点，当该节点终止时，launch文件中的其他节点也被终止**
- **ns=“namespace”：命名空间，为节点内的相对名称添加命名空间前缀**
- **args=“arguments”：节点需要的输入参数**

### 3.3 参数设置标签

#### 3.3.1 param

**代表parameter，是存储在参数服务器中的参数**

- 在launch文件中通过`<param>`元素加载parameter。launch文件执行后，parameter就加载到ROS的参数服务器上了

```xml
<param name="namespace/param_name" value="param_value">
```

- **运行launch文件后，param_name的值就设置param_value，并且加载到ROS参数服务器上了**
- **name="namespace/param_name"**：**定义参数名**。参数名中可以包含命名空间，但指定的全局名称应该避免
- **value="param_value"：定义参数的值**

其他属性：

- `textfile`：以字符串形式读取文件的内容（**该文件必须是本地可访问的**）

	```xml
	textfile="$(find package_name)/path/file.txt"
	```

#### 3.3.2 rosparam

- 在很多复杂的系统中参数的数量很多，如果用`<param>`一个一个地设置会非常麻烦，ROS就提供了`<rosparam>`这种加载方式

- **加载多个参数**

	```xml
	<rosparam file="$(find 2dnav_pr2)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
	```

- `<rosparam>`可以帮助我们将一个YAML格式文件中的参数全部加载到ROS参数服务器中

- 需要设置command属性为load，还可以选择设置命名空间ns

#### 3.3.3 arg

- 代表argument，**类似于launch文件内部的局部变量，*仅限于launch文件使用*，便于launch文件的重构，与ROS节点内部的实现没有关系**

- arg可以通过指定值来创建更多能重复使用和配置的文件

- 三种调用方式：

	1. **声明参数arg_name**。arg_name既可以作为命令行参数传递，也可以通过<include>传递（如果被包含的话）

		```xml
		<arg name="arg_name"/>
		```

	2. **声明有默认值arg_value的arg_name**。arg_name能被覆盖（通过命令行参数传递（顶层），也可以通过`<include>`传递（如果被包含的话））

		```xml
		<arg name="arg_name" default="arg_value"/>
		```

	3. **声明有常值的arg_name**。arg_name**不能被重写**。这种用法保证了启动文件内部的参数化

		```xml
		<arg name="arg_name" value="bar">
		```

launch文件中需要使用到argument时，可以使用如下方式调用：

```xml
<param name="foo" value="$(arg arg_name"/>
<node name="node" pkg="package" type="type" args="$(arg arg_name)"/>
```

### 3.4 重映射机制 —— `<remp>`

- ROS提供一种重映射的机制，简单来说就是**取别名**，类似于C++中的别名机制

- **不需要修改别人功能包的接口，只需要将接口名称重映射一下，取个别名，我们的系统就认识了（接口的数据类型必须相同）**

- **`<remp>`标签允许通过名称映射参数到ROS节点，其适用于在其范围内随后的所有声明**。用法如下：

	```xml
	<remap from="original_name" to="new_name"/>
	```

	- **from="original_name"**：定义需要映射的参数名称
	- **to="new_name"**：定义目标名称

### 3.5 嵌套复用 —— `<include>`

- 在复杂的系统当中，launch文件往往有很多，这些**launch文件之间也会存在依赖关系**

- 如果需要直接**复用一个已有launch文件中的内容，可以使用`<include>`标签包含其他launch文件**。使用方式：

	```xml
	<include file="$(dirname)/other.launch"/>
	```

	































