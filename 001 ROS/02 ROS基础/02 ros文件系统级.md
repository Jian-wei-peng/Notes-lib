## 一、文件系统级

### 1.1 基本概念

- 从系统实现的角度来看，ROS可以分为三个层次：**文件系统、计算图和开源社区**

	<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205271633125.png" alt="image-20220527163304085" style="zoom:67%;" />

- ros文件系统级**是用于描述可以在硬盘上查找到的代码及可执行文件程序，在这一级中将使用一组概念来解释ROS的内部组成、文件架结构以及工作所需的核心文件。**

<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205271645377.jpeg" alt="img" style="zoom:67%;" />

### 1.2 相关命令

- 创建功能包：`catkin_create_pkg 自定义包名 依赖包`
- 安装ros功能包：`sudo apt install xxx`
- 删除某个功能包：`sudo apt purge xxx`
- 搜索某个功能包：`apt search xxx`

### 1.3 创建工作空间

1.  步骤

```shell
# 1.创建工作空间目录，并初始化
mkdir -p ~/catkin_ws/src	# catkin_ws是工作空间的名称，创建时必须要有src目录
cd ~/catkin_ws/src
catkin_init_workspace		# 初始化工作空间

# 2.编译
cd ~/catkin_ws
catkin_make

# 3.设置环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2.  **工作空间的典型结构**

| src       | 代码空间，Source Space，存储所有ROS功能包的源码文件          |
| --------- | ------------------------------------------------------------ |
| **build** | **编译空间，Build Space，存储工作空间编译过程中产生的缓存信息和中间文件** |
| **devel** | **开发空间，Development Space，放置编译生成的可执行文件**    |
| install   | 安装空间，Install Space，编译成功后可用make install将可执行文件安装到该空间，运行该空间的环境变量脚本，即可在终端中运行这些可执行文件，该空间不是必需的 |

### 1.4 创建功能包

*   **ROS不允许在某个功能包中嵌套其他功能包，多个功能包必须平行放置在代码空间**

1. 创建功能包指令

```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend2]
```

例如：

```shell
cd ~/catkin_ws/src
catkin_create_pkg learning_communication std_msgs rospy roscpp

# 回到工作空间根目录下进行编译，source环境变量
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```

2. **功能包的典型结构**

| src                | 放置需要编译的C++代码                                  |
| ------------------ | ------------------------------------------------------ |
| **scripts**        | **放置可以直接运行的Python脚本**                       |
| **include**        | **放置功能包中需要用到的头文件**                       |
| **msg**            | **放置功能包自定义的消息类型**                         |
| **srv**            | **放置功能包自定义的服务类型**                         |
| **action**         | **放置功能包自定义的动作指令**                         |
| **config**         | **放置功能包中的配置文件，由用户创建，文件名可以不同** |
| **launch**         | **放置功能包中的所有启动文件**                         |
| **CMakeLists.txt** | **编译器编译功能包的规则**                             |
| **package.xml**    | **功能包清单**                                         |

## 二、功能包（package）

- 功能包是**ros中软件组织的基本形式**，**构成ros的基本单元**，**catkin编译的基本单元**

	- catkin是ROS的官方编译系统，是基于CMake的编译构建系统，其沿用了包管理的传统基础结构
	- 工作流程：首先在工作空间的src目录下**递归的查找每一个ros的package**。因为每一个package中都有package.xml和CMakeList.txt文件，所以catkin编译系统依据CMakeList.txt文件，生成makefile文件，放在工作空间的build文件夹中。然后make刚刚生成的makefiles等文件，编译链接生成可执行文件，放在工作空间下的devel文件夹中

- ros的应用程序是**以功能包为单位开发的**，一个功能包可包含多个可执行文件（节点）

- **在功能包文件下，一定有CMakeList.txt和package.xml这两个文件**

	- **CMakeList.txt规定了catkin编译的规则**，即告诉编译系统如何编译这个功能包下的代码，需要编译哪些文件，需要哪些依赖，生成哪些目标文件等。
		- 该文件在创建功能包时，系统会自动生成一个模板，只需懂得如何修改即可

	<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205271642894.png" alt="image-20220527164234857" style="zoom:67%;" />

	- **package.xml定义了功能包的属性，例如包名、版本号、作者、依赖包等等（功能包的自我描述）**
		- 创建功能包时也会自动生成该文件，通常需要修改的时**build_depend**和**run_depend**

	<img src="https://raw.githubusercontent.com/Jian-wei-peng/typora-pic/main/202205271645025.png" alt="image-20220527164512986" style="zoom:67%;" />

## 三、package.xml（功能包配置文件）

- 文件写法：遵循XML标签文本的写法，目前存在两种格式，但内容大致一样

```xml
<?xml version="1.0"?>
<!-- 格式: 以前是 1，推荐使用格式 2 -->
<package format="2">
  <!-- 包名 -->
  <name>demo01_hello_vscode</name>
  <!-- 版本 -->
  <version>0.0.0</version>
  <!-- 描述信息 -->
  <description>The demo01_hello_vscode package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <!-- 维护人员 -->
  <maintainer email="xuzuo@todo.todo">xuzuo</maintainer>

  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <!-- 许可证信息，ROS核心组件默认 BSD -->
  <license>TODO</license>

  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/demo01_hello_vscode</url> -->

  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->

  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <!-- 依赖的构建工具，这是必须的 -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 指定构建此软件包所需的软件包 -->
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <!-- 指定根据这个包构建库所需要的包 -->
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <!-- 运行该程序包中的代码所需的程序包 -->  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

## 四、CMakeList.txt文件（构建配置文件）

- CMakeList.txt原本是Cmake编译系统的规则文件，ROS的构建系统catkin基本上使用CMake，只是针对ROS工程添加了一些宏定义。因此在写法上CMakeList.txt和Cmake基本一致
- CMakeList.txt文件规定了catkin的编译规则，直接规定这个包需要依赖那些package，要编译生成那些目标，如何编译等等流程
- 写法：该文件的基本语法都是按照CMake，只是在其基础上添加了一些宏

```cmake
cmake_minimum_required(VERSION 3.0.2) #所需 cmake 版本
project(demo01_hello_vscode) #包名称，会被 ${PROJECT_NAME} 的方式调用

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
#默认添加系统依赖
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# 启动 python 模块支持
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
## 声明 ROS 消息、服务、动作... ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# 生成消息、服务时的依赖包
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
## 声明 ROS 动态参数配置 ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
## catkin 特定配置##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# 运行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo01_hello_vscode
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 声明 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo01_hello_vscode.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# 添加库的 cmake 目标依赖
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# 声明 C++ 可执行文件
add_executable(Hello_VSCode src/Hello_VSCode.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#重命名c++可执行文件
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#添加可执行文件的 cmake 目标依赖
add_dependencies(Hello_VSCode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#指定库、可执行文件的链接库
target_link_libraries(Hello_VSCode
  ${catkin_LIBRARIES}
)

#############
## Install ##
## 安装 ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#设置用于安装的可执行脚本
catkin_install_python(PROGRAMS
  scripts/Hi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo01_hello_vscode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```









