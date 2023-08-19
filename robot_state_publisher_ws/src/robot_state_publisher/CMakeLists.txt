# 最低CMake版本要求
cmake_minimum_required(VERSION 2.8)

# 项目名称
project(robot_state_publisher)

# 设置C++编译标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# 查找必需的依赖包
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole rostime tf tf2_ros tf2_kdl kdl_parser)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)

# 定义catkin包
catkin_package(
        LIBRARIES ${PROJECT_NAME}_solver
        INCLUDE_DIRS include
        DEPENDS roscpp rosconsole rostime tf2_ros tf2_kdl kdl_parser orocos_kdl urdfdom_headers
)

# 包括头文件路径
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 创建库
add_library(${PROJECT_NAME}_solver
        src/robot_state_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(joint_state_listener src/joint_state_listener.cpp)
target_link_libraries(joint_state_listener ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# 创建可执行文件
add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# 使用旧名称编译相同的可执行文件
add_executable(state_publisher src/joint_state_listener.cpp)
target_link_libraries(state_publisher ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# 测试部分
if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  # 添加各种测试
  # ...

  # 下载需要的数据文件
  catkin_download_test_data(
          joint_states_indexed_bag
          http://wiki.ros.org/robot_state_publisher/data?action=AttachFile&do=get&target=joint_states_indexed.bag
          DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
          FILENAME joint_states_indexed.bag
          MD5 793e0b566ebe4698265a936b92fa2bba)

  # 更多测试
  # ...

endif()

# 安装目标
install(TARGETS ${PROJECT_NAME}_solver joint_state_listener ${PROJECT_NAME} state_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
