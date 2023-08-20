#最低CMake版本要求：设置了CMake的最低版本要求为2.8.3。
#项目名称：定义项目名为 "urdf"。
#依赖包查找：查找了必要的依赖包，包括Boost、urdfdom、urdfdom_headers、catkin、TinyXML等。
#获取urdfdom_headers版本：解析urdfdom_headers的版本号，并将其分解为主要、次要和修订版本号。
#配置兼容性头文件：设置并配置兼容性头文件 "urdfdom_compatibility.h"。
#编译选项：添加了C++11编译选项。
#定义catkin包及依赖关系：定义了catkin包和相关依赖关系，指定了库、包括路径、catkin依赖、以及其他依赖。
#安装兼容性头文件：安装了之前配置的兼容性头文件。
#包括头文件和链接目录：添加了必要的头文件和链接目录。
#添加库：创建了名为 "urdf" 的库，并链接了所需的库文件。
#平台相关编译选项：根据不同的操作系统（Windows和Apple）设置了相关的编译选项。
#测试部分：如果启用测试，将构建和链接测试相关的目标。
#安装目标：指定了库、运行时等的安装目的地。
#安装头文件目录：安装了项目的头文件目录。

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 2.8.3)

# 定义项目名
project(urdf)

# 找到必要的依赖包
find_package(Boost REQUIRED thread)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        urdf_parser_plugin pluginlib rosconsole_bridge roscpp cmake_modules)
find_package(TinyXML REQUIRED)

# 获取urdfdom_headers的版本组件
if(NOT urdfdom_headers_VERSION)
  set(urdfdom_headers_VERSION "0.0.0")
endif()
string(REGEX REPLACE "^([0-9]+).*" "\\1" URDFDOM_HEADERS_MAJOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_MINOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_REVISION_VERSION "${urdfdom_headers_VERSION}")

# 配置兼容性头文件
set(generated_compat_header "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/urdfdom_compatibility.h")
include_directories("${CATKIN_DEVEL_PREFIX}/include")
configure_file(urdfdom_compatibility.h.in "${generated_compat_header}" @ONLY)

# 添加C++11编译选项
add_compile_options(-std=c++11)

# 定义catkin包和依赖关系
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/include
        CATKIN_DEPENDS rosconsole_bridge roscpp
        DEPENDS urdfdom_headers urdfdom Boost TinyXML
)

# 安装兼容性头文件
install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 包括必要的头文件和链接目录
include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS}
        ${urdfdom_headers_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

# 添加库
add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

# 平台相关的编译选项
if(WIN32)
target_compile_definitions(${PROJECT_NAME} PRIVATE "URDF_BUILDING_LIBRARY")
endif()

if(APPLE)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

# 测试部分（如果启用）
if(CATKIN_ENABLE_TESTING)
  #message("----------------------------------------------------------------------------Building tests")
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest_gtest(test_urdf_parser test/test_robot_model_parser.launch test/test_robot_model_parser.cpp)
  target_link_libraries(test_urdf_parser ${PROJECT_NAME})
  catkin_add_gtest(urdfdom_compatibility_test test/urdfdom_compatibility.cpp)
endif()

# 安装目标
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
