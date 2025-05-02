cmake_minimum_required(VERSION 3.5)
project(gazebo_sfm_plugin)

# ------------------------------
# 设置 C++ 标准
# ------------------------------
set(CMAKE_CXX_STANDARD 14)  # 升级到C++14，更符合ROS2推荐标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项，启用警告
if(NOT WIN32)
  add_compile_options(-Wall -Wextra)
endif()

# ------------------------------
# 查找必要的包
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ------------------------------
# 生成服务接口
# ------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PedState.srv"
  DEPENDENCIES geometry_msgs  # 添加消息依赖
)

# 生成导出依赖
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# ------------------------------
# 添加头文件目录
# ------------------------------
include_directories(
  include
  /usr/local/include  # 用于查找 lightsfm 库
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}  # 添加Gazebo的包含目录
)

# ------------------------------
# 编译共享库（插件）
# ------------------------------
add_library(PedestrianSFMPlugin SHARED src/pedestrian_sfm_plugin.cpp)

# 使用现代CMake方式设置属性
target_compile_features(PedestrianSFMPlugin PUBLIC cxx_std_14)

# 添加 ament 索引的包含路径和依赖
ament_target_dependencies(PedestrianSFMPlugin
  rclcpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# ------------------------------
# 链接依赖库
# ------------------------------
target_link_libraries(PedestrianSFMPlugin
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  "${cpp_typesupport_target}"  # 链接生成的服务接口
)

# ------------------------------
# 安装规则
# ------------------------------
# 安装共享库
install(TARGETS PedestrianSFMPlugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 将 launch和urdf文件夹拷贝到 share目录下，因为C++的功能包构建就是这样的
install(DIRECTORY launch test_world
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------
# 导出依赖
# ------------------------------
ament_export_include_directories(include)
ament_export_libraries(PedestrianSFMPlugin)
ament_export_dependencies(
  rclcpp 
  gazebo_ros 
  gazebo
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# ------------------------------
# 完成包配置
# ------------------------------
ament_package()
