cmake_minimum_required(VERSION 3.11)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)

add_executable(manus_data_publisher 
  src/manus_data_publisher.cpp
  src/ClientPlatformSpecific.cpp
  src/ManusDataPublisher.cpp)

# 这里用当前包里的 ManusSDK 目录
target_include_directories(manus_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/ManusSDK/include
)

target_link_directories(manus_data_publisher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/ManusSDK/lib
)

# ✅ 使用 integrated 版，避免外部 GRPC / Protobuf 依赖
target_link_libraries(manus_data_publisher
  ManusSDK_Integrated
  ncurses
)

target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)

install(TARGETS manus_data_publisher
  DESTINATION lib/${PROJECT_NAME})

# 这里也改成包内 ManusSDK/lib
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ManusSDK/lib)

# 同样使用 integrated 版本的 .so
set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK_Integrated.so)

install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})

# 安装校准文件目录
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration)

set_target_properties(manus_data_publisher PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs ament_index_cpp)

ament_package()
