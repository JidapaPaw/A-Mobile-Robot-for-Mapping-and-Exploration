cmake_minimum_required(VERSION 3.8)
project(serial_motor_demo)

# ---------- Build options ----------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- Dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# ถ้าโค้ดคุณมีใช้พวกนี้ ให้ "ปลดคอมเมนต์" แล้วเพิ่มใน ament_target_dependencies ด้านล่างด้วย
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

# ถ้าคุณมี custom message package เช่น serial_motor_demo_msgs ให้ปลดคอมเมนต์
# find_package(serial_motor_demo_msgs REQUIRED)

# ---------- Executable ----------
add_executable(serial_motor_demo_node
  src/serial_motor_demo_node.cpp
)

# ใส่ dependency ให้ตรงกับที่ include ในโค้ดคุณ
ament_target_dependencies(serial_motor_demo_node
  rclcpp
  geometry_msgs
  # std_msgs
  # nav_msgs
  # tf2
  # tf2_ros
  # serial_motor_demo_msgs
)

# ---------- Install ----------
install(TARGETS
  serial_motor_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# ถ้ามี launch/params ให้ติดตั้งด้วย (ไม่มีก็ไม่เป็นไร)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---------- Testing (optional) ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
