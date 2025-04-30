cmake_minimum_required(VERSION 3.0.2)
project(inspection_pkg)

## 声明CATKIN依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

## 声明Python模块安装（关键新增）
catkin_python_setup()  # 自动检测Python包结构

## 安装Python脚本（原有scripts目录）
catkin_install_python(
  PROGRAMS scripts/main_node.py
           scripts/color_detection.py
           scripts/obstacle_avoidance.py
           scripts/black_line_detection.py
           scripts/deepseek_agent.py
           scripts/deepseek_prompt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装Python模块（新增motion_library目录）
install(
  DIRECTORY motion_library/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/motion_library
  FILES_MATCHING PATTERN "*.py"
)

###
catkin_package(
    CATKIN_DEPENDS rospy cv_bridge sensor_msgs
)

catkin_install_python(
    PROGRAMS
        scripts/main_inspection.py
        scripts/color_detection.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)