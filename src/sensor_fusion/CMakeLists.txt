cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  sensor_fusion/adaptive_fusion.py
  sensor_fusion/status_display.py
  sensor_fusion/status_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
