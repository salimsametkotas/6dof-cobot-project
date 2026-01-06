cmake_minimum_required(VERSION 3.8)
project(cobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# BAĞIMLILIKLARIN BULUNMASI

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(cobot_interfaces REQUIRED)

# PARAMETRE KÜTÜPHANESİ OLUŞTURMA

generate_parameter_library(cobot_controller_parameters
  src/cobot_controller_parameter.yaml
)

# KÜTÜPHANE OLUŞTURMA 

add_library(
  ${PROJECT_NAME} SHARED
  src/cobot_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  cobot_controller_parameters
  rclcpp_action::rclcpp_action
  ${cobot_interfaces_LIBRARIES}
)

# Include dizinlerini ayarla
target_include_directories(cobot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cobot_controller>
  ${cobot_interfaces_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

# Tüm bağımlılıkları hedefe bağla
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  geometry_msgs
  tf2
  tf2_msgs
  pluginlib
)

# PLUGIN TANIMLAMASI 

pluginlib_export_plugin_description_file(controller_interface cobot_control_plugin.xml)

# YÜKLEME (INSTALL) KURALLARI

# Header dosyalarını yükle
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Kütüphaneleri ve Parametreleri yükle
install(TARGETS cobot_controller cobot_controller_parameters
  EXPORT export_cobot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES cobot_control_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

#EXPORT VE PAKETLEME

ament_export_targets(export_cobot_controller HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  controller_interface
  geometry_msgs
  cobot_interfaces 
)

ament_package()
