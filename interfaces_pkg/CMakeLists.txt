cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

# Encontrar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generar las interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Movimiento.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Soporte a tests (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
