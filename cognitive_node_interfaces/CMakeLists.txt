cmake_minimum_required(VERSION 3.8)
project(cognitive_node_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Activation.msg"
  "msg/Value.msg"
  "msg/Perception.msg"
  "msg/PerceptionLayout.msg"
  "msg/PerceptionParameters.msg"
  "srv/AddPoint.srv"
  "srv/AddNeighbor.srv"
  "srv/DeleteNeighbor.srv"
  "srv/Evaluate.srv"
  "srv/Execute.srv"
  "srv/GetActivation.srv"
  "srv/GetInformation.srv"
  "srv/GetReward.srv"
  "srv/GetSuccessRate.srv"
  "srv/IsCompatible.srv"
  "srv/IsReached.srv"
  "srv/IsSatisfied.srv"
  "srv/Predict.srv"
  "srv/SetActivation.srv"
  "srv/SetActivationTopic.srv"
  "srv/SetInputs.srv"
  "srv/SetWeight.srv"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
