cmake_minimum_required(VERSION 3.8)
project(cl2703)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install( # Launch files
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)
install( # script files
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
