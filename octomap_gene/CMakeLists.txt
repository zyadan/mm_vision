cmake_minimum_required(VERSION 3.8)
project(octomap_gene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(DEPENDENCIES
    rclcpp 
    pcl_conversions 
    sensor_msgs
    std_msgs
    geometry_msgs
    octomap_msgs
    octomap_ros
    tc5_msgs
    PCL 
    pcl_msgs 
    pcl_conversions 
)

# find dependencies

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(INCLUDES

    include/octomap_gene/OctomapGen.hpp

)

set(SOURCES

    src/OctomapGen.cpp
    # src/octo_map_node.cpp
    
    )

include_directories(include
    INCLUDES
)

add_executable(octomap_gene_node src/octo_map_node.cpp)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  )

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)  


target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}

)


target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES}

)

target_link_libraries(octomap_gene_node 
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(octomap_gene_node 
    ${DEPENDENCIES}
) 

install(TARGETS
  octomap_gene_node
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  octomap_gene
  DESTINATION lib)  

  
# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
#   )
  
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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
