cmake_minimum_required(VERSION 3.11)
project(ow_core)

###################
## Configuration ##
###################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# ROS dependecies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
    ow_msgs
)

# System dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Package configuration (this information is passed to projects/packages that
# depend on this package)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        tf
        tf_conversions
        ow_msgs
        
#    DEPENDS ${DEPENDS_VAR}
)

###########
## Build ##
###########

# Location of header files (First package headers always!)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



### Binaries ###
## C++ Library
set(SOURCES
    src/conversions.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

# Library dependencies. Always use:
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# Special CXXFlags for the libraries
# set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11

# linking libraries (from ros packages and system libraries)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## C++ Executables

set(APP_TARGETS
    ${PROJECT_NAME}_test2
    ${PROJECT_NAME}_test3
    ${PROJECT_NAME}_test4
    ${PROJECT_NAME}_test5
    ${PROJECT_NAME}_test6
    ${PROJECT_NAME}_test7
    ${PROJECT_NAME}_test8
    ${PROJECT_NAME}_test9
    ${PROJECT_NAME}_test10
    ${PROJECT_NAME}_test11
    ${PROJECT_NAME}_test12
    ${PROJECT_NAME}_test13
    ${PROJECT_NAME}_test14
    ${PROJECT_NAME}_test15
    ${PROJECT_NAME}_test17
    ${PROJECT_NAME}_test18
    ${PROJECT_NAME}_test19
    ${PROJECT_NAME}_test21
    )

foreach(target ${APP_TARGETS})
        add_executable(${target} src/applications/main_${target}.cpp)
        add_dependencies(${target}
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS})

        # Special CXXFlags for the binaries
        # set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

        # Generate a short name and define prefix for the target
        # set_target_properties(
        # ${target} PROPERTIES OUTPUT_NAME node PREFIX "")

        target_link_libraries(${target}
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
        )
endforeach()

#############
## Install ##
#############

# don't install private headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*_p.h" EXCLUDE
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )



##Additional dirs and files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

set(TEST_TARGETS
    #conversions
    vector3
    angular_position
    angular_velocity
    angular_acceleration
    state_updater
    force
    matrix
    linear_position
    linear_acceleration
    linear_velocity
    force_torque_sensor
    imu_sensor
    #cartesian_position
    #homogeneous_transformation
    #cartesian_velocity
    #cartesian_state
)
if(CATKIN_ENABLE_TESTING)
foreach(target ${TEST_TARGETS})
  catkin_add_gtest(${target} src/tests/test_${target}.cpp)
  add_dependencies(${target}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS})

  set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

  target_link_libraries(${target}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )
endforeach()
endif()

