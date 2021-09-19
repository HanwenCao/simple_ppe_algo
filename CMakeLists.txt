cmake_minimum_required(VERSION 3.5)
# auto supermarket marker detection & plane pose estimation
project(auto_market_ppe VERSION 0.0.1) 

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall )
endif()



# set params
set(PUBLIC_HEADER_FILES include/handeyeTarget.hpp
		    	 include/handeyeTargetConfig.hpp)

# Set realsense dependency
set(realsense2_FOUND true)
set(realsense2_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")
set(realsense2_LIBRARIES "${realsense2_LIBRARY_DIRS}/librealsense2.so")

# Find dependencies ####
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(apriltag REQUIRED)


#######################################
# Add handeye target library
add_library(${PROJECT_NAME} SHARED 
        src/handeyeTarget.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS} 
        # ${apriltag_INCLUDE_DIRS}
        ${realsense2_LIBRARY_DIRS}
)


target_link_libraries(${PROJECT_NAME} 
        ${OpenCV_LIBS} 
        # ${apriltag_LIBS}
        ${realsense2_LIBRARIES}
)


ament_export_dependencies(realsense2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


# # install
# install(TARGETS ${PROJECT_NAME}
#     DESTINATION $ENV{HOME}/SmartGraspingPlatform/Modules/handeye_calibration/lib
#     DESTINATION lib
# )

# install(DIRECTORY include/ 
#     DESTINATION $ENV{HOME}/SmartGraspingPlatform/Modules/handeye_calibration/include
#     DESTINATION include
# )

# executable for testing
add_executable(testHandeyeTarget test/handeyeTargetTest.cpp)
target_link_libraries(testHandeyeTarget ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
