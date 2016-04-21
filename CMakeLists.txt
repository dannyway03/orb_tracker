cmake_minimum_required(VERSION 2.8.3)

project(orb_tracker)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  image_geometry
  message_filters
  cv_bridge
  tf
  cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen REQUIRED)

catkin_package()

include_directories(include
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

set(LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

add_library(lib_orb_tracker
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
)
target_link_libraries(lib_orb_tracker ${LIBS})

# ROS node
add_executable(orb_tracker src/node.cc)
target_link_libraries(orb_tracker lib_orb_tracker ${LIBS})