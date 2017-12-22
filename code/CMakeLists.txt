cmake_minimum_required(VERSION 3.0)

project(PlaneStereo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/MRF2.2)

find_package(OpenCV REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen REQUIRED)

find_package(OpenMesh REQUIRED)

add_executable(PlaneStereo main.cpp mesh.h mesh.cc
        plane3D.h plane3D.cpp
        plane_stereo.h plane_stereo.cpp)
include_directories(${EIGEN_INCLUDE_DIR})

target_link_libraries(PlaneStereo MRF ${OpenCV_LIBS} ${GFLAGS_LIBRARY} ${GLOG_LIBRARY} ${OPENMESH_LIBRARIES})
