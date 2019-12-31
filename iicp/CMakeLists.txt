cmake_minimum_required(VERSION 2.8)
project(iaicp)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
  # SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -march=native")

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


message("PROJECT_SOURCE_DIR: " ${OpenCV_DIR})
find_package(OpenCV 4.1 REQUIRED)
include_directories( "/usr/include/eigen3" )
find_package(Boost REQUIRED)
if(Boost_FOUND)
    message("Boost was found!")
    message("Boost Headers DIRECTORY: " ${Boost_INCLUDE_DIRS})
    message("Boost LIBS DIRECTORY: " ${Boost_LIBRARY_DIRS})
    message("Found Libraries: " ${Boost_LIBRARIES})
endif()
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/PCLConfig.cmake")
# add for point cloud viewer and mapper
find_package(PCL 1.7 REQUIRED)
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
set( Sophus_LIBRARIES libSophus.so)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)


add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})
message("PCL_INCLUDE_DIR" ${PCL_INCLUDE_DIRS})
message("PCL_LIBRARIES_DIR" ${PCL_LIBRARY_DIRS})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}
src/iaicp.cpp
src/camera_tracker.cpp
src/config.cpp
src/frame.cpp
src/camera.cpp
src/map.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${Sophus_LIBRARIES}
)
message("PCL_LIBRARIES source path" ${PCL_LIBRARIES})
# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(main
src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})
