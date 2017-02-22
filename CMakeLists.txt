cmake_minimum_required(VERSION 3.6)
project(untitled)

find_package(PCL 1.4 REQUIRED)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})

add_executable(filter filter.cpp)
target_link_libraries (filter ${PCL_LIBRARIES})

set(SOURCE_FILES main.cpp pairwise_incremental_registration.cpp filter.cpp)
add_executable(untitled ${SOURCE_FILES})