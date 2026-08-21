cmake_minimum_required(VERSION 3.25)
project(City_Sweep)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/GVK-Engine)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/reactphysics3d)

add_executable(City_Sweep main.cpp)
target_link_libraries(City_Sweep PRIVATE
        gvk::gvk
        reactphysics3d
)