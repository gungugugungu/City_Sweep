cmake_minimum_required(VERSION 3.25)
project(City_Sweep)

set(CMAKE_CXX_STANDARD 23)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON  CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/GVK-Engine)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/reactphysics3d)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/SDL_mixer)

add_executable(City_Sweep main.cpp)

# hide console
if (WIN32)
    set_target_properties(City_Sweep PROPERTIES WIN32_EXECUTABLE TRUE)
    if (MSVC)
        target_link_options(City_Sweep PRIVATE /ENTRY:mainCRTStartup)
    endif()
endif()

target_link_libraries(City_Sweep PRIVATE
        gvk::gvk
        reactphysics3d
        SDL3_mixer::SDL3_mixer
)

install(TARGETS City_Sweep RUNTIME DESTINATION bin)
install(DIRECTORY textures/ DESTINATION textures)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY sounds/ DESTINATION sounds)
install(DIRECTORY fonts/ DESTINATION fonts)
install(DIRECTORY scenes/ DESTINATION scenes)