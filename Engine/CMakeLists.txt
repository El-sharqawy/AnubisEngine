cmake_minimum_required(VERSION 3.23)
project(Engine LANGUAGES CXX)

# Set the C standard to C23 or latest
if (MSVC)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    # This ensures C flags ONLY go to .c files
    add_compile_options($<$<COMPILE_LANGUAGE:C>:/std:clatest>)
    # This ensures CXX flags ONLY go to .cpp files
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/std:c++20>)
    add_compile_options(/utf-8)        # Treat source as UTF-8
    add_compile_options(/MP)
else()
	set(CMAKE_C_STANDARD 23)
	set(CMAKE_C_STANDARD_REQUIRED ON)
	set(CMAKE_C_EXTENSIONS ON) # Set to ON if you want 'gnu23' features
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED YES)
    add_compile_options(/utf-8)        # Treat source as UTF-8
endif()

set(CMAKE_BUILD_TYPE Debug)

add_library(Engine STATIC)

target_sources(Engine
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/AnubisEngine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Stdafx.cpp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/AnubisEngine.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Stdafx.h
)

# Make Engine root visible to itself and sub-libraries
target_include_directories(Engine
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}       # contains Stdafx.h
        ${CMAKE_SOURCE_DIR}/Extern/Include   # AnubisEngine/Extern/Include
)

include_directories(BEFORE "${CMAKE_SOURCE_DIR}/Extern/include")

# 2. Tell CMake where the compiled .a or .so or .lib files are
link_directories("${CMAKE_SOURCE_DIR}/Extern/lib")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# Global PCH for Engine target itself
target_precompile_headers(Engine
    PUBLIC         # export the PCH to dependants
        "Stdafx.h" # must be reachable through include dirs above
)

target_link_libraries(Engine
    PUBLIC
        Math
        Core
        Renderer

        # Extern Libraries
        glad
        stb_image
)

add_subdirectory(Math)
add_subdirectory(Core)
add_subdirectory(Renderer)

if (WIN32)
target_link_libraries(Engine PRIVATE 
    # Vulkan Library
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/vulkan/vulkan-1"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/vulkan/vulkan-1"> # .lib

    # External libraries (Windows specific)
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/glfw3_d"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/glfw3"> # .lib

    # Assimp Library and Mesh Parts
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/assimp-vc145-mtd"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/assimp-vc145-mt"> # .lib
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/zlibstatic_mtd"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/zlibstatic_mt"> # .lib

    # Python
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/python3_d"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/python3"> # .lib
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/python314_d"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/python314"> # .lib

    # FreeType GL
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/freetype_d"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/freetype"> # .lib
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Debug/freetypegl_d"> # .lib
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Windows/Release/freetypegl"> # .lib

    # Other standard Windows system 
    opengl32.lib
    gdi32.lib 
    user32.lib 
    shell32.lib
)
else()
target_link_libraries(Engine PRIVATE 
    $<$<CONFIG:Debug>:"${CMAKE_SOURCE_DIR}/Extern/lib/Linux/libglfw_d.so">
    $<$<CONFIG:Release>:"${CMAKE_SOURCE_DIR}/Extern/lib/Linux/libglfw.so">
    GL
    dl
    m
    pthread
)
endif()