cmake_minimum_required(VERSION 3.23)
project(glad LANGUAGES C CXX)

# Set the C++ standard to C20 or latest
if (MSVC)
    # We use 'target' level settings instead of global 'add_compile_options'
    # to prevent flag clashing.
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED YES)
endif()

# Then filter out the build directory
add_library(glad STATIC)

target_sources(glad
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/gl.c
        ${CMAKE_CURRENT_SOURCE_DIR}/vulkan.c
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/gl.h
        ${CMAKE_CURRENT_SOURCE_DIR}/vulkan.h
)

include_directories(BEFORE "${CMAKE_SOURCE_DIR}/Extern/include")

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes glad/include/

# Tells the compiler to optimize for the current CPU architecture (AVX2/FMA)
if(NOT MSVC)
target_compile_options(glad PRIVATE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-march=native>
    $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Debug>>:-O0 -g>
    $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Release>>:-O3>
)
endif()
