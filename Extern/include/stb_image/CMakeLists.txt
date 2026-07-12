cmake_minimum_required(VERSION 3.23)
project(stb_image LANGUAGES C CXX)

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

add_library(stb_image STATIC)

target_sources(stb_image
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/stb_image.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/stb_image_write.cpp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/stb_image.h
        ${CMAKE_CURRENT_SOURCE_DIR}/stb_image_write.h
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(stb_image PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes stb_image/include/

# Tells the compiler to optimize for the current CPU architecture (AVX2/FMA)
if(NOT MSVC)
target_compile_options(stb_image PRIVATE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-march=native>
    $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Debug>>:-O0 -g>
    $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Release>>:-O3>
)
endif()
