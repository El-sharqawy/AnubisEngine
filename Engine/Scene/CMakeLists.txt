cmake_minimum_required(VERSION 3.23)
project(Scene LANGUAGES CXX)

# Set the C++ standard to C20 or latest
if (MSVC)
    # We use 'target' level settings instead of global 'add_compile_options'
    # to prevent flag clashing.
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_compile_options(/MP)
else()
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED YES)
endif()

# Set all sources recursively
set(Scene_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Core/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Core/SceneManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Entity/LightComponent.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Entity/SkeletalMeshComponent.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Entity/TransformComponent.cpp
)

set(Scene_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Core/Scene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Core/SceneManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Entity/LightComponent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Entity/SkeletalMeshComponent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Entity/TransformComponent.h
)

add_library(Scene STATIC)

target_sources(Scene
	PRIVATE
		${Scene_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Scene_Headers}
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Scene/include/
target_include_directories(Scene PRIVATE "Extern/include")

target_link_libraries(Scene
    PUBLIC
        Engine
)

# Tells the compiler to optimize for the current CPU architecture (AVX2/FMA)
if(NOT MSVC)
    add_compile_options(
    -march=native
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3>
)
endif()
