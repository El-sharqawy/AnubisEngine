cmake_minimum_required(VERSION 3.23)
project(Terrain LANGUAGES CXX)

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
set(Terrain_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Terrain/Terrain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Terrain/TerrainPatch.cpp
)

set(Terrain_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Terrain/Terrain.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Terrain/TerrainData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Terrain/TerrainPatch.h
)

add_library(Terrain STATIC)

target_sources(Terrain
	PRIVATE
		${Terrain_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Terrain_Headers}
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Terrain PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Terrain/include/
target_include_directories(Terrain PRIVATE "Extern/include")

target_link_libraries(Terrain
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
