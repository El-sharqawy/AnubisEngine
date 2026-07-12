cmake_minimum_required(VERSION 3.23)
project(AnubisEngineRoot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the root of your engine directory as the output target
set(OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Redirect all Executables (like your metin2client_Debug.exe)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIR}")

add_subdirectory(Extern/include/glad)
add_subdirectory(Extern/include/stb_image)

# Options to enable/disable executables if you want
option(ANUBIS_BUILD_SANDBOX "Build Sandbox executable" ON)
option(ANUBIS_BUILD_EDITOR "Build Editor executable" OFF)

# Engine library (must be added first so Sandbox/Editor can link it)
add_subdirectory(Engine)

# Apps
if (ANUBIS_BUILD_SANDBOX)
    add_subdirectory(Sandbox)
endif()

if (ANUBIS_BUILD_EDITOR)
    add_subdirectory(Editor)
endif()

