cmake_minimum_required(VERSION 3.23)
project(Model LANGUAGES CXX)

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
set(Model_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Actor/SkeletalActor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Actor/SkeletalActorAsset.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Actor/StaticActor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Actor/StaticActorAsset.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/Animation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/Animator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/Bone.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/SkeletalMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/SkeletalModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/StaticMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Model/StaticModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Services/ActorsManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Services/AnimationsManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Services/AssimpModelImporter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Services/RenderQueue.cpp
)

set(Model_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Actor/SkeletalActor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Actor/SkeletalActorAsset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Actor/StaticActor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Actor/StaticActorAsset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/Animation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/Animator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/Bone.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/SkeletalMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/SkeletalMeshData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/SkeletalModel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/Skeleton.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/StaticMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/StaticMeshData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Model/StaticModel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Services/ActorsManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Services/AnimationsManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Services/AssimpModelImporter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Services/RenderQueue.h
)

add_library(Model STATIC)

target_sources(Model
	PRIVATE
		${Model_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Model_Headers}
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Model/include/
target_include_directories(Model PRIVATE "Extern/include")

target_link_libraries(Model
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
