cmake_minimum_required(VERSION 3.23)
project(Math LANGUAGES CXX)

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

set(Math_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BoundingBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/EngineMath.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/EngineMathMatrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/EngineMathQuaternion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/EngineMathVectors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Frustum.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MathPCH.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MathUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrix2x2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrix3x3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrix4x4.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Quaternion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Ray.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vector2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vector3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vector4.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vertex.cpp
)

set(Math_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/BoundingBox.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/EngineMath.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/EngineMathMatrix.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/EngineMathQuaternion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/EngineMathVectors.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Frustum.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Grid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/MathPCH.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/MathUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Matrix2x2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Matrix3x3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Matrix4x4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OrthographicProjection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/PrespectiveProjection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Quaternion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Ray.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Transform.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeMatrix2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeMatrix3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeMatrix4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeVector2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeVector3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/TypeVector4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vector2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vector3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vector4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vertex.h
)

add_library(Math STATIC)

target_sources(Math
	PRIVATE
		${Math_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Math_Headers}	        
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Math/include/
target_include_directories(Math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}) # exposes Math/include/
target_include_directories(Math PRIVATE "Extern/include")

target_precompile_headers(Math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/MathPCH.h)

# Tells the compiler to optimize for the current CPU architecture (AVX2/FMA)
if(NOT MSVC)
    add_compile_options(
    -march=native
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3>
)
endif()
