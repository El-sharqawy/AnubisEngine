cmake_minimum_required(VERSION 3.23)
project(Core LANGUAGES CXX)

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
set(Core_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Crc/CRC32.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Logging/LogManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Memory/MemoryManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils/CoreUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Config/ConfigManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Input/InputManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Time/TimerManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Window/WindowManager.cpp
)

set(Core_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/BindingContext.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/Buffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/CommandList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/Entity.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/IComponent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/Material.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/Pipeline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/RenderDevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/RenderObject.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/ShaderProgram.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/API/Texture.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Crc/CRC32.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/Event.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/EventBus.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/EventDispatcher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/EventKeys.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/EventMouse.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/Events.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Events/EventWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Logging/LogManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Logging/LogManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Memory/MemoryTags.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Utils/AnubisAssert.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Utils/CoreUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Config/ConfigManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Input/InputManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Time/TimerManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Window/WindowManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Dynamic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/ServiceLocator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Singleton.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/CoreDefines.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/CorePlatform.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/CoreEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/CoreTypes.h
)

add_library(Core STATIC)

target_sources(Core
	PRIVATE
		${Core_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Core_Headers}	        
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Core/include/
target_include_directories(Core PRIVATE "Extern/include")

target_link_libraries(Core
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
