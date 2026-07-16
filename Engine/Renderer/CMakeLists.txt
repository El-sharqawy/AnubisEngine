cmake_minimum_required(VERSION 3.23)
project(Renderer LANGUAGES CXX)

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
set(Renderer_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera/Camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Device/OpenGLRenderDevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Device/PipelinesManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Device/VulkanRenderDevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLBindingContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLBuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLCommandList.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLMaterial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLPipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLShader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLTexture2D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/OpenGLVertexArray.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Textures/TexturesManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanBuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanCommandList.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanDescriptorContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanDescriptorSet.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanDevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanMaterial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanPipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanShader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanSwapchain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanSyncObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanTexture2D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Vulkan/VulkanUtils.cpp
)

set(Renderer_Headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Camera/Camera.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Device/OpenGLRenderDevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Device/PipelinesManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Device/VulkanRenderDevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLBindingContext.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLBuffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLCommandList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLMaterial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLPipeline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLRenderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLShader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLTexture2D.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/OpenGL/OpenGLVertexArray.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Textures/TexturesManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanBuffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanCommandList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanDescriptorContext.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanDescriptorSet.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanDevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanMaterial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanPipeline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanRenderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanShader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanSwapchain.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanSyncObject.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanTexture2D.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/Vulkan/VulkanUtils.h
)

add_library(Renderer STATIC)

target_sources(Renderer
	PRIVATE
		${Renderer_Sources}

	PUBLIC
	FILE_SET HEADERS
	BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} # Use absolute path for safety
	FILES
		${Renderer_Headers}
)

# This ensures that library knows where its own headers are during its own compilation
target_include_directories(Renderer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include) # exposes Renderer/include/
target_include_directories(Renderer PRIVATE "Extern/include")

target_link_libraries(Renderer
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
