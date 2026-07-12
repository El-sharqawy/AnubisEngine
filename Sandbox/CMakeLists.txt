cmake_minimum_required(VERSION 3.23)

project(Sandbox LANGUAGES CXX)

set(Sandbox_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/Main.cpp
)

add_executable(Sandbox
    ${Sandbox_Sources}
)

target_link_libraries(Sandbox
    PRIVATE
        Engine
)

target_include_directories(Sandbox
    PRIVATE
        ${CMAKE_SOURCE_DIR}/Engine
)

set_target_properties(Sandbox PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

if (MSVC)
    target_compile_options(Sandbox PRIVATE /MP)
endif()
