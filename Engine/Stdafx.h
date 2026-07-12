// stdafx.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#pragma once

// add headers that you want to pre-compile here

#include "CoreDefines.h"
#include "CoreEnums.h"

#if defined(ANUBIS_PLATFORM_WINDOWS)
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h> // <<--- Here we go
#pragma comment(lib, "winmm.lib")
#endif

// OpenGL Related Files, Extern Libs
#include <vulkan/vulkan.h>

#define GLFW_INCLUDE_VULKAN
#define GLFW_EXPOSE_NATIVE_WIN32
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <nlohmann/json.hpp>
#include <stb_image/stb_image.h>
#include <stb_image/stb_image_write.h>
#include <Python/Python.h>

// Standard C++ Libraries
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <chrono>
#include <ctime>
#include <stdarg.h>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include <mutex>
#include <map>
#include <regex> // For std::sort

#include "ServiceLocator.h"
#include "Singleton.h"
#include "Utils/AnubisAssert.h"