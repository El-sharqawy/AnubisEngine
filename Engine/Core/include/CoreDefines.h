#pragma once

#include "Memory/MemoryManager.h"

// Helper macro — wraps the boilerplate call arguments
#define MEM_FUNC   __FUNCTION__, __FILE__, __LINE__

// Engine System Tagged Allocation (Explicit Tags)
#define AnubisCNew(type, tag, align) (type*)CServiceLocator::Get<CMemoryManager>().TrackedMalloc(MEM_FUNC, #type, sizeof(type), tag, align)
#define AnubisCNewZero(type, count, tag, align) (type*)CServiceLocator::Get<CMemoryManager>().TrackedCalloc(MEM_FUNC, #type, count, sizeof(type), tag, align)
#define AnubisCNewZeroArr(type, count, tag, align) (type*)CServiceLocator::Get<CMemoryManager>().TrackedCalloc(MEM_FUNC, #type "[]", count, sizeof(type), tag, align)

// Arrays/Bytes
#define AnubisMalloc(size, tag, align) CServiceLocator::Get<CMemoryManager>().TrackedMalloc(MEM_FUNC, "raw_bytes", size, tag, align)
#define AnubisMallocArray(size, count, tag, align) CServiceLocator::Get<CMemoryManager>().TrackedMalloc(MEM_FUNC, "raw_bytes []", sizeof(type) * (count), tag, align)
#define AnubisCalloc(count, size, tag, align) CServiceLocator::Get<CMemoryManager>().TrackedCalloc(MEM_FUNC, "raw_bytes", count, size, tag, align)

// Reallocation (Pass NULL for typeName to keep the old one)
#define AnubisRealloc(ptr, size) CServiceLocator::Get<CMemoryManager>().TrackedRealloc(MEM_FUNC, ptr, size);

// Strings
#define AnubisStrdup(szSource, tag) CServiceLocator::Get<CMemoryManager>().TrackedStrdup(MEM_FUNC, szSource, "strdup(" #szSource ")", tag)

// Cleanup
#define AnubisFree(ptr) CServiceLocator::Get<CMemoryManager>().TrackedFree(MEM_FUNC, (void*)(ptr)); (ptr) = nullptr

#define AnubisNew(type, tag, ...) CServiceLocator::Get<CMemoryManager>().TrackedNew<type>(MEM_FUNC, tag, ##__VA_ARGS__)

#define AnubisDelete(ptr) CServiceLocator::Get<CMemoryManager>().TrackedDelete(MEM_FUNC, ptr); (ptr) = nullptr

#define AnubisSafeDelete(ptr) if (ptr) CServiceLocator::Get<CMemoryManager>().TrackedDelete(MEM_FUNC, ptr); (ptr) = nullptr

#define AnubisNewArray(type, count, tag, align) CServiceLocator::Get<CMemoryManager>().TrackedNewArray<type>(MEM_FUNC, count, tag, align)

#define AnubisDeleteArray(ptr, type, count) CServiceLocator::Get<CMemoryManager>().TrackedDeleteArray<type>(MEM_FUNC, ptr, count); (ptr) = nullptr

// Default asset paths
#define LOG_PATH "Assets/Logs/EngineLog.txt"
#define ERROR_LOG_PATH "Assets/Logs/EngineErrorLog.txt"
#define CONFIG_PATH "Assets/Config/EngineConfig.json"
#define SHADERS_PATH "Assets/Config/EngineShaders.json"
#define TEXTURES_PATH "Assets/Config/EngineTextures.json"
#define ACTORS_PATH "Assets/Config/EngineActors.json"

#define DEFAULT_ERROR_TEXTURE_PATH "Assets/Textures/Default/error_texture.png"
#define DEFAULT_WHITE_TEXTURE_PATH "Assets/Textures/Default/white_texture.png"
#define DEFAULT_BLACK_TEXTURE_PATH "Assets/Textures/Default/black_texture.png"
#define DEFAULT_NORMAL_TEXTURE_PATH "Assets/Textures/Default/normal_texture.png"
#define DEFAULT_FONT_PATH "Assets/Fonts/Default/arial.ttf"

#define DEFAULT_WEAPON_PATH "Assets/Models/Weapons/00180.fbx"

#define COLOR_TEXTURE_UNIT              GL_TEXTURE0
#define COLOR_TEXTURE_UNIT_INDEX        0
#define SHADOW_TEXTURE_UNIT             GL_TEXTURE1
#define SHADOW_TEXTURE_UNIT_INDEX       1
#define NORMAL_TEXTURE_UNIT             GL_TEXTURE2
#define NORMAL_TEXTURE_UNIT_INDEX       2
#define RANDOM_TEXTURE_UNIT             GL_TEXTURE3
#define RANDOM_TEXTURE_UNIT_INDEX       3
#define DISPLACEMENT_TEXTURE_UNIT       GL_TEXTURE4
#define DISPLACEMENT_TEXTURE_UNIT_INDEX 4
#define ALBEDO_TEXTURE_UNIT             GL_TEXTURE5
#define ALBEDO_TEXTURE_UNIT_INDEX       5          
#define ROUGHNESS_TEXTURE_UNIT          GL_TEXTURE6
#define ROUGHNESS_TEXTURE_UNIT_INDEX    6
#define MOTION_TEXTURE_UNIT             GL_TEXTURE7
#define MOTION_TEXTURE_UNIT_INDEX       7
#define SPECULAR_EXPONENT_UNIT             GL_TEXTURE8
#define SPECULAR_EXPONENT_UNIT_INDEX       8
#define CASCACDE_SHADOW_TEXTURE_UNIT0       SHADOW_TEXTURE_UNIT
#define CASCACDE_SHADOW_TEXTURE_UNIT0_INDEX SHADOW_TEXTURE_UNIT_INDEX
#define CASCACDE_SHADOW_TEXTURE_UNIT1       GL_TEXTURE9
#define CASCACDE_SHADOW_TEXTURE_UNIT1_INDEX 9
#define CASCACDE_SHADOW_TEXTURE_UNIT2       GL_TEXTURE10
#define CASCACDE_SHADOW_TEXTURE_UNIT2_INDEX 10
#define SHADOW_CUBE_MAP_TEXTURE_UNIT        GL_TEXTURE11
#define SHADOW_CUBE_MAP_TEXTURE_UNIT_INDEX  11
#define SHADOW_MAP_RANDOM_OFFSET_TEXTURE_UNIT       GL_TEXTURE12
#define SHADOW_MAP_RANDOM_OFFSET_TEXTURE_UNIT_INDEX 12
#define DETAIL_MAP_TEXTURE_UNIT                     GL_TEXTURE13
#define DETAIL_MAP_TEXTURE_UNIT_INDEX               13
#define METALLIC_TEXTURE_UNIT                       GL_TEXTURE14
#define METALLIC_TEXTURE_UNIT_INDEX                 14
#define HEIGHT_TEXTURE_UNIT                         GL_TEXTURE15
#define HEIGHT_TEXTURE_UNIT_INDEX                   15
