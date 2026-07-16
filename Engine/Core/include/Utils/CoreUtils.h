#pragma once

#include <string>
#include <utility>
#include <assimp/types.h>
#include "API/ShaderProgram.h"
#include "TypeMatrix4.h"
#include <assimp/scene.h>			// Output data structure

namespace Anubis
{
    /**
     * @brief Extracts the file name from a full file path using modern C++ techniques.
     *
     * @param full_path The full file path as a C++ string.
     * @return The extracted file name as a std::string.
     */
    std::string GetFileNameModern(const char* full_path);

    /**
     * @brief Extracts the file name from a full file path using C-style techniques
     *
     * @param full_path The full file path as a C-style string.
     * @return The extracted file name as a std::string.
     */
    const char* GetFileName(const char* filepath);

    /**
     * @brief Extracts the dir name from a full file path using modern C++ techniques
     *
     * @param full_path The full file path as a C++ string.
     * @return The extracted file dir as a std::string.
     */
    std::string GetDirFromFilename(const std::string& Filename);

    /**
     * @brief Constructs a full file path by cleaning and combining a directory and a relative path
     *
     * @param Dir The base directory path as a C++ string.
     * @param Path The relative path (Assimp aiString) to be cleaned and appended.
     * @return The concatenated and formatted full path as a std::string.
     */
    std::string GetFullPath(const std::string& Dir, const aiString& Path);

    /**
     * @brief Safely deletes a pointer and sets it to nullptr.
     *
     * This template function checks if the provided pointer is non-null,
     * deletes the object it points to, and then sets the pointer to nullptr
     * to prevent dangling references.
     *
     * @tparam T The type of the object being pointed to.
     * @param ptr Reference to the pointer to be deleted.
     */
    template<typename T>
    void SafeDelete(T*& ptr)
    {
        if (ptr)
        {
            delete ptr;
            ptr = nullptr;
        }
    }

    /**
     * @brief Safely deletes an array pointer and sets it to nullptr.
     *
     * This template function checks if the provided array pointer is non-null,
     * deletes the array it points to, and then sets the pointer to nullptr
     * to prevent dangling references.
     *
     * @tparam T The type of the objects in the array.
     * @param ptr Reference to the array pointer to be deleted.
     */
    template<typename T>
    inline void SafeDeleteArray(T*& ptr)
    {
        if (ptr)
        {
            delete[] ptr;
            ptr = nullptr;
        }
    }

    /**
     * @brief Safely frees a dynamically allocated resource and sets the pointer to nullptr.
     *
     * This template function checks if the provided pointer is non-null,
     * frees the memory it points to using free(), and then sets the pointer to nullptr
     * to prevent dangling references.
     *
     * @tparam T The type of the resource being pointed to.
     * @param resource Reference to the pointer to be freed.
     */
    inline void SafeFree(void* resource)
    {
        if (resource)
        {
            std::free(resource);
            resource = nullptr;
        }
    }

    template<typename T>
    inline void SafeClearMap(T& mMap)
    {
        for (auto& pair : mMap)
        {
            SafeDelete(pair.second);
        }

        mMap.clear();
    }

    std::string GetShaderStageStr(EShaderStage stage);
    std::string ResolveShaderPath(EGraphicsAPI api, const std::string& baseName, EShaderStage stage);
    Matrix4 AssimpToMatrix4(const aiMatrix3x3& AssimpMatrix);
    Matrix4 AssimpToMatrix4(const aiMatrix4x4& AssimpMatrix);
    void PrintMatrix4(const Matrix4& Matrix);
}