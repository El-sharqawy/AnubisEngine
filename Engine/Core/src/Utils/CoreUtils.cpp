#include "Stdafx.h"
#include "Utils/CoreUtils.h"

/**
 * @brief Extracts the file name from a full file path using modern C++ techniques.
 *
 * @param full_path The full file path as a C-style string.
 * @return The extracted file name as a std::string.
 */
std::string Anubis::GetFileNameModern(const char* full_path)
{
    // 1. Create a path object from the macro output
    std::filesystem::path p(full_path);

    // 2. Use the filename() method
    // This correctly handles both '/' and '\' separators.
    return p.filename().string();
}

const char* Anubis::GetFileName(const char* filepath)
{
	// Find last backslash (Windows) or forward slash (Unix)
	// 1. Find the last backslash
	const char* last_backslash = strrchr(filepath, '\\');
	// 2. Find the last forward slash
	const char* last_forward = strrchr(filepath, '/');

	// Determine which one is later in the string
	// 3. We want the one that appears LATEST in the string
	const char* last_slash = (last_backslash > last_forward) ? last_backslash : last_forward;

	// 4. If we found any slash, return the char after it. Otherwise, return the original.
	return (last_slash) ? (last_slash + 1) : filepath;
}

/**
 * @brief Extracts the dir name from a full file path using modern C++ techniques
 *
 * @param full_path The full file path as a C++ string.
 * @return The extracted file dir as a std::string.
 */
std::string Anubis::GetDirFromFilename(const std::string& Filename)
{
    // Extract the directory part from the file name
    std::string::size_type SlashIndex;

#ifdef _WIN64
    SlashIndex = Filename.find_last_of("\\");

    if (SlashIndex == -1)
    {
        SlashIndex = Filename.find_last_of("/");
    }
#else
    SlashIndex = Filename.find_last_of("/");
#endif

    std::string Dir;

    if (SlashIndex == std::string::npos)
    {
        Dir = ".";
    }
    else if (SlashIndex == 0)
    {
        Dir = "/";
    }
    else
    {
        Dir = Filename.substr(0, SlashIndex);
    }

    return Dir;

}

/**
 * @brief Constructs a full file path by cleaning and combining a directory and a relative path
 *
 * @param Dir The base directory path as a C++ string.
 * @param Path The relative path (Assimp aiString) to be cleaned and appended.
 * @return The concatenated and formatted full path as a std::string.
 */
std::string Anubis::GetFullPath(const std::string& Dir, const aiString& Path)
{
    std::string p(Path.data);

    if (p == "C:\\\\")
    {
        p = "";
    }
    else if (p.substr(0, 2) == ".\\")
    {
        p = p.substr(2, p.size() - 2);
    }

    std::string FullPath = Dir + "/" + p;

    return FullPath;
}

std::string Anubis::GetShaderStageStr(EShaderStage stage)
{
    switch (stage)
    {
        case EShaderStage::SHADER_TYPE_VERTEX:
            return "vert";
        case EShaderStage::SHADER_TYPE_TESSELLATION_CONTROL:
            return "tcs";
        case EShaderStage::SHADER_TYPE_TESSELLATION_EVALUATION:
            return "tes";
        case EShaderStage::SHADER_TYPE_GEOMETRY:
            return "geom";
        case EShaderStage::SHADER_TYPE_FRAGMENT:
            return "frag";
        case EShaderStage::SHADER_TYPE_COMPUTE:
            return "comp";
        default:
            return "vert";
    }
}

std::string Anubis::ResolveShaderPath(EGraphicsAPI api, const std::string& baseName, EShaderStage stage)
{
    const std::string ext = GetShaderStageStr(stage);

    if (api == EGraphicsAPI::API_VULKAN)
    {
        return "shaders/vulkan/" + baseName + "_" + ext + ".spv";
    }
    return "shaders/opengl/" + baseName + "." + ext;
}