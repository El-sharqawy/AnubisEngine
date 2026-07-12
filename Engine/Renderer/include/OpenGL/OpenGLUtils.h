#pragma once

#include <string>
#include "API/ShaderProgram.h"
#include "API/Buffer.h"
#include "API/Texture.h"
#include <glad/gl.h>

namespace OpenGLUtils
{
    std::vector<char> ReadShaderData(const std::string& filename);

    bool CheckGLErrors(const std::string& stMessage);
    /**
     * @brief Checks if the current OpenGL version is greater than or equal to the specified version.
     *
     * @param MajorVer The major version to check (e.g., 4)
     * @param MinorVer The minor version to check (e.g., 5)
     * @return true if current version >= specified version
     */
    bool IsGLVersionHigher(int32_t MajorVer, int32_t MinorVer);

    /**
     * Checks for shader compilation or program linking errors.
     * Prints detailed error messages if compilation/linking fails.
     *
     * @param shader    Shader or program ID
     * @param type      "vertex", "fragment", "program", etc.
     * @param name      Shader filename for error reporting
     * @return true if no errors, false if compilation/linking failed
     */
    bool CheckShaderCompileErrors(uint32_t uiShaderID, EShaderStage shaderStage);
    bool CheckProgramLinkingError(uint32_t uiProgramID);

    // Bufffers
    GLbitfield ToGLBufferStorageFlags(EBufferMemoryType bufferMemoryType);
    GLenum ToGLBufferUsage(EBufferMemoryType bufferMemoryType);
    GLenum ToGLBufferType(EBufferType bufferType);
    GLenum ToGLBufferBindingPoint(EBufferBindingPoints bufferBindingPoint);

    // Textures
    GLenum ToGLTextureType(ETextureType textureType);
    GLenum ToGLTexureInternalFormat(ETextureFormats textureFormat); // Internal Format
    GLenum ToGLTexurePixelFormat(ETextureFormats textureFormat); // Pixel Format
    GLenum ToGLTexturePixelType(ETextureFormats textureFormat); // Pixels Type .. Floats .. Bytes
    GLenum ToGLTextureMinFilter(ETextureFilter textureMinFilter, ETextureMipmapMode textureMipmapMode);
    GLenum ToGLTextureMagFilter(ETextureFilter textureMagFilter);
    GLenum ToGlTextureWrap(ETextureWrap textureWrap);

    // Shaders
    GLenum ToGLShaderType(EShaderStage shaderStage);
}