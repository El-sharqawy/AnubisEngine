#include "OpenGL/OpenGLUtils.h"
#include "Logging/LogManager.h"
#include <glad/gl.h>

std::vector<char> OpenGLUtils::ReadShaderData(const std::string& filename)
{
	// Open at the end (ios::ate) and in binary mode
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open())
	{
		syserr("Failed to Open the file %s\n", filename.c_str());
		return {};
	}

	size_t fileSize = static_cast<size_t>(file.tellg());

	// Crucial for Vulkan SPIR-V: Ensure the vector data size accounts for 4-byte chunks
	// and is naturally aligned to uint32_t elements
	std::vector<char> buffer(fileSize);

	file.seekg(0);
	// Read directly into the uint32_t buffer data pointer safely casted
	if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize))
	{
		syserr("Failed to read shader file: {}", filename.c_str());
		file.close();
		return {};
	}

	file.close();

	return buffer;
}

bool OpenGLUtils::CheckGLErrors(const std::string& stMessage)
{
	uint32_t eError = glGetError();
	if (eError != GL_NO_ERROR)
	{
		syserr(stMessage);
		return false;
	}
	return true;
}

/**
 * @brief Checks if the current OpenGL version is greater than or equal to the specified version.
 *
 * @param MajorVer The major version to check (e.g., 4)
 * @param MinorVer The minor version to check (e.g., 5)
 * @return true if current version >= specified version
 */
bool OpenGLUtils::IsGLVersionHigher(int32_t MajorVer, int32_t MinorVer)
{
	static int32_t glMajorVersion = 0;
	static int32_t glMinorVersion = 0;

	// Only query once for performance
	if (glMajorVersion == 0)
	{
		glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
		glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);
	}

	// Check major version first
	if (glMajorVersion > MajorVer)
	{
		return true;
	}

	// Same major version, check minor
	if (glMajorVersion == MajorVer && glMinorVersion >= MinorVer)
	{
		return true;
	}

	// otherwise, return false
	return (false);
}

/**
 * Checks for shader compilation or program linking errors.
 * Prints detailed error messages if compilation/linking fails.
 *
 * @param shader    Shader or program ID
 * @param type      "vertex", "fragment", "compute", etc.
 * @param name      Shader filename for error reporting
 * @return true if no errors, false if compilation/linking failed
 */
bool OpenGLUtils::CheckShaderCompileErrors(uint32_t uiShaderID, EShaderStage shaderStage)
{
	int32_t iSuccess = 0;
	char c_szInfoLog[1024] = {};
	glGetShaderiv(uiShaderID, GL_COMPILE_STATUS, &iSuccess);
	if (!iSuccess)
	{
		glGetShaderInfoLog(uiShaderID, 1024, nullptr, c_szInfoLog);
		syserr("Error shader: {}, Compilation Error of Stage {}", uiShaderID, static_cast<uint32_t>(shaderStage));
		syserr("Error Log: {}", c_szInfoLog);
	}

	return (iSuccess > 0);
}

/**
 * Checks for shader compilation or program linking errors.
 * Prints detailed error messages if compilation/linking fails.
 *
 * @param shader    Shader or program ID
 * @param type      "vertex", "fragment", "compute", etc.
 * @param name      Shader filename for error reporting
 * @return true if no errors, false if compilation/linking failed
 */
bool OpenGLUtils::CheckProgramLinkingError(uint32_t uiProgramID)
{
	int32_t iSuccess = 0;
	char c_szInfoLog[1024] = {};
	glGetProgramiv(uiProgramID, GL_LINK_STATUS, &iSuccess);
	if (!iSuccess)
	{
		glGetProgramInfoLog(uiProgramID, 1024, nullptr, c_szInfoLog);
		syserr("Error Program: {}, Linking Error", uiProgramID);
		syserr("Error Log: {}", c_szInfoLog);
	}

	return (iSuccess > 0);
}

GLbitfield OpenGLUtils::ToGLBufferStorageFlags(EBufferMemoryType bufferMemoryType)
{
	switch (bufferMemoryType)
	{
	case EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY:
		return (0);

	case EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE:
		return (GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ:
		return (GL_MAP_READ_BIT);

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ_WRITE:
		return (GL_MAP_READ_BIT | GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);

	default:
		return (GL_DYNAMIC_STORAGE_BIT);
	}
}

GLenum OpenGLUtils::ToGLBufferUsage(EBufferMemoryType bufferMemoryType)
{
	switch (bufferMemoryType)
	{
	case EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY:
		return GL_STATIC_DRAW;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE:
		return GL_DYNAMIC_DRAW;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ:
		return GL_STREAM_READ;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ_WRITE:
		return GL_DYNAMIC_READ;

	default:
		return GL_STATIC_DRAW;
	}
}

GLenum OpenGLUtils::ToGLBufferType(EBufferType bufferType)
{
	switch (bufferType)
	{
	case EBufferType::BUFFER_TYPE_UNIFORM:
		return (GL_UNIFORM_BUFFER); // UBO

	case EBufferType::BUFFER_TYPE_STORAGE:
		return (GL_SHADER_STORAGE_BUFFER); // SSBO

	case EBufferType::BUFFER_TYPE_INDEX:
		return (GL_ELEMENT_ARRAY_BUFFER);

	case EBufferType::BUFFER_TYPE_VERTEX:
		return (GL_ARRAY_BUFFER);

	case EBufferType::BUFFER_TYPE_INDIRECT:
		return (GL_DRAW_INDIRECT_BUFFER);

	case EBufferType::BUFFER_TYPE_TRANSFER_SRC:
		return GL_COPY_READ_BUFFER;

	case EBufferType::BUFFER_TYPE_TRANSFER_DST:
		return GL_COPY_WRITE_BUFFER;

	case EBufferType::BUFFER_TYPE_STAGING:
		return GL_COPY_WRITE_BUFFER; // or GL_COPY_READ_BUFFER depending on intended direction

	default:
		syserr("Unknown OpenGL buffer type");
		return GL_ARRAY_BUFFER;
	}
}

GLenum OpenGLUtils::ToGLBufferBindingPoint(EBufferBindingPoints bufferBindingPoint)
{
	return (static_cast<uint32_t>(bufferBindingPoint));
}

GLenum OpenGLUtils::ToGLTextureType(ETextureType textureType)
{
	switch (textureType)
	{
	case ETextureType::TEXTURE_TYPE_1D:
		return (GL_TEXTURE_1D);
	case ETextureType::TEXTURE_TYPE_2D:
		return (GL_TEXTURE_2D);
	case ETextureType::TEXTURE_TYPE_3D:
		return (GL_TEXTURE_3D);
	case ETextureType::TEXTURE_TYPE_CUBE:
		return (GL_TEXTURE_CUBE_MAP);

	default:
		return (GL_TEXTURE_2D);
	}
}

GLenum OpenGLUtils::ToGLTexureInternalFormat(ETextureFormats textureFormat)
{
	switch (textureFormat)
	{
	case ETextureFormats::TEXTIRE_FORMAT_R8_UNORM:
		return (GL_R8);
	case ETextureFormats::TEXTIRE_FORMAT_RG8_UNORM:
		return (GL_RG8);
	case ETextureFormats::TEXTIRE_FORMAT_RGB8_UNORM:
		return (GL_RGB8);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_UNORM:
		return (GL_RGBA8);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB:
		return (GL_SRGB8_ALPHA8);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_UNORM:
		return (GL_RGBA8);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_SRGB:
		return (GL_SRGB8_ALPHA8);

	case ETextureFormats::TEXTIRE_FORMAT_R16_FLOAT:
		return (GL_R16F);
	case ETextureFormats::TEXTIRE_FORMAT_R32_FLOAT:
		return (GL_R32F);
	case ETextureFormats::TEXTIRE_FORMAT_RG16_FLOAT:
		return (GL_RG16F);
	case ETextureFormats::TEXTIRE_FORMAT_RG32_FLOAT:
		return (GL_RG32F);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA16_FLOAT:
		return (GL_RGBA16F);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA32_FLOAT:
		return (GL_RGBA32F);

	case ETextureFormats::TEXTIRE_FORMAT_DEPTH16:
		return (GL_DEPTH_COMPONENT16);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH24_STENCIL8:
		return (GL_DEPTH24_STENCIL8);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F:
		return (GL_DEPTH_COMPONENT32F);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F_STENCIL8:
		return (GL_DEPTH32F_STENCIL8);

	default:
		syserr("Unsupported OpenGL texture internal format {}", static_cast<uint32_t>(textureFormat));
		return GL_RGBA8;
	}
}

GLenum OpenGLUtils::ToGLTexurePixelFormat(ETextureFormats textureFormat)
{
	switch (textureFormat)
	{
	case ETextureFormats::TEXTIRE_FORMAT_R8_UNORM:
		return (GL_RED);
	case ETextureFormats::TEXTIRE_FORMAT_RG8_UNORM:
		return (GL_RG);
	case ETextureFormats::TEXTIRE_FORMAT_RGB8_UNORM:
		return (GL_RGB);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_UNORM:
		return (GL_RGBA);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB:
		return (GL_RGBA);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_UNORM:
		return (GL_BGRA);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_SRGB:
		return (GL_BGRA);

	case ETextureFormats::TEXTIRE_FORMAT_R16_FLOAT:
		return (GL_RED);
	case ETextureFormats::TEXTIRE_FORMAT_R32_FLOAT:
		return (GL_RED);
	case ETextureFormats::TEXTIRE_FORMAT_RG16_FLOAT:
		return (GL_RG);
	case ETextureFormats::TEXTIRE_FORMAT_RG32_FLOAT:
		return (GL_RG);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA16_FLOAT:
		return (GL_RGBA);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA32_FLOAT:
		return (GL_RGBA);

	case ETextureFormats::TEXTIRE_FORMAT_DEPTH16:
		return (GL_DEPTH_COMPONENT);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH24_STENCIL8:
		return (GL_DEPTH_STENCIL);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F:
		return (GL_DEPTH_COMPONENT);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F_STENCIL8:
		return (GL_DEPTH_STENCIL);

	default:
		syserr("Unsupported OpenGL texture pixel format {}", static_cast<uint32_t>(textureFormat));
		return GL_RGBA;
	}
}

GLenum OpenGLUtils::ToGLTexturePixelType(ETextureFormats textureFormat)
{
	switch (textureFormat)
	{
	case ETextureFormats::TEXTIRE_FORMAT_R8_UNORM:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_RG8_UNORM:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_RGB8_UNORM:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_UNORM:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_UNORM:
		return (GL_UNSIGNED_BYTE);
	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_SRGB:
		return (GL_UNSIGNED_BYTE);

	case ETextureFormats::TEXTIRE_FORMAT_R16_FLOAT:
		return (GL_HALF_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_R32_FLOAT:
		return (GL_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_RG16_FLOAT:
		return (GL_HALF_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_RG32_FLOAT:
		return (GL_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA16_FLOAT:
		return (GL_HALF_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_RGBA32_FLOAT:
		return (GL_FLOAT);

	case ETextureFormats::TEXTIRE_FORMAT_DEPTH16:
		return (GL_UNSIGNED_SHORT);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH24_STENCIL8:
		return (GL_UNSIGNED_INT_24_8);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F:
		return (GL_FLOAT);
	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F_STENCIL8:
		return (GL_FLOAT_32_UNSIGNED_INT_24_8_REV);

	default:
		syserr("Unsupported OpenGL texture pixel type {}", static_cast<uint32_t>(textureFormat));
		return GL_UNSIGNED_BYTE;
	}
}

GLenum OpenGLUtils::ToGLTextureMinFilter(ETextureFilter textureMinFilter, ETextureMipmapMode textureMipmapMode)
{
	if (textureMipmapMode == ETextureMipmapMode::TEXTURE_MIPMAP_MODE_NONE)
	{
		return (textureMinFilter == ETextureFilter::TEXTURE_FILTER_NEAREST) ? GL_NEAREST : GL_LINEAR;
	}

	if (textureMipmapMode == ETextureMipmapMode::TEXTURE_MIPMAP_MODE_NEAREST)
	{
		return (textureMinFilter == ETextureFilter::TEXTURE_FILTER_NEAREST) ? GL_NEAREST_MIPMAP_NEAREST : GL_LINEAR_MIPMAP_NEAREST;
	}

	// mipmap linear
	return (textureMinFilter == ETextureFilter::TEXTURE_FILTER_NEAREST) ? GL_NEAREST_MIPMAP_LINEAR : GL_LINEAR_MIPMAP_LINEAR;
}

GLenum OpenGLUtils::ToGLTextureMagFilter(ETextureFilter textureMagFilter)
{
	return (textureMagFilter == ETextureFilter::TEXTURE_FILTER_NEAREST) ? GL_NEAREST : GL_LINEAR;
}

GLenum OpenGLUtils::ToGlTextureWrap(ETextureWrap textureWrap)
{
	switch (textureWrap)
	{
	case ETextureWrap::TEXTURE_WRAP_REPEAT:
		return (GL_REPEAT);

	case ETextureWrap::TEXTURE_WRAP_CLAMP_TO_EDGE:
		return (GL_CLAMP_TO_EDGE);

	case ETextureWrap::TEXTURE_WRAP_CLAMP_TO_BORDER:
		return (GL_CLAMP_TO_BORDER);

	case ETextureWrap::TEXTURE_WRAP_MIRRORED_REPEAT:
		return (GL_MIRRORED_REPEAT);

	case ETextureWrap::TEXTURE_WRAP_MIRROR_CLAMP_TO_EDGE:
		return (GL_MIRROR_CLAMP_TO_EDGE);

	default:
		syserr("Unsupported OpenGL texture wrap type {}", static_cast<uint32_t>(textureWrap));
		return (GL_REPEAT);
	}
}

GLenum OpenGLUtils::ToGLShaderType(EShaderStage shaderStage)
{
	switch (shaderStage)
	{
	case EShaderStage::SHADER_TYPE_VERTEX:
		return (GL_VERTEX_SHADER);

	case EShaderStage::SHADER_TYPE_TESSELLATION_CONTROL:
		return (GL_TESS_CONTROL_SHADER);

	case EShaderStage::SHADER_TYPE_TESSELLATION_EVALUATION:
		return (GL_TESS_EVALUATION_SHADER);

	case EShaderStage::SHADER_TYPE_GEOMETRY:
		return (GL_GEOMETRY_SHADER);

	case EShaderStage::SHADER_TYPE_FRAGMENT:
		return (GL_FRAGMENT_SHADER);

	case EShaderStage::SHADER_TYPE_COMPUTE:
		return (GL_COMPUTE_SHADER);

	default:
		syserr("Unknown GL Shader Stage {}", static_cast<uint32_t>(shaderStage));
		return (GL_VERTEX_SHADER);
	}
}
