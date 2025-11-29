#include "Shader.h"
#include <fstream>
#include <sstream>
#include <utils.h>
#include <glm/gtc/type_ptr.hpp>

/**
 * Creates a new shader with given name.
 *
 * @param name Optional name for debugging (default: "Shader")
 */
CShader::CShader(const std::string& name)
{
	m_stName = name;
	m_uiProgramID = 0;
	m_bIsInitialized = false;
	m_bIsLinked = false;
	m_bIsCompute = false;
}

/**
 * Destructor - automatically cleans up OpenGL resources.
 */
CShader::~CShader()
{
	glDeleteProgram(m_uiProgramID);
}

/**
 * Initializes and creates a new shader program.
 */
void CShader::InitializeShader()
{
	if (m_bIsInitialized)
	{
		return;
	}

	m_uiProgramID = glCreateProgram();

	if (m_uiProgramID != 0)
	{
		m_bIsInitialized = true;
	}
}

/**
 * Attaches a shader from file to the program.
 * Shader type is automatically detected from file extension.
 *
 * @param stShaderPath Path to shader file (.vert, .frag, .geom, etc.)
 * @return true if successfully compiled and attached, false otherwise
 */
bool CShader::AttachShader(const std::string& stShaderPath)
{
	// Validate program state
	if (!m_bIsInitialized)
	{
		InitializeShader();
	}

	if (m_bIsLinked)
	{
		syserr("Cannot attach shader '%s': program '%s' already linked", GetShaderName(stShaderPath).c_str(), m_stName.c_str());
		return false;
	}

	if (m_bIsCompute)
	{
		// IDK YET
		return (false);
	}

	// Load shader source from file
	std::string shaderCode = LoadShaderFromFile(stShaderPath);
	if (shaderCode.empty())
	{
		syserr("Failed to load shader file: %s", stShaderPath.c_str());
		return (false);
	}

	// Determine shader type from file extension
	TShaderType shaderType = GetShaderType(stShaderPath);
	if (shaderType.m_uiType == 0)
	{
		syserr("Unknown shader type for file: %s", stShaderPath.c_str());
		return false;
	}

	// Compile shader
	GLuint uiShaderID = glCreateShader(shaderType.m_uiType);
	const char* shaderCodeStr = shaderCode.c_str();
	glShaderSource(uiShaderID, 1, &shaderCodeStr, nullptr);
	glCompileShader(uiShaderID);

	// Check for compilation errors
	if (CheckCompileErrors(uiShaderID, shaderType.m_stName, GetShaderName(stShaderPath)) == false)
	{
		glDeleteShader(uiShaderID);	
		return (false);
	}

	// Attach to program
	glAttachShader(m_uiProgramID, uiShaderID);
	m_vecShaders.push_back(uiShaderID);

	syslog("Successfully Attached shader: %s", GetShaderName(stShaderPath).c_str());
	return (true);
}

/**
 * Links all attached shaders into a complete program.
 * Must be called after all shaders are attached and before Use().
 *
 * @return true if linking succeeded, false otherwise
 */
bool CShader::LinkProgram()
{
	if (!m_bIsInitialized)
	{
		syserr("Cannot link program '%s': not initialized", m_stName.c_str());
		return (false);
	}

	if (m_bIsLinked)
	{
		syserr("Program '%s' is already linked", m_stName.c_str());
		return (false);
	}

	if (m_vecShaders.empty())
	{
		syserr("Cannot link program '%s': no shaders attached", m_stName.c_str());
		return (false);
	}

	// Link the program
	glLinkProgram(m_uiProgramID);

	// Check for linking errors
	if (!CheckCompileErrors(m_uiProgramID, "program", m_stName))
	{
		syserr("Failed to link program '%s'", m_stName.c_str());
		return (false);
	}

	m_bIsLinked = true;
	syslog("Program '%s' linked successfully", m_stName.c_str());

	// Clean up shader objects (no longer needed after linking)
	for (GLuint shaderID : m_vecShaders)
	{
		glDeleteShader(shaderID);
	}
	m_vecShaders.clear();

	return true;
}

/**
 * Activates this shader program for rendering.
 * Program must be linked before calling this.
 */
void CShader::Use()
{
	if (!m_bIsInitialized)
	{
		syserr("Cannot use program '%s': not initialized", m_stName.c_str());
		return;
	}

	if (!m_bIsLinked)
	{
		syserr("Cannot use program '%s': not linked", m_stName.c_str());
		return;
	}

	glUseProgram(m_uiProgramID);
}

/**
 * Gets the OpenGL program ID.
 *
 * @return Program ID, or 0 if not initialized
 */
GLuint CShader::GetProgramID() const
{
	return (m_uiProgramID);
}

/**
 * Checks if the program is ready to use.
 *
 * @return true if initialized and linked
 */
bool CShader::IsReady() const
{
	return (m_bIsInitialized) && (m_bIsLinked);
}

/**
 * Gets the shader program name.
 *
 * @return Program name for debugging
 */
const std::string& CShader::GetName() const
{
	return (m_stName);
}

/**
 * Loads shader source code from a file.
 *
 * @param stShaderPath Path to shader file
 * @return Shader source code, or empty string on error
 */
std::string CShader::LoadShaderFromFile(const std::string& stShaderPath)
{
	/* 1. retrieve the shader source code from filePath */
	std::string shaderCode;
	std::ifstream fShaderFile; /* shader file */

	/* ensure ifstream objects can throw exceptions: */
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		/* Open Files */
		fShaderFile.open(stShaderPath);

		std::stringstream sShaderStream;

		/* read file’s buffer contents into streams */
		sShaderStream << fShaderFile.rdbuf();

		/* close file handlers */
		fShaderFile.close();

		/* convert streams into strings */
		shaderCode = sShaderStream.str();
	}
	catch (std::ifstream::failure err)
	{
		syserr("Failed to Load the Shader File %s", GetShaderName(stShaderPath).c_str());
		return "";
	}

	return (shaderCode);
}

/**
 * Checks for shader compilation or program linking errors.
 * Prints detailed error messages if compilation/linking fails.
 *
 * @param shader    Shader or program ID
 * @param type      "vertex", "fragment", "program", etc.
 * @param name      Shader filename for error reporting
 * @return true if no errors, false if compilation/linking failed
 */
bool CShader::CheckCompileErrors(GLuint uiShaderID, const std::string& stShaderType, const std::string& stShaderName)
{
	GLint iSuccess = 0;
	char c_szInfoLog[1024];

	if (stShaderType != "program")
	{
		glGetShaderiv(uiShaderID, GL_COMPILE_STATUS, &iSuccess);
		if (!iSuccess)
		{
			glGetShaderInfoLog(uiShaderID, 1024, nullptr, c_szInfoLog);
			syserr("Error shader: %s, Compilation Error of Type %s", stShaderName.c_str(), stShaderType.c_str());
			syserr("Error Log: %s", c_szInfoLog);
		}
	}
	else
	{
		glGetProgramiv(uiShaderID, GL_LINK_STATUS, &iSuccess);
		if (!iSuccess)
		{
			glGetProgramInfoLog(uiShaderID, 1024, nullptr, c_szInfoLog);
			syserr("Error Linking Program of Type %s", stShaderType.c_str());
			syserr("Error Log: %s", c_szInfoLog);
		}
	}

	return (iSuccess);
}

/**
 * Extracts filename from full path for error messages.
 *
 * @param stShaderPath Full file path
 * @return Filename only
 */
std::string CShader::GetShaderName(const std::string& stShaderPath)
{
	std::string pathStr = stShaderPath;
	const size_t last_slash_idx = pathStr.find_last_of("/");
	if (std::string::npos != last_slash_idx)
	{
		pathStr.erase(0, last_slash_idx + 1);
	}

	return (pathStr);
}

/**
 * Extracts shader type from full path.
 *
 * @param stShaderPath Full file path
 * @return shader type str only
 */
std::string CShader::GetShaderTypeName(const std::string& stShaderPath)
{
	std::string stType = GetShaderName(stShaderPath);
	const size_t last_slash_idx = stType.find_last_of(".");
	if (std::string::npos != last_slash_idx)
	{
		stType.erase(0, last_slash_idx + 1);
	}

	return (stType);
}

/**
 * Determines shader type from file extension.
 * Supported: .vert, .frag, .geom, .tesc, .tese, .comp
 *
 * @param stShaderPath Path to shader file
 * @return Shader type info with OpenGL enum and name
 */
TShaderType CShader::GetShaderType(const std::string& stShaderPath)
{
	std::string stType = GetShaderTypeName(stShaderPath);
	TShaderType shaderType{};

	if (stType == "vert")
	{
		shaderType = TShaderType("vertex", GL_VERTEX_SHADER);
	}
	else if (stType == "frag")
	{
		shaderType = TShaderType("fragment", GL_FRAGMENT_SHADER);
	}
	else if (stType == "geom")
	{
		shaderType = TShaderType("geometry", GL_GEOMETRY_SHADER);
	}
	else if (stType == "comp")
	{
		shaderType = TShaderType("compute", GL_COMPUTE_SHADER);
	}
	else if (stType == "tes")
	{
		shaderType = TShaderType("tess_evaluation", GL_TESS_EVALUATION_SHADER);
	}
	else if (stType == "tcs")
	{
		shaderType = TShaderType("tess_control", GL_TESS_CONTROL_SHADER);
	}

	return (shaderType);
}

/**
 * Sets a boolean uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided boolean.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param value: The boolean value to be set.
 */
void CShader::SetBool(const std::string& name, bool value) const
{
	GLuint iboolLoc = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform1i(iboolLoc, (GLuint)value);
}

/**
 * Sets an integer uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided integer.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param value: The integer value to be set.
 *
 */
void CShader::SetInt(const std::string& name, GLint value) const
{
	GLuint iIntLoc = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform1i(iIntLoc, value);
}

/**
 * Sets an integer array uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided integer.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param index: The integer index to be set.
 * @param value: The integer value to be set.
 *
 */
void CShader::SetIntArray(const std::string& name, GLint index, GLint value) const
{
	std::string fullName = name + "[" + std::to_string(index) + "]";
	GLuint iIntLoc = glGetUniformLocation(GetProgramID(), fullName.c_str());
	glUniform1i(iIntLoc, value);
}

/**
 * Sets an float uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided float.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param value: The float value to be set.
 *
 */
void CShader::SetFloat(const std::string& name, GLfloat value) const
{
	GLuint iFloatLoc = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform1f(iFloatLoc, value);
}

/**
 *  Sets a 2D vector uniform in the shader program using two floats.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided x and y components.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param value1: The first float value (x-component).
 * @param value2: The second float value (y-component).
 *
 */
void CShader::Set2Float(const std::string& name, GLfloat value1, GLfloat value2) const
{
	GLuint iFloatLoc = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform2f(iFloatLoc, value1, value2);
}

/**
 * Sets a 2D vector uniform in the shader program using individual components.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value using the provided x and y components.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param x: The x-component of the vector.
 * @param y: The y-component of the vector.
 */
void CShader::SetVec2(const std::string& name, GLfloat x, GLfloat y) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform2f(iVectorLocation, x, y);
}

/**
 * Sets a 3D vector uniform in the shader program using individual components.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value using the provided x, y, and z components.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param x: The x-component of the vector.
 * @param y: The y-component of the vector.
 * @param z: The z-component of the vector.
 */
void CShader::SetVec3(const std::string& name, GLfloat x, GLfloat y, GLfloat z) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform3f(iVectorLocation, x, y, z);
}

/**
 * Sets a 4D vector uniform in the shader program using individual components.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value using the provided x, y, z, and w components.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param x: The x-component of the vector.
 * @param y: The y-component of the vector.
 * @param z: The z-component of the vector.
 * @param w: The w-component of the vector.
 */
void CShader::SetVec4(const std::string& name, GLfloat x, GLfloat y, GLfloat z, GLfloat w) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform4f(iVectorLocation, x, y, z, w);
}

void CShader::SetSampler2D(const std::string& name, GLuint iTextureID, GLint iTexValue) const
{
	if (IsGLVersionHigher(4, 5))
	{
		// Use glBindTextureUnit for OpenGL 4.5 and higher
		glBindTextureUnit(iTexValue, iTextureID);
	}
	else
	{
		glActiveTexture(GL_TEXTURE0 + iTexValue);
		glBindTexture(GL_TEXTURE_2D, iTextureID);
	}
	SetInt(name, iTexValue);
}

void CShader::SetSampler3D(const std::string& name, GLuint iTexValue, GLint iTextureID) const
{
	if (IsGLVersionHigher(4, 5))
	{
		// Use glBindTextureUnit for OpenGL 4.5 and higher
		glBindTextureUnit(iTexValue, iTextureID);
	}
	else
	{
		glActiveTexture(GL_TEXTURE0 + iTexValue);
		glBindTexture(GL_TEXTURE_3D, iTextureID);
	}
	SetInt(name, iTexValue);
}

/**
 * Sets an sampler2D Bindless texture uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided integer.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param value: The unsigned integer64 value of texture Handler.
 *
 */
void CShader::SetBindlessSampler2D(const std::string& name, GLuint64 value) const
{
	GLint iIntLoc = glGetUniformLocation(GetProgramID(), name.c_str());

	if (iIntLoc == -1)
	{
		syserr("[Shader] Warning: Uniform '%s' not found or optimized out.", name.c_str());
		return;
	}

	glUniformHandleui64ARB(iIntLoc, value);
}

/**
 * Sets a 2D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 2D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec2: The 2D GLM vector to be set.
 */
void CShader::SetVec2(const std::string& name, const glm::vec2& vec2) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform2fv(iVectorLocation, 1, glm::value_ptr(vec2));
}

/**
 * Sets a 3D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 3D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec3: The 3D GLM vector to be set.
 */
void CShader::SetVec3(const std::string& name, const glm::vec3& vec3) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform3fv(iVectorLocation, 1, glm::value_ptr(vec3));
}

/**
 * Sets a 4D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 4D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec4: The 4D GLM vector to be set.
 */
void CShader::SetVec4(const std::string& name, const glm::vec4& vec4) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform3fv(iVectorLocation, 1, glm::value_ptr(vec4));
}

/**
 * Sets a 2x2 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 2x2 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 2x2 GLM matrix to be set.
 */
void CShader::SetMat2(const std::string& name, const glm::mat2& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix2fv(iMatLocation, 1, GL_FALSE, glm::value_ptr(matrix));
}

/**
 * Sets a 3x3 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 3x3 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 3x3 GLM matrix to be set.
 */
void CShader::SetMat3(const std::string& name, const glm::mat3& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix3fv(iMatLocation, 1, GL_FALSE, glm::value_ptr(matrix));
}

/**
 * Sets a 4x4 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 4x4 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 4x4 GLM matrix to be set.
 */
void CShader::SetMat4(const std::string& name, const glm::mat4& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix4fv(iMatLocation, 1, GL_FALSE, glm::value_ptr(matrix));
}

/**
 * Sets a 2D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 2D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec2: The 2D vector to be set.
 */
void CShader::SetVec2(const std::string& name, const Vector2D& vec2) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform2fv(iVectorLocation, 1, vec2);
}

/**
 * Sets a 3D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 3D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec3: The 3D vector to be set.
 */
void CShader::SetVec3(const std::string& name, const Vector3D& vec3) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform3fv(iVectorLocation, 1, vec3);
}

/**
 * Sets a 4D vector uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 4D vector.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param vec4: The 4D vector to be set.
 */
void CShader::SetVec4(const std::string& name, const Vector4D& vec4) const
{
	GLuint iVectorLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniform4fv(iVectorLocation, 1, vec4);
}

/**
 * Sets a 2x2 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 2x2 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 2x2 matrix to be set.
 */
void CShader::SetMat2(const std::string& name, const Matrix2& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix2fv(iMatLocation, 1, GL_FALSE,(const GLfloat*)matrix);
}

/**
 * Sets a 3x3 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 3x3 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 3x3 matrix to be set.
 */
void CShader::SetMat3(const std::string& name, const Matrix3& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix3fv(iMatLocation, 1, GL_FALSE, (const GLfloat*)matrix);
}

/**
 * Sets a 4x4 matrix uniform in the shader program.
 *
 * This function locates the uniform variable in the shader by its name
 * and sets its value to the provided 4x4 matrix.
 *
 * @param name: The name of the uniform variable in the shader.
 * @param matrix: The 4x4 matrix to be set.
 */
void CShader::SetMat4(const std::string& name, const Matrix4& matrix) const
{
	GLuint iMatLocation = glGetUniformLocation(GetProgramID(), name.c_str());
	glUniformMatrix4fv(iMatLocation, 1, GL_FALSE, (const GLfloat*)matrix);
}