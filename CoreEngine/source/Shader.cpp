#include "Shader.h"
#include <fstream>
#include <sstream>
#include <utils.h>

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
