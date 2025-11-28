#pragma once

#include <glad/glad.h>
#include <string>
#include <vector>

typedef struct SShaderType
{
	std::string m_stName;
	GLuint m_uiType;

	SShaderType()
	{
		m_stName = "unkonwn";
		m_uiType = 0;
	}

	SShaderType(const std::string& stName, GLuint uiType)
	{
		m_stName = stName;
		m_uiType = uiType;
	}
} TShaderType;

class CShader
{
public:
	/**
	 * Creates a new shader with given name.
	 *
	 * @param name Optional name for debugging (default: "Shader")
	 */
	explicit CShader(const std::string& name = "Shader");

	/**
	 * Destructor - automatically cleans up OpenGL resources.
	 */
	~CShader();

	/**
	 * Prevent copying(OpenGL resources shouldn't be copied)
	 */
	CShader(const CShader&) = delete;
	CShader& operator=(const CShader&) = delete;

	/**
	 * Initializes and creates a new shader program.
	 */
	void InitializeShader();

	/**
	 * Attaches a shader from file to the program.
	 * Shader type is automatically detected from file extension.
	 *
	 * @param stShaderPath Path to shader file (.vert, .frag, .geom, etc.)
	 * @return true if successfully compiled and attached, false otherwise
	 */
	bool AttachShader(const std::string& stShaderPath);

	/**
	 * Links all attached shaders into a complete program.
	 * Must be called after all shaders are attached and before Use().
	 *
	 * @return true if linking succeeded, false otherwise
	 */
	bool LinkProgram();

	/**
	 * Activates this shader program for rendering.
	 * Program must be linked before calling this.
	 */
	void Use();

	/**
	 * Gets the OpenGL program ID.
	 *
	 * @return Program ID, or 0 if not initialized
	 */
	GLuint GetProgramID() const;

	/**
	 * Checks if the program is ready to use.
	 *
	 * @return true if initialized and linked
	 */
	bool IsReady() const;

	/**
	 * Gets the shader program name.
	 *
	 * @return Program name for debugging
	 */
	const std::string& GetName() const;

private:
	/**
	 * Loads shader source code from a file.
	 *
	 * @param stShaderPath Path to shader file
	 * @return Shader source code, or empty string on error
	 */
	std::string LoadShaderFromFile(const std::string& stShaderPath);

	/**
	 * Checks for shader compilation or program linking errors.
	 * Prints detailed error messages if compilation/linking fails.
	 *
	 * @param shader    Shader or program ID
	 * @param type      "vertex", "fragment", "program", etc.
	 * @param name      Shader filename for error reporting
	 * @return true if no errors, false if compilation/linking failed
	 */
	bool CheckCompileErrors(GLuint uiShaderID, const std::string& stShaderType, const std::string& stShaderName);

	/**
	 * Extracts filename from full path for error messages.
	 *
	 * @param stShaderPath Full file path
	 * @return Filename only
	 */
	std::string GetShaderName(const std::string& stShaderPath);

	/**
	 * Extracts shader type from full path.
	 *
	 * @param stShaderPath Full file path
	 * @return shader type str only
	 */
	std::string GetShaderTypeName(const std::string& stShaderPath);

	/**
	 * Determines shader type from file extension.
	 * Supported: .vert, .frag, .geom, .tesc, .tese, .comp
	 *
	 * @param stShaderPath Path to shader file
	 * @return Shader type info with OpenGL enum and name
	 */
	TShaderType GetShaderType(const std::string& stShaderPath);

public:
	/* general utility uniform functions */
	void SetBool(const std::string& name, bool value) const;
	void SetInt(const std::string& name, GLint value) const;
	void SetIntArray(const std::string& name, int index, int value) const;
	void SetFloat(const std::string& name, float value) const;
	void Set2Float(const std::string& name, float value1, float value2) const;
	void SetVec2(const std::string& name, float x, float y) const;
	void SetVec3(const std::string& name, float x, float y, float z) const;
	void SetVec4(const std::string& name, float x, float y, float z, float w) const;

private:
	std::string m_stName;               // Program name for debugging
	GLuint m_uiProgramID;               // OpenGL program object ID
	bool m_bIsInitialized;              // Program object created
	bool m_bIsLinked;                   // Shaders linked successfully
	bool m_bIsCompute;					// If his is compute shader
	std::vector<GLuint> m_vecShaders;   // Temporary storage for shader IDs
};

