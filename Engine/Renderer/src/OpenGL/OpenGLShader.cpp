#include "OpenGL/OpenGLShader.h"
#include "OpenGL/OpenGLUtils.h"
#include "Logging/LogManager.h"

bool COpenGLShader::Create(const SShaderDesc& desc)
{
	m_uiProgramID = glCreateProgram();

	if (m_uiProgramID == 0)
	{
		return false;
	}

	m_vModules.clear();

	for (const auto& stageDesc : desc.m_vStages)
	{
		std::vector<char> shaderCode = OpenGLUtils::ReadShaderData(stageDesc.path);

		if (shaderCode.empty())
		{
			syserr("Failed to compile stage for shader '{}': {}", desc.m_stName, stageDesc.path);
			for (auto& module : m_vModules)
			{
				glDeleteShader(module.shaderID);
			}

			Destroy();
			m_vModules.clear();
			return false;
		}

		const GLchar* shaderSrc = reinterpret_cast<const GLchar*>(shaderCode.data());

		uint32_t shaderType = OpenGLUtils::ToGLShaderType(stageDesc.stageType);
		uint32_t shaderID = glCreateShader(shaderType);
		if (shaderID == 0)
		{
			syserr("Failed to Create Shader {}", stageDesc.path);

			for (auto& module : m_vModules)
			{
				glDeleteShader(module.shaderID);
			}

			Destroy();
			m_vModules.clear();
			return (false);
		}

		int32_t sourceLen = static_cast<int32_t>(shaderCode.size());

		glShaderSource(shaderID, 1, &shaderSrc, &sourceLen);
		glCompileShader(shaderID);

		if (!OpenGLUtils::CheckShaderCompileErrors(shaderID, stageDesc.stageType))
		{
			syserr("Failed to compile shader '{}'", stageDesc.path);
			glDeleteShader(shaderID);

			for (auto& module : m_vModules)
			{
				glDeleteShader(module.shaderID);
			}

			Destroy();
			m_vModules.clear();
			return false;
		}

		glAttachShader(m_uiProgramID, shaderID);
		m_vModules.push_back({ stageDesc.stageType, shaderID });
	}

	glLinkProgram(m_uiProgramID);
	if (!OpenGLUtils::CheckProgramLinkingError(m_uiProgramID))
	{
		syserr("Failed to Link Program {}", desc.m_stName);

		for (auto& module : m_vModules)
		{
			glDeleteShader(module.shaderID);
		}

		Destroy();
		m_vModules.clear();

		return (false);
	}

	// Detach and delete stage objects — no longer needed after linking
	for (auto& module : m_vModules)
	{
		glDetachShader(m_uiProgramID, module.shaderID);
		glDeleteShader(module.shaderID);
		module.shaderID = 0;
	}

	return (true);
}

void COpenGLShader::Destroy()
{
	if (m_uiProgramID)
	{
		glDeleteProgram(m_uiProgramID);
		m_uiProgramID = 0;
	}
}

void COpenGLShader::Clear()
{
	m_uiProgramID = 0;
}

bool COpenGLShader::HasStage(EShaderStage stage) const
{
	return std::any_of(m_vModules.begin(), m_vModules.end(), [stage](const SStageModule& s) { return s.stage == stage; });
}

bool COpenGLShader::IsCompute() const
{
	const EShaderStage stage = EShaderStage::SHADER_TYPE_COMPUTE;
	return std::any_of(m_vModules.begin(), m_vModules.end(), [stage](const SStageModule& s) { return s.stage == stage; });
}

uint32_t COpenGLShader::GetProgramID() const
{
	return (m_uiProgramID);
}
