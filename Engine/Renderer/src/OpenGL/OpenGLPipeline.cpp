#include "OpenGL/OpenGLPipeline.h"
#include "Device/OpenGLRenderDevice.h"
#include "Logging/LogManager.h"

bool COpenGLPipeline::Initialize(const SPipelineDesc& desc)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& glRenderDevice = static_cast<COpenGLRenderDevice&>(renderDev);

	if (desc.pipelineType == EPipelineType::PIPELINE_TYPE_STATIC_MESH)
	{

		SShaderDesc shaderDesc = desc.shader;

		m_pPipelineShader = renderDev.CreateShaderProgram(shaderDesc);
		if (!m_pPipelineShader)
		{
			return (false);
		}

		SBufferDesc bufferDesc{};
		bufferDesc.m_stName = "Model UBO";
		bufferDesc.m_eType = EBufferType::BUFFER_TYPE_UNIFORM;
		bufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		bufferDesc.m_eBindingPoint = EBufferBindingPoints::BINDING_POINT_MODEL_UBO;
		bufferDesc.m_uiSize = sizeof(SUniformBufferBlockModel);
		bufferDesc.cpuWrite = true;

		m_pPipelineConstants = renderDev.CreateBuffer(bufferDesc);
		if (!m_pPipelineConstants)
		{
			syserr("Failed to create Pipeline Constants Uniform  Buffer");
		}
	}

	return (true);
}

void COpenGLPipeline::Destroy()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	if (m_pPipelineShader)
	{
		renderDev.DestroyShaderProgram(m_pPipelineShader);
		AnubisSafeDelete(m_pPipelineShader);
	}

	if (m_pPipelineConstants)
	{
		renderDev.DestroyBuffer(m_pPipelineConstants);
		AnubisSafeDelete(m_pPipelineConstants);
	}
}

EPipelineType COpenGLPipeline::GetPipelineType() const
{
	return (m_ePipelineType);
}

IShaderProgram* COpenGLPipeline::GetPipelineShader() const
{
	return (m_pPipelineShader);
}

IBuffer* COpenGLPipeline::GetPipelineConstants() const
{
	return (m_pPipelineConstants);
}
