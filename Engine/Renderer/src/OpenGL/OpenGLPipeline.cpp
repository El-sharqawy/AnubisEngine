#include "OpenGL/OpenGLPipeline.h"
#include "Device/OpenGLRenderDevice.h"
#include "OpenGL/OpenGLUtils.h"
#include "Logging/LogManager.h"

bool COpenGLPipeline::Initialize(const SPipelineDesc& desc)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& glRenderDevice = static_cast<COpenGLRenderDevice&>(renderDev);

	SShaderDesc shaderDesc = desc.shader;

	m_pPipelineShader = renderDev.CreateShaderProgram(shaderDesc);
	if (!m_pPipelineShader)
	{
		return (false);
	}

	SBufferDesc bufferDesc{};
	bufferDesc.m_stName = "Model SSBO";
	bufferDesc.m_eType = EBufferType::BUFFER_TYPE_UNIFORM;
	bufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	bufferDesc.m_eBindingPointOne = EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MODEL_UBO; // Binding Point 2
	bufferDesc.m_uiSize = sizeof(SUniformBufferBlockModel);
	bufferDesc.cpuWrite = true;

	m_pPipelineConstants = renderDev.CreateBuffer(bufferDesc);
	if (!m_pPipelineConstants)
	{
		syserr("Failed to create Pipeline Constants Uniform  Buffer");
	}

	// Toggles
	SetGLToggle(GL_DEPTH_TEST, desc.depthTest);
	SetGLToggle(GL_CULL_FACE, desc.cullFace);

	// State Functions
	glDepthFunc(OpenGLUtils::ToGLDepthFunc(desc.depthCompareOp));
	glDepthMask(desc.depthWrite ? GL_TRUE : GL_FALSE); // Note: glDepthMask natively accepts bool (GL_TRUE/GL_FALSE map to true/false)

	// Rasterization States
	glPolygonMode(GL_FRONT_AND_BACK, OpenGLUtils::ToGLPolygonMode(desc.polygonMode));
	glCullFace(OpenGLUtils::ToGLCullFace(desc.cullMode));
	glFrontFace(OpenGLUtils::ToGLFrontFace(desc.frontFace));

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

void COpenGLPipeline::SetGLToggle(GLenum cap, bool enable)
{
	enable ? glEnable(cap) : glDisable(cap);
}