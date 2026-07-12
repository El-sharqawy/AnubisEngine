#include "OpenGL/OpenGLCommandList.h"
#include "OpenGL/OpenGLPipeline.h"
#include "OpenGL/OpenGLShader.h"
#include "OpenGL/OpenGLBuffer.h"
#include "OpenGL/OpenGLMaterial.h"
#include "OpenGL/OpenGLBindingContext.h"
#include "OpenGL/OpenGLUtils.h"
#include "Logging/LogManager.h"

void COpenGLCommandList::Begin()
{
}

void COpenGLCommandList::End()
{
}

void COpenGLCommandList::SetViewport(const SViewport& vp)
{
	glViewport(vp.x, vp.y, vp.width, vp.height);
}

void COpenGLCommandList::SetScissor(const SRect2D& rect)
{
	glScissor(rect.x, rect.y, rect.width, rect.height);
}

void COpenGLCommandList::BindPipeline(IPipeline* pipeline)
{
	if (!pipeline)
	{
		return;
	}

	COpenGLPipeline* glPipeline = dynamic_cast<COpenGLPipeline*>(pipeline);
	IShaderProgram* pShaderProgram = glPipeline->GetPipelineShader();

	COpenGLShader* glShaderProgram = dynamic_cast<COpenGLShader*>(pShaderProgram);
	uint32_t uiShaderID = glShaderProgram->GetProgramID();
	glUseProgram(uiShaderID);

	m_pBoundPipeline = pipeline;
}

void COpenGLCommandList::BindVertexBuffer(IBuffer* buffer, uint64_t offset)
{
	COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(buffer);

	glBindBuffer(OpenGLUtils::ToGLBufferType(glBuffer->GetType()), glBuffer->GetBufferID());
}

void COpenGLCommandList::BindIndexBuffer(IBuffer* buffer, EIndexType type, uint64_t offset)
{
	COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(buffer);
	glBindBuffer(OpenGLUtils::ToGLBufferType(glBuffer->GetType()), glBuffer->GetBufferID());
}

void COpenGLCommandList::BindMaterial(IMaterial* material, uint32_t frameIndex)
{
	if (!material)
	{
		return;
	}

	COpenGLMaterial* glMaterial = dynamic_cast<COpenGLMaterial*>(material);
	if (!glMaterial)
	{
		return;
	}

	COpenGLBindingContext* glCtx = dynamic_cast<COpenGLBindingContext*>(glMaterial->GetDescriptorContext());
	if (!glCtx)
	{
		return;
	}

	COpenGLPipeline* glPipeline = dynamic_cast<COpenGLPipeline*>(m_pBoundPipeline);
	IShaderProgram* pShaderProgram = glPipeline->GetPipelineShader();

	COpenGLShader* glShaderProgram = dynamic_cast<COpenGLShader*>(pShaderProgram);
	uint32_t uiShaderID = glShaderProgram->GetProgramID();
	glCtx->Bind(frameIndex, uiShaderID);
}

void COpenGLCommandList::PushConstants(const void* data, uint32_t size, uint32_t offset)
{
	// 1. Safely retrieve the active pipeline
	COpenGLPipeline* glPipeline = dynamic_cast<COpenGLPipeline*>(m_pBoundPipeline);
	if (!glPipeline)
	{
		syserr("PushConstants called without a valid bound OpenGL pipeline.");
		return;
	}

	// 2. Get the UBO handle dedicated to Push Constants from the pipeline
	// (Alternatively, your CommandList could manage a transient dynamic buffer)
	COpenGLBuffer* glBufferUBO = dynamic_cast<COpenGLBuffer*>(glPipeline->GetPipelineConstants());
	uint32_t uiPushConstantUBO = glBufferUBO->GetBufferID();

	if (uiPushConstantUBO == 0)
	{
		syserr("Bound pipeline does not support Push Constants (No UBO initialized).");
		return;
	}
	// 3. Bind the UBO to the context
	glBindBuffer(GL_UNIFORM_BUFFER, uiPushConstantUBO);

	// 4. Directly stream the raw data block using the offset and size provided
	glBufferSubData(GL_UNIFORM_BUFFER, offset, size, data);

	// 5. Ensure it is bound to the correct block binding slot (e.g., slot 0)
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, uiPushConstantUBO);
}

void COpenGLCommandList::DrawIndexed(uint32_t indexCount, uint32_t instanceCount, uint32_t firstIndex, int32_t vertexOffset, uint32_t firstInstance)
{
	// 1. Determine the index type (GL_UNSIGNED_INT or GL_UNSIGNED_SHORT)
	// You should fetch this from your currently bound Index Buffer state.
	// Assuming 32-bit indices here for demonstration:
	GLenum indexType = GL_UNSIGNED_INT;
	uint32_t indexSize = sizeof(uint32_t);

	// 2. Calculate the byte offset into the index buffer
	// OpenGL expects a void* pointer representing the byte offset inside the bound GL_ELEMENT_ARRAY_BUFFER
	void* pIndicesOffset = reinterpret_cast<void*>(static_cast<uintptr_t>(firstIndex * indexSize));

	// 3. Multi-instance drawing check
	if (instanceCount > 1 || firstInstance > 0)
	{
		// Requires OpenGL 4.2+ or ARB_base_instance
		glDrawElementsInstancedBaseVertexBaseInstance(
			GL_TRIANGLES,         // Mode: assuming standard triangles
			indexCount,           // Number of indices to draw
			indexType,            // Type of indices (e.g., GL_UNSIGNED_INT)
			pIndicesOffset,       // Pointer to the starting index byte offset
			instanceCount,        // Number of instances to render
			vertexOffset,         // Value added to each index before fetching vertex data
			firstInstance         // Instance ID offset applied to gl_InstanceID
		);
	}
	else if (instanceCount == 1)
	{
		// Fallback optimized call for single instances (Requires OpenGL 3.2+)
		glDrawElementsBaseVertex(
			GL_TRIANGLES,
			indexCount,
			indexType,
			pIndicesOffset,
			vertexOffset
		);
	}
}
