#include "OpenGL/OpenGLRenderer.h"
#include "OpenGL/OpenGLCommandList.h"

void COpenGLRenderer::SubmitRenderItem(const SRenderItem& renderItem)
{
	m_vRenderItems.push_back(renderItem);
}

void COpenGLRenderer::SubimtRenderItems(const std::vector<SRenderItem>& vRenderItems)
{
	m_vRenderItems.insert(m_vRenderItems.end(), vRenderItems.begin(), vRenderItems.end());
}

void COpenGLRenderer::FlushRenderItems(ICommandList* pCmd)
{
	if (!pCmd)
	{
		return;
	}

	std::sort(m_vRenderItems.begin(), m_vRenderItems.end(), [](const SRenderItem& a, const SRenderItem& b) { return a.sortKey < b.sortKey; });

	COpenGLCommandList* glCmd = static_cast<COpenGLCommandList*>(pCmd);

	for (const auto& renderItem : m_vRenderItems)
	{
		glCmd->BindPipeline(renderItem.pPipeline);
		glCmd->BindVertexArray(renderItem.pVertexArray);
		// pCmd->BindVertexBuffer(renderItem.pVertexBuffer);
		// pCmd->BindIndexBuffer(renderItem.pIndexBuffer, EIndexType::INDEX_TYPE_UINT32);

		SUniformBufferBlockModel modelData{};
		modelData.matModel = renderItem.modelMatrix;
		glCmd->PushConstants(&modelData, sizeof(modelData));
		glCmd->BindMaterial(renderItem.pMaterial, 0);
		glCmd->DrawIndexed(renderItem.indexCount, 1, renderItem.firstIndex);
	}

	m_vRenderItems.clear();
}
