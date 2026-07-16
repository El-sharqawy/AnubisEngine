#include "Services/RenderQueue.h"

void CRenderQueue::BeginFrame()
{
	m_vFrameBoneMatrices.clear();
	m_vRenderItems.clear();
}

void CRenderQueue::EndFrame()
{
	m_vFrameBoneMatrices.clear();
	m_vRenderItems.clear();
}

uint32_t CRenderQueue::AllocateSkinPalette(const std::vector<Matrix4>& bonesMetrices)
{
	uint32_t startIndex = static_cast<uint32_t>(m_vFrameBoneMatrices.size());
	m_vFrameBoneMatrices.insert(m_vFrameBoneMatrices.end(), bonesMetrices.begin(), bonesMetrices.end()); // insert in the end

	return (startIndex);
}

const std::vector<Matrix4>& CRenderQueue::GetFrameBoneMatrices() const
{
	return m_vFrameBoneMatrices;
}

const std::vector<SRenderInstance>& CRenderQueue::GetRenderItems() const
{
	return m_vRenderItems;
}

void CRenderQueue::SubmitRenderItem(const SRenderInstance& renderItem)
{
	m_vRenderItems.push_back(renderItem);
}

void CRenderQueue::SubimtRenderItems(const std::vector<SRenderInstance>& vRenderItems)
{
	m_vRenderItems.insert(m_vRenderItems.end(), vRenderItems.begin(), vRenderItems.end());
}

void CRenderQueue::SortRenderItems()
{
	std::sort(m_vRenderItems.begin(), m_vRenderItems.end(),
		[](const SRenderInstance& a, const SRenderInstance& b)
		{
			return a.sortKey < b.sortKey;
		});
}
