#include "Services/RenderQueue.h"

void CRenderQueue::BeginFrame()
{
	m_vRenderItems.clear();
}

void CRenderQueue::EndFrame()
{
	m_vRenderItems.clear();
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
