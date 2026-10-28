#pragma once

#include "API/RenderObject.h"

class CRenderQueue
{
public:
	CRenderQueue() = default;
	~CRenderQueue() = default;

	void BeginFrame();
	void EndFrame();

	const std::vector<SRenderInstance>& GetRenderItems() const;
	void SubmitRenderItem(const SRenderInstance& renderItem);
	void SubimtRenderItems(const std::vector<SRenderInstance>& vRenderItems);
	void SortRenderItems();

private:
	std::vector<SRenderInstance> m_vRenderItems;
};