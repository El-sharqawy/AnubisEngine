#pragma once

#include "API/RenderObject.h"

class CRenderQueue
{
public:
	CRenderQueue() = default;
	~CRenderQueue() = default;

	void BeginFrame();
	void EndFrame();

	uint32_t AllocateSkinPalette(const std::vector<Matrix4>& bonesMetrices);
	const std::vector<Matrix4>& GetFrameBoneMatrices() const;
	const std::vector<SRenderInstance>& GetRenderItems() const;
	void SubmitRenderItem(const SRenderInstance& renderItem);
	void SubimtRenderItems(const std::vector<SRenderInstance>& vRenderItems);
	void SortRenderItems();

private:
	std::vector<Matrix4> m_vFrameBoneMatrices;
	std::vector<SRenderInstance> m_vRenderItems;
};