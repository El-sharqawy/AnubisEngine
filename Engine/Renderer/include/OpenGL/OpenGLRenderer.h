#pragma once

#include <vector>
#include "API/RenderObject.h"

class ICommandList;

class COpenGLRenderer
{
public:
    COpenGLRenderer() = default;
    ~COpenGLRenderer() = default;

    // Present
    void SubmitRenderItem(const SRenderItem& renderItem);
    void SubimtRenderItems(const std::vector<SRenderItem>& vRenderItems);
    void FlushRenderItems(ICommandList* pCmd);

private:
	std::vector<SRenderItem> m_vRenderItems = {};
};