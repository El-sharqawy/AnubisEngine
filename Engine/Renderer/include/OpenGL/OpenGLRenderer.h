#pragma once

#include <memory>
#include <vector>
#include "API/RenderObject.h"
#include "API/ActorData.h"

class ICommandList;

class COpenGLRenderer
{
public:
    COpenGLRenderer() = default;
    ~COpenGLRenderer() = default;

    bool Initialize();
    void Destroy();

    void BeginFrame();
    void Present();
    void EndFrame();

    // Present
    void SubmitRenderItem(const SRenderInstance& renderItem);
    void SubimtRenderItems(const std::vector<SRenderInstance>& vRenderItems);
    void FlushRenderItems(ICommandList* pCmd);

    uint32_t GetCurrentFrameIndex() const;

protected:
    bool InitializeRendererBuffers();
    void UpdateRendererBuffers();
    void DestroyRendererBuffers();

private:
    uint32_t m_uiCurrentFrame = 0;

	std::vector<SRenderInstance> m_vRenderItems = {};

    // Camera Matrix
    std::vector<IBuffer*> m_vCameraUBO = {}; // 2 Buffers .. 
    std::vector<IBuffer*> m_vJointsBuffer = {}; // 2 Buffer
    std::shared_ptr<CActor> m_pActor = nullptr;
};