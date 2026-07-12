#pragma once

#include <glad/gl.h>
#include "API/CommandList.h"

class COpenGLPipeline;

class COpenGLCommandList : public ICommandList
{
public:
	COpenGLCommandList() = default;
	~COpenGLCommandList() = default;

    void Begin() override;
    void End() override;

    void SetViewport(const SViewport& vp) override;
    void SetScissor(const SRect2D& rect) override;

    void BindPipeline(IPipeline* pipeline) override;
    void BindVertexBuffer(IBuffer* buffer, uint64_t offset = 0) override;
    void BindIndexBuffer(IBuffer* buffer, EIndexType type, uint64_t offset = 0) override;
    void BindMaterial(IMaterial* material, uint32_t frameIndex) override;
    void PushConstants(const void* data, uint32_t size, uint32_t offset = 0) override;

    void DrawIndexed(uint32_t indexCount,
        uint32_t instanceCount = 1,
        uint32_t firstIndex = 0,
        int32_t vertexOffset = 0,
        uint32_t firstInstance = 0) override;

private:
    IPipeline* m_pBoundPipeline = nullptr;
};