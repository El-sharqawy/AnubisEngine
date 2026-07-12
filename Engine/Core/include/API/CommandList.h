#pragma once

#include "Pipeline.h"
#include "Buffer.h"
#include "Material.h"
#include "CoreTypes.h"

class ICommandList
{
public:
    virtual ~ICommandList() = default;

    virtual void Begin() = 0;
    virtual void End() = 0;

    virtual void SetViewport(const SViewport& vp) = 0;
    virtual void SetScissor(const SRect2D& rect) = 0;

    virtual void BindPipeline(IPipeline* pipeline) = 0;
    virtual void BindVertexBuffer(IBuffer* buffer, uint64_t offset = 0) = 0;
    virtual void BindIndexBuffer(IBuffer* buffer, EIndexType type, uint64_t offset = 0) = 0;

    virtual void BindMaterial(IMaterial* material, uint32_t frameIndex) = 0;

    virtual void PushConstants(const void* data, uint32_t size, uint32_t offset = 0) = 0;

    virtual void DrawIndexed(uint32_t indexCount,
        uint32_t instanceCount = 1,
        uint32_t firstIndex = 0,
        int32_t vertexOffset = 0,
        uint32_t firstInstance = 0) = 0;
};
