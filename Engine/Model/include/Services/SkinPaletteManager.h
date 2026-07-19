#pragma once

#include "TypeMatrix4.h"
#include "Vulkan/VulkanDescriptorContext.h"
#include "API/Buffer.h"
#include "Utils/AnubisAssert.h"

using SkinPaletteHandle = uint32_t;
static constexpr SkinPaletteHandle INVALID_SKIN_PALETTE = 0;

struct SSkinPaletteEntry
{
    SkinPaletteHandle handle = INVALID_SKIN_PALETTE;
    uint32_t maxMatrices = 0;
    uint32_t lastUploadedFrame = UINT32_MAX;
    bool active = false;
};

struct SSkinPaletteGPUView
{
    uint32_t firstMatrix = 0;   // matrix index inside big palette buffer
    uint32_t matrixCount = 0;
};

struct SPerFramePaletteBuffer
{
    IBuffer* pBuffer;
    uint32_t capacityMatrices = 0;
    uint32_t usedMatrices = 0;
    bool descriptorDirty = false;
};

class CSkinPaletteManager
{
public:
    bool Initialize(uint32_t framesInFlight);
    void Shutdown();

    void BeginFrame(uint32_t frameIndex);

    SkinPaletteHandle RegisterPalette(uint32_t maxMatrices);
    void ReleasePalette(SkinPaletteHandle handle);

    SSkinPaletteGPUView UploadPalette(SkinPaletteHandle handle, std::vector<Matrix4> matrices);

    const IBuffer* GetCurrentPaletteBuffer() const;
    IBuffer* GetPaletteBuffer(uint32_t frameIndex);
    bool NeedsDescriptorUpdate(uint32_t frameIndex) const;
    void ClearDescriptorDirty(uint32_t frameIndex);
    CVulkanDescriptorContext* GetDescriptorContext();

private:
    bool EnsureFrameCapacity(uint32_t frameIndex, uint32_t requiredMatrices);

private:
    uint32_t m_uiNextHandle = 1;
    std::unordered_map<SkinPaletteHandle, SSkinPaletteEntry> m_mEntries = {};
    std::vector<SPerFramePaletteBuffer> m_vFrameBuffers;
    uint32_t m_uiCurrentFrame = 0;
    std::unique_ptr<CVulkanDescriptorContext> m_pBonesBindingContext = nullptr;
};