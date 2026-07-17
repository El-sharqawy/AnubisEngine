#pragma once

#include <vulkan/vulkan.h>
#include <vector>

#include "Buffer.h"
#include "Texture.h"

using TDeviceOffset = uint64_t;
using TDeviceSize = uint64_t;

enum class EBindingType
{
    BIND_TYPE_SAMPLER,
    BIND_TYPE_COMBINED_IMAGE_SAMPLER,
    BIND_TYPE_SAMPLED_IMAGE,
    BIND_TYPE_UNIFORM_BUFFER,
    BIND_TYPE_STORAGE_BUFFER,
};

enum class EImageBindingLayout
{
    IMAGE_LAYOUT_UNDEFINED,
    IMAGE_LAYOUT_GENERAL,
    IMAGE_LAYOUT_COLOR_ATTACHMENT,
    IMAGE_LAYOUT_SHADER_READ_ONLY,
    IMAGE_LAYOUT_TRANSFER_SRC,
    IMAGE_LAYOUT_TRANSFER_DST,
    IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT,
    IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY,
    IMAGE_LAYOUT_PRESENT,
};

enum class EBiningLayoutSetsPoints
{
    BINDING_POINT_MATERIAL,         // SET 0
    BINDING_POINT_FRAME_RESOURCES,  // SET 1
};

struct SBindingDesc
{
    uint32_t          m_uiBinding = 0;
    EBindingType      m_eType = EBindingType::BIND_TYPE_UNIFORM_BUFFER;
    uint32_t          m_uiArrayCount = 1;
    uint32_t          m_uiStageFlags = 0; // engine flags, later converted to VkShaderStageFlags
};

struct SBindingBufferResource
{
    uint32_t m_uiBinding = 0;
    std::vector<IBuffer*> m_vBuffers; // one per frame
    TDeviceOffset m_uiOffset = 0;
    TDeviceSize m_uiRange = VK_WHOLE_SIZE;
};

struct SBindingImageResource
{
    uint32_t m_uiBinding = 0;
    ITexture2D* m_pTexture = nullptr;
    EBindingType m_eBindingType = EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER;
    EImageBindingLayout m_eLayout = EImageBindingLayout::IMAGE_LAYOUT_SHADER_READ_ONLY;
};

struct SBindingContextDesc
{
    std::vector<SBindingDesc> m_vBindings;

    std::vector<SBindingBufferResource> m_vBufferResources;
    std::vector<SBindingImageResource>  m_vImageResources;

    uint32_t m_uiFrameCount = 0;
};

class IBindingContext
{
public:
    virtual ~IBindingContext() = default;

    virtual bool Initialize(const SBindingContextDesc& desc) = 0;
    virtual void Destroy() = 0;
};