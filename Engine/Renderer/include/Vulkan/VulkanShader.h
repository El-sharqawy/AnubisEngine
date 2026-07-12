#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <vector>
#include "API/ShaderProgram.h"

struct SVulkanContext;

class CVulkanShader : public IShaderProgram
{
public:
    CVulkanShader() = default;
    ~CVulkanShader() = default;

    bool Create(const SShaderDesc& desc);
    void Destroy();

    bool HasStage(EShaderStage stage) const override;
    bool IsCompute() const override;

    std::vector<VkPipelineShaderStageCreateInfo>& GetShaderStagesInfo();
    const std::vector<VkPipelineShaderStageCreateInfo>& GetShaderStagesInfo() const;
    SVulkanContext& GetVulkanContext() const;

private:
    struct SStageModule
    {
        EShaderStage stage;
        VkShaderModule module;
    };

    std::vector<SStageModule> m_vModules;
    std::vector<VkPipelineShaderStageCreateInfo> m_vStageInfos;
};