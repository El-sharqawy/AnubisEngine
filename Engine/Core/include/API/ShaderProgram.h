#pragma once

enum class EShaderStage : uint8_t
{
    SHADER_TYPE_VERTEX,
    SHADER_TYPE_TESSELLATION_CONTROL,
    SHADER_TYPE_TESSELLATION_EVALUATION,
    SHADER_TYPE_GEOMETRY,
    SHADER_TYPE_FRAGMENT,
    SHADER_TYPE_COMPUTE,
};

struct SShaderStageDesc
{
    EShaderStage stageType;
    std::string path;
};

struct SShaderDesc
{
    std::string m_stName = "Shader";
    std::vector<SShaderStageDesc> m_vStages;

    bool IsCompute() const
    {
        return m_vStages.size() == 1 && m_vStages[0].stageType == EShaderStage::SHADER_TYPE_COMPUTE;
    }

    bool HasStage(EShaderStage stage) const
    {
        return std::any_of(m_vStages.begin(), m_vStages.end(), [stage](const SShaderStageDesc& s) { return s.stageType == stage; });
    }
};

class IShaderProgram
{
public:
    virtual ~IShaderProgram() = default;
    virtual bool HasStage(EShaderStage stage) const = 0;
    virtual bool IsCompute() const = 0;

protected:
    std::string m_stName = "Shader";
};
