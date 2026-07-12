#pragma once

#include <string>
#include <vector>
#include "API/ShaderProgram.h"

class COpenGLShader : public IShaderProgram
{
public:
    COpenGLShader() = default;
    ~COpenGLShader() = default;

    bool Create(const SShaderDesc& desc);
    void Destroy();
    void Clear();

    bool HasStage(EShaderStage stage) const override;
    bool IsCompute() const override;
    uint32_t GetProgramID() const;

private:
    struct SStageModule
    {
        EShaderStage stage;
        uint32_t shaderID;
    };

    std::vector<SStageModule> m_vModules;
    uint32_t m_uiProgramID = 0;
};