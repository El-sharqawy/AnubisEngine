#pragma once

#include "TypeVector4.h"
#include "CoreEnums.h"
#include "CoreTypes.h"
#include "CommandList.h"
#include "ShaderProgram.h"
#include "Texture.h"

constexpr int32_t MAX_FRAMES_IN_FLIGHT = 2;

struct GLFWwindow;

class CIRenderDevice
{
public:
    virtual ~CIRenderDevice() = default;

    virtual EGraphicsAPI GetAPI() const = 0;

    virtual bool Initialize(GLFWwindow* pPlatformWindowHandle) = 0;
    virtual void Shutdown() = 0;

    virtual void BeginFrame() = 0;
    virtual void EndFrame() = 0;
    virtual void Present() = 0;

    virtual uint32_t GetCurrentFrameIndex() const = 0;

    virtual IBuffer* CreateBuffer(const SBufferDesc& bufferDesc, const void* initialData = nullptr) = 0;
    virtual ITexture2D* CreateTexture2D(const STextureDesc& textureDesc, const void* pixels = nullptr) = 0;
    virtual IShaderProgram* CreateShaderProgram(const SShaderDesc& shaderDesc) = 0;
    virtual IPipeline* CreatePipeline(const SPipelineDesc& pipelineDesc) = 0;
    virtual IMaterial* CreateMaterial() = 0;

    virtual void DestroyBuffer(IBuffer* pBuffer) = 0;
    virtual void DestroyTexture2D(ITexture2D* pTexture) = 0;
    virtual void DestroyShaderProgram(IShaderProgram* pShaderProgram) = 0;
    virtual void DestroyPipeline(IPipeline* pPipeline) = 0;
    virtual void DestroyMaterial(IMaterial* pMaterial) = 0;

    virtual bool UpdateBuffer(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset = 0) = 0;

    virtual ICommandList* GetCommandList() = 0;

    virtual void Resize(int32_t width, int32_t height) = 0;
};

