#pragma once

#include "API/RenderDevice.h"
#include "VulkanModel/StaticModel.h"
#include "OpenGL/OpenGLVertexArray.h"
#include "OpenGL/OpenGLCommandList.h"
#include "OpenGL/OpenGLRenderer.h"
#include <glad/gl.h>

struct SGLContext
{
    GLFWwindow* window = nullptr;
    int32_t     majorVersion = 4;
    int32_t     minorVersion = 6;
};

class COpenGLRenderDevice : public CIRenderDevice
{
public:
    COpenGLRenderDevice() = default;
    ~COpenGLRenderDevice() = default;

    EGraphicsAPI GetAPI() const override;

    bool Initialize(GLFWwindow* pPlatformWindowHandle) override;
    void Shutdown() override;

    void BeginFrame() override;
    void EndFrame() override;
    void Present() override;

    uint32_t GetCurrentFrameIndex() const override;
    
    IBuffer* CreateBuffer(const SBufferDesc& desc, const void* initialData = nullptr) override;
    ITexture2D* CreateTexture2D(const STextureDesc& desc, const void* pixels = nullptr) override;
    IShaderProgram* CreateShaderProgram(const SShaderDesc& desc) override;
    IPipeline* CreatePipeline(const SPipelineDesc& desc) override;
    IMaterial* CreateMaterial() override;
    CVertexArray* CreateVertexArray(const SVertexArrayDesc& vertexArrayDesc);

    void DestroyBuffer(IBuffer* pBuffer) override;
    void DestroyTexture2D(ITexture2D* pTexture) override;
    void DestroyShaderProgram(IShaderProgram* pShaderProgram) override;
    void DestroyPipeline(IPipeline* pPipeline) override;
    void DestroyMaterial(IMaterial* pMaterial) override;
    void DestroyVertexArray(CVertexArray* pVertexArray);

    bool UpdateBuffer(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset = 0) override;
    ICommandList* GetCommandList() override;
    void Resize(int32_t width, int32_t height) override;

protected:
    bool CreateGLContext(GLFWwindow* pPlatformWindowHandle);
    bool LoadExtensions();
    bool CreateFrameResources();
    void SetupDebugOutput();
    static void APIENTRY MyDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);
    void UpdateUniformBuffers(uint32_t currFrame);

private:
    EGraphicsAPI m_eGraphicsAPI = EGraphicsAPI::API_OPENGL;
    std::unique_ptr<COpenGLCommandList> m_pOpenGLCommandList = nullptr;
    std::unique_ptr<COpenGLRenderer> m_pOpenGLRenderer = nullptr;

    // GLFWwindow
    GLFWwindow* m_pGLFWwindow = nullptr;
    SGLContext m_gContext = {};

    uint32_t m_uiCurrentFrame = 0;

    // Camera Matrix
    IBuffer* m_pCameraUBO = nullptr;
    CStaticModel* m_pStaticModel = nullptr;
};