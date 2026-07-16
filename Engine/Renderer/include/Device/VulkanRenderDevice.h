#pragma once

#include "API/RenderDevice.h"
#include "Vulkan/VulkanDevice.h"
#include "Vulkan/VulkanSwapchain.h"
#include "Vulkan/VulkanSyncObject.h"
#include "Vulkan/VulkanPipeline.h"
#include "Vulkan/VulkanRenderer.h"
#include "Vulkan/VulkanCommandList.h"

class CActor;

struct SVulkanContext
{
    VkInstance       instance = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice         device = VK_NULL_HANDLE;
    VkQueue          graphicsQueue = VK_NULL_HANDLE;
    VkQueue          presentQueue = VK_NULL_HANDLE;
    VkCommandPool    commandPool = VK_NULL_HANDLE;
    VkSurfaceKHR     surface = VK_NULL_HANDLE;
    VkSwapchainKHR   swapchain = VK_NULL_HANDLE;
};

class CVulkanRenderDevice : public CIRenderDevice
{
public:
	CVulkanRenderDevice() = default;
	~CVulkanRenderDevice() = default;

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

    void DestroyBuffer(IBuffer* pBuffer) override;
    void DestroyTexture2D(ITexture2D* pTexture) override;
    void DestroyShaderProgram(IShaderProgram* pShaderProgram) override;
    void DestroyPipeline(IPipeline* pPipeline) override;
    void DestroyMaterial(IMaterial* pMaterial) override;

    bool UpdateBuffer(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset = 0) override;
    ICommandList* GetCommandList() override;
    void Resize(int32_t width, int32_t height) override;

    // Vulkan Only
    SVulkanContext& GetContext();
    VkFormat FindDepthFormat() const;
    SQueueFamilyIndices FindFamilyIndices(VkPhysicalDevice device) const;
    SSwapChainSupportDetails QuerySwapChainSupport() const;
    VkPhysicalDevice GetPhysicalDevice() const;
    uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
    CVulkanSwapchain* GetSwapChain();
    VkDevice GetDevice() const;

    bool UpdateBufferWithStaging(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset = 0);
    bool RecreateSwapchain();
    bool TransitionImageLayout(VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout);
    EBufferResizeResult EnsureBufferCapacity(IBuffer*& pBuffer, VkDeviceSize requiredSize, const SBufferDesc& templateDesc);

protected:
	bool CreateInstance();
	bool CreateSurface(GLFWwindow* pPlatformWindowHandle);
	bool CreateDevice();
	bool CreateSwapchain();
	bool CreateFrameResources();
	void CleanupSwapchain();

protected:
	bool UploadBufferInternal(class CVulkanBuffer* pBuffer, const void* pData, VkDeviceSize size, VkDeviceSize dstOffset);
	bool UploadWithStagingInternal(class CVulkanBuffer* pBuffer, const void* pData, VkDeviceSize size, VkDeviceSize dstOffset);
    bool CopyBufferInternal(VkBuffer srcBuffer, VkDeviceSize srcOffset, VkBuffer dstBuffer, VkDeviceSize dstOffset, VkDeviceSize size);
	bool TransitionImageLayoutInternal(VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout);
	bool CopyBufferToImageInternal(VkBuffer srcBuffer, VkImage dstImage, uint32_t width, uint32_t height);

private:
	EGraphicsAPI m_eGraphicsAPI = EGraphicsAPI::API_VULKAN;
	std::unique_ptr<CVulkanDevice> m_pVulkanDevice = nullptr;
	std::unique_ptr<CVulkanSwapchain> m_pVulkanSwapchain = nullptr;
    std::unique_ptr<CVulkanRenderer> m_pVulkanRenderer = nullptr;
    std::unique_ptr<CVulkanCommandList> m_pVulkanCommandList = nullptr;
    // Semaphores
    bool m_bFramebufferResized = false;
    VkCommandBuffer m_vkCurrentCommandBuffer = VK_NULL_HANDLE;

    // GLFWwindow
    GLFWwindow* m_pGLFWwindow = nullptr;
    SVulkanContext m_gContext = {};
};