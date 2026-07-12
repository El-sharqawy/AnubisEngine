#include "Device/VulkanRenderDevice.h"
#include "Device/PipelinesManager.h"
#include "Vulkan/VulkanDevice.h"
#include "Vulkan/VulkanSwapchain.h"
#include "Vulkan/VulkanBuffer.h"
#include "Vulkan/VulkanTexture2D.h"
#include "VulkanModel/StaticModel.h"
#include "Window/WindowManager.h"
#include "Camera/Camera.h"
#include "Textures/TexturesManager.h"

EGraphicsAPI CVulkanRenderDevice::GetAPI() const
{
	return (EGraphicsAPI::API_VULKAN);
}

bool CVulkanRenderDevice::Initialize(GLFWwindow* pPlatformWindowHandle)
{
	m_pGLFWwindow = pPlatformWindowHandle;

	// Device
	m_pVulkanDevice = std::make_unique<CVulkanDevice>();
	if (!CreateInstance())
	{
		return (false);
	}

	if (!CreateSurface(pPlatformWindowHandle))
	{
		return (false);
	}

	if (!CreateDevice())
	{
		return (false);
	}

	if (!m_pVulkanDevice->CreateCommandPool())
	{
		syserr("Failed to create command pool");
		return false;
	}

	// Swapchain
	m_pVulkanSwapchain = std::make_unique<CVulkanSwapchain>();
	if (!CreateSwapchain())
	{
		return (false);
	}

	if (!CreateFrameResources())
	{
		return (false);
	}

	m_pVulkanRenderer = std::make_unique<CVulkanRenderer>();
	if (!m_pVulkanRenderer->Initialize())
	{
		return (false);
	}

	m_vpUniformBuffer.resize(MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		SBufferDesc uniformBufferDesc{};
		uniformBufferDesc.m_stName = "Camera Uniform Buffer";
		uniformBufferDesc.m_eType = EBufferType::BUFFER_TYPE_UNIFORM;
		uniformBufferDesc.m_uiSize = sizeof(SUniformBufferBlock);
		uniformBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		uniformBufferDesc.cpuWrite = true;

		IBuffer* pBuffer = CreateBuffer(uniformBufferDesc);
		if (!pBuffer)
		{
			return (false);
		}

		m_vpUniformBuffer[i] = dynamic_cast<CVulkanBuffer*>(pBuffer);
	}

	// Initialize Before Models
	CTexturesManager::Instance().Initialize();
	CPipelinesManager::Instance().Initialize();

	m_pVulkanCommandList = std::make_unique<CVulkanCommandList>();
	m_pVulkanCommandList->SetRenderer(m_pVulkanRenderer.get());

	m_pStaticModel = AnubisNew(CStaticModel, MEM_TAG_RENDERING);
	m_pStaticModel->ImportModel("Assets/Models/Female/SK_Shaman_4_1.fbx", m_vpUniformBuffer);
	m_pStaticModel->UploadToGPU();

	return (true);
}

void CVulkanRenderDevice::Shutdown()
{
	// Destroy in Reverse order
	vkDeviceWaitIdle(m_pVulkanDevice->GetDevice());

	// Start Clearing
	if (m_pStaticModel)
	{
		m_pStaticModel->Clear();
		AnubisSafeDelete(m_pStaticModel);
	}

	CPipelinesManager::Instance().Clear();
	CTexturesManager::Instance().Destroy();

	for (auto& buffer : m_vpUniformBuffer)
	{
		DestroyBuffer(buffer);
		AnubisSafeDelete(buffer);
		buffer = nullptr;
	}

	m_pVulkanRenderer->Shutdown();
	CleanupSwapchain();

	m_pVulkanDevice->Destroy();
}

void CVulkanRenderDevice::BeginFrame()
{
	m_vkCurrentCommandBuffer = m_pVulkanRenderer->BeginFrame();
	if (m_vkCurrentCommandBuffer == VK_NULL_HANDLE)
	{
		return;
	}

	m_pVulkanRenderer->BeginSwapchainRenderPass(m_vkCurrentCommandBuffer);
}

void CVulkanRenderDevice::EndFrame()
{
	if (m_vkCurrentCommandBuffer == VK_NULL_HANDLE)
	{
		return;
	}

	m_pVulkanRenderer->EndSwapchainRenderPass(m_vkCurrentCommandBuffer);
	m_pVulkanRenderer->EndFrame();

	m_vkCurrentCommandBuffer = VK_NULL_HANDLE;
}

void CVulkanRenderDevice::Present()
{
	UpdateUniformBuffers(GetCurrentFrameIndex());
	if (m_pStaticModel)
	{
		std::vector<SRenderItem> renderItems = m_pStaticModel->BuildRenderItems();
		m_pVulkanRenderer->SubimtRenderItems(renderItems);
	}

	ICommandList* cmd = GetCommandList();

	m_pVulkanRenderer->FlushRenderItems(cmd);

	/*
	if (cmd)
	{
		cmd->Begin();

		cmd->BindPipeline(meshPipeline);
		cmd->BindVertexBuffer(vertexBuffer);
		cmd->BindIndexBuffer(indexBuffer, EIndexType::INDEX_UINT32);
		cmd->BindMaterial(material, renderDevice->GetCurrentFrameIndex());
		cmd->PushConstants(&pushData, sizeof(pushData));
		cmd->DrawIndexed(indexCount);

		cmd->End();
	}
	*/
}

uint32_t CVulkanRenderDevice::GetCurrentFrameIndex() const
{
	return m_pVulkanRenderer->GetCurrentFrameIndex();
}

IBuffer* CVulkanRenderDevice::CreateBuffer(const SBufferDesc& bufferDesc, const void* initialData)
{
	CVulkanBuffer* pBuffer = AnubisNew(CVulkanBuffer, MEM_TAG_GPU_BUFFER);
	assert(bufferDesc.m_uiSize > 0);

	if (bufferDesc.m_uiSize == 0)
	{
		return (nullptr);
	}

	VkBufferUsageFlags usageFlags = VulkanUtils::ToVulkanBufferUsage(bufferDesc.m_eType, bufferDesc.cpuWrite);
	VkMemoryPropertyFlags memPropertyFlags = VulkanUtils::ToVulkanMemoryProperties(bufferDesc.m_eMemoryType);
	VkSharingMode sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	VkBufferCreateInfo bufferInfo{};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.pNext = nullptr;
	bufferInfo.flags = 0;
	bufferInfo.size = static_cast<VkDeviceSize>(bufferDesc.m_uiSize);
	bufferInfo.usage = usageFlags;
	bufferInfo.sharingMode = sharingMode; // default


	VkBuffer buffer = VK_NULL_HANDLE;
	if (VK_CHECK_BOOL(vkCreateBuffer(GetDevice(), &bufferInfo, VK_NULL_HANDLE, &buffer)) == false)
	{
		syserr("Failed to Create Buffer {}", bufferDesc.m_stName);
		return (nullptr);
	}

	VkMemoryRequirements bufMemRequirements = {};
	vkGetBufferMemoryRequirements(GetDevice(), buffer, &bufMemRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.pNext = nullptr;
	allocInfo.allocationSize = bufMemRequirements.size;
	allocInfo.memoryTypeIndex = FindMemoryType(GetPhysicalDevice(), bufMemRequirements.memoryTypeBits, memPropertyFlags);

	VkDeviceMemory bufferMemory = VK_NULL_HANDLE;
	if (VK_CHECK_BOOL(vkAllocateMemory(GetDevice(), &allocInfo, VK_NULL_HANDLE, &bufferMemory)) == false)
	{
		vkDestroyBuffer(GetDevice(), buffer, VK_NULL_HANDLE);
		syserr("failed to allocate buffer %s memory!", bufferDesc.m_stName); // Cleanup Resources
		return (nullptr);
	}

	if (VK_CHECK_BOOL(vkBindBufferMemory(GetDevice(), buffer, bufferMemory, 0)) == false)
	{
		vkDestroyBuffer(GetDevice(), buffer, VK_NULL_HANDLE);
		vkFreeMemory(GetDevice(), bufferMemory, VK_NULL_HANDLE);
		syserr("Failed to bind buffer memory");  // Cleanup Resources
		return (nullptr);
	}


	SVulkanBufferData vulkanBufferData{};
	vulkanBufferData.m_vkBuffer = buffer;
	vulkanBufferData.m_vkBufferMemory = bufferMemory;
	vulkanBufferData.m_vkUsageFlags = usageFlags;
	vulkanBufferData.m_vkMemoryPropertyFlags = memPropertyFlags;
	vulkanBufferData.m_vkSharingMode = sharingMode;

	pBuffer->UpdateBufferData(vulkanBufferData, bufferDesc);

	if (initialData != nullptr)
	{
		if (!(memPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT))
		{
			syserr("Buffer {} memory is not host visible ", pBuffer->GetName());
			return pBuffer;
		}

		void* mappedData = nullptr;
		if (VK_CHECK_BOOL(vkMapMemory(GetDevice(), pBuffer->GetBufferMemory(), 0, pBuffer->GetSize(), 0, &mappedData)) == false)
		{
			syserr("Failed to map buffer memory for buffer {}", pBuffer->GetName());
			return pBuffer;
		}

		memcpy(mappedData, initialData, pBuffer->GetSize());

		// Flush our memory
		if (!(memPropertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
		{
			VkMappedMemoryRange range{};
			range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
			range.memory = pBuffer->GetBufferMemory();
			range.offset = static_cast<VkDeviceSize>(0);
			range.size = static_cast<VkDeviceSize>(pBuffer->GetSize());

			if (!VK_CHECK_BOOL(vkFlushMappedMemoryRanges(GetDevice(), 1, &range)))
			{
				vkUnmapMemory(GetDevice(), pBuffer->GetBufferMemory());
				syserr("Failed to flush mapped buffer memory for buffer {}", pBuffer->GetName());
				return pBuffer;
			}
		}

		vkUnmapMemory(GetDevice(), pBuffer->GetBufferMemory());
	}
	return (pBuffer);
}

ITexture2D* CVulkanRenderDevice::CreateTexture2D(const STextureDesc& textureDesc, const void* pixels)
{
	CVulkanTexture2D* pTexture = AnubisNew(CVulkanTexture2D, MEM_TAG_RENDERING);

	const void* pImageData = nullptr;
	bool bNeedsFree = false;

	int32_t iWidth = 0, iHeight = 0, iChannels = 0;

	if (textureDesc.m_fsFilePath.empty() == false)
	{
		stbi_set_flip_vertically_on_load(false);

		std::string stTexturePath = textureDesc.m_fsFilePath.string();
		stbi_uc* pixels = stbi_load(stTexturePath.c_str(), &iWidth, &iHeight, &iChannels, STBI_rgb_alpha);
		if (pixels == nullptr)
		{
			syserr("Failed to load texture: {}", stTexturePath.c_str());
			return (nullptr);
		}

		pImageData = pixels;
		iChannels = 4;
		bNeedsFree = true;
	}
	else if (pixels)
	{
		pImageData = pixels;
		iWidth = textureDesc.m_iWidth;
		iHeight = textureDesc.m_iHeight;
		iChannels = textureDesc.m_iChannels;
	}
	else
	{
		syserr("Texture '{}' has no file path and no image data", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	if (iWidth <= 0 || iHeight <= 0)
	{
		syserr("Texture '{}' has invalid dimensions", textureDesc.m_stName.c_str());

		if (bNeedsFree)
		{
			stbi_image_free(const_cast<void*>(pImageData));
		}

		return nullptr;
	}

	VkDeviceSize imageSize = static_cast<VkDeviceSize>(iWidth * iHeight * iChannels); // 4 channels

	SBufferDesc bufferDesc;
	bufferDesc.m_stName = textureDesc.m_stName + "_temporary_buffer";
	bufferDesc.m_eType = EBufferType::BUFFER_TYPE_STAGING;
	bufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	bufferDesc.cpuWrite = true;
	bufferDesc.m_uiSize = imageSize;

	CVulkanBuffer* pTempBuffer = dynamic_cast<CVulkanBuffer*>(CreateBuffer(bufferDesc, pImageData));

	// Free pixels data since we have it in our buffer
	if (bNeedsFree)
	{
		stbi_image_free(const_cast<void*>(pImageData));
	}

	// Create Vulkan Image
	uint32_t mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(iWidth, iHeight)))) + 1;

	VkImageType imageType = VulkanUtils::ToVkImageType(textureDesc.m_eType);
	VkImageViewType viewType = VulkanUtils::ToVkImageViewType(textureDesc.m_eType);
	VkFormat format = VulkanUtils::ToVkFormat(textureDesc.m_eFormat);
	VkImageUsageFlags usage = VulkanUtils::ToVkImageUsage(textureDesc.m_uiUsageFlags);

	VkFilter magFilter = VulkanUtils::ToVkFilter(textureDesc.m_eMagFilter);
	VkFilter minFilter = VulkanUtils::ToVkFilter(textureDesc.m_eMinFilter);
	VkSamplerMipmapMode mipMode = VulkanUtils::ToVkMipmapMode(textureDesc.m_eMinFilter);

	VkSamplerAddressMode wrapU = VulkanUtils::ToVkAddressMode(textureDesc.m_eWrapU);
	VkSamplerAddressMode wrapV = VulkanUtils::ToVkAddressMode(textureDesc.m_eWrapV);
	VkSamplerAddressMode wrapW = VulkanUtils::ToVkAddressMode(textureDesc.m_eWrapW);

	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.imageType = imageType;
	imageInfo.extent.width = static_cast<uint32_t>(iWidth);
	imageInfo.extent.height = static_cast<uint32_t>(iHeight);
	imageInfo.extent.depth = 1;
	imageInfo.mipLevels = 1;
	imageInfo.arrayLayers = 1;
	imageInfo.format = format;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	imageInfo.usage = usage;
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.flags = 0; // Optional

	VkImage vkImage = VK_NULL_HANDLE;

	if (VK_CHECK_BOOL(vkCreateImage(GetDevice(), &imageInfo, nullptr, &vkImage)) == false)
	{
		syserr("failed to create Vulkan image for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Allocate Memory for the Image
	VkMemoryRequirements memRequirements;
	vkGetImageMemoryRequirements(GetDevice(), vkImage, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;
	allocInfo.memoryTypeIndex = FindMemoryType(GetPhysicalDevice(), memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VkDeviceMemory vkImageMemory = pTexture->GetImageMemory();
	if (VK_CHECK_BOOL(vkAllocateMemory(GetDevice(), &allocInfo, nullptr, &vkImageMemory)) == false)
	{
		syserr("failed to allocate image memory! for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Bind the Image Memory
	if (VK_CHECK_BOOL(vkBindImageMemory(GetDevice(), vkImage, vkImageMemory, 0)) == false)
	{
		syserr("failed to bind image memory for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Set Texture Layout to recieve data
	if (!TransitionImageLayout(vkImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL))
	{
		syserr("failed to transition image layout 1! for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Copy Buffer Data to Image
	if (!CopyBufferToImageInternal(pTempBuffer->GetBuffer(), vkImage, static_cast<uint32_t>(iWidth), static_cast<uint32_t>(iHeight)))
	{
		syserr("failed to copy buffer to image for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Set Texture Layout to Shader data
	if (!TransitionImageLayout(vkImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL))
	{
		syserr("failed to transition image layout 2! for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Create Image View
	VkImageView vkImageView = VK_NULL_HANDLE;

	VkImageViewCreateInfo imageViewInfo{};
	imageViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	imageViewInfo.pNext = nullptr;
	imageViewInfo.flags = 0;
	imageViewInfo.image = vkImage;
	imageViewInfo.viewType = viewType;
	imageViewInfo.format = format;
	imageViewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	imageViewInfo.subresourceRange.baseMipLevel = 0;
	imageViewInfo.subresourceRange.levelCount = 1;
	imageViewInfo.subresourceRange.baseArrayLayer = 0;
	imageViewInfo.subresourceRange.layerCount = 1;

	if (VK_CHECK_BOOL(vkCreateImageView(GetDevice(), &imageViewInfo, VK_NULL_HANDLE, &vkImageView)) == false)
	{
		syserr("Failed to Create Texture Image View for texture '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	if (vkImageView == VK_NULL_HANDLE)
	{
		syserr("Failed to Create Image View for textuer '{}'", textureDesc.m_stName.c_str());
		return (nullptr);
	}

	// Create Texture Sampler 
	VkPhysicalDeviceProperties properties{};
	vkGetPhysicalDeviceProperties(GetPhysicalDevice(), &properties);

	VkSamplerCreateInfo samplerCreateInfo{};
	samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerCreateInfo.pNext = nullptr;
	samplerCreateInfo.flags = 0;
	samplerCreateInfo.magFilter = magFilter;
	samplerCreateInfo.minFilter = minFilter;
	samplerCreateInfo.addressModeU = wrapU;
	samplerCreateInfo.addressModeV = wrapV;
	samplerCreateInfo.addressModeW = wrapW;
	samplerCreateInfo.mipLodBias = 0.0f;
	samplerCreateInfo.anisotropyEnable = VK_TRUE;
	samplerCreateInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;
	samplerCreateInfo.compareEnable = VK_FALSE;
	samplerCreateInfo.compareOp = VK_COMPARE_OP_ALWAYS;
	samplerCreateInfo.minLod = 0.0f;
	samplerCreateInfo.maxLod = 0.0f;
	samplerCreateInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
	samplerCreateInfo.unnormalizedCoordinates = VK_FALSE;

	VkSampler vkImageSampler = VK_NULL_HANDLE;
	if (VK_CHECK_BOOL(vkCreateSampler(GetDevice(), &samplerCreateInfo, VK_NULL_HANDLE, &vkImageSampler)) == false)
	{
		syserr("Failed to Create Texture {} sampler", textureDesc.m_stName);
		return (nullptr);
	}

	// Free Memory
	DestroyBuffer(pTempBuffer);
	AnubisSafeDelete(pTempBuffer);

	SVulkanTexture2DInfo texInfo{};
	texInfo.m_vkImage = vkImage;
	texInfo.m_vkImageMemory = vkImageMemory;
	texInfo.m_vkImageView = vkImageView;
	texInfo.m_vkSampler = vkImageSampler;
	texInfo.iWidth = iWidth;
	texInfo.iHeight = iHeight;
	texInfo.iChannels = iChannels;
	pTexture->UpdateTextureData(texInfo, textureDesc);

	return (pTexture);
}

IShaderProgram* CVulkanRenderDevice::CreateShaderProgram(const SShaderDesc& shaderDesc)
{
	// Check for Shader
	CVulkanShader* pShader = AnubisNew(CVulkanShader, MEM_TAG_RENDERING);
	if (!pShader->Create(shaderDesc))
	{
		syserr("Failed to Create Shader {}", shaderDesc.m_stName);
		return (nullptr);
	}

	return pShader;
}

IPipeline* CVulkanRenderDevice::CreatePipeline(const SPipelineDesc& pipelineDesc)
{
	CVulkanPipeline* pPipeline = AnubisNew(CVulkanPipeline, MEM_TAG_RENDERING);
	if (!pPipeline->Initialize(pipelineDesc, m_pVulkanSwapchain->GetRenderPass(), m_pVulkanSwapchain->GetSwapchainExtent()))
	{
		syserr("Failed to Create Pipeline!");
		return (nullptr);
	}

	return (pPipeline);
}

IMaterial* CVulkanRenderDevice::CreateMaterial()
{
	CVulkanMaterial* pMaterial = AnubisNew(CVulkanMaterial, MEM_TAG_MATERIAL);
	if (!pMaterial)
	{
		syserr("CreateMaterial: allocation failed");
		return nullptr;
	}

	return pMaterial;
}

void CVulkanRenderDevice::DestroyBuffer(IBuffer* pBuffer)
{
	assert(pBuffer != nullptr);

	CVulkanBuffer* vkBuffer = dynamic_cast<CVulkanBuffer*>(pBuffer); // cast to nullptr if it's wrong

	if (!vkBuffer)
	{
		return;
	}

	VkBuffer buffer = vkBuffer->GetBuffer();
	if (buffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(GetDevice(), vkBuffer->GetBuffer(), VK_NULL_HANDLE);
		buffer = VK_NULL_HANDLE;
	}

	VkDeviceMemory bufferMemory = vkBuffer->GetBufferMemory();
	if (bufferMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(GetDevice(), bufferMemory, VK_NULL_HANDLE);
		bufferMemory = VK_NULL_HANDLE;
	}

	vkBuffer->Clear();
}

void CVulkanRenderDevice::DestroyTexture2D(ITexture2D* pTexture)
{
	// 1. Safety check
	if (!pTexture)
	{
		return;
	}

	// 2. Cast and safely verify it succeeded
	CVulkanTexture2D* vkTexture = dynamic_cast<CVulkanTexture2D*>(pTexture);
	if (!vkTexture)
	{
		assert(false && "Passed a non-Vulkan texture to CVulkanRenderDevice!");
		return;
	}

	// 3. Destroy Vulkan objects in the CORRECT reverse order of creation
	VkDevice device = GetDevice();

	if (vkTexture->GetSampler() != VK_NULL_HANDLE)
	{
		vkDestroySampler(device, vkTexture->GetSampler(), nullptr);
	}

	if (vkTexture->GetImageView() != VK_NULL_HANDLE)
	{
		vkDestroyImageView(device, vkTexture->GetImageView(), nullptr);
	}

	if (vkTexture->GetImage() != VK_NULL_HANDLE)
	{
		vkDestroyImage(device, vkTexture->GetImage(), nullptr);
	}

	if (vkTexture->GetImageMemory() != VK_NULL_HANDLE)
	{
		vkFreeMemory(device, vkTexture->GetImageMemory(), nullptr);
	}
	vkTexture->Clear();
}

void CVulkanRenderDevice::DestroyShaderProgram(IShaderProgram* pShaderProgram)
{
	if (!pShaderProgram)
	{
		return;
	}
}

void CVulkanRenderDevice::DestroyPipeline(IPipeline* pPipeline)
{
	// Safety Check
	if (!pPipeline)
	{
		return;
	}

	CVulkanPipeline* vkPipeline = dynamic_cast<CVulkanPipeline*>(pPipeline);
	if (!vkPipeline)
	{
		assert(false && "Passed a non-Vulkan Pipeline to CVulkanRenderDevice!");
		return;
	}

	// Just Call Destroy for it ..
	if (vkPipeline->GetGraphicsPipeline() != VK_NULL_HANDLE)
	{
		vkDestroyPipeline(GetDevice(), vkPipeline->GetGraphicsPipeline(), VK_NULL_HANDLE);
	}

	if (vkPipeline->GetPipelineLayout() != VK_NULL_HANDLE)
	{
		vkDestroyPipelineLayout(GetDevice(), vkPipeline->GetPipelineLayout(), VK_NULL_HANDLE);
	}

	vkPipeline->Clear();
}

void CVulkanRenderDevice::DestroyMaterial(IMaterial* pMaterial)
{
	// Safety Check
	if (!pMaterial)
	{
		return;
	}

	pMaterial->ClearMaterial();
}

bool CVulkanRenderDevice::UpdateBuffer(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset)
{
	assert(pBuffer != nullptr);
	assert(pData != nullptr);

	CVulkanBuffer* vkBuffer = dynamic_cast<CVulkanBuffer*>(pBuffer); // cast to nullptr if it's wrong
	if (!vkBuffer)
	{
		return (false); // dynaimc cast failed
	}

	return UploadBufferInternal(vkBuffer, pData, size, dstOffset);
}

ICommandList* CVulkanRenderDevice::GetCommandList()
{
	if (m_vkCurrentCommandBuffer == VK_NULL_HANDLE)
		return nullptr;

	return m_pVulkanCommandList.get();
}

void CVulkanRenderDevice::Resize(int32_t width, int32_t height)
{
}

bool CVulkanRenderDevice::CreateInstance()
{
	if (!m_pVulkanDevice->Initialize())
	{
		syserr("Failed to Initialize Vulkan Device");
		return (false);
	}

	return (true);
}

bool CVulkanRenderDevice::CreateSurface(GLFWwindow* pPlatformWindowHandle)
{
	if (!m_pVulkanDevice->InitializeSurface(pPlatformWindowHandle))
	{
		syserr("Failed to Initialize Vulkan Device Surface");
		return (false);
	}

	return (true);
}

bool CVulkanRenderDevice::CreateDevice()
{
	if (!m_pVulkanDevice->ChoosePhysicalDevice())
	{
		syserr("Faile to Choose Vulkan Physical Device");
		return (false);
	}

	if (!m_pVulkanDevice->CreateLogicalDevice())
	{
		syserr("Faile to Choose Vulkan Logical Device");
		return (false);
	}

	return (true);
}

bool CVulkanRenderDevice::CreateFrameResources()
{
	return true;
}

bool CVulkanRenderDevice::RecreateSwapchain()
{
	int32_t width = 0;
	int32_t height = 0;
	glfwGetFramebufferSize(m_pGLFWwindow, &width, &height);

	while (width == 0 || height == 0)
	{
		glfwGetFramebufferSize(m_pGLFWwindow, &width, &height);
		glfwWaitEvents();
	}

	if (!VK_CHECK_BOOL(vkDeviceWaitIdle(m_pVulkanDevice->GetDevice())))
	{
		syserr("Failed to wait for device idle");
		return false;
	}

	m_pVulkanSwapchain->Clear();

	if (!CreateSwapchain())
	{
		syserr("Failed to recreate swapchain");
		return false;
	}

	return true;
}

bool CVulkanRenderDevice::TransitionImageLayout(VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout)
{
	return TransitionImageLayoutInternal(image, oldLayout, newLayout);
}

bool CVulkanRenderDevice::UploadBufferInternal(CVulkanBuffer* pBuffer, const void* pData, VkDeviceSize size, VkDeviceSize dstOffset)
{
	VkMemoryPropertyFlags memFlags = VulkanUtils::ToVulkanMemoryProperties(pBuffer->GetMemoryType());

	if (!(memFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT))
	{
		syserr("Buffer {} memory is not host visible ", pBuffer->GetName());
		return (false);
	}

	if (dstOffset + size > pBuffer->GetSize())
	{
		syserr("Can't exceed buffer size and offset! ({} : {})", dstOffset + size, pBuffer->GetSize());
		return (false);
	}

	void* mappedData = nullptr;
	if (VK_CHECK_BOOL(vkMapMemory(GetDevice(), pBuffer->GetBufferMemory(), dstOffset, size, 0, &mappedData)) == false)
	{
		syserr("Failed to map buffer memory for buffer {}", pBuffer->GetName());
		return (false);
	}

	memcpy(mappedData, pData, size);

	// Flush our memory
	if (!(memFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
	{
		VkMappedMemoryRange range{};
		range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		range.memory = pBuffer->GetBufferMemory();
		range.offset = static_cast<VkDeviceSize>(dstOffset);
		range.size = static_cast<VkDeviceSize>(size);

		if (!VK_CHECK_BOOL(vkFlushMappedMemoryRanges(GetDevice(), 1, &range)))
		{
			vkUnmapMemory(GetDevice(), pBuffer->GetBufferMemory());
			syserr("Failed to flush mapped buffer memory for buffer {}", pBuffer->GetName());
			return (false);
		}
	}

	vkUnmapMemory(GetDevice(), pBuffer->GetBufferMemory());
	return (true);
}

bool CVulkanRenderDevice::UploadWithStagingInternal(CVulkanBuffer* pBuffer, const void* pData, VkDeviceSize size, VkDeviceSize dstOffset)
{
	assert(pBuffer != nullptr);
	assert(pData != nullptr);

	VkBufferUsageFlags bufferFLags = pBuffer->GetUsageFlags();

	if (!(bufferFLags & VK_BUFFER_USAGE_TRANSFER_DST_BIT))
	{
		syserr("Buffer was not created with VK_BUFFER_USAGE_TRANSFER_DST_BIT");
		return (false);
	}

	if (dstOffset + size > pBuffer->GetSize())
	{
		syserr("Can't exceed buffer size and offset! (%zu : %zu)", dstOffset + size, pBuffer->GetSize());
		return (false);
	}

	SBufferDesc stagingDesc{};
	stagingDesc.m_stName = pBuffer->GetName() + "_staging";
	stagingDesc.m_eType = EBufferType::BUFFER_TYPE_STAGING;
	stagingDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	stagingDesc.m_uiSize = size;
	stagingDesc.cpuWrite = true;

	IBuffer* stagingBuffer = CreateBuffer(stagingDesc, pData);

	if (!stagingBuffer)
	{
		syserr("Failed to create staging buffer");
		return (false);
	}

	auto* vkStagingBuffer = dynamic_cast<CVulkanBuffer*>(stagingBuffer);
	if (!vkStagingBuffer)
	{
		DestroyBuffer(stagingBuffer);
		AnubisSafeDelete(stagingBuffer);
		syserr("Failed to cast staging buffer");
		return false;
	}

	if (!CopyBufferInternal(vkStagingBuffer->GetBuffer(), 0, pBuffer->GetBuffer(), dstOffset, size))
	{
		DestroyBuffer(stagingBuffer);
		AnubisSafeDelete(stagingBuffer);
		syserr("Failed to Copy Buffer");
		return (false);
	}

	DestroyBuffer(stagingBuffer);
	AnubisSafeDelete(stagingBuffer);
	return (true);
}

bool CVulkanRenderDevice::CopyBufferInternal(VkBuffer srcBuffer, VkDeviceSize srcOffset, VkBuffer dstBuffer, VkDeviceSize dstOffset, VkDeviceSize size)
{
	VkCommandBuffer commandBuffer = m_pVulkanDevice->BeginSingleTimeCommands();
	if (commandBuffer == VK_NULL_HANDLE)
	{
		syserr("Failed to begin single time commands");
		return (false);
	}

	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = srcOffset;
	copyRegion.dstOffset = dstOffset;
	copyRegion.size = size;
	vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

	if (!m_pVulkanDevice->EndSingleTimeCommands(commandBuffer))
	{
		syserr("Failed to end single time commands");
		return (false);
	}

	return (true);
}

void CVulkanRenderDevice::CleanupSwapchain()
{
	m_pVulkanSwapchain->Clear();
}

bool CVulkanRenderDevice::CreateSwapchain()
{
	if (!m_pVulkanSwapchain->InitializeSwapchain())
	{
		return (false);
	}

	if (!m_pVulkanSwapchain->CreateImageViews())
	{
		return (false);
	}

	if (!m_pVulkanSwapchain->CreateDepthResources())
	{
		return (false);
	}

	if (!m_pVulkanSwapchain->CreateRenderPass())
	{
		return (false);
	}

	if (!m_pVulkanSwapchain->CreateFramebuffers())
	{
		return (false);
	}

	return (true);
}

SVulkanContext& CVulkanRenderDevice::GetContext()
{
	m_gContext.instance = m_pVulkanDevice->GetInstance();
	m_gContext.device = m_pVulkanDevice->GetDevice();
	m_gContext.physicalDevice = m_pVulkanDevice->GetPhysicalDevice();
	m_gContext.graphicsQueue = m_pVulkanDevice->GetGraphicsQueue();
	m_gContext.presentQueue = m_pVulkanDevice->GetPresentQueue();
	m_gContext.commandPool = m_pVulkanDevice->GetCommandPool();
	m_gContext.surface = m_pVulkanDevice->GetDeviceSurface();
	m_gContext.swapchain = m_pVulkanSwapchain->GetSwapchain();

	return (m_gContext);
}


VkFormat CVulkanRenderDevice::FindDepthFormat() const
{
	return (m_pVulkanDevice->FindDepthFormat());
}

SQueueFamilyIndices CVulkanRenderDevice::FindFamilyIndices(VkPhysicalDevice device) const
{
	return (m_pVulkanDevice->FindFamilyIndices(device));
}

SSwapChainSupportDetails CVulkanRenderDevice::QuerySwapChainSupport() const
{
	return (m_pVulkanDevice->QuerySwapChainSupport());
}

VkPhysicalDevice CVulkanRenderDevice::GetPhysicalDevice() const
{
	return (m_pVulkanDevice->GetPhysicalDevice());
}

uint32_t CVulkanRenderDevice::FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
	return (m_pVulkanDevice->FindMemoryType(physicalDevice, typeFilter, properties));
}

CVulkanSwapchain* CVulkanRenderDevice::GetSwapChain()
{
	return m_pVulkanSwapchain.get();
}

VkDevice CVulkanRenderDevice::GetDevice() const
{
	return (m_pVulkanDevice->GetDevice());
}

bool CVulkanRenderDevice::UpdateBufferWithStaging(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset)
{
	assert(pBuffer != nullptr);
	assert(pData != nullptr);

	CVulkanBuffer* vkBuffer = dynamic_cast<CVulkanBuffer*>(pBuffer); // cast to nullptr if it's wrong
	if (!vkBuffer)
	{
		return (false); // dynaimc cast failed
	}

	return UploadWithStagingInternal(vkBuffer, pData, size, dstOffset);
}

bool CVulkanRenderDevice::TransitionImageLayoutInternal(VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout)
{
	VkCommandBuffer commandBuffer = m_pVulkanDevice->BeginSingleTimeCommands();

	if (commandBuffer == VK_NULL_HANDLE)
	{
		syserr("Failed to begin single time commands");
		return (false);
	}

	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.pNext = VK_NULL_HANDLE;
	barrier.srcAccessMask = 0;
	barrier.dstAccessMask = 0;

	VkPipelineStageFlags sourceStage = 0;
	VkPipelineStageFlags destinationStage = 0;

	barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;

	if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
	{
		barrier.srcAccessMask = 0;
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

		sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
	{
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

		sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
		destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
	{
		barrier.srcAccessMask = 0;
		barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

		sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	}
	else
	{
		syserr("Unsupported layout transition: {} -> {}", static_cast<uint32_t>(oldLayout), static_cast<uint32_t>(newLayout));
		vkFreeCommandBuffers(GetDevice(), m_pVulkanDevice->GetCommandPool(), 1, &commandBuffer);
		return false;
	}

	barrier.oldLayout = oldLayout;
	barrier.newLayout = newLayout;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = image;
	barrier.subresourceRange.baseMipLevel = 0;
	barrier.subresourceRange.levelCount = 1;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 1;

	vkCmdPipelineBarrier(
		commandBuffer,
		sourceStage,
		destinationStage,
		0,
		0, nullptr,
		0, nullptr,
		1, &barrier
	);

	if (!m_pVulkanDevice->EndSingleTimeCommands(commandBuffer))
	{
		syserr("Failed to end single time commands");
		return (false);
	}

	return (true);
}

bool CVulkanRenderDevice::CopyBufferToImageInternal(VkBuffer srcBuffer, VkImage dstImage, uint32_t width, uint32_t height)
{
	VkCommandBuffer commandBuffer = m_pVulkanDevice->BeginSingleTimeCommands();

	if (commandBuffer == VK_NULL_HANDLE)
	{
		syserr("Failed to begin single time commands");
		return (false);
	}

	VkBufferImageCopy bufferImageCopyRegion{};
	bufferImageCopyRegion.bufferOffset = 0;
	bufferImageCopyRegion.bufferRowLength = 0; // Tightly packed
	bufferImageCopyRegion.bufferImageHeight = 0; // Tightly packed
	bufferImageCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; // Color image
	bufferImageCopyRegion.imageSubresource.mipLevel = 0; // Base mip level
	bufferImageCopyRegion.imageSubresource.baseArrayLayer = 0; // Base array layer
	bufferImageCopyRegion.imageSubresource.layerCount = 1; // Single layer
	bufferImageCopyRegion.imageOffset = { 0, 0, 0 }; // No offset
	bufferImageCopyRegion.imageExtent = { width, height, 1 }; // Full image extent

	vkCmdCopyBufferToImage(commandBuffer, srcBuffer, dstImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &bufferImageCopyRegion);

	if (!m_pVulkanDevice->EndSingleTimeCommands(commandBuffer))
	{
		syserr("Failed to end single time commands");
		return (false);
	}

	return (true);
}

void CVulkanRenderDevice::UpdateUniformBuffers(uint32_t currFrame)
{
	auto& windowMgr = CServiceLocator::Get<CWindowManager>();

	static auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

	SUniformBufferBlock bufferBlock{};

	bufferBlock.matView = windowMgr.GetCamera()->GetViewMatrix();

	bufferBlock.matProjection = windowMgr.GetCamera()->GetProjectionMatrix();
	bufferBlock.matProjection[1][1] *= -1.0f;

	bufferBlock.matViewProjection = bufferBlock.matProjection * bufferBlock.matView;

	UpdateBuffer(m_vpUniformBuffer[currFrame], &bufferBlock, sizeof(SUniformBufferBlock), 0);
}
