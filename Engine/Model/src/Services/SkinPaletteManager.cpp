#include "Services/SkinPaletteManager.h"
#include "Logging/LogManager.h"
#include "API/RenderDevice.h"
#include "Device/VulkanRenderDevice.h"
#include "Vulkan/VulkanDescriptorSet.h"

bool CSkinPaletteManager::Initialize(uint32_t framesInFlight)
{
	ANUBIS_ASSERT(framesInFlight > 0);

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	m_uiNextHandle = 1;
	m_uiCurrentFrame = 0;
	m_mEntries.clear();
	
	m_vFrameBuffers.clear();
	constexpr uint32_t INITIAL_PALETTE_CAPACITY = 1024; // matrices, tune later

	m_vFrameBuffers.resize(framesInFlight);
	std::vector<IBuffer*> pBuffers;
    for (int32_t i = 0; i < framesInFlight; i++)
    {
		SPerFramePaletteBuffer& frameBuffer = m_vFrameBuffers[i];
		frameBuffer.capacityMatrices = 0;
		frameBuffer.usedMatrices = 0;
		frameBuffer.descriptorDirty = false;
		frameBuffer.pBuffer = nullptr;

		if (!EnsureFrameCapacity(i, INITIAL_PALETTE_CAPACITY))
		{
			syserr("Failed to initialize skin palette frame buffer {}", i);
			return false;
		}

		pBuffers.push_back(frameBuffer.pBuffer);
    }

	if (renderDev.GetAPI() == EGraphicsAPI::API_VULKAN)
	{
		SBindingContextDesc ctxDesc{};
		ctxDesc.m_uiFrameCount = MAX_FRAMES_IN_FLIGHT;
		ctxDesc.m_vBindings = CDescriptorSetLayouts::GetBonesBindings();

		ctxDesc.m_vBufferResources.push_back({ 0, pBuffers });

		m_pBonesBindingContext = std::make_unique<CVulkanDescriptorContext>();
		if (!m_pBonesBindingContext->Initialize(ctxDesc))
		{
			return (false);
		}
	}

	return (true);
}

void CSkinPaletteManager::Shutdown()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	for (auto& buffer : m_vFrameBuffers)
	{
		if (buffer.pBuffer)
		{
			renderDev.DestroyBuffer(buffer.pBuffer);
			AnubisSafeDelete(buffer.pBuffer);
		}
	}
	m_pBonesBindingContext->Destroy();
}

void CSkinPaletteManager::BeginFrame(uint32_t frameIndex)
{
	ANUBIS_ASSERT(frameIndex < m_vFrameBuffers.size());
	m_uiCurrentFrame = frameIndex;
	m_vFrameBuffers[frameIndex].usedMatrices = 0;
}

SkinPaletteHandle CSkinPaletteManager::RegisterPalette(uint32_t maxMatrices)
{
	SkinPaletteHandle handle = m_uiNextHandle++;

	SSkinPaletteEntry entry{};
	entry.handle = handle;
	entry.maxMatrices = maxMatrices;
	entry.lastUploadedFrame = UINT32_MAX;
	entry.active = true;

	m_mEntries.emplace(handle, entry);

	return handle;
}

void CSkinPaletteManager::ReleasePalette(SkinPaletteHandle handle)
{
	auto it = m_mEntries.find(handle);
	if (it == m_mEntries.end())
	{
		syswarn("ReleasePalette: handle {} not found", handle);
		return;
	}

	m_mEntries.erase(it);
}

SSkinPaletteGPUView CSkinPaletteManager::UploadPalette(SkinPaletteHandle handle, std::vector<Matrix4> matrices)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	SSkinPaletteGPUView view{};

	auto it = m_mEntries.find(handle);
	if (it == m_mEntries.end())
	{
		syserr("invalid handle {}", handle);
		return view;
	}

	SSkinPaletteEntry& entry = it->second;
	uint32_t matrixCount = static_cast<uint32_t>(matrices.size());

	if (matrixCount == 0)
	{
		return view;
	}

	SPerFramePaletteBuffer& frameBuffer = m_vFrameBuffers[m_uiCurrentFrame];
	uint32_t requiredTotal = frameBuffer.usedMatrices + matrixCount;

	if (!EnsureFrameCapacity(m_uiCurrentFrame, requiredTotal))
	{
		syserr("failed to grow palette buffer for handle {}", handle);
		return view;
	}

	uint32_t firstMatrix = frameBuffer.usedMatrices;
	uint64_t byteOffset = static_cast<uint64_t>(firstMatrix) * sizeof(Matrix4);
	uint64_t byteSize = static_cast<uint64_t>(matrixCount) * sizeof(Matrix4);

	if (NeedsDescriptorUpdate(m_uiCurrentFrame))
	{
		uint32_t bindingPoint = static_cast<uint32_t>(EStorageBufferBindingSets::BINDING_POINT_BONES_SSBO);
		m_pBonesBindingContext->UpdateBufferBinding(m_uiCurrentFrame, bindingPoint, frameBuffer.pBuffer, 0, frameBuffer.pBuffer->GetSize());
		ClearDescriptorDirty(m_uiCurrentFrame);
	}

	renderDev.UpdateBuffer(frameBuffer.pBuffer, matrices.data(), byteSize, byteOffset);

	frameBuffer.usedMatrices += matrixCount;
	entry.lastUploadedFrame = m_uiCurrentFrame;
	view.firstMatrix = firstMatrix;
	view.matrixCount = matrixCount;

	return view;
}

const IBuffer* CSkinPaletteManager::GetCurrentPaletteBuffer() const
{
	SPerFramePaletteBuffer frameBuffer = m_vFrameBuffers[m_uiCurrentFrame];
	return (frameBuffer.pBuffer);
}

IBuffer* CSkinPaletteManager::GetPaletteBuffer(uint32_t frameIndex)
{
	SPerFramePaletteBuffer frameBuffer = m_vFrameBuffers[frameIndex];
	return (frameBuffer.pBuffer);
}

bool CSkinPaletteManager::NeedsDescriptorUpdate(uint32_t frameIndex) const
{
	ANUBIS_ASSERT(frameIndex < m_vFrameBuffers.size());
	return m_vFrameBuffers[frameIndex].descriptorDirty;
}

void CSkinPaletteManager::ClearDescriptorDirty(uint32_t frameIndex)
{
	ANUBIS_ASSERT(frameIndex < m_vFrameBuffers.size());
	m_vFrameBuffers[frameIndex].descriptorDirty = false;
}

CVulkanDescriptorContext* CSkinPaletteManager::GetDescriptorContext()
{
	return (m_pBonesBindingContext.get());
}

bool CSkinPaletteManager::EnsureFrameCapacity(uint32_t frameIndex, uint32_t requiredMatrices)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	SPerFramePaletteBuffer& frameBuffer = m_vFrameBuffers[frameIndex];

	if (requiredMatrices <= frameBuffer.capacityMatrices && frameBuffer.pBuffer)
	{
		return true;
	}

	uint64_t bufferSize = static_cast<uint64_t>(requiredMatrices) * sizeof(Matrix4);

	SBufferDesc desc{};
	desc.m_stName = "Skin Palette SSBO";
	desc.m_eType = EBufferType::BUFFER_TYPE_STORAGE;
	desc.m_uiSize = bufferSize;
	desc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	desc.cpuWrite = true;

	EBufferResizeResult result = renderDev.EnsureBufferCapacity(frameBuffer.pBuffer, bufferSize, desc);
	if (result == EBufferResizeResult::BUFFER_RESIZE_FAILED)
	{
		return false;
	}

	frameBuffer.capacityMatrices = requiredMatrices;

	if (result == EBufferResizeResult::BUFFER_RESIZE_RECREATED)
	{
		frameBuffer.descriptorDirty = true;
	}

	return (true);
}

