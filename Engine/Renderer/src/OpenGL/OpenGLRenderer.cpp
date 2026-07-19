#include "OpenGL/OpenGLRenderer.h"
#include "OpenGL/OpenGLCommandList.h"
#include "Services/RenderQueue.h"
#include "Services/ActorsManager.h"
#include "Services/AssimpModelImporter.h"
#include "Device/OpenGLRenderDevice.h"
#include "Logging/LogManager.h"
#include "Window/WindowManager.h"
#include "Camera/Camera.h"
#include "Services/SkinPaletteManager.h"

bool COpenGLRenderer::Initialize()
{
	if (!InitializeRendererBuffers())
	{
		return (false);
	}

	CServiceLocator::Get<CSkinPaletteManager>().Initialize(MAX_FRAMES_IN_FLIGHT);

	return (true);
}

void COpenGLRenderer::Destroy()
{
	CServiceLocator::Get<CSkinPaletteManager>().Shutdown();

	DestroyRendererBuffers();
}

void COpenGLRenderer::BeginFrame()
{
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void COpenGLRenderer::Present()
{
	UpdateRendererBuffers();
	CServiceLocator::Get<CSkinPaletteManager>().BeginFrame(GetCurrentFrameIndex());
}

void COpenGLRenderer::EndFrame()
{
	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	renderQueue.EndFrame();
	m_uiCurrentFrame = (m_uiCurrentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void COpenGLRenderer::SubmitRenderItem(const SRenderInstance& renderItem)
{
	m_vRenderItems.push_back(renderItem);
}

void COpenGLRenderer::SubimtRenderItems(const std::vector<SRenderInstance>& vRenderItems)
{
	m_vRenderItems.insert(m_vRenderItems.end(), vRenderItems.begin(), vRenderItems.end());
}

void COpenGLRenderer::FlushRenderItems(ICommandList* pCmd)
{
	if (!pCmd)
	{
		return;
	}

	COpenGLCommandList* glCmd = static_cast<COpenGLCommandList*>(pCmd);

	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	renderQueue.SortRenderItems();
	std::vector<SRenderInstance> renderItems = renderQueue.GetRenderItems();

	for (const auto& renderItem : renderItems)
	{
		const auto batch = renderItem.pBatch;

		glCmd->BindPipeline(batch->pPipeline);
		glCmd->BindVertexArray(batch->pVertexArray);
		glCmd->BindMaterial(batch->pMaterial, GetCurrentFrameIndex());

		SUniformBufferBlockModel modelData{};
		modelData.matModel = renderItem.modelMatrix;
		modelData.skinPaletteFirstMatrix = renderItem.skinPaletteFirstMatrix;
		modelData.skinMatrixCount = renderItem.skinMatrixCount;
		modelData.flags = renderItem.flags;
		glCmd->PushConstants(&modelData, sizeof(modelData));
		glCmd->DrawIndexed(batch->indexCount, 1, batch->firstIndex);
	}

	m_vRenderItems.clear();
}

uint32_t COpenGLRenderer::GetCurrentFrameIndex() const
{
	return (m_uiCurrentFrame);
}

bool COpenGLRenderer::InitializeRendererBuffers()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	// Initialize Buffers
	m_vCameraUBO.reserve(MAX_FRAMES_IN_FLIGHT);
	m_vJointsBuffer.reserve(MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		SBufferDesc cameraUBODesc{};
		cameraUBODesc.m_stName = "Camera Uniform Buffer";
		cameraUBODesc.m_eType = EBufferType::BUFFER_TYPE_UNIFORM;
		cameraUBODesc.m_uiSize = sizeof(SUniformBufferBlock);
		cameraUBODesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		cameraUBODesc.m_sBindingSets.bindingSet = EBindingLayoutSetsPoints::BINDING_POINT_SET_FRAME_RESOURCES;
		cameraUBODesc.m_sBindingSets.bindingPoint = static_cast<uint32_t>(EUniformBuffersBindingSets::BINDING_POINT_UBO_CAMERA);
		cameraUBODesc.cpuWrite = true;

		IBuffer* pCameraBuffer = renderDev.CreateBuffer(cameraUBODesc);
		if (!pCameraBuffer)
		{
			syserr("Failed to Create Camera Buffer");
			return (false);
		}

		m_vCameraUBO.push_back(pCameraBuffer);
	}

	return (true);
}

void COpenGLRenderer::UpdateRendererBuffers()
{
	auto& windowMgr = CServiceLocator::Get<CWindowManager>();
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	static auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

	SUniformBufferBlock bufferBlock{};

	bufferBlock.matView = windowMgr.GetCamera()->GetViewMatrix();

	bufferBlock.matProjection = windowMgr.GetCamera()->GetProjectionMatrix();
	// bufferBlock.matProjection[1][1] *= -1.0f;

	bufferBlock.matViewProjection = bufferBlock.matProjection * bufferBlock.matView;

	SUniformBufferBlockModel bufferModelBlock{};
	bufferModelBlock.matModel = Matrix4(1.0f);

	renderDev.UpdateBuffer(m_vCameraUBO[GetCurrentFrameIndex()], &bufferBlock, sizeof(SUniformBufferBlock), 0);
}

void COpenGLRenderer::DestroyRendererBuffers()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	for (auto& cameraUBO : m_vCameraUBO)
	{
		renderDev.DestroyBuffer(cameraUBO);
		AnubisSafeDelete(cameraUBO);
		cameraUBO = nullptr;
	}
	m_vCameraUBO.clear();
}