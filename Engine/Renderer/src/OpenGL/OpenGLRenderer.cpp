#include "OpenGL/OpenGLRenderer.h"
#include "OpenGL/OpenGLCommandList.h"
#include "Services/RenderQueue.h"
#include "Services/ActorsManager.h"
#include "Services/AssimpModelImporter.h"
#include "Device/OpenGLRenderDevice.h"
#include "Logging/LogManager.h"
#include "Window/WindowManager.h"
#include "Camera/Camera.h"
#include "Actor/SkeletalActor.h"

bool COpenGLRenderer::Initialize()
{
	auto& assimpImporter = CServiceLocator::Get<CAssimpModelImporter>();
	auto& actorsMgr = CServiceLocator::Get<CActorsManager>();
	const SActorInfo pInfo = actorsMgr.GetActorInfo("Warrior_Male");
	m_pActor = pInfo.pActor;

	std::shared_ptr<CSkeletalActor> pSkeletalActor = std::dynamic_pointer_cast<CSkeletalActor>(m_pActor);
	pSkeletalActor->GetAnimator()->SetAnimationLibrary(pSkeletalActor->GetSkeletalAsset()->GetAnimations());
	pSkeletalActor->GetAnimator()->PlayAnimation("WarriorMale/OnehandSword/Idle", true);

	if (!InitializeRendererBuffers())
	{
		return (false);
	}

	return (true);
}

void COpenGLRenderer::Destroy()
{
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

	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	std::vector<SRenderInstance> renderInstances{};

	if (m_pActor)
	{
		if (m_pActor->GetAsset()->GetModelAsset())
		{
			m_pActor->BuildRenderItemsNew(renderInstances);
			renderQueue.SubimtRenderItems(renderInstances);
		}
	}
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
		modelData.skinPaletteIndex = renderItem.skinPaletteIndex;
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
		cameraUBODesc.m_eBindingPointOne = EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_CAMERA_UBO; // binding point 0
		cameraUBODesc.cpuWrite = true;

		IBuffer* pCameraBuffer = renderDev.CreateBuffer(cameraUBODesc);
		if (!pCameraBuffer)
		{
			syserr("Failed to Create Camera Buffer");
			return (false);
		}

		m_vCameraUBO.push_back(pCameraBuffer);

		uint32_t initialSize = 1 * sizeof(Matrix4);

		SBufferDesc joinsBufferDesc{};
		joinsBufferDesc.m_stName = "Model Storage Uniform Buffer";
		joinsBufferDesc.m_eType = EBufferType::BUFFER_TYPE_STORAGE;
		joinsBufferDesc.m_uiSize = initialSize; // Initialize as 100 Metrices
		joinsBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		joinsBufferDesc.m_eBindingPointOne = EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_BONES_SSBO;
		joinsBufferDesc.cpuWrite = true;

		IBuffer* pJoinsBuffer = renderDev.CreateBuffer(joinsBufferDesc);
		if (!pJoinsBuffer)
		{
			syserr("Failed to Create Joints Buffer");
			return (false);
		}

		m_vJointsBuffer.push_back(pJoinsBuffer);
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

	// Update Joints Buffer
	std::shared_ptr<CSkeletalActor> pSkeletalActor = std::dynamic_pointer_cast<CSkeletalActor>(m_pActor);
	std::vector<Matrix4> jointsMertices = pSkeletalActor->GetAnimator()->GetFinalBoneMatrices();

	uint64_t jointsBufferSize = jointsMertices.size() * sizeof(Matrix4);
	SBufferDesc joinsBufferDesc{};
	joinsBufferDesc.m_stName = "Model Joints Storage Buffer";
	joinsBufferDesc.m_eType = EBufferType::BUFFER_TYPE_STORAGE;
	joinsBufferDesc.m_uiSize = jointsBufferSize; // Initialize as 100 Metrices
	joinsBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	joinsBufferDesc.cpuWrite = true;

	renderDev.UpdateBuffer(m_vJointsBuffer[GetCurrentFrameIndex()], jointsMertices.data(), jointsBufferSize, 0);
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
	for (auto& jointsSSBO : m_vJointsBuffer)
	{
		renderDev.DestroyBuffer(jointsSSBO);
		AnubisSafeDelete(jointsSSBO);
		jointsSSBO = nullptr;
	}

	m_vCameraUBO.clear();
	m_vJointsBuffer.clear();
}