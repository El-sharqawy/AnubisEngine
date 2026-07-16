#include "Device/OpenGLRenderDevice.h"
#include "Device/PipelinesManager.h"
#include "Logging/LogManager.h"
#include "Window/WindowManager.h"
#include "Textures/TexturesManager.h"
#include "OpenGL/OpenGLBuffer.h"
#include "OpenGL/OpenGLTexture2D.h"
#include "OpenGL/OpenGLMaterial.h"
#include "OpenGL/OpenGLPipeline.h"
#include "OpenGL/OpenGLShader.h"
#include "OpenGL/OpenGLUtils.h"
#include "Camera/Camera.h"
#include "API/ActorData.h"
#include "Model/StaticModel.h"
#include "Model/SkeletalModel.h"
#include "Services/AssimpModelImporter.h"
#include "Services/ActorsManager.h"

#define ENABLE_DEBUG_LOGS

EGraphicsAPI COpenGLRenderDevice::GetAPI() const
{
	return (m_eGraphicsAPI);
}

bool COpenGLRenderDevice::Initialize(GLFWwindow* pPlatformWindowHandle)
{
	m_pGLFWwindow = pPlatformWindowHandle;
	m_gContext.window = pPlatformWindowHandle;

	if (!CreateGLContext(pPlatformWindowHandle))
	{
		return (false);
	}

	if (!LoadExtensions())
	{
		return false;
	}

#if defined(_DEBUG)
	SetupDebugOutput();
#endif

	CTexturesManager::Instance().Initialize();
	CPipelinesManager::Instance().Initialize();

	// Initialize Actors Manager
	CServiceLocator::Get<CActorsManager>().LoadMeshesFromJson(ACTORS_PATH);

	m_pOpenGLCommandList = std::make_unique<COpenGLCommandList>();
	m_pOpenGLRenderer = std::make_unique<COpenGLRenderer>();
	if (!m_pOpenGLRenderer->Initialize())
	{
		return (false);
	}

	return (true);
}

void COpenGLRenderDevice::Shutdown()
{
	CServiceLocator::Get<CActorsManager>().Destroy();
	CPipelinesManager::Instance().Clear();
	CTexturesManager::Instance().Destroy();

	gladLoaderUnloadGL();
}

void COpenGLRenderDevice::BeginFrame()
{
	auto& windowMgr = CServiceLocator::Get<CWindowManager>();
	windowMgr.HandleOsInput();

	m_pOpenGLRenderer->BeginFrame();
}

void COpenGLRenderDevice::EndFrame()
{
	m_pOpenGLRenderer->EndFrame();

	auto& windowMgr = CServiceLocator::Get<CWindowManager>();
	windowMgr.SwapBuffers();
}

void COpenGLRenderDevice::Present()
{
	// Render Stuff
	m_pOpenGLRenderer->Present();

	ICommandList* cmd = GetCommandList();
	m_pOpenGLRenderer->FlushRenderItems(cmd);
}

uint32_t COpenGLRenderDevice::GetCurrentFrameIndex() const
{
	return (m_pOpenGLRenderer->GetCurrentFrameIndex());
}

IBuffer* COpenGLRenderDevice::CreateBuffer(const SBufferDesc& bufferDesc, const void* initialData)
{
	assert(bufferDesc.m_uiSize > 0);
	COpenGLBuffer* pBuffer = AnubisNew(COpenGLBuffer, MEM_TAG_GPU_BUFFER);

	GLenum bufferType = OpenGLUtils::ToGLBufferType(bufferDesc.m_eType);

	uint32_t uiID = 0;
	if (OpenGLUtils::IsGLVersionHigher(4, 5))
	{
		glCreateBuffers(1, &uiID);

		GLbitfield storageFlags = OpenGLUtils::ToGLBufferStorageFlags(bufferDesc.m_eMemoryType);

		glNamedBufferStorage(uiID, static_cast<GLsizeiptr>(bufferDesc.m_uiSize), initialData, storageFlags);
	}
	else
	{
		GLenum usage = OpenGLUtils::ToGLBufferUsage(bufferDesc.m_eMemoryType);

		glGenBuffers(1, &uiID);
		glBindBuffer(bufferType, uiID);
		glBufferData(bufferType, static_cast<GLsizeiptr>(bufferDesc.m_uiSize), initialData, usage);
		glBindBuffer(bufferType, 0);
	}
	
	if (uiID == 0)
	{
		syserr("failed to create buffer '{}'", bufferDesc.m_stName);
		delete pBuffer;
		return (nullptr);
	}

	pBuffer->UpdateBufferData(bufferDesc, uiID); // no binding point

	GLenum bindingPoint = OpenGLUtils::ToGLBufferBindingPoint(pBuffer->GetBindingPoint());

	if (pBuffer->IsBoundToBase())
	{
		glBindBufferBase(bufferType, bindingPoint, pBuffer->GetBufferID()); // Bind no buffer to this binding point
	}

#if defined(ENABLE_DEBUG_LOGS)
	glObjectLabel(GL_BUFFER, uiID, -1, bufferDesc.m_stName.c_str());
	syslog("Buffer '{}' created ({} bytes)", bufferDesc.m_stName, bufferDesc.m_uiSize);
#endif

	return (pBuffer);
}

ITexture2D* COpenGLRenderDevice::CreateTexture2D(const STextureDesc& textureDesc, const void* pixels)
{
	COpenGLTexture2D* pTexture = AnubisNew(COpenGLTexture2D, MEM_TAG_TEXTURE);

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

		AnubisSafeDelete(pTexture);
		return nullptr;
	}

	GLuint uiTextureID = 0;

	GLenum textureType = OpenGLUtils::ToGLTextureType(textureDesc.m_eType);

	if (OpenGLUtils::IsGLVersionHigher(4, 5))
	{
		glCreateTextures(textureType, 1, &uiTextureID);
	}
	else
	{
		glGenTextures(1, &uiTextureID);
		glBindTexture(textureType, uiTextureID);
	}

	// Upload Data
	// Calculate Mip Levels
	GLint iLevels = 1;
	if (textureDesc.m_bGenMipmaps && iWidth > 0)
	{
		iLevels = static_cast<GLint>(std::floor(std::log2(std::max(iWidth, iHeight)))) + 1;
	}

	// Determine Formats and parameters
	GLenum internalForamat = OpenGLUtils::ToGLTexureInternalFormat(textureDesc.m_eFormat);
	GLenum pixelFormat = OpenGLUtils::ToGLTexurePixelFormat(textureDesc.m_eFormat);
	GLenum pixelType = OpenGLUtils::ToGLTexturePixelType(textureDesc.m_eFormat);
	GLenum minFilter = OpenGLUtils::ToGLTextureMinFilter(textureDesc.m_eMinFilter, textureDesc.m_eMipmapMode);
	GLenum magFilter = OpenGLUtils::ToGLTextureMagFilter(textureDesc.m_eMagFilter);
	GLenum wrapS = OpenGLUtils::ToGlTextureWrap(textureDesc.m_eWrapU);
	GLenum wrapT = OpenGLUtils::ToGlTextureWrap(textureDesc.m_eWrapV);
	const bool bSingleChannelFormat = textureDesc.m_eFormat == ETextureFormats::TEXTIRE_FORMAT_R8_UNORM;

	if (OpenGLUtils::IsGLVersionHigher(4, 5))
	{
		// Reset Swizzle to standard
		GLint defaultSwizzle[] = { GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA };
		glTextureParameteriv(uiTextureID, GL_TEXTURE_SWIZZLE_RGBA, defaultSwizzle);

		// Allocate Immutable Storage
		glTextureStorage2D(uiTextureID, iLevels, internalForamat, iWidth, iHeight);

		// Upload Data
		if (pImageData)
		{
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTextureSubImage2D(uiTextureID, 0, 0, 0, iWidth, iHeight, pixelFormat, pixelType, pImageData);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		}

		// Handle Grayscale Swizzle
		if (bSingleChannelFormat)
		{
			GLint greySwizzle[] = { GL_RED, GL_RED, GL_RED, GL_ONE };
			glTextureParameteriv(uiTextureID, GL_TEXTURE_SWIZZLE_RGBA, greySwizzle);
		}

		glTextureParameteri(uiTextureID, GL_TEXTURE_MIN_FILTER, minFilter);
		glTextureParameteri(uiTextureID, GL_TEXTURE_MAG_FILTER, magFilter);
		glTextureParameteri(uiTextureID, GL_TEXTURE_WRAP_S, wrapS);
		glTextureParameteri(uiTextureID, GL_TEXTURE_WRAP_T, wrapT);

		if (iLevels > 1 && textureDesc.m_bGenMipmaps)
		{
			glGenerateTextureMipmap(uiTextureID);
		}

	}
	else
	{
		// Bind the Texture
		glBindTexture(textureType, uiTextureID);

		// Calculate Mip Levels (In Legacy, we don't pre-allocate levels, but we need to know if we want them)
		bool bUseMips = textureDesc.m_bGenMipmaps && iWidth > 1;

		if (pImageData)
		{
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexImage2D(textureType, 0, internalForamat, iWidth, iHeight, 0, pixelFormat, pixelType, pImageData);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 4); // Back to normal
		}

		// Set Parameters (Using target-based functions)
		glTexParameteri(textureType, GL_TEXTURE_MIN_FILTER, minFilter);
		glTexParameteri(textureType, GL_TEXTURE_MAG_FILTER, magFilter);
		glTexParameteri(textureType, GL_TEXTURE_WRAP_S, wrapS);
		glTexParameteri(textureType, GL_TEXTURE_WRAP_T, wrapT);

		// Handle Swizzle (If 1-channel)
		if (bSingleChannelFormat)
		{
			GLint greySwizzle[] = { GL_RED, GL_RED, GL_RED, GL_ONE };
			glTexParameteriv(textureType, GL_TEXTURE_SWIZZLE_RGBA, greySwizzle);
		}

		// 7. Generate Mipmaps if needed
		if (bUseMips && pImageData)
		{
			glGenerateMipmap(textureType);
		}

		// 8. Unbind to avoid accidental state changes elsewhere
		glBindTexture(textureType, 0);
	}

	// Free pixels data since we have it in our buffer
	if (bNeedsFree)
	{
		stbi_image_free(const_cast<void*>(pImageData));
	}

	STextureData textureData{};
	textureData.iWidth = iWidth;
	textureData.iHeight = iHeight;
	textureData.iChannels = iChannels;
	textureData.uiTextureID = uiTextureID;

	pTexture->UpdateTextureData(textureDesc, textureData);

	return (pTexture);
}

IShaderProgram* COpenGLRenderDevice::CreateShaderProgram(const SShaderDesc& shaderDesc)
{
	// Check for Shader
	COpenGLShader* pShader = AnubisNew(COpenGLShader, MEM_TAG_RENDERING);
	if (!pShader->Create(shaderDesc))
	{
		syserr("Failed to Create Shader {}", shaderDesc.m_stName);
		return (nullptr);
	}

	return pShader;
}

IPipeline* COpenGLRenderDevice::CreatePipeline(const SPipelineDesc& pipelineDesc)
{
	COpenGLPipeline* pPipeline = AnubisNew(COpenGLPipeline, MEM_TAG_RENDERING);
	if (!pPipeline->Initialize(pipelineDesc))
	{
		syserr("Failed to Create Pipeline!");
		return (nullptr);
	}

	return (pPipeline);
}

IMaterial* COpenGLRenderDevice::CreateMaterial()
{
	COpenGLMaterial* pMaterial = AnubisNew(COpenGLMaterial, MEM_TAG_TEXTURE);
	if (!pMaterial)
	{
		syserr("CreateMaterial: allocation failed");
		return nullptr;
	}

	return pMaterial;
}

CVertexArray* COpenGLRenderDevice::CreateVertexArray(const SVertexArrayDesc& vertexArrayDesc)
{
	ANUBIS_ASSERT(!vertexArrayDesc.m_vBindings.empty());
	ANUBIS_ASSERT(!vertexArrayDesc.m_vAttribs.empty());

	CVertexArray* pVertexArray = AnubisNew(CVertexArray, MEM_TAG_GPU_BUFFER);

	GLuint uiVaoID = 0;
	if (OpenGLUtils::IsGLVersionHigher(4, 5))
	{
		glCreateVertexArrays(1, &uiVaoID);

		for (const auto& binding : vertexArrayDesc.m_vBindings)
		{
			ANUBIS_ASSERT(binding.m_pBuffer && binding.m_pBuffer->IsValid());
			ANUBIS_ASSERT(binding.m_iStride > 0);

			COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(binding.m_pBuffer);

			glVertexArrayVertexBuffer(
				uiVaoID,
				binding.m_uiBinding,
				glBuffer->GetBufferID(),
				binding.m_uiOffset,
				binding.m_iStride
			);

			if (binding.m_uiDivisor > 0)
			{
				glVertexArrayBindingDivisor(uiVaoID, binding.m_uiBinding, binding.m_uiDivisor);
			}
		}
		if (vertexArrayDesc.m_pIndexBuffer && vertexArrayDesc.m_pIndexBuffer->IsValid())
		{
			COpenGLBuffer* glIndexBuffer = dynamic_cast<COpenGLBuffer*>(vertexArrayDesc.m_pIndexBuffer);

			glVertexArrayElementBuffer(uiVaoID, glIndexBuffer->GetBufferID());
		}

		for (const auto& attrib : vertexArrayDesc.m_vAttribs)
		{
			glEnableVertexArrayAttrib(uiVaoID, attrib.m_uiLocation);

			switch (attrib.m_eClass)
			{
			case EVertexAttribClass::Float:
				glVertexArrayAttribFormat(
					uiVaoID, attrib.m_uiLocation, attrib.m_iCount,
					attrib.m_eDataType, attrib.m_bNormalize, attrib.m_uiOffset
				);
				break;

			case EVertexAttribClass::Integer:
				glVertexArrayAttribIFormat(
					uiVaoID, attrib.m_uiLocation, attrib.m_iCount,
					attrib.m_eDataType, attrib.m_uiOffset
				);
				break;

			case EVertexAttribClass::Double:
				glVertexArrayAttribLFormat(
					uiVaoID, attrib.m_uiLocation, attrib.m_iCount,
					attrib.m_eDataType, attrib.m_uiOffset
				);
				break;
			}

			glVertexArrayAttribBinding(uiVaoID, attrib.m_uiLocation, attrib.m_uiBinding);
		}
	}
	else
	{
		glGenVertexArrays(1, &uiVaoID);
		glBindVertexArray(uiVaoID);

		for (const auto& binding : vertexArrayDesc.m_vBindings)
		{
			ANUBIS_ASSERT(binding.m_pBuffer && binding.m_pBuffer->IsValid());
			ANUBIS_ASSERT(binding.m_iStride > 0);

			COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(binding.m_pBuffer);

			glBindBuffer(GL_ARRAY_BUFFER, glBuffer->GetBufferID());

			for (const auto& attrib : vertexArrayDesc.m_vAttribs)
			{
				if (attrib.m_uiBinding != binding.m_uiBinding)
				{
					continue;
				}

				const void* pOff = reinterpret_cast<const void*>(static_cast<uintptr_t>(binding.m_uiOffset + attrib.m_uiOffset));

				glEnableVertexAttribArray(attrib.m_uiLocation);

				switch (attrib.m_eClass)
				{
				case EVertexAttribClass::Float:
					glVertexAttribPointer(
						attrib.m_uiLocation, attrib.m_iCount, attrib.m_eDataType,
						attrib.m_bNormalize, binding.m_iStride, pOff
					);
					break;

				case EVertexAttribClass::Integer:
					glVertexAttribIPointer(
						attrib.m_uiLocation, attrib.m_iCount, attrib.m_eDataType,
						binding.m_iStride, pOff
					);
					break;

				case EVertexAttribClass::Double:
					glVertexAttribLPointer(
						attrib.m_uiLocation, attrib.m_iCount, attrib.m_eDataType,
						binding.m_iStride, pOff
					);
					break;
				}

				if (attrib.m_uiDivisor > 0)
				{
					glVertexAttribDivisor(attrib.m_uiLocation, attrib.m_uiDivisor);
				}
			}

			if (vertexArrayDesc.m_pIndexBuffer && vertexArrayDesc.m_pIndexBuffer->IsValid())
			{
				COpenGLBuffer* glIndexBuffer = dynamic_cast<COpenGLBuffer*>(vertexArrayDesc.m_pIndexBuffer);

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glIndexBuffer->GetBufferID());
			}
		}
	}

	if (uiVaoID == 0)
	{
		syserr("failed to create VertexArray '{}'", vertexArrayDesc.m_stName);
		return (nullptr);
	}

	// Populate — derive counts from buffer sizes
	SVertexArrayData vertexArrayData{};
	vertexArrayData.m_stName = vertexArrayDesc.m_stName;
	vertexArrayData.m_uiVaoID = uiVaoID;
	if (!vertexArrayDesc.m_vBindings.empty())
	{
		const auto& firstBinding = vertexArrayDesc.m_vBindings[0];
		if (firstBinding.m_pBuffer && firstBinding.m_pBuffer->IsValid() && firstBinding.m_iStride > 0)
		{
			vertexArrayData.m_uiVertexCount = firstBinding.m_pBuffer->GetSize() / firstBinding.m_iStride;
		}
	}
	vertexArrayData.m_eIndexType = vertexArrayDesc.m_eIndexType;
	vertexArrayData.m_bHasIndexBuffer = vertexArrayDesc.m_pIndexBuffer && vertexArrayDesc.m_pIndexBuffer->IsValid();

	pVertexArray->UpdateVertexArrayData(vertexArrayData);

	return (pVertexArray);
}

void COpenGLRenderDevice::DestroyBuffer(IBuffer* pBuffer)
{
	ANUBIS_ASSERT(pBuffer != nullptr);
	if (!pBuffer->IsValid())
	{
		return;
	}

	COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(pBuffer);

	GLenum bufferType = OpenGLUtils::ToGLBufferType(pBuffer->GetType());
	GLenum bindingPoint = OpenGLUtils::ToGLBufferBindingPoint(glBuffer->GetBindingPoint());

	if (glBuffer->IsBoundToBase())
	{
		glBindBufferBase(bufferType, bindingPoint, 0); // Bind no buffer to this binding point
	}

	uint32_t uiBufferID = glBuffer->GetBufferID();
	glDeleteBuffers(1, &uiBufferID);

	const std::string& stName = pBuffer->GetName();
	glBuffer->Clear();
	
#if defined(ENABLE_DEBUG_LOGS)
	syslog("Buffer '{}' destroyed", stName);  // save name before clear
#endif
}

void COpenGLRenderDevice::DestroyTexture2D(ITexture2D* pTexture)
{
	ANUBIS_ASSERT(pTexture != nullptr);

	COpenGLTexture2D* glTexture = dynamic_cast<COpenGLTexture2D*>(pTexture);

	uint32_t uiTextureID = glTexture->GetTextureID();

	glDeleteTextures(1, &uiTextureID);

	glTexture->Clear();
}

void COpenGLRenderDevice::DestroyShaderProgram(IShaderProgram* pShaderProgram)
{
	if (!pShaderProgram)
	{
		return;
	}

	COpenGLShader* glShaderProgram = dynamic_cast<COpenGLShader*>(pShaderProgram);
	uint32_t uiProgramID = glShaderProgram->GetProgramID();
	if (uiProgramID)
	{
		glDeleteProgram(uiProgramID);
		uiProgramID = 0;
	}

	glShaderProgram->Clear();
}

void COpenGLRenderDevice::DestroyPipeline(IPipeline* pPipeline)
{
	// Safety Check
	if (!pPipeline)
	{
		return;
	}

	COpenGLPipeline* glPipeline = dynamic_cast<COpenGLPipeline*>(pPipeline);
	if (!glPipeline)
	{
		assert(false && "Passed a non-OpenGL Pipeline to COpenGLPipeline!");
		return;
	}

	glPipeline->Destroy();
}

void COpenGLRenderDevice::DestroyMaterial(IMaterial* pMaterial)
{
	// Safety Check
	if (!pMaterial)
	{
		return;
	}

	COpenGLMaterial* glMaterial = dynamic_cast<COpenGLMaterial*>(pMaterial);
	if (!glMaterial)
	{
		assert(false && "Passed a non-OpenGL Mateiral to COpenGLMaterial!");
		return;
	}

	glMaterial->ClearMaterial();
}

void COpenGLRenderDevice::DestroyVertexArray(CVertexArray* pVertexArray)
{
#if defined(ENABLE_DEBUG_LOGS)
	const std::string stName = pVertexArray->GetName();
	uint32_t uiVAOID = pVertexArray->GetID();
#endif

	glDeleteVertexArrays(1, &uiVAOID);

	SVertexArrayData vertexArrayData{};
	vertexArrayData.m_stName = "";
	vertexArrayData.m_uiVaoID = 0;
	vertexArrayData.m_uiVertexCount = 0;
	vertexArrayData.m_eIndexType = GL_UNSIGNED_INT;
	vertexArrayData.m_bHasIndexBuffer = false;

	pVertexArray->UpdateVertexArrayData(vertexArrayData);

#if defined(ENABLE_DEBUG_LOGS)
	syslog("VAO '{}' : {} destroyed", stName, uiVAOID);
#endif

}

bool COpenGLRenderDevice::UpdateBuffer(IBuffer* pBuffer, const void* pData, size_t size, size_t dstOffset)
{
	ANUBIS_ASSERT(pBuffer != nullptr);
	ANUBIS_ASSERT(pData != nullptr);
	ANUBIS_ASSERT(size > 0);

	if (!pBuffer->IsValid())
	{
		syswarn("Buffer: '{}' is not valid — skipping", pBuffer->GetName());
		return (false);
	}

	COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(pBuffer);
	if (!glBuffer)
	{
		assert(false && "Passed a non-OpenGL Buffer to COpenGLBuffer!");
		return (false);
	}

	const GLuint oldID = glBuffer->GetBufferID();
	const size_t oldSize = glBuffer->GetSize();

	size_t uiRequired = size + dstOffset;
	size_t uiNewSize = std::max(oldSize * 2, uiRequired);

	// Clamp the growth to the maximum allowed
	if (uiNewSize > MAX_MAIN_VRAM_BYTES)
	{
		uiNewSize = MAX_MAIN_VRAM_BYTES;
	}

	const GLenum bufferType = OpenGLUtils::ToGLBufferType(pBuffer->GetType());

	// ---- Reallocation path ----
	if (uiRequired > glBuffer->GetSize())
	{
		syswarn("{} write range [{} -> {}] exceeds buffer size {} — reallocation", glBuffer->GetName(), dstOffset, uiRequired, glBuffer->GetSize());

		GLuint newID = 0;
		// Recreate with new size
		if (OpenGLUtils::IsGLVersionHigher(4, 5))
		{
			const GLbitfield storageFlags = OpenGLUtils::ToGLBufferStorageFlags(pBuffer->GetMemoryType());

			glCreateBuffers(1, &newID);
			glNamedBufferStorage(newID, static_cast<GLsizeiptr>(uiNewSize), nullptr, storageFlags);

			if (oldID && oldSize > 0)
			{
				glCopyNamedBufferSubData(oldID, newID, 0, 0, static_cast<GLsizeiptr>(oldSize));
			}
		}
		else
		{
			const GLenum storageFlags = OpenGLUtils::ToGLBufferUsage(pBuffer->GetMemoryType());

			glGenBuffers(1, &newID);
			glBindBuffer(bufferType, newID);
			glBufferData(bufferType, static_cast<GLsizeiptr>(uiNewSize), nullptr, storageFlags);

			if (oldID && oldSize > 0)
			{
				glBindBuffer(GL_COPY_READ_BUFFER, oldID);
				glBindBuffer(GL_COPY_WRITE_BUFFER, newID);

				glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, static_cast<GLsizeiptr>(oldSize));

				glBindBuffer(GL_COPY_READ_BUFFER, 0);
				glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
			}

			glBindBuffer(bufferType, 0);
		}

		glDeleteBuffers(1, &oldID);
		glBuffer->UpdateBufferID(newID);
		glBuffer->UpdateBufferSize(uiNewSize);

		// Re-bind to base point if it was a UBO/SSBO
		GLenum eBingingPoint = OpenGLUtils::ToGLBufferBindingPoint(glBuffer->GetBindingPoint());
		if (glBuffer->IsBoundToBase())
		{
			glBindBufferBase(bufferType, eBingingPoint, glBuffer->GetBufferID());
			syslog("Buffer: {} is bound to point {}", glBuffer->GetName(), static_cast<uint32_t>(glBuffer->GetBindingPoint()));
		}
	}
	else
	{
		if (OpenGLUtils::IsGLVersionHigher(4, 5))
		{
			// DSA — glNamedBufferStorage is immutable, only SubData is allowed
			// Requires GL_DYNAMIC_STORAGE_BIT set at creation
			glNamedBufferSubData(glBuffer->GetBufferID(), static_cast<GLintptr>(dstOffset), static_cast<GLsizeiptr>(size), pData);
		}
		else
		{
			const GLenum bufferType = OpenGLUtils::ToGLBufferType(pBuffer->GetType());

			glBindBuffer(bufferType, glBuffer->GetBufferID());
			glBufferSubData(bufferType, static_cast<GLintptr>(dstOffset), static_cast<GLsizeiptr>(size), pData);
			glBindBuffer(bufferType, 0);
		}

	}

	return (true);
}

ICommandList* COpenGLRenderDevice::GetCommandList()
{
	return (m_pOpenGLCommandList.get());
}

void COpenGLRenderDevice::Resize(int32_t width, int32_t height)
{
	glViewport(0, 0, width, height);
	glScissor(0, 0, width, height);
}

bool COpenGLRenderDevice::CreateGLContext(GLFWwindow* pPlatformWindowHandle)
{
	glfwMakeContextCurrent(pPlatformWindowHandle);
	return (true);
}

bool COpenGLRenderDevice::LoadExtensions()
{
	// Glad belongs here — it's completing the context setup, not rendering
	if (gladLoaderLoadGL() == false)
	{
		syserr("Failed to Initialize GLAD Library");
		return (false);
	}

	// Print OpenGL Version and GPU used
	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);
	const GLubyte* vendor = glGetString(GL_VENDOR);
	const GLubyte* renderer = glGetString(GL_RENDERER);
	syslog("GPU: {}", reinterpret_cast<const char*>(renderer));
	syslog("GL version : {}.{}", major, minor);

	// Match Vulkan matrix, depth [0 .. 1]
	glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
	glEnable(GL_FRAMEBUFFER_SRGB);

	return (true);
}

bool COpenGLRenderDevice::CreateFrameResources()
{
	return false;
}

void COpenGLRenderDevice::SetupDebugOutput()
{
	glDebugMessageCallback(MyDebugCallback, nullptr);
}

void APIENTRY COpenGLRenderDevice::MyDebugCallback(GLenum source, GLenum type, GLuint id,
	GLenum severity, GLsizei length,
	const GLchar* message, const void* userParam)
{
	// Suppress low-level informational warnings (GL_DEBUG_SEVERITY_NOTIFICATION)
	if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
	{
		return; // Exit the function without printing the log
	}

	if (type == GL_DEBUG_TYPE_ERROR)
	{
		// This will trigger a breakpoint in Visual Studio only when an error occurs
		__debugbreak();
	}

	syslog("{}", message);
}
