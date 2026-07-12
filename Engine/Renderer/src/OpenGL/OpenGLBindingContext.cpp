#include "OpenGL/OpenGLBindingContext.h"
#include "OpenGL/OpenGLBuffer.h"
#include "OpenGL/OpenGLTexture2D.h"
#include "Logging/LogManager.h"

bool COpenGLBindingContext::Initialize(const SBindingContextDesc& desc)
{
    if (desc.m_uiFrameCount == 0)
    {
        syserr("frame count is 0");
        return false;
    }

    if (desc.m_vBindings.empty())
    {
        syserr("no bindings");
        return false;
    }


    for (const SBindingDesc& binding : desc.m_vBindings)
    {
        if (binding.m_uiArrayCount == 0)
        {
            syserr("binding {} has arrayCount = 0", binding.m_uiBinding);
            return false;
        }

        switch (binding.m_eType)
        {
        case EBindingType::BIND_TYPE_UNIFORM_BUFFER:
        case EBindingType::BIND_TYPE_STORAGE_BUFFER:
        case EBindingType::BIND_TYPE_SAMPLER:
        case EBindingType::BIND_TYPE_SAMPLED_IMAGE:
        case EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER:
            break;

        default:
            syserr("unsupported binding type at binding {}", binding.m_uiBinding);
            return false;
        }
    }

    for (const SBindingBufferResource& bufferRes : desc.m_vBufferResources)
    {
        if (bufferRes.m_vBuffers.size() < desc.m_uiFrameCount)
        {
            syserr("buffer binding {} has only %{} buffers, expected {}", bufferRes.m_uiBinding, bufferRes.m_vBuffers.size(), desc.m_uiFrameCount);
            return false;
        }

        for (uint32_t i = 0; i < desc.m_uiFrameCount; ++i)
        {
            if (bufferRes.m_vBuffers[i] == nullptr)
            {
                syserr("null buffer at binding {} frame {}", bufferRes.m_uiBinding, i);
                return false;
            }
        }
    }

    for (const SBindingImageResource& imageRes : desc.m_vImageResources)
    {
        if (imageRes.m_pTexture == nullptr)
        {
            syserr("null texture at binding {}", imageRes.m_uiBinding);
            return false;
        }
    }

    m_gDesc = desc;
    m_bInitialized = true;
    return (true);
}

void COpenGLBindingContext::Destroy()
{
    m_gDesc = {};
}

bool COpenGLBindingContext::Bind(uint32_t frameIndex, uint32_t programID)
{
    if (!m_bInitialized)
    {
        return false;
    }

    if (frameIndex >= m_gDesc.m_uiFrameCount)
    {
        return false;
    }

    for (const SBindingBufferResource& bufferRes : m_gDesc.m_vBufferResources)
    {
        IBuffer* pBuffer = bufferRes.m_vBuffers[frameIndex];
        COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(pBuffer);
        if (!glBuffer)
        {
            return false;
        }

        const uint32_t buffer = glBuffer->GetBufferID();
        const SBindingDesc* pBindingDesc = nullptr;
        for (const auto& binding : m_gDesc.m_vBindings)
        {
            if (binding.m_uiBinding == bufferRes.m_uiBinding)
            {
                pBindingDesc = &binding;
                break;
            }
        }

        if (!pBindingDesc)
        {
            return false;
        }

        switch (pBindingDesc->m_eType)
        {
        case EBindingType::BIND_TYPE_UNIFORM_BUFFER:
            glBindBufferRange(GL_UNIFORM_BUFFER,
                bufferRes.m_uiBinding,
                buffer,
                static_cast<GLintptr>(bufferRes.m_uiOffset),
                static_cast<GLsizeiptr>(bufferRes.m_uiRange == VK_WHOLE_SIZE ? glBuffer->GetSize() : bufferRes.m_uiRange));
            break;

        case EBindingType::BIND_TYPE_STORAGE_BUFFER:
            glBindBufferRange(GL_SHADER_STORAGE_BUFFER,
                bufferRes.m_uiBinding,
                buffer,
                static_cast<GLintptr>(bufferRes.m_uiOffset),
                static_cast<GLsizeiptr>(bufferRes.m_uiRange == VK_WHOLE_SIZE ? glBuffer->GetSize() : bufferRes.m_uiRange));
            break;

        default:
            syserr("invalid buffer binding type at binding %u", bufferRes.m_uiBinding);
            return false;

        }
    }

    for (const SBindingImageResource& imageRes : m_gDesc.m_vImageResources)
    {
        COpenGLTexture2D* glTexture = dynamic_cast<COpenGLTexture2D*>(imageRes.m_pTexture);
        if (!glTexture)
        {
            return false;
        }

        GLuint unit = imageRes.m_uiBinding;
        glBindTextureUnit(unit, glTexture->GetTextureID());

        // If we support separate sampler objects:
        // glBindSampler(unit, glTexture->GetSamplerID());
    }

    return (true);
}
