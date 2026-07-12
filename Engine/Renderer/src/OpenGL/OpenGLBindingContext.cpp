#include "OpenGL/OpenGLBindingContext.h"
#include "OpenGL/OpenGLBuffer.h"
#include "OpenGL/OpenGLTexture2D.h"
#include "Logging/LogManager.h"

bool COpenGLBindingContext::Initialize(const SBindingContextDesc& desc)
{
    Destroy();

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

    m_uiFrameCount = desc.m_uiFrameCount;

    for (const SBindingDesc& binding : desc.m_vBindings)
    {
        if (binding.m_uiArrayCount == 0)
        {
            syserr("binding {} has arrayCount = 0", binding.m_uiBinding);
            Destroy();
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
            Destroy();
            return false;
        }

        m_vBindings.push_back(binding);
    }

    for (const SBindingBufferResource& bufferRes : desc.m_vBufferResources)
    {
        const SBindingDesc* pBindingDesc = FindBindingDesc(m_vBindings, bufferRes.m_uiBinding);
        if (!pBindingDesc)
        {
            syserr("buffer resource references unknown binding {}", bufferRes.m_uiBinding);
            Destroy();
            return false;
        }

        if (pBindingDesc->m_eType != EBindingType::BIND_TYPE_UNIFORM_BUFFER &&
            pBindingDesc->m_eType != EBindingType::BIND_TYPE_STORAGE_BUFFER)
        {
            syserr("buffer resource at binding {} does not match buffer binding type", bufferRes.m_uiBinding);
            Destroy();
            return false;
        }

        if (bufferRes.m_vBuffers.size() < desc.m_uiFrameCount)
        {
            syserr("buffer binding {} has only {} buffers, expected {}",
                bufferRes.m_uiBinding,
                static_cast<uint32_t>(bufferRes.m_vBuffers.size()),
                desc.m_uiFrameCount);
            Destroy();
            return false;
        }

        for (uint32_t i = 0; i < desc.m_uiFrameCount; ++i)
        {
            IBuffer* pBuffer = bufferRes.m_vBuffers[i];
            if (pBuffer == nullptr)
            {
                syserr("null buffer at binding {} frame {}", bufferRes.m_uiBinding, i);
                Destroy();
                return false;
            }

            COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(pBuffer);
            if (!glBuffer)
            {
                syserr("buffer at binding {} frame {} is not an OpenGL buffer", bufferRes.m_uiBinding, i);
                Destroy();
                return false;
            }

            if (!glBuffer->IsValid() || glBuffer->GetBufferID() == 0)
            {
                syserr("buffer at binding {} frame {} is invalid", bufferRes.m_uiBinding, i);
                Destroy();
                return false;
            }
        }

        m_vBufferResources.push_back(bufferRes);
    }

    for (const SBindingImageResource& imageRes : desc.m_vImageResources)
    {
        const SBindingDesc* pBindingDesc = FindBindingDesc(m_vBindings, imageRes.m_uiBinding);
        if (!pBindingDesc)
        {
            syserr("image resource references unknown binding {}", imageRes.m_uiBinding);
            Destroy();
            return false;
        }

        if (pBindingDesc->m_eType != EBindingType::BIND_TYPE_SAMPLER &&
            pBindingDesc->m_eType != EBindingType::BIND_TYPE_SAMPLED_IMAGE &&
            pBindingDesc->m_eType != EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER)
        {
            syserr("image resource at binding {} does not match image binding type", imageRes.m_uiBinding);
            Destroy();
            return false;
        }

        if (imageRes.m_pTexture == nullptr)
        {
            syserr("null texture at binding {}", imageRes.m_uiBinding);
            Destroy();
            return false;
        }

        COpenGLTexture2D* glTexture = dynamic_cast<COpenGLTexture2D*>(imageRes.m_pTexture);
        if (!glTexture)
        {
            syserr("texture at binding {} is not an OpenGL texture", imageRes.m_uiBinding);
            Destroy();
            return false;
        }

        if (!glTexture->IsValid() || glTexture->GetTextureID() == 0)
        {
            syserr("texture at binding {} is invalid", imageRes.m_uiBinding);
            Destroy();
            return false;
        }

        m_vImageResources.push_back(imageRes);
    }

    m_bInitialized = true;
    return true;
}

void COpenGLBindingContext::Destroy()
{
    m_bInitialized = false;
    m_uiFrameCount = 0;
    m_vBindings.clear();
    m_vBufferResources.clear();
    m_vImageResources.clear();
}

bool COpenGLBindingContext::Bind(uint32_t frameIndex, uint32_t programID)
{
    (void)programID;

    if (!m_bInitialized)
    {
        syserr("binding context is not initialized");
        return false;
    }

    if (frameIndex >= m_uiFrameCount)
    {
        syserr("invalid frame index {} (frame count {})", frameIndex, m_uiFrameCount);
        return false;
    }

    for (const SBindingBufferResource& bufferRes : m_vBufferResources)
    {
        if (frameIndex >= bufferRes.m_vBuffers.size())
        {
            syserr("frame index {} out of range for buffer binding {}", frameIndex, bufferRes.m_uiBinding);
            return false;
        }

        IBuffer* pBuffer = bufferRes.m_vBuffers[frameIndex];
        if (!pBuffer)
        {
            syserr("null buffer at binding {} frame {}", bufferRes.m_uiBinding, frameIndex);
            return false;
        }

        COpenGLBuffer* glBuffer = dynamic_cast<COpenGLBuffer*>(pBuffer);
        if (!glBuffer)
        {
            syserr("buffer at binding {} frame {} is not an OpenGL buffer", bufferRes.m_uiBinding, frameIndex);
            return false;
        }

        if (!glBuffer->IsValid() || glBuffer->GetBufferID() == 0)
        {
            syserr("buffer at binding {} frame {} is invalid", bufferRes.m_uiBinding, frameIndex);
            return false;
        }

        const SBindingDesc* pBindingDesc = FindBindingDesc(m_vBindings, bufferRes.m_uiBinding);
        if (!pBindingDesc)
        {
            syserr("missing binding description for buffer binding {}", bufferRes.m_uiBinding);
            return false;
        }

        const GLuint buffer = glBuffer->GetBufferID();
        const GLintptr offset = static_cast<GLintptr>(bufferRes.m_uiOffset);
        const GLsizeiptr range = static_cast<GLsizeiptr>(
            bufferRes.m_uiRange == VK_WHOLE_SIZE ? glBuffer->GetSize() : bufferRes.m_uiRange);

        if (range <= 0)
        {
            syserr("invalid range for buffer binding {}", bufferRes.m_uiBinding);
            return false;
        }

        switch (pBindingDesc->m_eType)
        {
        case EBindingType::BIND_TYPE_UNIFORM_BUFFER:
            glBindBufferRange(GL_UNIFORM_BUFFER, bufferRes.m_uiBinding, buffer, offset, range);
            break;

        case EBindingType::BIND_TYPE_STORAGE_BUFFER:
            glBindBufferRange(GL_SHADER_STORAGE_BUFFER, bufferRes.m_uiBinding, buffer, offset, range);
            break;

        default:
            syserr("invalid buffer binding type at binding {}", bufferRes.m_uiBinding);
            return false;
        }
    }

    for (const SBindingImageResource& imageRes : m_vImageResources)
    {
        if (!imageRes.m_pTexture)
        {
            syserr("null texture at binding {}", imageRes.m_uiBinding);
            return false;
        }

        COpenGLTexture2D* glTexture = dynamic_cast<COpenGLTexture2D*>(imageRes.m_pTexture);
        if (!glTexture)
        {
            syserr("texture at binding {} is not an OpenGL texture", imageRes.m_uiBinding);
            return false;
        }

        if (!glTexture->IsValid() || glTexture->GetTextureID() == 0)
        {
            syserr("texture at binding {} is invalid", imageRes.m_uiBinding);
            return false;
        }

        const GLuint unit = imageRes.m_uiBinding;
        glBindTextureUnit(unit, glTexture->GetTextureID());
    }

    return true;
}
