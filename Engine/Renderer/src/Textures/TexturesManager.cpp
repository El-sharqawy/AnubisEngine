#include "Textures/TexturesManager.h"
#include "Device/VulkanRenderDevice.h"

bool CTexturesManager::Initialize()
{
    uint8_t white[4] = { 255, 255, 255, 255 };
    uint8_t black[4] = { 0,   0,   0,   255 };
    uint8_t normal[4] = { 128, 128, 255, 255 }; // flat normal in tangent space


    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    STextureDesc textureDesc{};
    textureDesc.m_stName = "Fallback Texture";
    textureDesc.m_iWidth = 1;
    textureDesc.m_iHeight = 1;
    textureDesc.m_iChannels = 4;
    textureDesc.m_eType = ETextureType::TEXTURE_TYPE_2D;
    textureDesc.m_eFormat = ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB;
    textureDesc.m_eMagFilter = ETextureFilter::TEXTURE_FILTER_NEAREST;
    textureDesc.m_eMinFilter = ETextureFilter::TEXTURE_FILTER_NEAREST;
    textureDesc.m_eWrapU = ETextureWrap::TEXTURE_WRAP_REPEAT;
    textureDesc.m_eWrapV = ETextureWrap::TEXTURE_WRAP_REPEAT;
    textureDesc.m_eWrapW = ETextureWrap::TEXTURE_WRAP_REPEAT;

    m_pFallbackWhite = renderDev.CreateTexture2D(textureDesc, white);
    m_pFallbackBlack = renderDev.CreateTexture2D(textureDesc, black);
    m_pFallbackNormal = renderDev.CreateTexture2D(textureDesc, normal);

    if (!m_pFallbackWhite || !m_pFallbackBlack || !m_pFallbackNormal)
    {
        syserr("Failed to Initialize Default fallback Textures");
        return (false);
    }

    return (true);
}

void CTexturesManager::Destroy()
{
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    renderDev.DestroyTexture2D(m_pFallbackWhite);
    renderDev.DestroyTexture2D(m_pFallbackBlack);
    renderDev.DestroyTexture2D(m_pFallbackNormal);

    AnubisSafeDelete(m_pFallbackWhite);
    AnubisSafeDelete(m_pFallbackBlack);
    AnubisSafeDelete(m_pFallbackNormal);
}

ITexture2D* CTexturesManager::GetFallBackWhiteTexture()
{
    return (m_pFallbackWhite);
}

ITexture2D* CTexturesManager::GetFallBackNormalTexture()
{
    return (m_pFallbackNormal);
}

ITexture2D* CTexturesManager::GetFallBackBlackTexture()
{
    return (m_pFallbackBlack);
}
