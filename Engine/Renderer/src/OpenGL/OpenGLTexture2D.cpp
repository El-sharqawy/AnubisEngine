#pragma once

#include "OpenGL/OpenGLTexture2D.h"

void COpenGLTexture2D::Clear()
{
    // Texture Properties
    m_stName = "Texture";
    m_fsFilePath.clear();           // empty = procedural texture
    m_iWidth = 0;
    m_iHeight = 0;
    m_iDepth = 1;
    m_iChannels = 0;
    m_eType = ETextureType::TEXTURE_TYPE_2D;
    m_eFormat = ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB;
    m_uiUsageFlags = static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_SAMPLED) | static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_DST);
    m_eMagFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    m_eMinFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    m_eMipmapMode = ETextureMipmapMode::TEXTURE_MIPMAP_MODE_LINEAR;

    m_eWrapU = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_eWrapV = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_eWrapW = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_bGenMipmaps = true;
    m_bEnableAnisotropy = true;
    m_fMaxAnisotropy = 16.0f;
    m_bEnableCompare = false;

}
void COpenGLTexture2D::UpdateTextureData(const STextureDesc& textureDesc, const STextureData& textureData)
{
    // OpenGL
    m_uiTextureID = textureData.uiTextureID;

    m_stName = textureDesc.m_stName;
    // empty = procedural texture
    m_fsFilePath = textureDesc.m_fsFilePath;

    m_iWidth = textureData.iWidth;
    m_iHeight = textureData.iHeight;
    m_iDepth = textureData.iDepth;			// depth is mostly 1 unless 3D
    m_iChannels = textureData.iChannels;

    m_eType = textureDesc.m_eType;
    m_eFormat = textureDesc.m_eFormat;
    m_uiUsageFlags = textureDesc.m_uiUsageFlags;
    m_eMagFilter = textureDesc.m_eMagFilter;
    m_eMinFilter = textureDesc.m_eMinFilter;
    m_eMipmapMode = textureDesc.m_eMipmapMode;

    m_eWrapU = textureDesc.m_eWrapU;
    m_eWrapV = textureDesc.m_eWrapV;
    m_eWrapW = textureDesc.m_eWrapW;
    m_bGenMipmaps = textureDesc.m_bGenMipmaps;
    m_bEnableAnisotropy = textureDesc.m_bEnableAnisotropy;
    m_fMaxAnisotropy = textureDesc.m_fMaxAnisotropy;
    m_bEnableCompare = textureDesc.m_bEnableCompare;
}

uint32_t COpenGLTexture2D::GetTextureID() const
{
    return (m_uiTextureID);
}

uint32_t COpenGLTexture2D::GetWidth() const
{
    return (m_iWidth);
}

uint32_t COpenGLTexture2D::GetHeight() const
{
    return (m_iHeight);
}

std::string COpenGLTexture2D::GetName() const
{
    return (m_stName);
}
