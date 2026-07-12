#pragma once

#include "API/Texture.h"

struct STextureData
{
	uint32_t uiTextureID;
	int32_t iWidth = 0;
	int32_t iHeight = 0;
	int32_t iDepth = 1;
	int32_t iChannels = 0;
};

class COpenGLTexture2D : public ITexture2D
{
public:
	COpenGLTexture2D() = default;
	~COpenGLTexture2D() = default;

	void Clear();
	void UpdateTextureData(const STextureDesc& textureDesc, const STextureData& textureData);

	// OpenGL Properties
	uint32_t GetTextureID() const;

	uint32_t GetWidth() const override;
	uint32_t GetHeight() const override;
	std::string GetName() const;

private:
	uint32_t m_uiTextureID = 0;
};