#pragma once

#include "Singleton.h"
#include "Vulkan/VulkanTexture2D.h"

class CTexturesManager : public CSingleton<CTexturesManager>
{
public:
	CTexturesManager() = default;
	~CTexturesManager() = default;

	bool Initialize();
	void Destroy();

	ITexture2D* GetFallBackWhiteTexture();
	ITexture2D* GetFallBackNormalTexture();
	ITexture2D* GetFallBackBlackTexture();

private:
	ITexture2D* m_pFallbackWhite;
	ITexture2D* m_pFallbackBlack;
	ITexture2D* m_pFallbackNormal;
};