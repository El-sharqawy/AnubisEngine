#pragma once

#include <glad/gl.h>
#include "API/BindingContext.h"

class COpenGLBindingContext : public IBindingContext
{
public:
	COpenGLBindingContext() = default;
	~COpenGLBindingContext() override = default;

	bool Initialize(const SBindingContextDesc& desc) override;
	void Destroy() override;

	bool Bind(uint32_t frameIndex, uint32_t programID);

	const SBindingDesc* FindBindingDesc(const std::vector<SBindingDesc>& bindings, uint32_t bindingIndex)
	{
		for (const auto& binding : bindings)
		{
			if (binding.m_uiBinding == bindingIndex)
			{
				return &binding;
			}
		}
		return nullptr;
	}

private:
	bool m_bInitialized = false;
	std::vector<SBindingDesc> m_vBindings;
	std::vector<SBindingBufferResource> m_vBufferResources; // borrowed
	std::vector<SBindingImageResource>  m_vImageResources;  // borrowed
	uint32_t m_uiFrameCount = 0;
};