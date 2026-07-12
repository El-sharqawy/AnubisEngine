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

private:
	bool m_bInitialized = false;
	SBindingContextDesc m_gDesc = {};
};