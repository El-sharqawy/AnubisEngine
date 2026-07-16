#pragma once

#include "API/Pipeline.h"
#include "API/Buffer.h"

class COpenGLPipeline : public IPipeline
{
public:
	COpenGLPipeline() = default;
	~COpenGLPipeline() = default;

	bool Initialize(const SPipelineDesc& desc);
	void Destroy();
	EPipelineType GetPipelineType() const override;
	IShaderProgram* GetPipelineShader() const;
	IBuffer* GetPipelineConstants() const;
	void SetGLToggle(GLenum cap, bool enable);

private:
	IShaderProgram* m_pPipelineShader = nullptr;
	IBuffer* m_pPipelineConstants = nullptr;
	GLStateCache m_gGLStateCache = {};
};