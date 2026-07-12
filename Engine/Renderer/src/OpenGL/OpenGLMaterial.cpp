#include "OpenGL/OpenGLMaterial.h"
#include "OpenGL/OpenGLBindingContext.h"
#include "API/RenderDevice.h"

bool COpenGLMaterial::InitializeMaterial(const SBindingContextDesc& ctxDesc)
{
	m_pDescriptorContext = AnubisNew(COpenGLBindingContext, MEM_TAG_MATERIAL);
	if (!m_pDescriptorContext->Initialize(ctxDesc))
	{
		return (false);
	}

	return (true);
}

IBindingContext* COpenGLMaterial::GetDescriptorContext() const
{
	return (m_pDescriptorContext);
}

void COpenGLMaterial::ClearMaterial()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	// Clear Diffuse And Specular
	if (m_pDiffuseMap)
	{
		renderDev.DestroyTexture2D(m_pDiffuseMap);
		AnubisSafeDelete(m_pDiffuseMap);
	}
	if (m_pSpecularMap)
	{
		renderDev.DestroyTexture2D(m_pSpecularMap);
		AnubisSafeDelete(m_pSpecularMap);
	}

	// Clear PBR Materials
	if (m_sPbr.m_pAlbedoMap)
	{
		renderDev.DestroyTexture2D(m_sPbr.m_pAlbedoMap);
		AnubisSafeDelete(m_sPbr.m_pAlbedoMap);
	}
	if (m_sPbr.m_pRoughnessMap)
	{
		renderDev.DestroyTexture2D(m_sPbr.m_pRoughnessMap);
		AnubisSafeDelete(m_sPbr.m_pRoughnessMap);
	}
	if (m_sPbr.m_pMetallicMap)
	{
		renderDev.DestroyTexture2D(m_sPbr.m_pMetallicMap);
		AnubisSafeDelete(m_sPbr.m_pMetallicMap);
	}
	if (m_sPbr.m_pNormalMap)
	{
		renderDev.DestroyTexture2D(m_sPbr.m_pNormalMap);
		AnubisSafeDelete(m_sPbr.m_pNormalMap);
	}

	if (m_pDescriptorContext)
	{
		m_pDescriptorContext->Destroy();
		AnubisSafeDelete(m_pDescriptorContext);
	}
}

void COpenGLMaterial::SetMaterialName(const std::string& stName)
{
	m_stName = stName;
}

const std::string& COpenGLMaterial::GetMaterialName() const
{
	return (m_stName);
}

void COpenGLMaterial::SetMaterialWorkflow(const EMaterialWorkflow& materialWorkFlow)
{
	m_eWorkflow = materialWorkFlow;
}

EMaterialWorkflow COpenGLMaterial::GetMaterialWorkflow() const
{
	return (m_eWorkflow);
}

void COpenGLMaterial::SetMaterialAmbientColor(const Vector4D& v4AmbientColor)
{
	m_v4AmbientColor = v4AmbientColor;
}

Vector4D COpenGLMaterial::GetMaterialAmbientColor() const
{
	return (m_v4AmbientColor);
}

void COpenGLMaterial::SetMaterialDiffuseColor(const Vector4D& v4DiffuseColor)
{
	m_v4DiffuseColor = v4DiffuseColor;
}

Vector4D COpenGLMaterial::GetMaterialDiffuseColor() const
{
	return (m_v4DiffuseColor);
}

void COpenGLMaterial::SetMaterialSpecularColor(const Vector4D& v4SpecularColor)
{
	m_v4SpecularColor = v4SpecularColor;
}

Vector4D COpenGLMaterial::GetMaterialSpecularColor() const
{
	return (m_v4SpecularColor);
}

void COpenGLMaterial::SetMaterialDiffuseMap(ITexture2D* pDiffuseMap)
{
	m_pDiffuseMap = pDiffuseMap;
}

ITexture2D* COpenGLMaterial::GetMaterialDiffuseMap() const
{
	return (m_pDiffuseMap);
}

void COpenGLMaterial::SetMaterialSpecularMap(ITexture2D* pSpecularMap)
{
	m_pSpecularMap = pSpecularMap;
}

ITexture2D* COpenGLMaterial::GetMaterialSpecularMap() const
{
	return (m_pSpecularMap);
}

void COpenGLMaterial::SetMaterialTransparency(float fTransparency)
{
	m_fTransparency = fTransparency;
}

float COpenGLMaterial::GetMaterialTransparency() const
{
	return (m_fTransparency);
}

void COpenGLMaterial::SetMaterialAlpha(float fAlpha)
{
	m_fAlpha = fAlpha;
}

float COpenGLMaterial::GetMaterialAlpha() const
{
	return (m_fAlpha);
}

void COpenGLMaterial::SetMaterialPBRAlbedoMap(ITexture2D* pAlbedoMap)
{
	m_sPbr.m_pAlbedoMap = pAlbedoMap;
}

ITexture2D* COpenGLMaterial::GetMaterialPBRAlbedoMap() const
{
	return (m_sPbr.m_pAlbedoMap);
}

void COpenGLMaterial::SetMaterialPBRRoughnessMap(ITexture2D* pRoughnessMap)
{
	m_sPbr.m_pRoughnessMap = pRoughnessMap;
}

ITexture2D* COpenGLMaterial::GetMaterialPBRRoughnessMap() const
{
	return (m_sPbr.m_pRoughnessMap);
}

void COpenGLMaterial::SetMaterialPBRMetallicMap(ITexture2D* pMetallicMap)
{
	m_sPbr.m_pMetallicMap = pMetallicMap;
}

ITexture2D* COpenGLMaterial::GetMaterialPBRMetallicMap() const
{
	return (m_sPbr.m_pMetallicMap);
}

void COpenGLMaterial::SetMaterialPBRNormalMap(ITexture2D* pNormalMap)
{
	m_sPbr.m_pNormalMap = pNormalMap;
}

ITexture2D* COpenGLMaterial::GetMaterialPBRNormalMap() const
{
	return (m_sPbr.m_pNormalMap);
}

void COpenGLMaterial::SetMaterialPBRRoughness(float fRoughness)
{
	m_sPbr.m_fRoughness = fRoughness;
}

float COpenGLMaterial::GetMaterialPBRRoughness() const
{
	return (m_sPbr.m_fRoughness);
}

void COpenGLMaterial::SetMaterialPBRMetallic(float fMetallic)
{
	m_sPbr.m_fMetallic = fMetallic;
}

float COpenGLMaterial::GetMaterialPBRMetallic() const
{
	return (m_sPbr.m_fMetallic);
}

void COpenGLMaterial::SetMaterialPBRBaseColor(const Vector3D& v3BaseColor)
{
	m_sPbr.m_v3BaseColor = v3BaseColor;
}

Vector3D COpenGLMaterial::GetMaterialPBRBaseColor() const
{
	return (m_sPbr.m_v3BaseColor);
}

void COpenGLMaterial::SetMaterialPBR(const SPBRMaterial& pbr)
{
	m_sPbr = pbr;
}

SPBRMaterial& COpenGLMaterial::GetMaterialPBR()
{
	return (m_sPbr);
}

const SPBRMaterial& COpenGLMaterial::GetMaterialPBR() const
{
	return (m_sPbr);
}
