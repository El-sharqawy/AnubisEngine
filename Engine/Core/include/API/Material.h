#pragma once

#include "API/BindingContext.h"
#include "API/Texture.h"
#include "TypeVector3.h"
#include "TypeVector4.h"

enum class EMaterialWorkflow
{
	MATERIAL_WORKFLOW_LEGACY,
	MATERIAL_WORKFLOW_PBR
};

struct SPBRMaterial
{
	ITexture2D* m_pAlbedoMap = nullptr;
	ITexture2D* m_pRoughnessMap = nullptr;
	ITexture2D* m_pMetallicMap = nullptr;
	ITexture2D* m_pNormalMap = nullptr;

	float m_fRoughness = 1.0f;
	float m_fMetallic = 0.0f;
	SVector3Df m_v3BaseColor = SVector3Df(1.0f, 1.0f, 1.0f);
};

class IMaterial
{
public:
    virtual ~IMaterial() = default;
	virtual bool InitializeMaterial(const SBindingContextDesc& desc) = 0;
	virtual IBindingContext* GetDescriptorContext() const = 0;

    virtual void ClearMaterial() = 0;

    virtual void SetMaterialName(const std::string& stName) = 0;
    virtual const std::string& GetMaterialName() const = 0;

	virtual void SetMaterialWorkflow(const EMaterialWorkflow& materialWorkFlow) = 0;
	virtual EMaterialWorkflow GetMaterialWorkflow() const = 0;

	virtual void SetMaterialAmbientColor(const Vector4D& v4AmbientColor) = 0;
	virtual Vector4D GetMaterialAmbientColor() const = 0;

	virtual void SetMaterialDiffuseColor(const Vector4D& v4DiffuseColor) = 0;
	virtual Vector4D GetMaterialDiffuseColor() const = 0;

	virtual void SetMaterialSpecularColor(const Vector4D& v4SpecularColor) = 0;
	virtual Vector4D GetMaterialSpecularColor() const = 0;

	virtual void SetMaterialDiffuseMap(ITexture2D* pDiffuseMap) = 0;
	virtual ITexture2D* GetMaterialDiffuseMap() const = 0;

	virtual void SetMaterialSpecularMap(ITexture2D* pSpecularMap) = 0;
	virtual ITexture2D* GetMaterialSpecularMap() const = 0;

	virtual void SetMaterialTransparency(float fTransparency) = 0;
	virtual float GetMaterialTransparency() const = 0;

	virtual void SetMaterialAlpha(float fAlpha) = 0;
	virtual float GetMaterialAlpha() const = 0;

	virtual void SetMaterialPBRAlbedoMap(ITexture2D* pAlbedoMap) = 0;
	virtual ITexture2D* GetMaterialPBRAlbedoMap() const = 0;

	virtual void SetMaterialPBRRoughnessMap(ITexture2D* pRoughnessMap) = 0;
	virtual ITexture2D* GetMaterialPBRRoughnessMap() const = 0;

	virtual void SetMaterialPBRMetallicMap(ITexture2D* pMetallicMap) = 0;
	virtual ITexture2D* GetMaterialPBRMetallicMap() const = 0;

	virtual void SetMaterialPBRNormalMap(ITexture2D* pNormalMap) = 0;
	virtual ITexture2D* GetMaterialPBRNormalMap() const = 0;

	virtual void SetMaterialPBRRoughness(float fRoughness) = 0;
	virtual float GetMaterialPBRRoughness() const = 0;

	virtual void SetMaterialPBRMetallic(float fMetallic) = 0;
	virtual float GetMaterialPBRMetallic() const = 0;

	virtual void SetMaterialPBRBaseColor(const Vector3D& v3BaseColor) = 0;
	virtual Vector3D GetMaterialPBRBaseColor() const = 0;

	virtual void SetMaterialPBR(const SPBRMaterial& pbr) = 0;
	virtual SPBRMaterial& GetMaterialPBR() = 0;
	virtual const SPBRMaterial& GetMaterialPBR() const = 0;

protected:
	std::string m_stName = "Material";
	EMaterialWorkflow m_eWorkflow = EMaterialWorkflow::MATERIAL_WORKFLOW_LEGACY;

	SVector4Df m_v4AmbientColor = SVector4Df(0, 0, 0, 1);
	SVector4Df  m_v4DiffuseColor = SVector4Df(1, 1, 1, 1);
	SVector4Df  m_v4SpecularColor = SVector4Df(0, 0, 0, 1);

	ITexture2D* m_pDiffuseMap = nullptr;
	ITexture2D* m_pSpecularMap = nullptr;

	float m_fTransparency = 1.0f;
	float m_fAlpha = 1.0f;

	SPBRMaterial m_sPbr = {};
};
