#include "Vulkan/VulkanMaterial.h"
#include "Device/VulkanRenderDevice.h"

bool CVulkanMaterial::InitializeMaterial(const SBindingContextDesc& ctxDesc)
{
	m_vkVulkanDescriptorContext = AnubisNew(CVulkanDescriptorContext, MEM_TAG_RENDERING);
	if (!m_vkVulkanDescriptorContext->Initialize(ctxDesc))
	{
		syserr("Failed to Initialize Vulkan Descriptor Context for Material {}", GetMaterialName());
		delete m_vkVulkanDescriptorContext;
		return (false);
	}

	m_vkvDescriptorSets = m_vkVulkanDescriptorContext->GetDescriptorSets();
	return (true);
}

IBindingContext* CVulkanMaterial::GetDescriptorContext() const
{
	return (m_vkVulkanDescriptorContext);
}

void CVulkanMaterial::ClearMaterial()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	// Clear Diffuse And Specular
	if (m_pDiffuseMap)
	{
		vkRenderDevice.DestroyTexture2D(m_pDiffuseMap);
		AnubisSafeDelete(m_pDiffuseMap);
	}
	if (m_pSpecularMap)
	{
		vkRenderDevice.DestroyTexture2D(m_pSpecularMap);
		AnubisSafeDelete(m_pSpecularMap);
	}

	// Clear PBR Materials
	if (m_sPbr.m_pAlbedoMap)
	{
		vkRenderDevice.DestroyTexture2D(m_sPbr.m_pAlbedoMap);
		AnubisSafeDelete(m_sPbr.m_pAlbedoMap);
	}
	if (m_sPbr.m_pRoughnessMap)
	{
		vkRenderDevice.DestroyTexture2D(m_sPbr.m_pRoughnessMap);
		AnubisSafeDelete(m_sPbr.m_pRoughnessMap);
	}
	if (m_sPbr.m_pMetallicMap)
	{
		vkRenderDevice.DestroyTexture2D(m_sPbr.m_pMetallicMap);
		AnubisSafeDelete(m_sPbr.m_pMetallicMap);
	}
	if (m_sPbr.m_pNormalMap)
	{
		vkRenderDevice.DestroyTexture2D(m_sPbr.m_pNormalMap);
		AnubisSafeDelete(m_sPbr.m_pNormalMap);
	}

	m_vkVulkanDescriptorContext->Destroy();
	AnubisSafeDelete(m_vkVulkanDescriptorContext);

	m_vkvDescriptorSets.clear();
}

void CVulkanMaterial::SetMaterialName(const std::string& stName)
{
	m_stName = stName;
}

const std::string& CVulkanMaterial::GetMaterialName() const
{
	return (m_stName);
}

void CVulkanMaterial::SetMaterialWorkflow(const EMaterialWorkflow& materialWorkFlow)
{
	m_eWorkflow = materialWorkFlow;
}

EMaterialWorkflow CVulkanMaterial::GetMaterialWorkflow() const
{
	return (m_eWorkflow);
}

void CVulkanMaterial::SetMaterialAmbientColor(const Vector4D& v4AmbientColor)
{
	m_v4AmbientColor = v4AmbientColor;
}

Vector4D CVulkanMaterial::GetMaterialAmbientColor() const
{
	return (m_v4AmbientColor);
}

void CVulkanMaterial::SetMaterialDiffuseColor(const Vector4D& v4DiffuseColor)
{
	m_v4DiffuseColor = v4DiffuseColor;
}

Vector4D CVulkanMaterial::GetMaterialDiffuseColor() const
{
	return (m_v4DiffuseColor);
}

void CVulkanMaterial::SetMaterialSpecularColor(const Vector4D& v4SpecularColor)
{
	m_v4SpecularColor = v4SpecularColor;
}

Vector4D CVulkanMaterial::GetMaterialSpecularColor() const
{
	return (m_v4SpecularColor);
}

void CVulkanMaterial::SetMaterialDiffuseMap(ITexture2D* pDiffuseMap)
{
	m_pDiffuseMap = pDiffuseMap;
}

ITexture2D* CVulkanMaterial::GetMaterialDiffuseMap() const
{
	return (m_pDiffuseMap);
}

void CVulkanMaterial::SetMaterialSpecularMap(ITexture2D* pSpecularMap)
{
	m_pSpecularMap = pSpecularMap;
}

ITexture2D* CVulkanMaterial::GetMaterialSpecularMap() const
{
	return (m_pSpecularMap);
}

void CVulkanMaterial::SetMaterialTransparency(float fTransparency)
{
	m_fTransparency = fTransparency;
}

float CVulkanMaterial::GetMaterialTransparency() const
{
	return (m_fTransparency);
}

void CVulkanMaterial::SetMaterialAlpha(float fAlpha)
{
	m_fAlpha = fAlpha;
}

float CVulkanMaterial::GetMaterialAlpha() const
{
	return (m_fAlpha);
}

void CVulkanMaterial::SetMaterialPBRAlbedoMap(ITexture2D* pAlbedoMap)
{
	m_sPbr.m_pAlbedoMap = pAlbedoMap;
}

ITexture2D* CVulkanMaterial::GetMaterialPBRAlbedoMap() const
{
	return (m_sPbr.m_pAlbedoMap);
}

void CVulkanMaterial::SetMaterialPBRRoughnessMap(ITexture2D* pRoughnessMap)
{
	m_sPbr.m_pRoughnessMap = pRoughnessMap;
}

ITexture2D* CVulkanMaterial::GetMaterialPBRRoughnessMap() const
{
	return (m_sPbr.m_pRoughnessMap);
}

void CVulkanMaterial::SetMaterialPBRMetallicMap(ITexture2D* pMetallicMap)
{
	m_sPbr.m_pMetallicMap = pMetallicMap;
}

ITexture2D* CVulkanMaterial::GetMaterialPBRMetallicMap() const
{
	return (m_sPbr.m_pMetallicMap);
}

void CVulkanMaterial::SetMaterialPBRNormalMap(ITexture2D* pNormalMap)
{
	m_sPbr.m_pNormalMap = pNormalMap;
}

ITexture2D* CVulkanMaterial::GetMaterialPBRNormalMap() const
{
	return (m_sPbr.m_pNormalMap);
}

void CVulkanMaterial::SetMaterialPBRRoughness(float fRoughness)
{
	m_sPbr.m_fRoughness = fRoughness;
}

float CVulkanMaterial::GetMaterialPBRRoughness() const
{
	return (m_sPbr.m_fRoughness);
}

void CVulkanMaterial::SetMaterialPBRMetallic(float fMetallic)
{
	m_sPbr.m_fMetallic = fMetallic;
}

float CVulkanMaterial::GetMaterialPBRMetallic() const
{
	return (m_sPbr.m_fMetallic);
}

void CVulkanMaterial::SetMaterialPBRBaseColor(const Vector3D& v3BaseColor)
{
	m_sPbr.m_v3BaseColor = v3BaseColor;
}

Vector3D CVulkanMaterial::GetMaterialPBRBaseColor() const
{
	return (m_sPbr.m_v3BaseColor);
}

void CVulkanMaterial::SetMaterialPBR(const SPBRMaterial& pbr)
{
	m_sPbr = pbr;
}

SPBRMaterial& CVulkanMaterial::GetMaterialPBR()
{
	return (m_sPbr);
}

const SPBRMaterial& CVulkanMaterial::GetMaterialPBR() const
{
	return (m_sPbr);
}

std::vector<VkDescriptorSet>& CVulkanMaterial::GetDescriptorSets()
{
	return (m_vkvDescriptorSets);
}

const std::vector<VkDescriptorSet>& CVulkanMaterial::GetDescriptorSets() const
{
	return (m_vkvDescriptorSets);
}

VkDescriptorSet CVulkanMaterial::GetDescriptorSet(size_t index) const
{
	if (index >= m_vkvDescriptorSets.size())
	{
		return (VK_NULL_HANDLE);
	}

	return m_vkvDescriptorSets.at(index);
}

CVulkanDescriptorContext* CVulkanMaterial::GetDescriptorContext()
{
	return (m_vkVulkanDescriptorContext);
}

void CVulkanMaterial::SetOwningPipeline(CVulkanPipeline* pPipeline)
{
	m_pOwningPipeline = pPipeline;
}
