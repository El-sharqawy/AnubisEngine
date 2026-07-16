#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <vector>

#include "API/Material.h"
#include "API/BindingContext.h"

class CVulkanDescriptorContext;
class CVulkanPipeline;

class CVulkanMaterial : public IMaterial
{
public:
	CVulkanMaterial() = default;
	~CVulkanMaterial() = default;

	bool InitializeMaterial(const SBindingContextDesc& ctxDesc) override;
	IBindingContext* GetDescriptorContext() const override;

	void ClearMaterial() override;

	void SetMaterialName(const std::string& stName) override;
	const std::string& GetMaterialName() const override;

	void SetMaterialWorkflow(const EMaterialWorkflow& materialWorkFlow) override;
	EMaterialWorkflow GetMaterialWorkflow() const override;

	void SetMaterialAmbientColor(const Vector4D& v4AmbientColor) override;
	Vector4D GetMaterialAmbientColor() const override;

	void SetMaterialDiffuseColor(const Vector4D& v4DiffuseColor) override;
	Vector4D GetMaterialDiffuseColor() const override;

	void SetMaterialSpecularColor(const Vector4D& v4SpecularColor) override;
	Vector4D GetMaterialSpecularColor() const override;

	void SetMaterialDiffuseMap(ITexture2D* pDiffuseMap) override;
	ITexture2D* GetMaterialDiffuseMap() const override;

	void SetMaterialSpecularMap(ITexture2D* pSpecularMap) override;
	ITexture2D* GetMaterialSpecularMap() const override;

	void SetMaterialTransparency(float fTransparency) override;
	float GetMaterialTransparency() const override;

	void SetMaterialAlpha(float fAlpha) override;
	float GetMaterialAlpha() const override;
	
	void SetMaterialPBRAlbedoMap(ITexture2D* pAlbedoMap) override;
	ITexture2D* GetMaterialPBRAlbedoMap() const override;

	void SetMaterialPBRRoughnessMap(ITexture2D* pRoughnessMap) override;
	ITexture2D* GetMaterialPBRRoughnessMap() const override;

	void SetMaterialPBRMetallicMap(ITexture2D* pMetallicMap) override;
	ITexture2D* GetMaterialPBRMetallicMap() const override;

	void SetMaterialPBRNormalMap(ITexture2D* pNormalMap) override;
	ITexture2D* GetMaterialPBRNormalMap() const override;

	void SetMaterialPBRRoughness(float fRoughness) override;
	float GetMaterialPBRRoughness() const override;

	void SetMaterialPBRMetallic(float fMetallic) override;
	float GetMaterialPBRMetallic() const override;

	void SetMaterialPBRBaseColor(const Vector3D& v3BaseColor) override;
	Vector3D GetMaterialPBRBaseColor() const override;

	void SetMaterialPBR(const SPBRMaterial& pbr) override;
	SPBRMaterial& GetMaterialPBR() override;
	const SPBRMaterial& GetMaterialPBR() const override;

	std::vector<VkDescriptorSet>& GetDescriptorSets() ;
	const std::vector<VkDescriptorSet>& GetDescriptorSets() const;
	VkDescriptorSet GetDescriptorSet(size_t index) const;

	CVulkanDescriptorContext* GetDescriptorContext();
	void SetOwningPipeline(CVulkanPipeline* pPipeline);

private:
	CVulkanPipeline* m_pOwningPipeline = nullptr;
	std::vector<VkDescriptorSet> m_vkvDescriptorSets = {}; // size = MAX_FRAMES_IN_FLIGHT
	CVulkanDescriptorContext* m_pVulkanDescriptorContext = nullptr;
};