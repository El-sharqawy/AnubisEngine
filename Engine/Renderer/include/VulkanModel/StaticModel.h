#pragma once

#include "API/RenderObject.h"
#include "Quaternion.h"
#include "StaticMesh.h"
#include "Vulkan/VulkanMaterial.h"
#include "Vulkan/VulkanDescriptorContext.h"
#include "Vulkan/VulkanCommandList.h"

#include "OpenGL/OpenGLVertexArray.h"

#include <filesystem>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>			// Output data structure
#include <assimp/postprocess.h>		// Post processing flags

#define ENABLE_MESH_LOGS

#define ASSIMP_LOAD_FLAGS (aiProcess_JoinIdenticalVertices |    \
                           aiProcess_Triangulate |              \
                           aiProcess_GenSmoothNormals |         \
                           aiProcess_LimitBoneWeights |         \
                           aiProcess_SplitLargeMeshes |         \
                           aiProcess_ImproveCacheLocality |     \
                           aiProcess_RemoveRedundantMaterials | \
                           aiProcess_FindDegenerates |          \
                           aiProcess_FindInvalidData |          \
                           aiProcess_GenUVCoords |              \
                           aiProcess_CalcTangentSpace)

class CVulkanBuffer;
class CVulkanDevice;
class CVulkanRenderer;
class CVulkanPipeline;

struct SModelBatch
{
	uint32_t materialIndex = 0;
	uint32_t firstIndex = 0;   // index into merged index buffer
	uint32_t indexCount = 0;
	int32_t  baseVertex = 0;   // usually 0 if indices are already rebased
};

struct SModelImportOptions
{
	bool m_bFlipUVs = false;
};

class CStaticModel : public IRenderObject
{
public:
	CStaticModel() = default;
	~CStaticModel() = default;

	// Non-copyable — owns GPU resources
	CStaticModel(const CStaticModel&) = delete;
	CStaticModel& operator=(const CStaticModel&) = delete;

	const std::vector<CStaticMesh>& GetMeshes() const { return m_vMeshes; }
	std::vector<CStaticMesh>& GetMeshes() { return m_vMeshes; }
	const std::vector<IMaterial*>& GetMaterials() const { return m_vMaterials; }
	std::vector<IMaterial*>& GetMaterials() { return m_vMaterials; }
	IMaterial* GetMaterial(size_t index) { return m_vMaterials.at(index); }

	const std::string& GetMeshFilePath() const { return m_stMeshFilePath; }
	void SetMeshFilePath(const std::string& stFilePath) { m_stMeshFilePath = stFilePath; }
	const std::string& GetMeshName() const { return m_stMeshName; }
	void SetMeshName(const std::string& stName) { m_stMeshName = stName; }

	const std::vector<SStaticMeshVertex>& GetMergedVertices() const { return m_vMergedVertices; }
	const std::vector<uint32_t>& GetMergedIndices() const { return m_vMergedIndices; }
	const std::vector<SModelBatch>& GetBatches() const { return m_vBatches; }

	// Load Model
	bool ImportModel(const std::filesystem::path& filePath, const std::vector<IBuffer*>& vpUniformBuffer, const SModelImportOptions& options = {});
	bool ReadScene(const std::filesystem::path& filePath, const SModelImportOptions& options = {});
	void BuildMeshes();
	void BuildMaterials();
	IMaterial* ProcessMaterial(const aiMaterial* mat, const aiScene* scene);
	ITexture2D* LoadMaterialTexture(const std::string& stName, const aiMaterial* mat, aiTextureType type, const aiScene* scene);

	void Clear();
	void BuildMergedGeometry();
	void UploadToGPU();
	void RenderModel(uint32_t currentFrame, ICommandList* pCmd);
	std::vector<SRenderItem> BuildRenderItems();

	Matrix4 GetModelMatrix();

	void SetPosition(const Vector3D& v3Pos);
	void SetScale(const Vector3D& v3Scale);

protected:
	bool InitializeMaterialBindings(const std::vector<IBuffer*>& vpUniformBuffer);

private:
	std::vector<CStaticMesh> m_vMeshes = {};
	std::vector<IMaterial*> m_vMaterials = {};
	std::string m_stDirectory;
	std::string m_stMeshFilePath;
	std::string m_stMeshName;

	// Actor Model Data
	size_t m_iIndexOffset = 0; // Offset for instancing
	size_t m_iVertexOffset = 0; // Offset for instancing
	std::vector<SStaticMeshVertex> m_vMergedVertices = {};
	std::vector<uint32_t> m_vMergedIndices = {};
	std::vector<SModelBatch> m_vBatches = {};

	// GPU Data
	CVertexArray* m_pVertexArray = nullptr;
	IBuffer* m_pVertexBuffer = nullptr;
	IBuffer* m_pIndexBuffer = nullptr;

	// Loader
	Assimp::Importer m_Importer = {};
	const aiScene* m_pScene = nullptr; // owns m_pScene lifetime
	std::filesystem::path m_modelPath;
	std::filesystem::path m_modelDirectory;
	Matrix4 m_matGlobalInverseTransform = Matrix4(1.0f);

	Vector3D m_v3Position = Vector3D(0.0f);
	SQuaternion m_qRotation = SQuaternion(1.0f);
	Vector3D m_v3Scale = Vector3D(1.0f);
	Matrix4 m_matModel = Matrix4(1.0f);
	bool m_bTransformDirty = true;
};