#pragma once

#include "API/RenderObject.h"
#include "API/Buffer.h"
#include "Quaternion.h"
#include "StaticMesh.h"

#include <filesystem>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>			// Output data structure
#include <assimp/postprocess.h>		// Post processing flags

#define ENABLE_MESH_LOGS

class CVertexArray;

class CStaticModel : public CModelAssetBase
{
public:
	CStaticModel() = default;
	~CStaticModel() = default;

	// Non-copyable — owns GPU resources
	CStaticModel(const CStaticModel&) = delete;
	CStaticModel& operator=(const CStaticModel&) = delete;

	const std::vector<CStaticMesh>& GetMeshes() const { return m_vMeshes; }
	std::vector<CStaticMesh>& GetMeshes() { return m_vMeshes; }

	const std::vector<SStaticMeshVertex>& GetMergedVertices() const { return m_vMergedVertices; }
	const std::vector<uint32_t>& GetMergedIndices() const { return m_vMergedIndices; }

	void Clear() override;
	void BuildMergedGeometry();
	void UploadToGPU();

	CVertexArray* GetVertexArray();
	IBuffer* GetVertexBuffer();
	IBuffer* GetIndexBuffer();

	void SetPosition(const Vector3D& v3Pos);
	void SetScale(const Vector3D& v3Scale);
	bool InitializeMaterialBindings() override;

private:
	std::vector<CStaticMesh> m_vMeshes = {};

	// Actor Model Data
	size_t m_iIndexOffset = 0; // Offset for instancing
	size_t m_iVertexOffset = 0; // Offset for instancing
	std::vector<SStaticMeshVertex> m_vMergedVertices = {};
	std::vector<uint32_t> m_vMergedIndices = {};

	// GPU Data
	CVertexArray* m_pVertexArray = nullptr;
	IBuffer* m_pVertexBuffer = nullptr;
	IBuffer* m_pIndexBuffer = nullptr;
};