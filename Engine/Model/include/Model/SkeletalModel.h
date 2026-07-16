#pragma once

#include "API/RenderObject.h"
#include "API/Buffer.h"
#include "Quaternion.h"
#include "Model/SkeletalMesh.h"

class CSkeletalModel : public CModelAssetBase
{
public:
	CSkeletalModel() = default;
	~CSkeletalModel() = default;

	// Non-copyable — owns GPU resources
	CSkeletalModel(const CSkeletalModel&) = delete;
	CSkeletalModel& operator=(const CSkeletalModel&) = delete;

	const std::vector<CSkeletalMesh>& GetMeshes() const { return m_vMeshes; }
	std::vector<CSkeletalMesh>& GetMeshes() { return m_vMeshes; }

	void SetSkeleton(const std::shared_ptr<CSkeleton>& pSkeleton) { m_pSkeleton = pSkeleton; }
	const std::shared_ptr<CSkeleton>& GetSkeleton() const { return m_pSkeleton; }

	const std::vector<SSkeletalMeshVertex>& GetMergedVertices() const { return m_vMergedVertices; }
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
	std::vector<CSkeletalMesh> m_vMeshes = {};
	std::shared_ptr<CSkeleton> m_pSkeleton = nullptr;

	// Actor Model Data
	size_t m_iIndexOffset = 0; // Offset for instancing
	size_t m_iVertexOffset = 0; // Offset for instancing
	std::vector<SSkeletalMeshVertex> m_vMergedVertices = {};
	std::vector<uint32_t> m_vMergedIndices = {};

	// GPU Data
	CVertexArray* m_pVertexArray = nullptr;
	IBuffer* m_pVertexBuffer = nullptr;
	IBuffer* m_pIndexBuffer = nullptr;
};