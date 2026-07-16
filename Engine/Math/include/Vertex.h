#pragma once

#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include "TypeMatrix4.h"

/**
 * @brief Vertex structure (PNT) for rendering.
 *
 * This structure defines the layout of a vertex used in rendering,
 * including position, texture coordinates, and normals.
 */
struct SVertex
{
	Vector3D m_v3Position;		// World position
	Vector3D m_v3Normals;		// Normal
	Vector2D m_v2TexCoords;		// UVs (For Texturing)
	Vector4D m_v4Color;			// Color

	SVertex();
	SVertex(float fX, float fY, float fZ);
	SVertex(float fX, float fY, float fZ, float fU, float fV);
	SVertex(float fX, float fY, float fZ, float fU, float fV, float fR, float fG, float fB, float fA);
	SVertex(const Vector3D& v3Position);
	SVertex(const Vector3D& v3Position, const Vector2D& v2TexCoords);
	SVertex(const Vector3D& v3Position, const Vector2D& v2TexCoords, const Vector4D& v4Color);
	SVertex(const Vector3D& v3Position, const Vector4D& v4Color); // No UVs
};

using Vertex = SVertex;

struct SDebugLineInstance
{
	Vector3D start;
	Vector3D end;
	Vector4D color;
};

struct SDebugBoxInstance
{
	Matrix4 world;
	Vector4D color;
};

/**
 * @brief Vertex structure for rendering lines.
 *
 * This structure defines the layout of a vertex used specifically for line rendering,
 * including position and color.
 */
struct SLinesVertex
{
	Vector3D m_v3Position;		// World position
	Vector4D m_v4Color;			// Color

	SLinesVertex();
	SLinesVertex(float fX, float fY, float fZ);
	SLinesVertex(float fX, float fY, float fZ, float fR, float fG, float fB, float fA);
	SLinesVertex(const Vector3D& v3Position);
	SLinesVertex(const Vector3D& v3Position, const Vector4D& v4Color);
};

using LinesVertex = SLinesVertex;

struct SUIVertex
{
	Vector2D m_v2Position;
	Vector2D m_v2TexCoords;
	Vector4D m_v4Color;			// Color

	SUIVertex();
	SUIVertex(float fX, float fY);
	SUIVertex(float fX, float fY, float fU, float fV);
	SUIVertex(float fX, float fY, float fU, float fV, float fR, float fG, float fB, float fA);
	SUIVertex(const Vector2D& v2Position);
	SUIVertex(const Vector2D& v2Position, const Vector2D& v2TexCoords);
	SUIVertex(const Vector2D& v2Position, const Vector2D& v2TexCoords, const Vector4D& v4Color);
};

using UIVertex = SUIVertex;

#define MAX_BONE_INFLUENCE 4

typedef struct SSkeletalMeshVertex
{
	// Basic geometry
	SVector3Df position;     // vec3
	SVector3Df normal;       // vec3
	SVector2Df texCoord;     // vec2

	// For normal mapping / PBR
	SVector3Df tangent;      // vec3
	SVector3Df bitangent;    // vec3

	// Optional per-vertex color
	SVector4Df color;        // vec4 (RGBA)

	// Skinning (up to 4 bones)
	int32_t boneIndices[MAX_BONE_INFLUENCE];  // indices into a bone palette
	float   boneWeights[MAX_BONE_INFLUENCE];  // weights, usually sum to 1.0

	SSkeletalMeshVertex() = default;

	SSkeletalMeshVertex(const SVector3Df& pos,
		const SVector3Df& nrm,
		const SVector2Df& uv,
		const SVector3Df& tang = SVector3Df(0.f, 0.f, 0.f),
		const SVector3Df& bitang = SVector3Df(0.f, 0.f, 0.f),
		const SVector4Df& col = SVector4Df(1.f, 1.f, 1.f, 1.f),
		const int32_t* boneIdx = nullptr,
		const float* boneWgt = nullptr)
		: position(pos)
		, normal(nrm)
		, texCoord(uv)
		, tangent(tang)
		, bitangent(bitang)
		, color(col)
	{
		// Initialize skinning data
		for (int32_t i = 0; i < 4; ++i)
		{
			boneIndices[i] = boneIdx ? boneIdx[i] : 0;
			boneWeights[i] = boneWgt ? boneWgt[i] : 0.0f;
		}
	}

} TMeshVertex;