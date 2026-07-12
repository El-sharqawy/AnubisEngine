#pragma once

#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include <vulkan/vulkan.h>
#include <vector>

#define MAX_BONE_INFLUENCE 4

struct SAnimatedMeshVertex
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

	SAnimatedMeshVertex() = default;

	SAnimatedMeshVertex(const SVector3Df& pos,
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

	static VkVertexInputBindingDescription GetBindingDescription()
	{
		VkVertexInputBindingDescription bindingDescription{};
		bindingDescription.binding = 0;
		bindingDescription.stride = sizeof(SAnimatedMeshVertex);
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
		return (bindingDescription);
	}

	static std::vector<VkVertexInputAttributeDescription> GetAttributeDescriptions()
	{
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
		attributeDescriptions.resize(8); // 8 attributes

		attributeDescriptions[0].binding = 0;
		attributeDescriptions[0].location = 0;
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[0].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::position);

		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::normal);

		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::texCoord);

		attributeDescriptions[3].binding = 0;
		attributeDescriptions[3].location = 3;
		attributeDescriptions[3].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[3].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::tangent);

		attributeDescriptions[4].binding = 0;
		attributeDescriptions[4].location = 4;
		attributeDescriptions[4].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[4].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::bitangent);

		attributeDescriptions[5].binding = 0;
		attributeDescriptions[5].location = 5;
		attributeDescriptions[5].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[5].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::color);

		attributeDescriptions[6].binding = 0;
		attributeDescriptions[6].location = 6;
		attributeDescriptions[6].format = VK_FORMAT_R32G32B32A32_SINT;
		attributeDescriptions[6].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::boneIndices);

		attributeDescriptions[7].binding = 0;
		attributeDescriptions[7].location = 7;
		attributeDescriptions[7].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[7].offset = offsetof(SAnimatedMeshVertex, SAnimatedMeshVertex::boneWeights);

		return (attributeDescriptions);
	}

};