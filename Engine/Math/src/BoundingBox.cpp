#include "MathPCH.h"
#include "BoundingBox.h"
#include <limits>

SBoundingSphere::SBoundingSphere()
{
	v3Center = SVector3Df(0.0f);
	fRadius = 0.0f;
}

SBoundingBox::SBoundingBox()
{
	Reset();
}

SBoundingBox::SBoundingBox(const SVector3Df& v3MinVal, const SVector3Df& v3MaxVal)
{
	v3Min = v3MinVal;
	v3Max = v3MaxVal;
	v3Center = 0.0f;
	v3Size = 0.0f;
	memset(v3Corners, 0, sizeof(v3Corners));
}

void SBoundingBox::Reset()
{
	v3Min = SVector3Df(FLT_MAX, FLT_MAX, FLT_MAX);
	v3Max = SVector3Df(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	v3Center = 0.0f;
	v3Size = 0.0f;
}

void SBoundingBox::ExpandToInclude(const SVector3Df& v3Point)
{
	v3Min.x = std::fmin(v3Min.x, v3Point.x);
	v3Min.y = std::fmin(v3Min.y, v3Point.y);
	v3Min.z = std::fmin(v3Min.z, v3Point.z);

	v3Max.x = std::fmax(v3Max.x, v3Point.x);
	v3Max.y = std::fmax(v3Max.y, v3Point.y);
	v3Max.z = std::fmax(v3Max.z, v3Point.z);
}

void SBoundingBox::ComputeBox()
{
	v3Center = (v3Min + v3Max) * 0.5f;
	v3Size = v3Max - v3Min;
}

// Check if point is inside the bounding box
bool SBoundingBox::Contains(const SVector3Df& point) const
{
	return point.x >= v3Min.x && point.x <= v3Max.x &&
		point.y >= v3Min.y && point.y <= v3Max.y &&
		point.z >= v3Min.z && point.z <= v3Max.z;
}

// Returns the center of the bounding box
SVector3Df SBoundingBox::GetCenter()
{
	v3Center = (v3Min + v3Max) * 0.5f;
	return (v3Center);
}

// Returns the size (extent) of the bounding box
SVector3Df SBoundingBox::GetSize()
{
	v3Size = v3Max - v3Min;
	return (v3Size);
}

// Move Bounding Box by a given offset in world space
SBoundingBox SBoundingBox::MoveBox(const SVector3Df& offset) const
{
	return SBoundingBox(v3Min + offset, v3Max + offset);
}

// Returns the 8 corners of the box in model space
void SBoundingBox::SetCorners()
{
    v3Corners[0] = SVector3Df(v3Min.x, v3Min.y, v3Min.z);
    v3Corners[1] = SVector3Df(v3Max.x, v3Min.y, v3Min.z);
    v3Corners[2] = SVector3Df(v3Max.x, v3Max.y, v3Min.z);
    v3Corners[3] = SVector3Df(v3Min.x, v3Max.y, v3Min.z);
    v3Corners[4] = SVector3Df(v3Min.x, v3Min.y, v3Max.z);
    v3Corners[5] = SVector3Df(v3Max.x, v3Min.y, v3Max.z);
    v3Corners[6] = SVector3Df(v3Max.x, v3Max.y, v3Max.z);
    v3Corners[7] = SVector3Df(v3Min.x, v3Max.y, v3Max.z);
}

// Transforms the bounding box by a matrix and returns the world-space AABB
SBoundingBox SBoundingBox::Transform(const Matrix4& mat)
{
	SetCorners();

	// Transform all corners
	for (int32_t i = 0; i < 8; ++i)
	{
		v3Corners[i] = mat.TransformPoint(v3Corners[i]);
	}

	// Compute new AABB in world space
	SVector3Df min = v3Corners[0];
	SVector3Df max = v3Corners[0];
	for (int32_t i = 1; i < 8; ++i)
	{
		min.x = std::fmin(min.x, v3Corners[i].x);
		min.y = std::fmin(min.y, v3Corners[i].y);
		min.z = std::fmin(min.z, v3Corners[i].z);
		max.x = std::fmax(max.x, v3Corners[i].x);
		max.y = std::fmax(max.y, v3Corners[i].y);
		max.z = std::fmax(max.z, v3Corners[i].z);
	}
	return SBoundingBox(min, max);
}

void SBoundingBox::Scale(const SVector3Df& scaleFactor)
{
	// Multiply the min and max extents by the scale factor
	v3Min = v3Min * scaleFactor;
	v3Max = v3Max * scaleFactor;
}

bool SBoundingBox::Intersects(const SBoundingBox& other) const
{
	return (v3Min.x <= other.v3Max.x && v3Max.x >= other.v3Min.x) &&
		(v3Min.y <= other.v3Max.y && v3Max.y >= other.v3Min.y) &&
		(v3Min.z <= other.v3Max.z && v3Max.z >= other.v3Min.z);
}
