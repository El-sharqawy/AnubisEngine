#include "MathPCH.h"
#include "EngineMath.h"
#include "PrespectiveProjection.h"
#include "OrthographicProjection.h"
#include <algorithm>

#include "Ray.h"
#include "EngineMathVectors.h"

/**
 * @brief Provides a const pointer to the underlying float array.
 *
 * This allows direct access to the components as a float array.
 *
 * @return A const pointer to the float array.
 */
const float* EngineMath::value_ptr(const SVector2Df& vec2)
{
	return (&(vec2.x));
}

const float* EngineMath::value_ptr(const SVector3Df& vec3)
{
	return (&(vec3.x));
}
const float* EngineMath::value_ptr(const SVector4Df& vec4)
{
	return (&(vec4.x));
}

const float* EngineMath::value_ptr(const SMatrix2x2& mat2)
{
	return (&(mat2[0].x));
}

const float* EngineMath::value_ptr(const SMatrix3x3& mat3)
{
	return (&(mat3[0].x));
}

const float* EngineMath::value_ptr(const SMatrix4x4& mat4)
{
	return (&(mat4[0].x));
}

const float* EngineMath::value_ptr(const SQuaternion& quat)
{
	return (&(quat.w));
}

/**
 * Provides a pointer to the underlying float array.
 *
 * This allows direct access to the components as a float array.
 *
 * @return A pointer to the float array.
 */
float* EngineMath::value_ptr(SVector2Df& vec2)
{
	return (&(vec2.x));
}

float* EngineMath::value_ptr(SVector3Df& vec3)
{
	return (&(vec3.x));
}
float* EngineMath::value_ptr(SVector4Df& vec4)
{
	return (&(vec4.x));
}

float* EngineMath::value_ptr(SMatrix2x2& mat2)
{
	return (&(mat2[0].x));
}

float* EngineMath::value_ptr(SMatrix3x3& mat3)
{
	return (&(mat3[0].x));
}

float* EngineMath::value_ptr(SMatrix4x4& mat4)
{
	return (&(mat4[0].x));
}

float* EngineMath::value_ptr(SQuaternion& quat)
{
	return (&(quat.w));
}

/**
 * @brief Converts degrees to radians.
 *
 * @param fDegrees The angle in degrees.
 * @return The angle in radians.
 */
float EngineMath::ToRadians(const float fDegrees)
{
	float fAngleRad = ((fDegrees) * static_cast<float>(M_PI) / 180.0f);
	return (fAngleRad);
}

/**
 * @brief Converts radians to degrees.
 *
 * @param fRadians The angle in radians.
 * @return The angle in degrees.
 */
float EngineMath::ToDegrees(const float fRadians)
{
	float fAngleDeg = ((fRadians) * 180.0f / static_cast<float>(M_PI));
	return (fAngleDeg);
}

/**
 * @brief Tests for intersection between a ray and a triangle using the Mِller–Trumbore algorithm.
 *
 * This function checks if the given ray intersects with the triangle defined by the vertices v0, v1, and v2.
 * If an intersection occurs, it calculates the distance 't' from the ray's origin to the intersection point.
 *
 * @param ray The ray to test for intersection.
 * @param v0 The first vertex of the triangle.
 * @param v1 The second vertex of the triangle.
 * @param v2 The third vertex of the triangle.
 * @param t Output parameter that will hold the distance from the ray origin to the intersection point if an intersection occurs.
 * @return True if the ray intersects the triangle; otherwise, false.
 */
bool EngineMath::IntersectRayTriangle(const SRay& ray, const SVector3Df& v0, const SVector3Df& v1, const SVector3Df& v2, float& outDistance)
{
	const float EPSILON = 0.0000001f;
	// --- STEP 1: Calculate the "Edges" of the triangle ---
	// These vectors represent two sides of the triangle starting from v0.
	Vector3D edge1 = v1 - v0;
	Vector3D edge2 = v2 - v0;

	// --- STEP 2: Start calculating the Determinant ---
	// We use the Cross Product to find a vector perpendicular to the ray and one edge.

	Vector3D h = EngineMath::Cross(ray.GetDirection(), edge2);

	// The Dot Product here tells us how "parallel" the ray is to the triangle.
	float det = EngineMath::Dot(edge1, h);

	// If the determinant is near zero, the ray is lying in the plane of the 
	// triangle or is perfectly parallel to it. No intersection.
	if (det > -EPSILON && det < EPSILON)
	{
		return (false);
	}

	// --- STEP 3: Calculate the 'U' Barycentric Coordinate ---
	// This tells us how far the hit point is from v0 along edge1.
	float invDet = 1.0f / det;
	Vector3D s = ray.GetOrigin() - v0;
	float u = invDet * EngineMath::Dot(s, h);

	// If U is outside [0, 1], the hit point is outside the triangle's boundaries.
	if (u < 0.0f || u > 1.0f)
	{
		return (false);
	}

	// --- STEP 4: Calculate the 'V' Barycentric Coordinate ---
	// This tells us how far the hit point is from v0 along edge2.
	Vector3D q = EngineMath::Cross(s, edge1);
	float v = invDet * EngineMath::Dot(ray.GetDirection(), q);

	// If V is negative, or U+V > 1, the point is outside the triangle.
	if (v < 0.0f || u + v > 1.0f)
	{
		return (false);
	}

	// --- STEP 5: Calculate 'T' (The distance along the ray) ---
	// If we reach here, the ray definitely hits the triangle!
	// We just need to know how far away it is.
	float t = invDet * EngineMath::Dot(edge2, q);

	// Only return true if the hit is in front of the camera (t > 0)
	// and within our ray's maximum travel distance.
	if (t > EPSILON && t < ray.GetMaxDistance())
	{
		outDistance = t;
		return (true);
	}

	return (false);
}

/**
 * @brief Performs linear interpolation between two 3D vectors.
 *
 * This function calculates a point along the line connecting v3Start and v3End
 * based on the interpolation factor t. When t=0, the result is v3Start;
 * when t=1, the result is v3End; values in between yield points along the line.
 *
 * @param v3Start The starting vector.
 * @param v3End The ending vector.
 * @param t The interpolation factor, typically in the range [0, 1].
 * @return The interpolated vector.
 */
SVector3Df EngineMath::Vec3Lerp(const SVector3Df& v3Start, const SVector3Df& v3End, float t)
{
	return((1.0f - t) * v3Start + t * v3End);
	//return (v3Start + t * (v3End - v3Start));
}
