#include "MathPCH.h"
#include "Ray.h"
#include "EngineMathMatrix.h"

/**
 * @brief Parameterized constructor.
 *
 * @param origin The starting point of the ray.
 * @param direction The direction vector of the ray (should be normalized).
 * @param maxDistance The maximum distance the ray can travel.
 */
SRay::SRay(const Vector3D& origin, const Vector3D& direction, float maxDistance)
	: v3Origin(origin), v3Direction(direction), fMaxDistance(maxDistance)
{
	v3Direction.normalize();
}

/**
 * @brief Sets the origin of the ray.
 *
 * @param origin The new origin point for the ray.
 */
void SRay::SetOrigin(const Vector3D& origin)
{
	v3Origin = origin;
}

/**
 * @brief Retrieves the origin of the ray.
 *
 * @return The origin point of the ray.
 */
Vector3D SRay::GetOrigin() const
{
	return v3Origin;
}

/**
 * @brief Sets the direction of the ray.
 *
 * @param direction The new direction vector for the ray (should be normalized).
 */
void SRay::SetDirection(const Vector3D& direction)
{
	v3Direction = direction;
	v3Direction.normalize();
}

/**
 * @brief Retrieves the direction of the ray.
 *
 * @return The direction vector of the ray.
 */
Vector3D SRay::GetDirection() const
{
	return (v3Direction);
}

/**
 * @brief Sets the maximum distance the ray can travel.
 *
 * @param maxDistance The maximum distance of the ray.
 */
void SRay::SetMaxDistance(float maxDistance)
{
	fMaxDistance = maxDistance;
}

/**
 * @brief Retrieves the maximum distance the ray can travel.
 *
 * @return The maximum distance of the ray.
 */
float SRay::GetMaxDistance() const
{
	return fMaxDistance;
}

/**
 * @brief Calculates a point along the ray at a specified distance from the origin.
 *
 * @param distance The distance from the origin along the ray's direction.
 * @return The calculated point as a Vector3D.
 */
Vector3D SRay::GetPointAtDistance(float distance) const
{
	return (v3Origin + (distance * v3Direction));
}

/**
 * @brief Calculates the end point of the ray based on its origin, direction, and maximum distance.
 *
 * @return The end point of the ray as a Vector3D.
 */
Vector3D SRay::GetEndPoint() const
{
	return (v3Origin + (fMaxDistance * v3Direction));
}

/**
 * @brief Checks if a distance 't' is within the valid range of this ray.
 * Useful for intersection tests.
 *
 * @param t The distance to check.
 * @return true if 't' is between 0 and fMaxDistance, false otherwise.
 */
bool SRay::IsValidDistance(float t) const
{
	return (t >= 0.0f && t <= fMaxDistance);
}

/**
 * @brief Re-normalizes the direction vector.
 * Call this if you've modified the direction manually.
 */
void SRay::Normalize()
{
	// Re-normalize the direction vector
	float lengthSq = v3Direction.lengthSquared(); // Calculate squared length

	// Only normalize if length is non-zero and not already normalized
	if (lengthSq > 0.0f && lengthSq != 1.0f)
	{
		v3Direction.normalize();
	}
}

/**
 * @brief Transforms the ray by a given 4x4 matrix.
 *
 * @param matrix The transformation matrix to apply.
 */
void SRay::Transform(const Matrix4& matrix)
{
	v3Origin = EngineMath::TransformPoint(matrix, v3Origin);
	v3Direction = EngineMath::TransformDirection(matrix, v3Direction);
	v3Direction.normalize();
}
