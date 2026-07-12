#pragma once

#include "TypeVector3.h"
#include "TypeMatrix4.h"

/**
 * @brief SRay: A structure representing a 3D ray with an origin, direction, and maximum distance.
 *
 * This structure encapsulates the properties of a ray in 3D space, including its starting point (origin),
 * its direction (a normalized vector), and the maximum distance it can travel. It is commonly used in
 * graphics programming, physics simulations, and ray tracing algorithms.
 */
struct SRay
{
	SVector3Df v3Origin{ 0.0f, 0.0f, 0.0f }; // Origin point of the ray
	SVector3Df v3Direction{ 0.0f, 0.0f, -1.0f }; // Normalized direction vector
	float fMaxDistance{ 10000.0f }; // Use this instead of an End Point, 10 Kilometers by default

	/**
	* @brief Default constructor.
	*/
    SRay() = default;
	/**
	 * @brief Parameterized constructor.
	 * 
	 * @param origin The starting point of the ray.
	 * @param direction The direction vector of the ray (should be normalized).
	 * @param maxDistance The maximum distance the ray can travel.
	 */
	SRay(const Vector3D& origin, const Vector3D& direction, float maxDistance = 10000.0f);

	/**
	 * @brief Copy constructor.
	 * 
	 * @param other The SRay instance to copy from.
	 */
	SRay(const SRay& other) = default;

	/**
	 * @brief Copy assignment operator.
	 * 
	 * @param other The SRay instance to assign from.
	 * @return Reference to the assigned SRay instance.
	 */
	SRay& operator=(const SRay& other) = default;

	/**
	 * @brief Sets the origin of the ray.
	 * 
	 * @param origin The new origin point for the ray.
	 */
	void SetOrigin(const Vector3D& origin);

	/**
	 * @brief Retrieves the origin of the ray.
	 * 
	 * @return The origin point of the ray.
	 */
	Vector3D GetOrigin() const;

	/**
	 * @brief Sets the direction of the ray.
	 * 
	 * @param direction The new direction vector for the ray (should be normalized).
	 */
	void SetDirection(const Vector3D& direction);

	/**
	 * @brief Retrieves the direction of the ray.
	 * 
	 * @return The direction vector of the ray.
	 */
	Vector3D GetDirection() const;

	/**
	 * @brief Sets the maximum distance the ray can travel.
	 *
	 * @param maxDistance The maximum distance of the ray.
	 */
	void SetMaxDistance(float maxDistance);

	/**
	 * @brief Retrieves the maximum distance the ray can travel.
	 *
	 * @return The maximum distance of the ray.
	 */
	float GetMaxDistance() const;

	/**
	 * @brief Calculates a point along the ray at a specified distance from the origin.
	 *
	 * @param distance The distance from the origin along the ray's direction.
	 * @return The calculated point as a Vector3D.
	 */
	Vector3D GetPointAtDistance(float distance) const;

	/**
	 * @brief Calculates the end point of the ray based on its origin, direction, and maximum distance.
	 *
	 * @return The end point of the ray as a Vector3D.
	 */
	Vector3D GetEndPoint() const;

	/**
	 * @brief Checks if a distance 't' is within the valid range of this ray.
	 * Useful for intersection tests.
	 * 
	 * @param t The distance to check.
	 * @return true if 't' is between 0 and fMaxDistance, false otherwise.
	 */
	bool IsValidDistance(float t) const;

	/**
	 * @brief Re-normalizes the direction vector.
	 * Call this if you've modified the direction manually.
	 */
	void Normalize();

	/**
	 * @brief Transforms the ray by a given 4x4 matrix.
	 * 
	 * @param matrix The transformation matrix to apply.
	 */
	void Transform(const Matrix4& matrix);
};