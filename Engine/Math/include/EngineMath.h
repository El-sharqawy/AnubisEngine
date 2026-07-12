#pragma once

/**
 * @file EngineMath.h
 * @brief Core linear algebra and graphics utility functions.
 *
 * This file contains all essential mathematical structures (vectors, matrices, quaternions)
 * and corresponding operations (transformations, normalization, geometry calculations)
 * designed specifically for the engine's 3D graphics and physics systems.
 *
 * All functions are implemented to be highly optimized and concise.
 */

struct SVector2Df;
struct SVector3Df;
struct SVector4Df;
struct SMatrix2x2;
struct SMatrix3x3;
struct SMatrix4x4;
struct SQuaternion;
struct SRay;

/**
 * @brief Provides core linear algebra and graphics utility functions.
 *
 * This namespace contains all essential mathematical structures (vectors, matrices, quaternions)
 * and corresponding operations (transformations, normalization, geometry calculations)
 * designed specifically for the engine's 3D graphics and physics systems.
 *
 * All functions are implemented to be highly optimized and concise.
 */
namespace EngineMath
{
	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	const float* value_ptr(const SVector2Df& vec2);
	const float* value_ptr(const SVector3Df& vec3);
	const float* value_ptr(const SVector4Df& vec4);
	const float* value_ptr(const SMatrix2x2& mat2);
	const float* value_ptr(const SMatrix3x3& mat3);
	const float* value_ptr(const SMatrix4x4& mat4);
	const float* value_ptr(const SQuaternion& quat);

	/**
	 * Provides a pointer to the underlying float array.
	 *
	 * This allows direct access to the components as a float array.
	 *
	 * @return A pointer to the float array.
	 */
	float* value_ptr(SVector2Df& vec2);
	float* value_ptr(SVector3Df& vec3);
	float* value_ptr(SVector4Df& vec4);
	float* value_ptr(SMatrix2x2& mat2);
	float* value_ptr(SMatrix3x3& mat3);
	float* value_ptr(SMatrix4x4& mat4);
	float* value_ptr(SQuaternion& quat);

	/**
	 * @brief Converts degrees to radians.
	 *
	 * @param fDegrees The angle in degrees.
	 * @return The angle in radians.
	 */
	float ToRadians(const float fDegrees);

	/**
	 * @brief Converts radians to degrees.
	 *
	 * @param fRadians The angle in radians.
	 * @return The angle in degrees.
	 */
	float ToDegrees(const float fRadians);

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
	 * @param outDistance Output parameter that will hold the distance from the ray origin to the intersection point if an intersection occurs.
	 * @return True if the ray intersects the triangle; otherwise, false.
	 */
	bool IntersectRayTriangle(const SRay& ray, const SVector3Df& v0, const SVector3Df& v1, const SVector3Df& v2, float& outDistance);

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
	SVector3Df Vec3Lerp(const SVector3Df& v3Start, const SVector3Df& v3End, float t);

	/**
	 * @brief Gets the size of a static array.
	 *
	 * This function calculates the number of elements in a static array type T.
	 *
	 * @tparam T The static array type.
	 * @return The number of elements in the array.
	 */
	template <typename T>
	size_t GetArraySize()
	{
		return (sizeof(T) / sizeof(T[0]));
	}

} // namespace EngineMath - General Part