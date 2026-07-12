#pragma once

struct SVector2Df;
struct SVector3Df;
struct SVector4Df;

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
	//////////////////////////////////////////////////////////////////
	/////////////////////////////VECTOR2//////////////////////////////
	//////////////////////////////////////////////////////////////////

	/**
	 * float Calculates the squared length (magnitude) of the SVector2Df object.
	 *
	 * @param vec The SVector2Df object to calculate it's squared length.
	 * @return The squared length of the vector.
	 */
	float LengthSquared(const SVector2Df& v3Vec);

	/**
	 * float Calculates the length (magnitude) of the SVector2Df object.
	 *
	 * @param vec The SVector2Df object to calculate it's length.
	 * @return The length of the vector.
	 */
	float Length(const SVector2Df& v2Vec);

	/**
	 * float Normalizes the SVector2Df object, making its length 1.
	 *
	 * @param vec The SVector2Df object to normalize.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df Normalize(const SVector2Df& vec);

	/**
	 * float Calculates the dot product of two SVector2Df objects.
	 *
	 * @param vec1 The SVector2Df object to calculate the dot product with.
	 * @param vec2 The SVector2Df object to calculate the dot product with.
	 * @return The dot product of the two vectors.
	 */
	float Dot(const SVector2Df& vec1, const SVector2Df& vec2);

	/**
	 * @brief Calculates the Euclidean distance between two SVector2Df objects.
	 *
	 * @param vec1 The SVector2Df object to calculate the distance from.
	 * @param vec2 The SVector2Df object to calculate the distance to.
	 * @return The distance between the two vectors.
	 */
	float Distance(const SVector2Df& vec1, const SVector2Df& vec2);

	//////////////////////////////////////////////////////////////////
	/////////////////////////////VECTOR3//////////////////////////////
	//////////////////////////////////////////////////////////////////

	/**
	 * float Calculates the squared length (magnitude) of the SVector3Df object.
	 *
	 * @param vec The SVector3Df object to calculate it's squared length.
	 * @return The squared length of the vector.
	 */
	float LengthSquared(const SVector3Df& v3Vec);

	/**
	 * float Calculates the length (magnitude) of the SVector3Df object.
	 *
	 * @param vec The SVector3Df object to calculate it's length.
	 * @return The length of the vector.
	 */
	float Length(const SVector3Df& v3Vec);

	/**
	 * float Normalizes the SVector3Df object, making its length 1.
	 *
	 * @param vec The SVector3Df object to normalize.
	 * @return the modified SVector3Df object.
	 */
	SVector3Df Normalize(const SVector3Df& vec);

	/**
	 * float Calculates the dot product of two SVector3Df objects.
	 *
	 * @param vec1 The SVector3Df object to calculate the dot product with.
	 * @param vec2 The SVector3Df object to calculate the dot product with.
	 * @return The dot product of the two vectors.
	 */
	float Dot(const SVector3Df& vec1, const SVector3Df& vec2);

	/**
	 * @brief Calculates the cross product of two SVector3Df objects.
	 *
	 * @param vec1 The first SVector3Df object to calculate the cross product with.
	 * @param vec2 The second SVector3Df object to calculate the cross product with.
	 * @return The cross product of the two vectors.
	 */
	SVector3Df Cross(const SVector3Df& vec1, const SVector3Df& vec2);

	/**
	 * @brief Calculates the Euclidean distance between two SVector3Df objects.
	 *
	 * @param vec1 The SVector3Df object to calculate the distance from.
	 * @param vec2 The SVector3Df object to calculate the distance to.
	 * @return The distance between the two vectors.
	 */
	float Distance(const SVector3Df& vec1, const SVector3Df& vec2);

	/**
	 * @brief Calculates the squared Euclidean distance between two SVector3Df objects.
	 *
	 * @param vec1 The SVector3Df object to calculate the distance from.
	 * @param vec2 The SVector3Df object to calculate the distance to.
	 * @return The squared distance between the two vectors.
	 */
	float DistanceSquared(const SVector3Df& vec1, const SVector3Df& vec2);

	/**
	 * float Calculates the Angle between two SVector3Df objects.
	 *
	 * @param vec1 The first SVector3Df object to calculate the angle0 with.
	 * @param vec2 The second SVector3Df object to calculate the angle0 with.
	 * @return The angle between the two vectors.
	 */
	float Angle(const SVector3Df& vec1, const SVector3Df& vec2);

	/**
	 * Rotates the vector using quaternion by giving angle and vector.
	 *
	 * @param fAngle The Angle of Rotation.
	 * @param vec The vector to rotate around, could represent an Axis
	 * @return The rotated vector
	 */
	SVector3Df Rotate(const SVector3Df& vec, const float fAngle, const SVector3Df& v3Axis);

	//////////////////////////////////////////////////////////////////
	/////////////////////////////VECTOR4//////////////////////////////
	//////////////////////////////////////////////////////////////////
	/**
	 * float Calculates the squared length (magnitude) of the SVector4Df object.
	 *
	 * @param v4Vec The SVector4Df object to calculate it's squared length.
	 * @return The squared length of the vector.
	 */
	float LengthSquared(const SVector4Df& v4Vec);

	/**
	 * float Calculates the length (magnitude) of the SVector4Df object.
	 *
	 * @param v4Vec The SVector4Df object to calculate it's length.
	 * @return The length of the vector.
	 */
	float Length(const SVector4Df& v4Vec);

	/**
	 * float Normalizes the SVector4Df object, making its length 1.
	 *
	 * @param vec The SVector4Df object to normalize.
	 * @return the modified SVector4Df object.
	 */
	SVector4Df Normalize(const SVector4Df& vec);

	/**
	 * float Calculates the dot product of two SVector4Df objects.
	 *
	 * @param vec1 The SVector4Df object to calculate the dot product with.
	 * @param vec2 The SVector4Df object to calculate the dot product with.
	 * @return The dot product of the two vectors.
	 */
	float Dot(const SVector4Df& vec1, const SVector4Df& vec2);

	/**
	 * @brief Calculates the Euclidean distance between two SVector4Df objects.
	 *
	 * @param vec1 The SVector4Df object to calculate the distance from.
	 * @param vec2 The SVector4Df object to calculate the distance to.
	 * @return The distance between the two vectors.
	 */
	float Distance(const SVector4Df& vec1, const SVector4Df& vec2);

	/**
	 * @brief Linearly interpolates between two Vector3D objects (equivalent to glm::mix).
	 *
	 * @param vec1 The starting Vector3D object (returned when interpolator is 0.0f).
	 * @param vec2 The ending Vector3D object (returned when interpolator is 1.0f).
	 * @param interpolator The blend factor, clamped or expected between 0.0f and 1.0f.
	 * @return The interpolated Vector3D between the two input vectors.
	 */
	Vector3D Mix(const Vector3D& vec1, const Vector3D& vec2, float interpolator);

	Vector3D Lerp(const Vector3D& a, const Vector3D& b, float alpha);

} // namespace EngineMath - Vector Part