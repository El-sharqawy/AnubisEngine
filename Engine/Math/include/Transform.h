#pragma once

#include "TypeVector3.h"
#include "TypeMatrix4.h"
#include "Quaternion.h"

/** 
 * @brief Structure representing a 3D transformation (position, rotation, scale).
 */
struct STransform
{
	Vector3D m_v3Position;	// Translation vector
	Quaternion m_qOrientation;	// Quaternion representation of rotation
	Vector3D m_v3Scale;		// Scale factors along each axis

	mutable bool m_bMatrixDirty = true; // Flag indicating if the cached matrix needs updating
	mutable Matrix4 m_CachedMatrix; // Cached transformation matrix

	/** 
	 * @brief Default constructor.
	 *
	 * Initializes the transform components using default member initialization.
	 * This typically results in zero-initialized position and scale, and an identity quaternion for orientation.
	 */
	STransform();

	/** 
	 * @brief Constructor to initialize all components of the transform. 
	 *
	 * @param v3Position The position vector.
	 * @param qOrientation The orientation quaternion.
	 * @param v3Scale The scale vector.
	 */
	STransform(const Vector3D& v3Position, const Quaternion& qOrientation, const Vector3D& v3Scale);

	/** 
	 * @brief Sets the position component of the transform.
	 *
	 * @param v3Position The new position vector.
	 */
	void SetPosition(const Vector3D& v3Position);

	/**
	 * @brief Sets the orientation component of the transform.
	 *
	 * @param qOrientation The new orientation quaternion.
	 */
	void SetOrientation(const Quaternion& qOrientation);
	void SetRotation(const Quaternion& qOrientation);

	/**
	 * @brief Sets the scale component of the transform.
	 *
	 * @param v3Scale The new scale vector.
	 */
	void SetScale(const Vector3D& v3Scale);

	/** 
	 * @brief Rotates the transform around a specified axis by a given angle.
	 *
	 * This function creates a rotation quaternion based on the provided axis
	 * and angle, then applies it to the current orientation.
	 *
	 * @param v3Axis The axis to rotate around (should be a normalized vector).
	 * @param fDegrees The angle to rotate by, in degrees.
	 */
	void RotateAroundAxis(const Vector3D& v3Axis, float fDegrees);

	/**
	 * @brief Translates the transform by a given offset vector.
	 *
	 * @param v3Offset The translation offset vector.
	 */
	void Translate(const Vector3D& v3Offset);

	/**
	 * @brief Rotates the transform by specified Euler angles (in degrees).
	 *
	 * @param v3EulerAnglesDegrees The Euler angles for rotation around the Z, Y, and X axes, in degrees.
	 */
	void Rotate(const Vector3D& v3EulerAnglesDegrees);

	/**
	 * @brief Scales the transform by specified scale factors along each axis.
	 *
	 * @param v3ScaleFactor The scale factors for the X, Y, and Z axes.
	 */
	void Scale(const Vector3D& v3ScaleFactor);

	/**
	 * @brief Computes and returns the combined transformation matrix.
	 *
	 * This matrix incorporates translation, rotation, and scaling.
	 *
	 * @return The resulting 4x4 transformation matrix.
	 */
	Matrix4 GetMatrix() const;

	/** 
	 * @brief Assignment operator.
	 *
	 * Copies the components from another STransform instance.
	 *
	 * @param other The other STransform instance to copy from.
	 * @return Reference to this STransform instance after assignment.
	 */
	STransform operator = (const STransform& other);
};

