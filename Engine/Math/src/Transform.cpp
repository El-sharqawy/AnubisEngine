#include "MathPCH.h"
#include "Transform.h"
#include "EngineMathVectors.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"

/*
* @brief Default constructor.
*
* Initializes the transform components using default member initialization.
* This typically results in zero - initialized position and scale, and an identity quaternion for orientation.
*/
STransform::STransform()
{
	m_v3Position = Vector3D(0.0f, 0.0f, 0.0f);
	m_qOrientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
	m_v3Scale = Vector3D(1.0f, 1.0f, 1.0f);
	m_CachedMatrix = Matrix4(1.0f);
	m_bMatrixDirty = true;
}

/**
* @brief Constructor to initialize all components of the transform.
*
* @param v3Position The position vector.
* @param qOrientation The orientation quaternion.
* @param v3Scale The scale vector.
*/
STransform::STransform(const Vector3D& v3Position, const Quaternion& qOrientation, const Vector3D& v3Scale)
{
	m_v3Position = v3Position;
	m_qOrientation = qOrientation;
	m_v3Scale = v3Scale;
	m_CachedMatrix = Matrix4(1.0f);
	m_bMatrixDirty = true;
}

/**
*@brief Sets the position component of the transform.
*
* @param v3Position The new position vector.
*/
void STransform::SetPosition(const Vector3D& v3Position)
{
	m_v3Position = v3Position;
	m_bMatrixDirty = true;
}

/**
 * @brief Sets the orientation component of the transform.
 *
 * @param qOrientation The new orientation quaternion.
 */
void STransform::SetOrientation(const Quaternion& qOrientation)
{
	m_qOrientation = EngineMath::Normalize(qOrientation);
	m_bMatrixDirty = true;
}

void STransform::SetRotation(const Quaternion& qOrientation)
{
	SetOrientation(qOrientation);
	m_bMatrixDirty = true;
}

/**
 * @brief Sets the scale component of the transform.
 *
 * @param v3Scale The new scale vector.
 */
void STransform::SetScale(const Vector3D& v3Scale)
{
	m_v3Scale = v3Scale;
	m_bMatrixDirty = true;
}

/**
 * @brief Rotates the transform around a specified axis by a given angle.
 *
 * This function creates a rotation quaternion based on the provided axis
 * and angle, then applies it to the current orientation.
 *
 * @param v3Axis The axis to rotate around (should be a normalized vector).
 * @param fDegrees The angle to rotate by, in degrees.
 */
void STransform::RotateAroundAxis(const Vector3D& v3Axis, float fDegrees)
{
	float angleRad = EngineMath::ToRadians(fDegrees);
	Vector3D Axis = EngineMath::Normalize(v3Axis);
	Quaternion qDelta = EngineMath::FromAxisAngle(Axis, angleRad, false);
	m_qOrientation = EngineMath::Normalize(qDelta * m_qOrientation);

	m_bMatrixDirty = true;
}

/**
 * @brief Translates the transform by a given offset vector.
 *
 * @param v3Offset The translation offset vector.
 */
void STransform::Translate(const Vector3D& v3Offset)
{
	m_v3Position += v3Offset;
	m_bMatrixDirty = true;
}

/**
 * @brief Rotates the transform by specified Euler angles (in degrees).
 *
 * @param v3EulerAnglesDegrees The Euler angles for rotation around the Z, Y, and X axes, in degrees.
 */
void STransform::Rotate(const Vector3D& v3EulerAnglesDegrees)
{
	STransform tempTransform{};
	tempTransform.m_qOrientation = EngineMath::FromEulerZYX(v3EulerAnglesDegrees);
	m_qOrientation = EngineMath::Normalize(tempTransform.m_qOrientation * m_qOrientation);
	m_bMatrixDirty = true;
}

/**
 * @brief Scales the transform by specified scale factors along each axis.
 *
 * @param v3ScaleFactor The scale factors for the X, Y, and Z axes.
 */
void STransform::Scale(const Vector3D& v3ScaleFactor)
{
	m_v3Scale.x *= v3ScaleFactor.x;
	m_v3Scale.y *= v3ScaleFactor.y;
	m_v3Scale.z *= v3ScaleFactor.z;
	m_bMatrixDirty = true;
}


/**
 * @brief Computes and returns the combined transformation matrix.
 *
 * This matrix incorporates translation, rotation, and scaling.
 *
 * @return The resulting 4x4 transformation matrix.
 */
Matrix4 STransform::GetMatrix() const
{
	if (m_bMatrixDirty)
	{
		Matrix4 matTranslation{}, matRotation{}, matScale{};

		matTranslation = EngineMath::Translate(m_v3Position);
		matRotation = EngineMath::ToMatrix4(m_qOrientation);
		matScale = EngineMath::Scale(m_v3Scale);

		m_CachedMatrix = matTranslation * matRotation * matScale;
		m_bMatrixDirty = false;
	}

	return (m_CachedMatrix);
}

/*
*@brief Assignment operator.
*
* Copies the components from another STransform instance.
*
* @param other The other STransform instance to copy from.
* @return Reference to this STransform instance after assignment.
*/
STransform STransform::operator=(const STransform& other)
{
	m_v3Position = other.m_v3Position;
	m_qOrientation = other.m_qOrientation;
	m_v3Scale = other.m_v3Scale;
	m_bMatrixDirty = true;
	return (*this);
}
