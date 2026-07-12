#include "MathPCH.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include <glm/gtx/quaternion.hpp>

/**
 * @brief Calculates the product of two quaternions (non-modifying).
 *
 * Computes the Hamilton product of q1 multiplied by q2, and returns the result
 * in a new quaternion instance. This is equivalent to q1 * q2.
 *
 * @param q1 The constant reference to the left-hand side quaternion (first operand).
 * @param q2 The constant reference to the right-hand side quaternion (second operand).
 * @return SQuaternion A new quaternion instance resulting from the multiplication.
 */
SQuaternion EngineMath::Multiply(const SQuaternion& q1, const SQuaternion& q2)
{
	SQuaternion result{};

	/*
	Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
			a*e - b*f - c*g - d*h
		+ i (b*e + a*f + c*h- d*g)
		+ j (a*g - b*h + c*e + d*f)
		+ k (a*h + b*g - c*f + d*e)
	*/

	result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	result.x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
	result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	result.z = q1.w * q2.z + q1.x * q1.y - q1.y * q2.x + q1.z * q2.w;

	result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w; // Corrected: Term changed from q1.x * q1.y to q1.x * q2.y

	return (result);
}

/**
 * @brief Calculates the squared length (or squared norm) of a quaternion.
 *
 * This function computes the squared magnitude of the quaternion. This is generally
 * preferred over calculating the actual length (using a square root) when only
 * comparisons are needed, as it is computationally faster.
 *
 * The formula calculated is: Length² = (w² + x² + y² + z²).
 * @param quat [in] The SQuaternion object for which the squared length is calculated.
 * @return float The squared length (w² + x² + y² + z²) of the quaternion.
 */
float EngineMath::LengthSquared(const SQuaternion& quat)
{
	return (quat.LengthSquared());
}

/**
 * @brief Calculates the quaternion magnitude (length) of a quaternion.
 *
 * Computes the Euclidean length (or magnitude) of the quaternion, defined as
 * the square root of the sum of the squares of its components: sqrt{x^2 + y^2 + z^2 + w^2}.
 *
 * @param quat [in] The SQuaternion object for which the squared length is calculated.
 *
 * @return float The magnitude (length) of the quaternion.
 */
float EngineMath::Length(const SQuaternion& quat)
{
	return (quat.Length());
}

/**
 * @brief Calculates the conjugate of a quaternion.
 *
 * The conjugate of a quaternion is found by negating its vector components (x, y, z).
 * If the original quaternion is ${q} = (w, x, y, z), the conjugate is
 * q = (w, -x, -y, -z).
 *
 * For unit quaternions, the conjugate is also equal to the inverse (q = q^-1).
 * @param quat [in] The SQuaternion object for which the conjugate is calculated.
 * @return SQuaternion A new quaternion object representing the conjugate.
 */
SQuaternion EngineMath::Conjugate(const SQuaternion& quat)
{
	SQuaternion result{};
	result.x = -quat.x;
	result.y = -quat.y;
	result.z = -quat.z;
	result.w = quat.w;
	return (result);
}

/**
 * @brief Normalizes a quaternion, ensuring its length (magnitude) is 1.0.
 *
 * Normalization is crucial for rotation quaternions, as it preserves the rotation
 * and prevents scaling or distortion over time due to floating-point errors.
 *
 * The formula is: q_normalized = q / ||q|| (Quaternion divided by its length).
 * @param quat [in] The SQuaternion object to be normalized.
 * @return SQuaternion A new unit quaternion (normalized) that represents the same rotation.
 */
SQuaternion EngineMath::Normalize(const SQuaternion& quat)
{
	SQuaternion result = quat;
	float fLen = Length(quat);

	// Check to prevent division by zero for the zero quaternion
	if (fLen != 0.0f)
	{
		// Normalization factor (1 / length)
		float fInvLength = 1.0f / fLen;

		// Scale (normalize) each component
		result.w *= fInvLength;
		result.x *= fInvLength;
		result.y *= fInvLength;
		result.z *= fInvLength;
	}

	return (result);
}

/**
 * @brief Returns the Identity Quaternion.
 *
 * The Identity Quaternion represents no rotation and is the quaternion equivalent
 * of a multiplication factor of 1 or an identity matrix. It has the components
 * q_identity = (1, 0, 0, 0) (w=1, x=0, y=0, z=0).
 *
 * It is often used to initialize rotation variables or as the starting point
 * for incremental rotations.
 *
 * @return SQuaternion The Identity Quaternion (w=1, x=0, y=0, z=0).
 */
SQuaternion EngineMath::Identity()
{
	return SQuaternion(0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 * @brief Performs Spherical Linear Interpolation (Slerp) between two quaternions.
 *
 * Slerp interpolates between two rotations (quat1 and quat2) along the shortest
 * arc on the unit sphere, providing a constant angular velocity. This results
 * in smooth, visually correct rotation paths.
 *
 * The interpolation parameter 't' controls the position on the arc:
 * - If t = 0.0, the result is quat1.
 * - If t = 1.0, the result is quat2.
 * - If t is between 0.0 and 1.0, the result is a rotation between quat1 and quat2.
 *
 * @param quat1 [in] The starting rotation quaternion (t=0).
 * @param quat2 [in] The ending rotation quaternion (t=1).
 * @param t [in] The interpolation factor (typically between 0.0 and 1.0).
 * @return SQuaternion The interpolated rotation quaternion.
 */
SQuaternion EngineMath::Slerp(const SQuaternion& quat1, const SQuaternion& quat2, float t)
{
	// Clamp t to [0,1] for safety
	t = glm::clamp(t, 0.0f, 1.0f);

	// Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
	// t is interpolation factor
	SQuaternion result{};

	// Use local variables for q2, which we might negate for the shortest path
	float w2 = quat2.w;
	float x2 = quat2.x;
	float y2 = quat2.y;
	float z2 = quat2.z;

	// 1. Calculate the dot product (cosHalfTheta = q1 . q2)
	// The dot product is the cosine of the half-angle between the two quaternions.
	float cosHalfTheta = quat1.w * w2 + quat1.x * x2 + quat1.y * y2 + quat1.z * z2;

	/**
	 * @brief CRITICAL: Shortest Path Check (Double Cover Property)
	 *
	 * If the dot product is negative, the angle between the quaternions is > 90 degrees,
	 * meaning Slerp would take the long arc. We negate one quaternion (q2) to force
	 * the interpolation along the shortest path, as q and -q represent the same rotation.
	 */
	if (cosHalfTheta < 0.0f)
	{
		// Negate all components of our given quaternion
		w2 = -w2;
		x2 = -x2;
		y2 = -y2;
		z2 = -z2;

		// Flip the sign of the dot product to positive
		cosHalfTheta = -cosHalfTheta;
	}

	/**
	 * @brief Trivial/Fallback Check
	 *
	 * If the angle is very small (dot product is near 1.0), Slerp is unstable due to
	 * division by zero. We fall back to standard Linear Interpolation (Lerp) which
	 * is accurate for small angles.
	 */
	if (cosHalfTheta >= 1.0f - QUATERNION_EPS)
	{
		// Use Lerp as a stable approximation for small angles
		result.x = quat1.x * (1.0f - t) + x2 * t;
		result.y = quat1.y * (1.0f - t) + y2 * t;
		result.z = quat1.z * (1.0f - t) + z2 * t;

		result.w = quat1.w * (1.0f - t) + w2 * t;
	}
	else
	{
		/**
		 * @brief Standard Slerp Calculation
		 *
		 * Formula: qt = [sin((1-t)O) / sin(O)] * q1 + [sin(tO) / sin(O)] * q2
		 * where O is the angle between q1 and q2.
		 */
		float halfTheta = std::acos(cosHalfTheta);
		float sinHalfTheta = std::sin(halfTheta);

		// Calculate ratios
		float fRatioA = std::sin((1.0f - t) * halfTheta) / sinHalfTheta;
		float fRatioB = std::sin(t * halfTheta) / sinHalfTheta;

		// Apply Slerp formula component-wise
		result.x = (quat1.x * fRatioA) + (x2 * fRatioB);
		result.y = (quat1.y * fRatioA) + (y2 * fRatioB);
		result.z = (quat1.z * fRatioA) + (z2 * fRatioB);
		result.w = (quat1.w * fRatioA) + (w2 * fRatioB);
	}

	// Add normalization (optional but recommended for numerical drift)
	result.Normalize();

	return (result);
}

/**
 * @brief Converts an axis-angle rotation into a unit quaternion from a Right-Handed axis-angle rotation.
 *
 * This function computes a quaternion representation for a rotation
 * around a specified axis by a given angle. Quaternions are ideal for
 * representing 3D rotations in computer graphics and physics due to their
 * immunity to issues like Gimbal Lock.
 *
 * The resulting quaternion q = (x, y, z, w) is calculated using the
 * standard axis-angle conversion formula:
 * * * **Real (W) component:** w = cos(Angle / 2)
 * * **Vector (X, Y, Z) components:** (x, y, z) = (Axis_x, Axis_y, Axis_z)  * sin(Angle / 2)
 *
 * The implementation includes an **angle negation** when the input is in
 * radians (`fAngle = ToRadians(fAngle);`). This suggests the function is
 * designed to work with a standard or **right-handed rotation** convention,
 * where a positive angle produces the opposite rotation of the standard
 * right-hand rule.
 *
 * **Note on Input Axis:** For the output quaternion to be a true **unit**
 * quaternion (which is required for pure rotation), the input axis `vAxis`
 * **must** be a unit vector (normalized to a length of 1). The caller is
 * responsible for ensuring `vAxis` is normalized.
 *
 * @param vAxis [in] A float array of 3 elements representing the rotation axis vector (x, y, z).
 * *Must be a unit vector for the output to be a proper unit quaternion.*
 * @param fAngle [in] The rotation angle. The units (radians or degrees) are determined by `bRadian`.
 * @param bRadian [in] A boolean flag. Set to `false` if `fAngle` is in radians, `true` if in degrees.
 * *If `bRadian` is true, the angle is negated internally, indicating a
 * right-handed or normal rotation convention.*
 */
SQuaternion EngineMath::FromAxisAngle(float v3Axis[3], float fAngle, bool bRadian)
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
	}

	// Calculate sine and cosine of half the rotation angle
	float sinHalfTheta = std::sinf(angleRad / 2.0f);

	// Vector components: q.v = v_axis * sin(theta/2)
	result.x = sinHalfTheta * v3Axis[0];
	result.y = sinHalfTheta * v3Axis[1];
	result.z = sinHalfTheta * v3Axis[2];

	// Real component: q.w = cos(theta/2)
	result.w = std::cos(angleRad / 2.0f);

	return (result);
}

SQuaternion EngineMath::FromAxisAngle(const SVector3Df& v3Axis, float fAngle, bool bRadian)
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
	}

	// Calculate sine and cosine of half the rotation angle
	float sinHalfTheta = std::sinf(angleRad / 2.0f);

	// Vector components: q.v = v_axis * sin(theta/2)
	result.x = sinHalfTheta * v3Axis.x;
	result.y = sinHalfTheta * v3Axis.y;
	result.z = sinHalfTheta * v3Axis.z;

	// Real component: q.w = cos(theta/2)
	result.w = std::cos(angleRad / 2.0f);

	return (result);
}

SQuaternion EngineMath::FromAxisAngle(const glm::vec3& v3Axis, float fAngle, bool bRadian)
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
	}

	// Calculate sine and cosine of half the rotation angle
	float sinHalfTheta = std::sinf(angleRad / 2.0f);

	// Vector components: q.v = v_axis * sin(theta/2)
	result.x = sinHalfTheta * v3Axis.x;
	result.y = sinHalfTheta * v3Axis.y;
	result.z = sinHalfTheta * v3Axis.z;

	// Real component: q.w = cos(theta/2)
	result.w = std::cos(angleRad / 2.0f);

	return (result);
}

/**
 * @brief Converts a unit quaternion into an axis-angle representation.
 *
 * This function extracts the rotation axis and rotation angle from a
 * quaternion. The input quaternion **must be normalized** (unit length)
 * for the resulting axis and angle to represent a valid rotation.
 *
 * The conversion is based on the unit quaternion definition:
 *     q = (x, y, z, w) = v * sin(theta / 2) + cos(theta / 2)
 *
 * The resulting axis-angle representation is computed as:
 *
 *   - Angle (theta):
 *       theta = 2 * acos(q.w)
 *
 *   - Axis:
 *       axis = (q.x, q.y, q.z) / sqrt(1 - q.w^2)
 *
 * @note Due to floating-point precision errors, q.w should be clamped to
 *       the range [-1, 1] before calling acos.
 *
 * @note Special Case (Identity Rotation):
 *       When |q.w| is close to 1, the rotation angle approaches 0 (or 2PI),
 *       and the axis becomes mathematically undefined due to division by
 *       zero. In this case, the function assigns an arbitrary normalized
 *       axis (1, 0, 0), since rotating by zero around any axis produces
 *       no rotation.
 *
 * @param quat [in] The SQuaternion object to store the resulting quaternion.
 *
 * @param outAxis [out] A float array of 3 elements that receives the
 *        normalized rotation axis (x, y, z).
 * @param bRadian If true, the returned angle is in radians; if false,
 *        the angle is returned in degrees.
 *
 * @return float The rotation angle, in radians or degrees depending on
 *         the value of bRadian.
 */
float EngineMath::ToAxisAngle(const SQuaternion& quat, float outAxis[3], bool bRadian)
{
	// Ensure numerical safety
	float wClamped = glm::clamp(quat.w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis[0] = quat.x / sinHalfAngle;
		outAxis[1] = quat.y / sinHalfAngle;
		outAxis[2] = quat.z / sinHalfAngle;
	}
	else
	{
		// Identity rotation: axis is arbitrary
		outAxis[0] = 1.0f;
		outAxis[1] = 0.0f;
		outAxis[2] = 0.0f;
	}

	if (bRadian == false)
	{
		angle = ToDegrees(angle);
	}
	return float(angle);
}

float EngineMath::ToAxisAngle(const SQuaternion& quat, SVector3Df& outAxis, bool bRadian)
{
	// Ensure numerical safety
	float wClamped = glm::clamp(quat.w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis.x = quat.x / sinHalfAngle;
		outAxis.y = quat.y / sinHalfAngle;
		outAxis.z = quat.z / sinHalfAngle;
	}
	else
	{
		// Identity rotation: axis is arbitrary
		outAxis.x = 1.0f;
		outAxis.y = 0.0f;
		outAxis.z = 0.0f;
	}

	if (bRadian == false)
	{
		angle = ToDegrees(angle);
	}
	return float(angle);
}

float EngineMath::ToAxisAngle(const SQuaternion& quat, glm::vec3& outAxis, bool bRadian)
{
	// Ensure numerical safety
	float wClamped = glm::clamp(quat.w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis.x = quat.x / sinHalfAngle;
		outAxis.y = quat.y / sinHalfAngle;
		outAxis.z = quat.z / sinHalfAngle;
	}
	else
	{
		// Identity rotation: axis is arbitrary
		outAxis.x = 1.0f;
		outAxis.y = 0.0f;
		outAxis.z = 0.0f;
	}

	if (bRadian == false)
	{
		angle = ToDegrees(angle);
	}
	return float(angle);
}

/**
 * @brief Creates a quaternion representing a rotation about the X-axis.
 *
 * This function returns a new quaternion corresponding to a rotation of
 * the specified angle around the positive X-axis (1, 0, 0). The resulting
 * quaternion can be used for 3D transformations, animation, and orientation
 * calculations.
 *
 * Internally, this function is a convenience wrapper around the
 * axis-angle construction logic, using the X-axis as the rotation axis.
 *
 * @param fAngle  The rotation angle. The unit (radians or degrees) must
 *                match the convention used by the underlying axis-angle
 *                conversion logic.
 * @param bRadian Convert the given angels to radian, set to false if the given values are in radians
 * @return SQuaternion  A new quaternion representing the X-axis rotation.
 *
 * @note The returned quaternion will be a unit quaternion if the underlying
 *       axis-angle construction produces normalized results.
 */
SQuaternion EngineMath::FromXRotation(float fAngle, bool bRadian)
{
	SVector3Df v3XAxis(1.0f, 0.0f, 0.0f);
	return FromAxisAngle(v3XAxis, fAngle, bRadian);		// false = degrees, true = radians
}

/**
 * @brief Creates a quaternion representing a rotation about the Y-axis.
 *
 * This function returns a new quaternion corresponding to a rotation of
 * the specified angle around the positive Y-axis (0, 1, 0). The resulting
 * quaternion can be used for 3D transformations, animation, and orientation
 * calculations.
 *
 * Internally, this function is a convenience wrapper around the
 * axis-angle construction logic, using the Y-axis as the rotation axis.
 *
 * @param fAngle  The rotation angle. The unit (radians or degrees) must
 *                match the convention used by the underlying axis-angle
 *                conversion logic.
 *
 * @return SQuaternion  A new quaternion representing the Y-axis rotation.
 */
SQuaternion EngineMath::FromYRotation(float fAngle, bool bRadian)
{
	SVector3Df v3YAxis(0.0f, 1.0f, 0.0f);
	return FromAxisAngle(v3YAxis, fAngle, bRadian);		// false = degrees, true = radians
}

/**
 * @brief Creates a quaternion representing a rotation about the Z-axis.
 *
 * This function returns a new quaternion corresponding to a rotation of
 * the specified angle around the positive Z-axis (0, 0, 1). The resulting
 * quaternion can be used for 3D transformations, animation, and orientation
 * calculations.
 *
 * Internally, this function is a convenience wrapper around the
 * axis-angle construction logic, using the Z-axis as the rotation axis.
 *
 * @param fAngle  The rotation angle. The unit (radians or degrees) must
 *                match the convention used by the underlying axis-angle
 *                conversion logic.
 *
 * @return SQuaternion  A new quaternion representing the Z-axis rotation.
 */
SQuaternion EngineMath::FromZRotation(float fAngle, bool bRadian)
{
	SVector3Df v3ZAxis(0.0f, 0.0f, 1.0f);
	return FromAxisAngle(v3ZAxis, fAngle, bRadian);		// false = degrees, true = radians
}


/**
 * @brief Creates a quaternion representing a rotation about a primary axis.
 *
 * This function returns a new quaternion corresponding to a rotation of
 * the specified angle around one of the principal axes: X, Y, or Z.
 *
 * @param axis      enum specifying the axis:
 *                  - AXIS_X = X-axis
 *                  - AXIS_Y = Y-axis
 *                  - AXIS_Z = Z-axis
 *
 * @param fAngle  The rotation angle. The unit (radians or degrees) must
 *                match the convention used by the underlying axis-angle
 *                conversion logic.
 *
 * @return SQuaternion A new quaternion representing the rotation about the selected axis.
 */
SQuaternion EngineMath::FromRotation(EAxis axis, float fAngle, bool bRadian)
{
	if (axis == EAxis::AXIS_X)
	{
		return FromXRotation(fAngle, bRadian);
	}
	else if (axis == EAxis::AXIS_Y)
	{
		return FromYRotation(fAngle, bRadian);
	}
	else if (axis == EAxis::AXIS_Z)
	{
		return FromZRotation(fAngle, bRadian);
	}
	return SQuaternion();
}

/**
 * @brief Creates a quaternion from Euler angles using the ZYX (Yaw-Pitch-Roll) order.
 *
 * This function converts a set of Euler rotations into a single quaternion.
 * The rotations are applied in this specific order:
 * 1. First, rotation around the Z-axis (Yaw).
 * 2. Second, rotation around the Y-axis (Pitch).
 * 3. Third, rotation around the X-axis (Roll).
 *
 * This is the standard "Aerospace" or "Aviation" sequence. It is highly
 * recommended for game characters and vehicles.
 *
 * @param eulerRad [in] An array of 3 floats representing rotations in radians:
 * - eulerRad[0]: Roll (X-axis)
 * - eulerRad[1]: Pitch (Y-axis)
 * - eulerRad[2]: Yaw (Z-axis)
 *
 * @return SQuaternion A new quaternion representing the combined rotation.
 *
 * @note This function is 'const' and does not modify the current object.
 * It returns a brand new quaternion instead.
 */
SQuaternion EngineMath::FromEulerZYX(float eulerRad[3])
{
	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad[2] * 0.5f);
	const float sinYaw = std::sin(eulerRad[2] * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad[1] * 0.5f);
	const float sinPitch = std::sin(eulerRad[1] * 0.5f);

	// Roll - X axis
	const float cosRoll = std::cos(eulerRad[0] * 0.5f);
	const float sinRoll = std::sin(eulerRad[0] * 0.5f);

	// Compute quaternion components
	SQuaternion result{};
	result.x = cosYaw * sinRoll * cosPitch - sinYaw * cosRoll * sinPitch;
	result.y = cosYaw * cosRoll * sinPitch + sinYaw * sinRoll * cosPitch;
	result.z = sinYaw * cosRoll * cosPitch - cosYaw * sinRoll * sinPitch;
	result.w = cosYaw * cosRoll * cosPitch + sinYaw * sinRoll * sinPitch;
	return (result);
}

SQuaternion EngineMath::FromEulerZYX(const SVector3Df& eulerRad)
{
	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad.z * 0.5f);
	const float sinYaw = std::sin(eulerRad.z * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad.y * 0.5f);
	const float sinPitch = std::sin(eulerRad.y * 0.5f);

	// Roll - X axis
	const float cosRoll = std::cos(eulerRad.x * 0.5f);
	const float sinRoll = std::sin(eulerRad.x * 0.5f);

	// Compute quaternion components
	SQuaternion result{};
	result.x = cosYaw * sinRoll * cosPitch - sinYaw * cosRoll * sinPitch;
	result.y = cosYaw * cosRoll * sinPitch + sinYaw * sinRoll * cosPitch;
	result.z = sinYaw * cosRoll * cosPitch - cosYaw * sinRoll * sinPitch;
	result.w = cosYaw * cosRoll * cosPitch + sinYaw * sinRoll * sinPitch;
	return (result);
}

SQuaternion EngineMath::FromEulerZYX(const glm::vec3& eulerRad)
{
	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad.z * 0.5f);
	const float sinYaw = std::sin(eulerRad.z * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad.y * 0.5f);
	const float sinPitch = std::sin(eulerRad.y * 0.5f);

	// Roll - X axis
	const float cosRoll = std::cos(eulerRad.x * 0.5f);
	const float sinRoll = std::sin(eulerRad.x * 0.5f);

	// Compute quaternion components (ZYX sequence formula)
	SQuaternion result{};
	result.x = cosYaw * sinRoll * cosPitch - sinYaw * cosRoll * sinPitch;
	result.y = cosYaw * cosRoll * sinPitch + sinYaw * sinRoll * cosPitch;
	result.z = sinYaw * cosRoll * cosPitch - cosYaw * sinRoll * sinPitch;
	result.w = cosYaw * cosRoll * cosPitch + sinYaw * sinRoll * sinPitch;
	return (result);
}

/**
 * @brief Creates a quaternion from Euler angles using the XYZ order (Roll-Pitch-Yaw).
 *
 * This function converts a set of Euler rotations into a single quaternion.
 * The rotations are applied in this specific order:
 * 1. First, rotation around the X-axis (Roll).
 * 2. Second, rotation around the Y-axis (Pitch).
 * 3. Third, rotation around the Z-axis (Yaw).
 *
 * This sequence is often used in general 3D modeling and graphics packages.
 *
 * @param eulerRad [in] An array of 3 floats representing rotations in radians:
 * - eulerRad[0]: Roll (X-axis)
 * - eulerRad[1]: Pitch (Y-axis)
 * - eulerRad[2]: Yaw (Z-axis)
 *
 * @return SQuaternion A new quaternion representing the combined rotation.
 *
 * @note This function is 'const' and does not modify the current object.
 * It returns a brand new quaternion instead.
 */
SQuaternion EngineMath::FromEulerXYZ(float eulerRad[3])
{
	// Roll - X axis
	const float cosRoll = std::cos(eulerRad[0] * 0.5f);
	const float sinRoll = std::sin(eulerRad[0] * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad[1] * 0.5f);
	const float sinPitch = std::sin(eulerRad[1] * 0.5f);

	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad[2] * 0.5f);
	const float sinYaw = std::sin(eulerRad[2] * 0.5f);

	// Compute quaternion components (XYZ sequence formula)
	SQuaternion result{};

	// The formulas are different from ZYX due to non-commutative rotation
	result.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	result.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	result.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	result.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	return (result);
}

SQuaternion EngineMath::FromEulerXYZ(const SVector3Df& eulerRad)
{
	// Roll - X axis
	const float cosRoll = std::cos(eulerRad.x * 0.5f);
	const float sinRoll = std::sin(eulerRad.x * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad.y * 0.5f);
	const float sinPitch = std::sin(eulerRad.y * 0.5f);

	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad.z * 0.5f);
	const float sinYaw = std::sin(eulerRad.z * 0.5f);

	// Compute quaternion components (XYZ sequence formula)
	SQuaternion result{};

	// The formulas are different from ZYX due to non-commutative rotation
	result.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	result.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	result.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	result.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	return (result);
}

SQuaternion EngineMath::FromEulerXYZ(const glm::vec3& eulerRad)
{
	// Roll - X axis
	const float cosRoll = std::cos(eulerRad.x * 0.5f);
	const float sinRoll = std::sin(eulerRad.x * 0.5f);

	// Pitch - Y axis
	const float cosPitch = std::cos(eulerRad.y * 0.5f);
	const float sinPitch = std::sin(eulerRad.y * 0.5f);

	// Yaw - Z axis
	const float cosYaw = std::cos(eulerRad.z * 0.5f);
	const float sinYaw = std::sin(eulerRad.z * 0.5f);

	// Compute quaternion components (XYZ sequence formula)
	SQuaternion result{};

	// The formulas are different from ZYX due to non-commutative rotation
	result.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	result.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	result.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	result.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	return (result);
}

/**
 * @brief Converts a quaternion into Euler angles using the ZYX (Yaw-Pitch-Roll) rotation sequence.
 *
 * This function extracts the three Euler angles (Roll, Pitch, Yaw) from a unit quaternion.
 * The angles are calculated in **radians** and correspond to rotations applied in the order:
 * Z (Yaw) -> Y (Pitch) -> X (Roll).
 *
 * @details
 * The conversion formulas are derived from the quaternion-to-rotation-matrix representation:
 * - **Roll (X-axis):**  atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
 * - **Pitch (Y-axis):** asin(2*(w*y - x*z)) (or an equivalent stable formulation using atan2)
 * - **Yaw (Z-axis):**   atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
 *
 * **Gimbal Lock / Singularity:**
 * Pitch approaches +/- 90° (+/- PI/2 radians) may cause gimbal lock, where Roll and Yaw rotations
 * are no longer uniquely defined. This implementation uses atan2 for Roll and Yaw and a
 * robust method for Pitch to mitigate singularities as much as possible.
 *
 * @param quat [in] The SQuaternion object to use and store the resulting quaternion.
 * @param eulerRad [out] Array of 3 floats receiving the Euler angles in radians:
 *   - eulerRad[0]: Roll (X-axis)
 *   - eulerRad[1]: Pitch (Y-axis)
 *   - eulerRad[2]: Yaw (Z-axis)
 *
 * @post `eulerRad` contains the Euler angles corresponding to the ZYX rotation sequence.
 */
void EngineMath::ToEulerZYX(const SQuaternion& quat, float eulerRad[3])
{
	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (quat.w * quat.x + quat.y * quat.z);
	const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
	eulerRad[0] = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (quat.w * quat.y - quat.x * quat.z);

	// Check for gimbal lock
	if (std::abs(sinPitch) >= 1.0f) // Near +/- 90 degrees Pitch
	{
		// Pitch is +/-90°, Roll + Yaw is ambiguous
		eulerRad[1] = std::copysign(static_cast<float>(M_PI) / 2.0f, sinPitch); // +/-90°
		eulerRad[2] = 0.0f; // Yaw set to 0, could be any value since Roll+Yaw is ambiguous
	}
	else
	{
		// Safe zone
		eulerRad[1] = std::asin(sinPitch);

		// Compute Yaw (Z-axis)
		const float siny_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
		const float cosy_cosp = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
		eulerRad[2] = std::atan2(siny_cosp, cosy_cosp);
	}
}

SVector3Df EngineMath::ToEulerZYX(const SQuaternion& quat)
{
	SVector3Df eulerRad{}; // eulerRad[0] = Roll, eulerRad[1] = Pitch, eulerRad[2] = Yaw

	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (quat.w * quat.x + quat.y * quat.z);
	const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
	eulerRad.x = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (quat.w * quat.y - quat.x * quat.z);

	// Check for gimbal lock
	if (std::abs(sinPitch) >= 1.0f) // Near +/- 90 degrees Pitch
	{
		// Pitch is +/-90°, Roll + Yaw is ambiguous
		eulerRad.y = std::copysign(static_cast<float>(M_PI) / 2.0f, sinPitch); // +/-90°
		eulerRad.z = 0.0f; // Yaw set to 0, could be any value since Roll+Yaw is ambiguous
	}
	else
	{
		// Safe zone
		eulerRad.y = std::asin(sinPitch);

		// Compute Yaw (Z-axis)
		const float siny_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
		const float cosy_cosp = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
		eulerRad.z = std::atan2(siny_cosp, cosy_cosp);
	}

	return (eulerRad);
}

glm::vec3 EngineMath::ToEulerZYXGLM(const SQuaternion& quat)
{
	glm::vec3 eulerRad{}; // eulerRad[0] = Roll, eulerRad[1] = Pitch, eulerRad[2] = Yaw

	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (quat.w * quat.x + quat.y * quat.z);
	const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
	eulerRad.x = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (quat.w * quat.y - quat.x * quat.z);

	// Check for gimbal lock
	if (std::abs(sinPitch) >= 1.0f) // Near +/- 90 degrees Pitch
	{
		// Pitch is +/-90°, Roll + Yaw is ambiguous
		eulerRad.y = std::copysign(static_cast<float>(M_PI) / 2.0f, sinPitch); // +/-90°
		eulerRad.z = 0.0f; // Yaw set to 0, could be any value since Roll+Yaw is ambiguous
	}
	else
	{
		// Safe zone
		eulerRad.y = std::asin(sinPitch);

		// Compute Yaw (Z-axis)
		const float siny_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
		const float cosy_cosp = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
		eulerRad.z = std::atan2(siny_cosp, cosy_cosp);
	}

	return (eulerRad);
}

/**
 * @brief Rotates a 3D vector using the quaternion rotation formula.
 *
 * This function rotates the input vector 'vec3' by the rotation represented
 * by the current quaternion (*this). It uses the standard vector identity
 * derived from quaternion multiplication:
 *
 * v' = v + 2 * w * (q_xyz cross v) + 2 * (q_xyz cross (q_xyz cross v))
 *
 * where q_xyz is the vector part and w is the scalar part of the quaternion.
 *
 * This version uses explicit cross calls for clarity and maintainability.
 *
 * @param quat [in] The SQuaternion object to use and store the resulting quaternion.
 * @param vec3 [in] The input 3D vector to rotate.
 * @returns SVector3Df The rotated vector.
 *
 * @pre The quaternion (*this) must be normalized (unit quaternion) for correct results.
 */
SVector3Df EngineMath::RotateVec(const SQuaternion& quat, const SVector3Df& vec3)
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	// Vector part of quaternion
	SVector3Df v(quat.x, quat.y, quat.z);

	// 1. Calculate the first cross product: t = v x p
	SVector3Df t = v.cross(vec3);

	// 2. Calculate the second cross product: t' = v x t
	SVector3Df t_prime = v.cross(t);

	// 3. Apply the final formula: p' = p + 2w(t) + 2(t')
	SVector3Df rotatedVec = vec3 + t * (2.0f * quat.w) + t_prime * 2.0f;

	return (rotatedVec);
}

glm::vec3 EngineMath::RotateVec(const SQuaternion& quat, const glm::vec3& vec3)
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	// Vector part of quaternion
	glm::vec3 v(quat.x, quat.y, quat.z);

	// 1. Calculate the first cross product: t = v x p
	glm::vec3 t = glm::cross(v, vec3);

	// 2. Calculate the second cross product: t' = v x t
	glm::vec3 t_prime = glm::cross(v, t);

	// 3. Apply the final formula: p' = p + 2w(t) + 2(t')
	glm::vec3 rotatedVec = vec3 + t * (2.0f * quat.w) + t_prime * 2.0f;

	return (rotatedVec);
}

/**
 * @brief Rotates a 3D vector using an optimized, fully inlined quaternion formula.
 *
 * This version performs the same rotation as 'Rotate' but avoids temporary
 * vectors and cross-product function calls for maximum performance.
 *
 * It computes the rotated vector using direct scalar arithmetic:
 *
 * t = 2 * (q_xyz cross v)
 * v' = v + w * t + (q_xyz cross t)
 *
 * where q_xyz is the vector part and w is the scalar part of the quaternion.
 *
 * @param quat [in] The SQuaternion object to use and store the resulting quaternion.
 * @param v [in] The input 3D vector to rotate.
 * @returns SVector3Df The rotated vector.
 *
 * @note This method is faster than the standard 'RotateVec' but less readable.
 * @pre The quaternion (*this) must be normalized (unit quaternion) for correct results.
 */
SVector3Df EngineMath::Rotate(const SQuaternion& quat, const SVector3Df& vec3)
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	const float qx = quat.x, qy = quat.y, qz = quat.z, qw = quat.w;
	const float vx = vec3.x, vy = vec3.y, vz = vec3.z;

	const float t_x = 2.0f * (qy * vz - qz * vy);
	const float t_y = 2.0f * (qz * vx - qx * vz);
	const float t_z = 2.0f * (qx * vy - qy * vx);

	const float rotated_x = vx + qw * t_x + (qy * t_z - qz * t_y);
	const float rotated_y = vy + qw * t_y + (qz * t_x - qx * t_z);
	const float rotated_z = vz + qw * t_z + (qx * t_y - qy * t_x);

	return SVector3Df(rotated_x, rotated_y, rotated_z);
}

glm::vec3 EngineMath::Rotate(const SQuaternion& quat, const glm::vec3& vec3)
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	const float qx = quat.x, qy = quat.y, qz = quat.z, qw = quat.w;
	const float vx = vec3.x, vy = vec3.y, vz = vec3.z;

	const float t_x = 2.0f * (qy * vz - qz * vy);
	const float t_y = 2.0f * (qz * vx - qx * vz);
	const float t_z = 2.0f * (qx * vy - qy * vx);

	const float rotated_x = vx + qw * t_x + (qy * t_z - qz * t_y);
	const float rotated_y = vy + qw * t_y + (qz * t_x - qx * t_z);
	const float rotated_z = vz + qw * t_z + (qx * t_y - qy * t_x);

	return glm::vec3(rotated_x, rotated_y, rotated_z);
}

/**
 * @brief Converts the quaternion to a 4x4 rotation matrix in Column-Major order.
 *
 * This function converts the quaternion into its equivalent rotation matrix.
 * It generates the standard rotation matrix but performs an explicit transpose
 * during assignment to ensure the output is in **Column-Major** format,
 * which is required by graphics APIs like OpenGL.
 * * The assignment pattern matrix[row][col] is used, where:
 * - matrix[0-2][0]: X-Axis (First Column)
 * - matrix[0-2][1]: Y-Axis (Second Column)
 * - matrix[0-2][2]: Z-Axis (Third Column)
 *
 * @param quat [in] The SQuaternion object to use and store the resulting quaternion.
 * @returns SMatrix4x4 The resulting rotation matrix.
 *
 * @pre The quaternion must be normalized (unit quaternion).
 */
SMatrix4x4 EngineMath::ToMatrix4(const SQuaternion& quat)
{
	// [ 1-2(y^2+z^2)		2(xy+zw)		2(xz-yw)		0 ]
	// [ 2(xy-zw)			1-2(x^2+z^2)	2(yz+xw)		0 ]
	// [ 2(xz+yw)			2(yz-xw)		1-2(x^2+y^2)	0 ]
	// [ 0					0				0				1 ]

	SMatrix4x4 matrix{};

	// Diagonal terms (remain the same)
	const float xx = quat.x * quat.x;
	const float yy = quat.y * quat.y;
	const float zz = quat.z * quat.z;

	const float two_x = 2.0f * quat.x;
	const float two_y = 2.0f * quat.y;
	const float two_z = 2.0f * quat.z;

	// Cross products
	const float xy2 = two_x * quat.y;
	const float xz2 = two_x * quat.z;
	const float yz2 = two_y * quat.z;

	// W terms
	const float wx = two_x * quat.w; // Note: xw term from formula
	const float wy = two_y * quat.w; // Note: yw term from formula
	const float wz = two_z * quat.w; // Note: zw term from formula

	// Column 0 (X-Axis)
	matrix[0][0] = 1.0f - 2.0f * (yy + zz);
	matrix[0][1] = xy2 + wz; // (2xy + 2zw)
	matrix[0][2] = xz2 - wy; // (2xz - 2yw)
	matrix[0][3] = 0.0f;

	// Column 1 (Y-Axis)
	matrix[1][0] = xy2 - wz; // (2xy - 2zw)
	matrix[1][1] = 1.0f - 2.0f * (xx + zz);
	matrix[1][2] = yz2 + wx; // (2yz + 2xw)]
	matrix[1][3] = 0.0f;

	// Column 2 (Z-Axis)
	matrix[2][0] = xz2 + wy; // (2xz + 2yw)
	matrix[2][1] = yz2 - wx; // (2yz - 2xw)
	matrix[2][2] = 1.0f - 2.0f * (xx + yy);
	matrix[2][3] = 0.0f;

	// Column 3 (Translation/Perspective)
	matrix[3][0] = 0.0f;
	matrix[3][1] = 0.0f;
	matrix[3][2] = 0.0f;
	matrix[3][3] = 1.0f;

	return (matrix);
}

glm::mat4 EngineMath::ToMatrix4GLM(const SQuaternion& quat)
{
	// [ 1-2(y^2+z^2)		2(xy+zw)		2(xz-yw)		0 ]
	// [ 2(xy-zw)			1-2(x^2+z^2)	2(yz+xw)		0 ]
	// [ 2(xz+yw)			2(yz-xw)		1-2(x^2+y^2)	0 ]
	// [ 0					0				0				1 ]

	glm::mat4 matrix{};

	// Diagonal terms (remain the same)
	const float xx = quat.x * quat.x;
	const float yy = quat.y * quat.y;
	const float zz = quat.z * quat.z;

	const float two_x = 2.0f * quat.x;
	const float two_y = 2.0f * quat.y;
	const float two_z = 2.0f * quat.z;

	// Cross products
	const float xy2 = two_x * quat.y;
	const float xz2 = two_x * quat.z;
	const float yz2 = two_y * quat.z;

	// W terms
	const float wx = two_x * quat.w; // Note: xw term from formula
	const float wy = two_y * quat.w; // Note: yw term from formula
	const float wz = two_z * quat.w; // Note: zw term from formula

	// Column 0 (X-Axis)
	matrix[0][0] = 1.0f - 2.0f * (yy + zz);
	matrix[1][0] = xy2 + wz; // (2xy + 2zw)
	matrix[2][0] = xz2 - wy; // (2xz - 2yw)
	matrix[3][0] = 0.0f;

	// Column 1 (Y-Axis)
	matrix[0][1] = xy2 - wz; // (2xy - 2zw)
	matrix[1][1] = 1.0f - 2.0f * (xx + zz);
	matrix[2][1] = yz2 + wx; // (2yz + 2xw)]
	matrix[3][1] = 0.0f;

	// Column 2 (Z-Axis)
	matrix[0][2] = xz2 + wy; // (2xz + 2yw)
	matrix[1][2] = yz2 - wx; // (2yz - 2xw)
	matrix[2][2] = 1.0f - 2.0f * (xx + yy);
	matrix[3][2] = 0.0f;

	// Column 3 (Translation/Perspective)
	matrix[0][3] = 0.0f;
	matrix[1][3] = 0.0f;
	matrix[2][3] = 0.0f;
	matrix[3][3] = 1.0f;

	return (matrix);
}

/**
 * @brief Creates a quaternion from a 4x4 rotation matrix.
 *
 * This function extracts the quaternion representation from a given
 * 4x4 rotation matrix. The input matrix is assumed to be a valid rotation
 * matrix (orthonormal with a determinant of +1).
 *
 * The conversion uses the standard algorithm for extracting quaternions
 * from rotation matrices, ensuring numerical stability and correctness.
 *
 * @param mat [in] The SMatrix4x4 rotation matrix from which to extract the quaternion.
 * @returns SQuaternion A new quaternion representing the same rotation as the input matrix.
 *
 * @note The resulting quaternion will be a unit quaternion if the input
 *       matrix is a proper rotation matrix.
 */
SQuaternion EngineMath::FromMatrix4(const SMatrix4x4& mat)
{
	SQuaternion quat{};

	// Calculate the trace (sum of the diagonal elements: R00 + R11 + R22)
	// R00 is m.mat[0][0]
	// R11 is m.mat[1][1]
	// R22 is m.mat[2][2]
	const float trace = mat[0][0] + mat[1][1] + mat[2][2];

	// We determine which component (w, x, y, or z) is the largest.
	// This maintains numerical stability during the calculation.
	if (trace > 0)
	{
		// Case 1: W (Real part) is the largest component
		const float s = std::sqrt(trace + 1.0f) * 2.0f; // S=4*qw
		quat.w = 0.25f * s;

		// The remaining terms use the off-diagonal elements:
		// mat[col][row]
		quat.x = (mat[1][2] - mat[2][1]) / s; // (R21 - R12) / S]
		quat.y = (mat[2][0] - mat[0][2]) / s; // (R02 - R20) / S]
		quat.z = (mat[0][1] - mat[1][0]) / s; // (R10 - R01) / S]
	}
	else if ((mat[0][0] > mat[1][1]) && (mat[0][0] > mat[2][2]))
	{
		// Case 2: X is the largest component
		const float s = std::sqrt(1.0f + mat[0][0] - mat[1][1] - mat[2][2]) * 2.0f; // S=4*qx
		quat.w = (mat[1][2] - mat[2][1]) / s;
		quat.x = 0.25f * s;
		quat.y = (mat[0][1] + mat[1][0]) / s;
		quat.z = (mat[0][2] + mat[2][0]) / s;
	}
	else if (mat[1][1] > mat[2][2])
	{
		// Case 3: Y is the largest component
		const float s = std::sqrt(1.0f + mat[1][1] - mat[0][0] - mat[2][2]) * 2.0f; // S=4*qy
		quat.w = (mat[2][0] - mat[0][2]) / s;
		quat.x = (mat[0][1] + mat[1][0]) / s;
		quat.y = 0.25f * s;
		quat.z = (mat[2][1] + mat[1][2]) / s;
	}
	else
	{
		// Case 4: Z is the largest component
		const float s = std::sqrt(1.0f + mat[2][2] - mat[0][0] - mat[1][1]) * 2.0f; // S=4*qz
		quat.w = (mat[0][1] - mat[1][0]) / s;
		quat.x = (mat[0][2] + mat[2][0]) / s;
		quat.y = (mat[2][1] + mat[1][2]) / s;
		quat.z = 0.25f * s;
	}

	return (quat);
}

SQuaternion EngineMath::FromMatrix4(const glm::mat4& mat)
{
	SQuaternion quat{};

	// Calculate the trace (sum of the diagonal elements: R00 + R11 + R22)
	// R00 is m.mat[0][0]
	// R11 is m.mat[1][1]
	// R22 is m.mat[2][2]
	const float trace = mat[0][0] + mat[1][1] + mat[2][2];

	// We determine which component (w, x, y, or z) is the largest.
	// This maintains numerical stability during the calculation.
	if (trace > 0)
	{
		// Case 1: W (Real part) is the largest component
		const float s = std::sqrt(trace + 1.0f) * 2.0f; // S=4*qw
		quat.w = 0.25f * s;

		// The remaining terms use the off-diagonal elements:
		// mat[col][row]
		quat.x = (mat[1][2] - mat[2][1]) / s; // (R21 - R12) / S]
		quat.y = (mat[2][0] - mat[0][2]) / s; // (R02 - R20) / S]
		quat.z = (mat[0][1] - mat[1][0]) / s; // (R10 - R01) / S]
	}
	else if ((mat[0][0] > mat[1][1]) && (mat[0][0] > mat[2][2]))
	{
		// Case 2: X is the largest component
		const float s = std::sqrt(1.0f + mat[0][0] - mat[1][1] - mat[2][2]) * 2.0f; // S=4*qx
		quat.w = (mat[1][2] - mat[2][1]) / s;
		quat.x = 0.25f * s;
		quat.y = (mat[0][1] + mat[1][0]) / s;
		quat.z = (mat[0][2] + mat[2][0]) / s;
	}
	else if (mat[1][1] > mat[2][2])
	{
		// Case 3: Y is the largest component
		const float s = std::sqrt(1.0f + mat[1][1] - mat[0][0] - mat[2][2]) * 2.0f; // S=4*qy
		quat.w = (mat[2][0] - mat[0][2]) / s;
		quat.x = (mat[0][1] + mat[1][0]) / s;
		quat.y = 0.25f * s;
		quat.z = (mat[2][1] + mat[1][2]) / s;
	}
	else
	{
		// Case 4: Z is the largest component
		const float s = std::sqrt(1.0f + mat[2][2] - mat[0][0] - mat[1][1]) * 2.0f; // S=4*qz
		quat.w = (mat[0][1] - mat[1][0]) / s;
		quat.x = (mat[0][2] + mat[2][0]) / s;
		quat.y = (mat[2][1] + mat[1][2]) / s;
		quat.z = 0.25f * s;
	}

	return (quat);
}

/**
 * @brief Converts a quaternion to Euler angles in degrees using the ZYX rotation sequence.
 *
 * This function extracts the three Euler angles (Roll, Pitch, Yaw) from a unit quaternion.
 * The angles are calculated in **degrees** and correspond to rotations applied in the order:
 * Z (Yaw) -> Y (Pitch) -> X (Roll).
 *
 * @details
 * The conversion formulas are derived from the quaternion-to-rotation-matrix representation:
 * - **Roll (X-axis):**  atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
 * - **Pitch (Y-axis):** asin(2*(w*y - x*z)) (or an equivalent stable formulation using atan2)
 * - **Yaw (Z-axis):**   atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
 *
 * **Gimbal Lock / Singularity:**
 * Pitch approaches +/- 90° (+/- PI/2 radians) may cause gimbal lock, where Roll and Yaw rotations
 * are no longer uniquely defined. This implementation uses atan2 for Roll and Yaw and a
 * robust method for Pitch to mitigate singularities as much as possible.
 *
 * @param quat [in] The SQuaternion object to use and store the resulting quaternion.
 * @return SEulerAngles A struct containing the Euler angles in degrees:
 *   - Roll (X-axis)
 *   - Pitch (Y-axis)
 *   - Yaw (Z-axis)
 *
 * @post The returned SEulerAngles contains the Euler angles corresponding to the ZYX rotation sequence.
 */
SEulerAngles EngineMath::ToEulerAnglesDegrees(const SQuaternion& quat)
{
	SEulerAngles angles = { 0.0f, 0.0f, 0.0f };

	// Use the standard formulas for converting quaternion to Euler angles (Y-X-Z order)
	// The components of the quaternion are q.w, q.x, q.y, q.z

	// --- PITCH (Rotation around X-axis) ---
	// Calculated from sin(Pitch) term, which can be derived from the rotation matrix element M21
	// This angle depends on the term: 2 * (qw*qx - qy*qz)
	const float sinPitch = 2.0f * (quat.w * quat.x - quat.y * quat.z);

	// Check for singularity (Pitch near +/- 90 degrees)
	if (std::abs(sinPitch) >= 1.0f)
	{
		// Pitch is +/- 90°. This is a Gimbal Lock scenario.
		// We set Pitch to +/- 90 degrees.
		angles.fPitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinPitch);

		// The sum of Yaw and Roll (Y+Z) is ambiguous. We typically set one to zero 
		// and calculate the other, but since FPS typically has zero roll:
		// We can calculate Roll (Z) from the remaining rotation around the camera's Z-axis.
		// Since your camera doesn't *add* roll, we can set both Yaw and Roll to 0 (or try to extract Roll).

		// Let's calculate the remaining Roll (Z)
		const float sinr_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
		const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.z * quat.z); // Note the changed terms here
		angles.fRoll = std::atan2f(sinr_cosp, cosr_cosp);

		angles.fYaw = 0.0f; // Force Yaw to zero when looking straight up/down.
	}
	else
	{
		// --- 2. Safe Zone: Compute Pitch (X) ---
		angles.fPitch = std::asin(sinPitch);

		// --- 3. Safe Zone: Compute Yaw (Y-axis, eulerRad.y) ---
		// Calculated from terms: 2*(qw*qy + qx*qz) and 1 - 2*(qx^2 + qy^2)
		const float siny_cosp = 2.0f * (quat.w * quat.y + quat.x * quat.z);
		const float cosy_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
		angles.fYaw = std::atan2f(siny_cosp, cosy_cosp);

		// --- 4. Safe Zone: Compute Roll (Z-axis, eulerRad.z) ---
		// Calculated from terms: 2*(qw*qz - qx*qy) and 1 - 2*(qx^2 + qz^2)
		// Since your FPS system should not have roll, this value should be near zero.
		const float sinr_cosp = 2.0f * (quat.w * quat.z - quat.x * quat.y);
		const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.z * quat.z);
		angles.fRoll = std::atan2f(sinr_cosp, cosr_cosp);
	}

	// Convert to Degrees (if your utility requires it)
	// You can do this outside or inside, but typically the engine uses radians, 
	// and the UI layer converts to degrees.
	angles.fPitch = ToDegrees(angles.fPitch); // Pitch
	angles.fYaw = ToDegrees(angles.fYaw); // Yaw
	angles.fRoll = ToDegrees(angles.fRoll); // Roll (Should be near zero)

	return (angles);
}

SQuaternion EngineMath::ExtractRotation(const Matrix4& m)
{
	Vector3D scale = EngineMath::ExtractScale(m);

	float m00 = m[0][0];
	float m01 = m[1][0];
	float m02 = m[2][0];

	float m10 = m[0][1];
	float m11 = m[1][1];
	float m12 = m[2][1];

	float m20 = m[0][2];
	float m21 = m[1][2];
	float m22 = m[2][2];

	if (scale.x > VECTOR3_EPS)
	{
		m00 /= scale.x;
		m10 /= scale.x;
		m20 /= scale.x;
	}

	if (scale.y > VECTOR3_EPS)
	{
		m01 /= scale.y;
		m11 /= scale.y;
		m21 /= scale.y;
	}

	if (scale.z > VECTOR3_EPS)
	{
		m02 /= scale.z;
		m12 /= scale.z;
		m22 /= scale.z;
	}

	SQuaternion q;

	float trace = m00 + m11 + m22;
	if (trace > 0.0f)
	{
		float s = std::sqrt(trace + 1.0f) * 2.0f;
		q.w = 0.25f * s;
		q.x = (m21 - m12) / s;
		q.y = (m02 - m20) / s;
		q.z = (m10 - m01) / s;
	}
	else if (m00 > m11 && m00 > m22)
	{
		float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
		q.w = (m21 - m12) / s;
		q.x = 0.25f * s;
		q.y = (m01 + m10) / s;
		q.z = (m02 + m20) / s;
	}
	else if (m11 > m22)
	{
		float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
		q.w = (m02 - m20) / s;
		q.x = (m01 + m10) / s;
		q.y = 0.25f * s;
		q.z = (m12 + m21) / s;
	}
	else
	{
		float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
		q.w = (m10 - m01) / s;
		q.x = (m02 + m20) / s;
		q.y = (m12 + m21) / s;
		q.z = 0.25f * s;
	}

	q.Normalize();
	return q;
}

Matrix4 EngineMath::ComposeTransform(const Vector3D& pos, const SQuaternion& rot, const Vector3D& scale)
{
	SQuaternion q = rot;
	q.Normalize();

	float xx = q.x * q.x;
	float yy = q.y * q.y;
	float zz = q.z * q.z;
	float xy = q.x * q.y;
	float xz = q.x * q.z;
	float yz = q.y * q.z;
	float wx = q.w * q.x;
	float wy = q.w * q.y;
	float wz = q.w * q.z;

	Matrix4 out(1.0f);

	out[0][0] = (1.0f - 2.0f * (yy + zz)) * scale.x;
	out[0][1] = (2.0f * (xy + wz)) * scale.x;
	out[0][2] = (2.0f * (xz - wy)) * scale.x;
	out[0][3] = 0.0f;

	out[1][0] = (2.0f * (xy - wz)) * scale.y;
	out[1][1] = (1.0f - 2.0f * (xx + zz)) * scale.y;
	out[1][2] = (2.0f * (yz + wx)) * scale.y;
	out[1][3] = 0.0f;

	out[2][0] = (2.0f * (xz + wy)) * scale.z;
	out[2][1] = (2.0f * (yz - wx)) * scale.z;
	out[2][2] = (1.0f - 2.0f * (xx + yy)) * scale.z;
	out[2][3] = 0.0f;

	out[3][0] = pos.x;
	out[3][1] = pos.y;
	out[3][2] = pos.z;
	out[3][3] = 1.0f;

	return out;
}
