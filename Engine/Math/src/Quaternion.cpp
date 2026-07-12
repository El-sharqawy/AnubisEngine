#include "MathPCH.h"
#include "Quaternion.h"

/**
 * @brief Initializes the quaternion to the Identity quaternion.
 *
 * Sets all diagonal components (w) to fW and all off-diagonal components to 0.0f.
 */
SQuaternion::SQuaternion(float fW)
{
	w = fW;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}
/**
 * @brief Explicit component constructor.
 *
 * Constructs an SQuaternion object using explicitly specified values
 * for the vector (x, y, z) and scalar (w) components.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 * @param fZ The value for the z component.
 * @param fW The value for the w component.
 */
SQuaternion::SQuaternion(float fW, float fX, float fY, float fZ)
{
	w = fW;
	x = fX;
	y = fY;
	z = fZ;
}

/**
 * @brief Explicit component constructor.
 *
 * Constructs an SQuaternion object using explicitly specified values
 * for the vector (x, y, z) and scalar (w) components.
 *
 * @param vec3 The value for the x, y, z component.
 * @param fW The value for the w component.
 */
SQuaternion::SQuaternion(float fW, const SVector3Df& vec3)
{
	w = fW;
	x = vec3.x;
	y = vec3.y;
	z = vec3.z;
}

/**
 * @brief Explicit component constructor.
 *
 * Constructs an SQuaternion object using explicitly specified values
 * for the vector (x, y, z) and scalar (w) components.
 *
 * @param vec4 The value for the x, y, z and w component.
 */
SQuaternion::SQuaternion(const SVector4Df& vec4)
{
	w = vec4.w;
	x = vec4.x;
	y = vec4.y;
	z = vec4.z;
}

/**
 * @brief Explicit component constructor.
 *
 * Constructs an SQuaternion object using explicitly specified values
 * for the vector (x, y, z) and scalar (w) components from another quaternion.
 *
 * @param quat The value for the x, y, z and w component.
 */
SQuaternion::SQuaternion(const SQuaternion& quat)
{
	x = quat.x;
	y = quat.y;
	z = quat.z;
	w = quat.w;
}

/**
 * @brief Quaternion addition operator (non-modifying).
 *
 * Calculates a new quaternion by adding the components of the input quaternion 'quat'
 * to the corresponding components of this quaternion (*this).
 *
 * @param quat The constant reference to the SQuaternion object to add.
 * @return SQuaternion A new quaternion instance resulting from the component-wise addition.
 */
SQuaternion SQuaternion::operator+(const SQuaternion& quat) const
{
	SQuaternion result{};
	result.x = x + quat.x;
	result.y = y + quat.y;
	result.z = z + quat.z;
	result.w = w + quat.w;
	return (result);
}

/**
 * @brief Quaternion subtraction operator (non-modifying).
 *
 * Calculates a new quaternion by subtracting the components of the input quaternion 'quat'
 * from the corresponding components of this quaternion (*this).
 *
 * @param quat The constant reference to the SQuaternion object to subtract.
 * @return SQuaternion A new quaternion instance resulting from the component-wise subtraction.
 */
SQuaternion SQuaternion::operator-(const SQuaternion& quat) const
{
	SQuaternion result{};
	result.x = x - quat.x;
	result.y = y - quat.y;
	result.z = z - quat.z;
	result.w = w - quat.w;
	return (result);
}

/**
 * @brief Quaternion multiplication operator (non-modifying).
 *
 * Computes the Hamilton product of this quaternion (*this) and the input quaternion 'quat'.
 * This operation is not commutative (q1 * q2 != q2 * q1).
 *
 * @param quat The constant reference to the SQuaternion object to multiply by.
 * @return SQuaternion A new quaternion instance resulting from the multiplication.
 */
SQuaternion SQuaternion::operator*(const SQuaternion& quat) const
{
	SQuaternion result{};

	/*
	Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
			a*e - b*f - c*g - d*h
		+ i (b*e + a*f + c*h- d*g)
		+ j (a*g - b*h + c*e + d*f)
		+ k (a*h + b*g - c*f + d*e)
	*/

	result.w = w * quat.w - x * quat.x - y * quat.y - z * quat.z;
	result.x = w * quat.x + x * quat.w + y * quat.z - z * quat.y;
	result.y = w * quat.y - x * quat.z + y * quat.w + z * quat.x;
	result.z = w * quat.z + x * quat.y - y * quat.x + z * quat.w;

	return (result);
}

/**
 * @brief Quaternion division operator (non-modifying).
 *
 * Computes the division of this quaternion (*this) and the input quaternion 'quat'.
 *
 * @param quat The constant reference to the SQuaternion object to divide by.
 * @return SQuaternion A new quaternion instance resulting from the division.
 */
SQuaternion SQuaternion::operator/(const SQuaternion& quat) const
{
	// Compute conjugate of other: (w, -x, -y, -z)
	SQuaternion conj = quat.GetConjugate();

	// Multiply *this by conjugate
	SQuaternion productQuat = *this * conj;

	// Divide by |other|^2
	float fMagSq = quat.LengthSquared();
	if (fMagSq == 0.0f)
	{
		// Division by zero - return original quaternion as a safe fallback
		return *this;
	}

	return productQuat / fMagSq;  // Reuse scalar division
}

/**
 * @brief Quaternion addition with a scalar value (non-modifying).
 *
 * Calculates a new Quaternion by adding the scalar value 'fVal' to all components
 * of this Quaternion (*this).
 *
 * @param fVal The scalar float value to add (passed by value for efficiency).
 * @return SQuaternion A new Quaternion instance resulting from the scalar addition.
 */
SQuaternion SQuaternion::operator+(float fVal) const
{
	return SQuaternion(w + fVal, x + fVal, y + fVal, z + fVal);
}

/**
 * @brief Quaternion subtraction with a scalar value (non-modifying).
 *
 * Calculates a new Quaternion by subtracting the scalar value 'fVal' from all
 * components of this Quaternion (*this).
 *
 * @param fVal The scalar float value to subtract (passed by value for efficiency).
 * @return SQuaternion A new Quaternion instance resulting from the scalar subtraction.
 */
SQuaternion SQuaternion::operator-(float fVal) const
{
	return SQuaternion(w - fVal, x - fVal, y - fVal, z - fVal);
}

/**
 * @brief Quaternion multiplication by a scalar value (non-modifying).
 *
 * Calculates a new Quaternion by multiplying all components of this Quaternion by the
 * scalar value 'fVal'. This is standard Quaternion scaling.
 *
 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
 * @return SQuaternion A new Quaternion instance resulting from the scalar multiplication.
 */
SQuaternion SQuaternion::operator*(float fVal) const
{
	return SQuaternion(w * fVal, x * fVal, y * fVal, z * fVal);
}

/**
 * @brief Quaternion division by a scalar (non-modifying).
 *
 * Calculates a new Quaternion where each component is divided by the scalar fVal.
 * Performs a zero-check: if fVal is 0, the original Quaternion is returned unchanged
 * to prevent division by zero errors.
 *
 * @param fVal The scalar float value to divide by (passed by value for efficiency).
 * @return SQuaternion A new Quaternion instance resulting from the division.
 */
SQuaternion SQuaternion::operator/(float fVal) const
{
	// 1. Check for division by zero.
	if (fVal != 0.0f)
	{
		// 2. Return the new Quaternion initialized with the divided components.
		return SQuaternion(w / fVal, x / fVal, y / fVal, z / fVal);
	}

	// 3. If division by zero, return a copy of the original Quaternion (safe fallback).
	return *this;
}

/**
 * @brief Compound assignment operator for quaternion addition (in-place modification).
 *
 * Adds the components of the input quaternion 'quat' to the corresponding components
 * of this quaternion (*this), modifying this object directly.
 *
 * @param vec The constant reference to the SQuaternion object to add.
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator+=(const SQuaternion& quat)
{
	x += quat.x;
	y += quat.y;
	z += quat.z;
	w += quat.w;
	return (*this);
}

/**
 * @brief Compound assignment operator for quaternion subtraction (in-place modification).
 *
 * Subtracts the components of the input quaternion 'quat' from the corresponding components
 * of this quaternion (*this), modifying this object directly.
 *
 * @param quat The constant reference to the SQuaternion object to subtract.
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator-=(const SQuaternion& quat)
{
	x -= quat.x;
	y -= quat.y;
	z -= quat.z;
	w -= quat.w;
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise multiplication (in-place modification).
 *
 * Multiplies the components of this quaternion (*this) by the corresponding components
 * of the input quaternion 'quat' (Hadamard product), modifying this object directly.
 *
 * @param quat The constant reference to the SQuaternion object to multiply by.
 * @return SQuaternion& A reference to this modified SQuaternion object.
 */
SQuaternion& SQuaternion::operator*=(const SQuaternion& quat)
{
	// Store the original components of *this before calculation
	// This is ESSENTIAL because the w, x, y, and z calculations depend on 
	// the original values of all four components.
	float fX = x;
	float fY = y;
	float fZ = z;
	float fW = w;

	// 1. Calculate the new W component
	w = fW * quat.w - fX * quat.x - fY * quat.y - fZ * quat.z;

	// 2. Calculate the new X component (i)
	x = fW * quat.x + fX * quat.w + fY * quat.z - fZ * quat.y;

	// 3. Calculate the new Y component (j)
	y = fW * quat.y - fX * quat.z + fY * quat.w + fZ * quat.x;

	// 4. Calculate the new Z component (k)
	z = fW * quat.z + fX * quat.y - fY * quat.x + fZ * quat.w;

	// Return a reference to the modified object
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise division (in-place modification).
 *
 * Divides the components of this quaternion (*this) by the corresponding components
 * of the input quaternion 'quat', modifying this object directly.
 * Includes a safety check for division by zero on each component.
 *
 * @param quat The constant reference to the SQuaternion object to divide by.
 * @return SQuaternion& A reference to this modified SQuaternion object.
 */
SQuaternion& SQuaternion::operator/=(const SQuaternion& quat)
{
	// Divide by |other|^2
	float fMagSq = quat.LengthSquared();
	if (fMagSq == 0.0f)
	{
		// Division by zero - return original quaternion as a safe fallback
		return *this;
	}

	// Compute conjugate of other: (w, -x, -y, -z)
	SQuaternion conj = quat.GetConjugate();

	// Multiply *this by conjugate
	SQuaternion tempResult = *this * conj;

	// Assign the new components to *this
	w = tempResult.w;
	x = tempResult.x;
	y = tempResult.y;
	z = tempResult.z;

	return (*this) /= fMagSq;
}

/**
 * @brief Compound assignment operator for scalar addition (in-place modification).
 *
 * Adds the scalar float value 'fVal' to all components (x, y, z, w) of this quaternion
 * (*this), modifying the quaternion directly.
 *
 * @param fVal The scalar float value to add (passed by value for efficiency).
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator+=(float fVal)
{
	x += fVal;
	y += fVal;
	z += fVal;
	w += fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar subtraction (in-place modification).
 *
 * Subtracts the scalar float value 'fVal' from all components (x, y, z, w) of this quaternion
 * (*this), modifying the quaternion directly.
 *
 * @param fVal The scalar float value to subtract (passed by value for efficiency).
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator-=(float fVal)
{
	x -= fVal;
	y -= fVal;
	z -= fVal;
	w -= fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar multiplication (in-place modification).
 *
 * Multiplies all components (x, y, z) of this quaternion (*this) by the scalar float value
 * 'fVal', modifying the quaternion directly (quaternion scaling).
 *
 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator*=(float fVal)
{
	x *= fVal;
	y *= fVal;
	z *= fVal;
	w *= fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar division (in-place modification).
 *
 * Divides all components (x, y, z, w) of this quaternion (*this) by the scalar float value
 * 'fVal', modifying the quaternion directly. Includes a safety check to prevent
 * division by zero.
 *
 * @param fVal The scalar float value to divide by (passed by value for efficiency).
 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
 */
SQuaternion& SQuaternion::operator/=(float fVal)
{
	if (fVal != 0.0f)
	{
		x /= fVal;
		y /= fVal;
		z /= fVal;
		w /= fVal;
	}
	return (*this);
}

/**
 * @brief Component - wise equality comparison operator.
 *
 * Compares this quaternion(*this) with the input quaternion 'quat'.
 * Returns true if and only if all corresponding components(x, y, z, and w) are equal.
 * This operator is marked 'const' as it does not modify the quaternion.
 *
 * @param quat The constant reference to the SQuaternion quaternion to compare against.
 * @return bool True if the quaternions are equal component - wise, false otherwise.
 */
bool SQuaternion::operator == (const SQuaternion& quat) const
{
	// Check for direct equality (q1 == q2)
	bool bEqualX = std::fabs(x - quat.x) <= QUATERNION_EPS;
	bool bEqualY = std::fabs(y - quat.y) <= QUATERNION_EPS;
	bool bEqualZ = std::fabs(z - quat.z) <= QUATERNION_EPS;
	bool bEqualW = std::fabs(w - quat.w) <= QUATERNION_EPS;

	bool is_quat_equal = (bEqualW && bEqualX && bEqualY && bEqualZ);

	// Here we check if the components are equal but with opposite signs (q1 == -q2) due to double cover property.
	bool bEqualNegaX = std::fabs(x - (-quat.x)) <= QUATERNION_EPS;
	bool bEqualNegaY = std::fabs(y - (-quat.y)) <= QUATERNION_EPS;
	bool bEqualNegaZ = std::fabs(z - (-quat.z)) <= QUATERNION_EPS;
	bool bEqualNegaW = std::fabs(w - (-quat.w)) <= QUATERNION_EPS;

	bool is_nega_quat_equal = (bEqualNegaX && bEqualNegaY && bEqualNegaZ && bEqualNegaW);

	return (is_quat_equal || is_nega_quat_equal);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this quaternion (*this) with the input quaternion 'vec'.
 * Returns true if any corresponding component (x, y, z, or w) is not equal.
 * This operator is marked 'const' as it does not modify the quaternion.
 *
 * @param quat The constant reference to the SQuaternion quaternion to compare against.
 * @return bool True if the quaternions are not equal component-wise, false otherwise.
 */
bool SQuaternion::operator != (const SQuaternion& quat) const
{
	return !(*this == quat);
}

/**
 * @brief Calculate the squared norm of the denominator
 *
 * Calculates the squared norm (magnitude) of the quaternion.
 * @return float The squared norm of the quaternion.
 */
float SQuaternion::LengthSquared() const
{
	float norm_sq = x * x + y * y + z * z + w * w;
	return float(norm_sq);
}

/**
 * @brief Calculate the norm of the denominator
 *
 * Calculates the norm (magnitude) of the quaternion.
 * @return float The norm of the quaternion.
 */
float SQuaternion::Length() const
{
	return std::sqrt(LengthSquared());
}

/**
 * @brief Calculates the conjugate of an input quaternion and stores it here
 *
 * Calculates the conjugate of the input quaternion 'quat' and assigns the result
 *
 * @param quat The constant reference to the source quaternion.
 * @return SQuaternion A modified SQuaternion object.
 */
SQuaternion SQuaternion::GetConjugate(const SQuaternion& quat) const
{
	SQuaternion result{};
	result.x = -quat.x;
	result.y = -quat.y;
	result.z = -quat.z;
	result.w = quat.w;
	return (result);
}

/**
 * @brief Calculates the conjugate of the current quaternion
 *
 * Calculates the conjugate of the current quaternion (*this).
 * @return SQuaternion A modified SQuaternion object.
 */
SQuaternion SQuaternion::GetConjugate() const
{
	SQuaternion result{};
	result.x = -x;
	result.y = -y;
	result.z = -z;
	result.w = w;
	return (result);
}

/**
 * @brief Calculates the conjugate of the current quaternion (in-place modification).
 *
 * Calculates the conjugate of the current quaternion (*this).
 * @return SQuaternion A modified SQuaternion object.
 */
SQuaternion& SQuaternion::Conjugate()
{
	x = -x;
	y = -y;
	z = -z;
	w = w;
	return (*this);
}

/**
 * @brief Initializes the quaternion to the Identity quaternion.
 *
 * Sets all diagonal components (w) to 1.0f and all off-diagonal components to 0.0f.
 */
SQuaternion& SQuaternion::Identity()
{
	w = 1.0f;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
	return (*this);
}

/**
 * @brief Normalizes the Quaternion (in-place modification).
 *
 * Modifies the quaternion such that its length becomes 1.0, preserving its direction.
 * If the current vector length is zero, the quaternion is left unchanged for safety.
 *
 * @return SQuaternion& A reference to the modified (normalized) SQuaternion object (*this).
 */
SQuaternion& SQuaternion::Normalize()
{
	float fLen = Length();

	// Check to prevent division by zero for the zero quaternion
	if (fLen != 0.0f)
	{
		// Normalization factor (1 / length)
		float fInvLength = 1.0f / fLen;

		// Scale (normalize) each component
		w *= fInvLength;
		x *= fInvLength;
		y *= fInvLength;
		z *= fInvLength;
	}
	return (*this);
}

/**
 * @brief Performs Spherical Linear Interpolation (Slerp) between two quaternions.
 *
 * Slerp calculates a rotation that smoothly interpolates between a start
 * quaternion (q1) and an end quaternion (q2) over the shortest arc,
 * based on the interpolation factor 't'.
 *
 * @param quat Pointer to the ending quaternion.
 * @param t The interpolation factor (0.0 for *this, 1.0 for quat).
 */
SQuaternion SQuaternion::Slerp(const SQuaternion& quat, float t) const
{
	// Clamp t to [0,1] for safety
	t = std::clamp(t, 0.0f, 1.0f);

	// Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
	// t is interpolation factor
	SQuaternion result{};

	// Use local variables for q2, which we might negate for the shortest path
	float w2 = quat.w;
	float x2 = quat.x;
	float y2 = quat.y;
	float z2 = quat.z;

	// 1. Calculate the dot product (cosHalfTheta = q1 . q2)
	// The dot product is the cosine of the half-angle between the two quaternions.
	float cosHalfTheta = w * w2 + x * x2 + y * y2 + z * z2;

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
		result.x = x * (1.0f - t) + x2 * t;
		result.y = y * (1.0f - t) + y2 * t;
		result.z = z * (1.0f - t) + z2 * t;

		result.w = w * (1.0f - t) + w2 * t;
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
		result.x = (x * fRatioA) + (x2 * fRatioB);
		result.y = (y * fRatioA) + (y2 * fRatioB);
		result.z = (z * fRatioA) + (z2 * fRatioB);
		result.w = (w * fRatioA) + (w2 * fRatioB);
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
 * The resulting quaternion {q} = (x, y, z, w) is calculated using the
 * standard axis-angle conversion formula:
 * * * **Real (W) component:** $w = cos({Angle} / 2)
 * * **Vector (X, Y, Z) components:** (x, y, z) = ({Axis}_x, {Axis}_y, {Axis}_z)  * sin({Angle} / 2)
 *
 * The implementation includes an **angle negation** when the input is in
 * radians (`angleRad = ToRadian(fAngle);`). This suggests the function is
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
 * @param qOutput [out] A pointer to the LPQUAT structure where the resulting quaternion (x, y, z, w) will be stored.
 * @param bRadian [in] A boolean flag. Set to `true` if `fAngle` is in radians, `false` if in degrees.
 * *If `bRadian` is true, the angle is negated internally, indicating a
 * right-handed or normal rotation convention.*
 */
SQuaternion SQuaternion::FromAxisAngle(float v3Axis[3], float fAngle, bool bRadian) const
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = EngineMath::ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
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

SQuaternion SQuaternion::FromAxisAngle(const SVector3Df& v3Axis, float fAngle, bool bRadian) const
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = EngineMath::ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
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

SQuaternion SQuaternion::FromAxisAngle(const glm::vec3& v3Axis, float fAngle, bool bRadian) const
{
	// result Quaternion
	SQuaternion result{};

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
	{
		angleRad = EngineMath::ToRadians(angleRad); // Converts angle to radians, the standard Right-Handed rotation convention, negate the angle to match left handed systems (-ToRadian(angleRad))
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
 * @param outAxis [out] A float array of 3 elements that receives the
 *        normalized rotation axis (x, y, z).
 * @param bRadian If true, the returned angle is in radians; if false,
 *        the angle is returned in degrees.
 *
 * @return float The rotation angle, in radians or degrees depending on
 *         the value of bRadian.
 */
float SQuaternion::ToAxisAngle(float outAxis[3], bool bRadian) const
{
	// Ensure numerical safety
	float wClamped = std::clamp(this->w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis[0] = this->x / sinHalfAngle;
		outAxis[1] = this->y / sinHalfAngle;
		outAxis[2] = this->z / sinHalfAngle;
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
		angle = EngineMath::ToDegrees(angle);
	}
	return float(angle);
}

float SQuaternion::ToAxisAngle(SVector3Df& outAxis, bool bRadian) const
{
	// Ensure numerical safety
	float wClamped = std::clamp(this->w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis.x = this->x / sinHalfAngle;
		outAxis.y = this->y / sinHalfAngle;
		outAxis.z = this->z / sinHalfAngle;
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
		angle = EngineMath::ToDegrees(angle);
	}
	return float(angle);
}

float SQuaternion::ToAxisAngle(glm::vec3& outAxis, bool bRadian) const
{
	// Ensure numerical safety
	float wClamped = std::clamp(this->w, -1.0f, 1.0f);

	// Angle in radians
	float angle = 2.0f * std::acosf(wClamped);

	float sinHalfAngle = std::sqrtf(1.0f - wClamped * wClamped);

	if (sinHalfAngle > QUATERNION_EPS)
	{
		outAxis.x = this->x / sinHalfAngle;
		outAxis.y = this->y / sinHalfAngle;
		outAxis.z = this->z / sinHalfAngle;
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
		angle = EngineMath::ToDegrees(angle);
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
 *
 * @return SQuaternion  A new quaternion representing the X-axis rotation.
 *
 * @note The returned quaternion will be a unit quaternion if the underlying
 *       axis-angle construction produces normalized results.
 */
SQuaternion SQuaternion::FromXRotation(float fAngle, bool bRadian) const
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
SQuaternion SQuaternion::FromYRotation(float fAngle, bool bRadian) const
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
SQuaternion SQuaternion::FromZRotation(float fAngle, bool bRadian) const
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
SQuaternion SQuaternion::FromRotation(EAxis axis, float fAngle, bool bRadian) const
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
SQuaternion SQuaternion::FromEulerZYX(float eulerRad[3]) const
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

SQuaternion SQuaternion::FromEulerZYX(const SVector3Df& eulerRad) const
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

SQuaternion SQuaternion::FromEulerZYX(const glm::vec3& eulerRad) const
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
SQuaternion SQuaternion::FromEulerXYZ(float eulerRad[3]) const
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

SQuaternion SQuaternion::FromEulerXYZ(const SVector3Df& eulerRad) const
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

SQuaternion SQuaternion::FromEulerXYZ(const glm::vec3& eulerRad) const
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
 * @param eulerRad [out] Array of 3 floats receiving the Euler angles in radians:
 *   - eulerRad[0]: Roll (X-axis)
 *   - eulerRad[1]: Pitch (Y-axis)
 *   - eulerRad[2]: Yaw (Z-axis)
 *
 * @post `eulerRad` contains the Euler angles corresponding to the ZYX rotation sequence.
 */
void SQuaternion::ToEulerZYX(float eulerRad[3]) const
{
	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (w * x + y * z);
	const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	eulerRad[0] = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (w * y - x * z);

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
		const float siny_cosp = 2.0f * (w * z + x * y);
		const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
		eulerRad[2] = std::atan2(siny_cosp, cosy_cosp);
	}
}

SVector3Df SQuaternion::ToEulerZYX() const
{
	SVector3Df eulerRad{}; // eulerRad[0] = Roll, eulerRad[1] = Pitch, eulerRad[2] = Yaw

	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (w * x + y * z);
	const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	eulerRad.x = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (w * y - x * z);

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
		const float siny_cosp = 2.0f * (w * z + x * y);
		const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
		eulerRad.z = std::atan2(siny_cosp, cosy_cosp);
	}

	return (eulerRad);
}

glm::vec3 SQuaternion::ToEulerZYXGLM() const
{
	glm::vec3 eulerRad{}; // eulerRad[0] = Roll, eulerRad[1] = Pitch, eulerRad[2] = Yaw

	// Compute Roll (X-axis)
	const float sinr_cosp = 2.0f * (w * x + y * z);
	const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	eulerRad.x = std::atan2f(sinr_cosp, cosr_cosp);

	// Compute Pitch (Y-axis)
	const float sinPitch = 2.0f * (w * y - x * z);

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
		const float siny_cosp = 2.0f * (w * z + x * y);
		const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
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
 * @param vec3 [in] The input 3D vector to rotate.
 * @returns SVector3Df The rotated vector.
 *
 * @pre The quaternion (*this) must be normalized (unit quaternion) for correct results.
 */
SVector3Df SQuaternion::RotateVec(const SVector3Df& vec3) const
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	// Vector part of quaternion
	SVector3Df v(x, y, z);

	// 1. Calculate the first cross product: t = v x p
	SVector3Df t = v.cross(vec3);

	// 2. Calculate the second cross product: t' = v x t
	SVector3Df t_prime = v.cross(t);

	// 3. Apply the final formula: p' = p + 2w(t) + 2(t')
	SVector3Df rotatedVec = vec3 + t * (2.0f * w) + t_prime * 2.0f;

	return (rotatedVec);
}

glm::vec3 SQuaternion::RotateVec(const glm::vec3& vec3) const
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	// Vector part of quaternion
	glm::vec3 v(x, y, z);

	// 1. Calculate the first cross product: t = v x p
	glm::vec3 t = glm::cross(v, vec3);

	// 2. Calculate the second cross product: t' = v x t
	glm::vec3 t_prime = glm::cross(v, t);

	// 3. Apply the final formula: p' = p + 2w(t) + 2(t')
	glm::vec3 rotatedVec = vec3 + t * (2.0f * w) + t_prime * 2.0f;

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
 * @param v [in] The input 3D vector to rotate.
 * @returns SVector3Df The rotated vector.
 *
 * @note This method is faster than the standard 'Rotate' but less readable.
 * @pre The quaternion (*this) must be normalized (unit quaternion) for correct results.
 */
SVector3Df SQuaternion::Rotate(const SVector3Df& vec3) const
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	const float qx = x, qy = y, qz = z, qw = w;
	const float vx = vec3.x, vy = vec3.y, vz = vec3.z;

	const float t_x = 2.0f * (qy * vz - qz * vy);
	const float t_y = 2.0f * (qz * vx - qx * vz);
	const float t_z = 2.0f * (qx * vy - qy * vx);

	const float rotated_x = vx + qw * t_x + (qy * t_z - qz * t_y);
	const float rotated_y = vy + qw * t_y + (qz * t_x - qx * t_z);
	const float rotated_z = vz + qw * t_z + (qx * t_y - qy * t_x);

	return SVector3Df(rotated_x, rotated_y, rotated_z);
}

glm::vec3 SQuaternion::Rotate(const glm::vec3& vec3) const
{
	// Quaternion rotation formula (from euclideanspace.com):
	//   p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x
	//   p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y
	//   p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z
	// The current implementation uses the equivalent cross-product identity for clarity and efficiency:
	//   t = q_xyz × v, t' = q_xyz × t, v' = v + 2*w*t + 2*t'

	const float qx = x, qy = y, qz = z, qw = w;
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
 * @returns SMatrix4x4 The resulting rotation matrix.
 *
 * @pre The quaternion must be normalized (unit quaternion).
 */
SMatrix4x4 SQuaternion::ToMatrix4() const
{
	// [ 1-2(y^2+z^2)		2(xy+zw)		2(xz-yw)		0 ]
	// [ 2(xy-zw)			1-2(x^2+z^2)	2(yz+xw)		0 ]
	// [ 2(xz+yw)			2(yz-xw)		1-2(x^2+y^2)	0 ]
	// [ 0					0				0				1 ]

	SMatrix4x4 matrix{};

	// Diagonal terms (remain the same)
	const float xx = x * x;
	const float yy = y * y;
	const float zz = z * z;

	const float two_x = 2.0f * x;
	const float two_y = 2.0f * y;
	const float two_z = 2.0f * z;

	// Cross products
	const float xy2 = two_x * y;
	const float xz2 = two_x * z;
	const float yz2 = two_y * z;

	// W terms
	const float wx = two_x * w; // Note: xw term from formula
	const float wy = two_y * w; // Note: yw term from formula
	const float wz = two_z * w; // Note: zw term from formula

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

glm::mat4 SQuaternion::ToMatrix4GLM() const
{
	// [ 1-2(y^2+z^2)		2(xy+zw)		2(xz-yw)		0 ]
	// [ 2(xy-zw)			1-2(x^2+z^2)	2(yz+xw)		0 ]
	// [ 2(xz+yw)			2(yz-xw)		1-2(x^2+y^2)	0 ]
	// [ 0					0				0				1 ]

	glm::mat4 matrix{};

	// Diagonal terms (remain the same)
	const float xx = x * x;
	const float yy = y * y;
	const float zz = z * z;

	const float two_x = 2.0f * x;
	const float two_y = 2.0f * y;
	const float two_z = 2.0f * z;

	// Cross products
	const float xy2 = two_x * y;
	const float xz2 = two_x * z;
	const float yz2 = two_y * z;

	// W terms
	const float wx = two_x * w; // Note: xw term from formula
	const float wy = two_y * w; // Note: yw term from formula
	const float wz = two_z * w; // Note: zw term from formula

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
