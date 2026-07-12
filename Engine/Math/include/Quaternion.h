#pragma once

#include <glm/glm.hpp>
#include "MathUtils.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include "TypeMatrix4.h"

/**
 * Maximum floating point difference that is considered as equal.
 */
constexpr float QUATERNION_EPS = (1e-6f);	// quaternion math
constexpr float ANGLE_EPS = (1e-6f);		// radians

struct SQuaternion
{
	/**< @brief Scalar part (real component). */
	float w;

	/**< @brief Vector part (i component). */
	float x;

	/**< @brief Vector part (j component).	*/
	float y;

	/**< @brief Vector part (k component). */
	float z;

	/**
	 * @brief Default constructor.
	 *
	 * Initializes all quaternion components (x, y, z, w) to zero.
	 * Uses the default C++ compiler implementation for efficiency.
	 */
	SQuaternion() = default;

	/**
	 * @brief Initializes the quaternion to the Identity quaternion.
	 *
	 * Sets all diagonal components (w) to fW and all off-diagonal components to 0.0f.
	 */
	SQuaternion(float fW);

	/**
	 * @brief Explicit component constructor.
	 *
	 * Constructs an SQuaternion object using explicitly specified values
	 * for the vector (x, y, z) and scalar (w) components.
	 *
	 * @param fW The value for the w component.
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 * @param fZ The value for the z component.
	 */
	SQuaternion(float fW, float fX, float fY, float fZ);

	/**
	 * @brief Explicit component constructor.
	 *
	 * Constructs an SQuaternion object using explicitly specified values
	 * for the vector (x, y, z) and scalar (w) components.
	 *
	 * @param fW The value for the w component.
	 * @param vec3 The value for the x, y, z component.
	 */
	SQuaternion(float fW, const SVector3Df& vec3);

	/**
	 * @brief Explicit component constructor.
	 *
	 * Constructs an SQuaternion object using explicitly specified values
	 * for the vector (x, y, z) and scalar (w) components.
	 *
	 * @param vec4 The value for the x, y, z and w component.
	 */
	SQuaternion(const SVector4Df& vec4);

	/**
	 * @brief Explicit component constructor.
	 *
	 * Constructs an SQuaternion object using explicitly specified values
	 * for the vector (x, y, z) and scalar (w) components from another quaternion.
	 *
	 * @param quat The value for the x, y, z and w component.
	 */
	SQuaternion(const SQuaternion& quat);

	/**
	 * @brief Quaternion addition operator (non-modifying).
	 *
	 * Calculates a new quaternion by adding the components of the input quaternion 'quat'
	 * to the corresponding components of this quaternion (*this).
	 *
	 * @param quat The constant reference to the SQuaternion object to add.
	 * @return SQuaternion A new quaternion instance resulting from the component-wise addition.
	 */
	SQuaternion operator+(const SQuaternion& quat) const;

	/**
	 * @brief Quaternion subtraction operator (non-modifying).
	 *
	 * Calculates a new quaternion by subtracting the components of the input quaternion 'quat'
	 * from the corresponding components of this quaternion (*this).
	 *
	 * @param quat The constant reference to the SQuaternion object to subtract.
	 * @return SQuaternion A new quaternion instance resulting from the component-wise subtraction.
	 */
	SQuaternion operator-(const SQuaternion& quat) const;

	/**
	 * @brief Quaternion multiplication operator (non-modifying).
	 *
	 * Computes the Hamilton product of this quaternion (*this) and the input quaternion 'quat'.
	 * This operation is not commutative (q1 * q2 != q2 * q1).
	 *
	 * @param quat The constant reference to the SQuaternion object to multiply by.
	 * @return SQuaternion A new quaternion instance resulting from the multiplication.
	 */
	SQuaternion operator*(const SQuaternion& quat) const;

	/**
	 * @brief Quaternion division operator (non-modifying).
	 *
	 * Computes the division of this quaternion (*this) and the input quaternion 'quat'.
	 *
	 * @param quat The constant reference to the SQuaternion object to divide by.
	 * @return SQuaternion A new quaternion instance resulting from the division.
	 */
	SQuaternion operator/(const SQuaternion& quat) const;

	/**
	 * @brief Quaternion addition with a scalar value (non-modifying).
	 *
	 * Calculates a new Quaternion by adding the scalar value 'fVal' to all components
	 * of this Quaternion (*this).
	 *
	 * @param fVal The scalar float value to add (passed by value for efficiency).
	 * @return SQuaternion A new Quaternion instance resulting from the scalar addition.
	 */
	SQuaternion operator+(float fVal) const;

	/**
	 * @brief Quaternion subtraction with a scalar value (non-modifying).
	 *
	 * Calculates a new Quaternion by subtracting the scalar value 'fVal' from all
	 * components of this Quaternion (*this).
	 *
	 * @param fVal The scalar float value to subtract (passed by value for efficiency).
	 * @return SQuaternion A new Quaternion instance resulting from the scalar subtraction.
	 */
	SQuaternion operator-(float fVal) const;

	/**
	 * @brief Quaternion multiplication by a scalar value (non-modifying).
	 *
	 * Calculates a new Quaternion by multiplying all components of this Quaternion by the
	 * scalar value 'fVal'. This is standard Quaternion scaling.
	 *
	 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
	 * @return SQuaternion A new Quaternion instance resulting from the scalar multiplication.
	 */
	SQuaternion operator*(float fVal) const;

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
	SQuaternion operator/(float fVal) const;

	/**
	 * @brief Compound assignment operator for quaternion addition (in-place modification).
	 *
	 * Adds the components of the input quaternion 'quat' to the corresponding components
	 * of this quaternion (*this), modifying this object directly.
	 *
	 * @param quat The constant reference to the SQuaternion object to add.
	 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
	 */
	SQuaternion& operator+=(const SQuaternion& quat);

	/**
	 * @brief Compound assignment operator for quaternion subtraction (in-place modification).
	 *
	 * Subtracts the components of the input quaternion 'quat' from the corresponding components
	 * of this quaternion (*this), modifying this object directly.
	 *
	 * @param quat The constant reference to the SQuaternion object to subtract.
	 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
	 */
	SQuaternion& operator-=(const SQuaternion& quat);

	/**
	 * @brief Compound assignment operator for component-wise multiplication (in-place modification).
	 *
	 * Multiplies the components of this quaternion (*this) by the corresponding components
	 * of the input quaternion 'quat' (Hadamard product), modifying this object directly.
	 *
	 * @param quat The constant reference to the SQuaternion object to multiply by.
	 * @return SQuaternion& A reference to this modified SQuaternion object.
	 */
	SQuaternion& operator*=(const SQuaternion& quat);

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
	SQuaternion& operator/=(const SQuaternion& quat);

	/**
	 * @brief Compound assignment operator for scalar addition (in-place modification).
	 *
	 * Adds the scalar float value 'fVal' to all components (x, y, z, w) of this quaternion
	 * (*this), modifying the quaternion directly.
	 *
	 * @param fVal The scalar float value to add (passed by value for efficiency).
	 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
	 */
	SQuaternion& operator+=(float fVal);

	/**
	 * @brief Compound assignment operator for scalar subtraction (in-place modification).
	 *
	 * Subtracts the scalar float value 'fVal' from all components (x, y, z, w) of this quaternion
	 * (*this), modifying the quaternion directly.
	 *
	 * @param fVal The scalar float value to subtract (passed by value for efficiency).
	 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
	 */
	SQuaternion& operator-=(float fVal);

	/**
	 * @brief Compound assignment operator for scalar multiplication (in-place modification).
	 *
	 * Multiplies all components (x, y, z) of this quaternion (*this) by the scalar float value
	 * 'fVal', modifying the quaternion directly (quaternion scaling).
	 *
	 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
	 * @return SQuaternion& A reference to the modified SQuaternion object (*this).
	 */
	SQuaternion& operator*=(float fVal);

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
	SQuaternion& operator/=(float fVal);

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
	bool operator == (const SQuaternion& quat) const;

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
	bool operator != (const SQuaternion& quat) const;

	/**
	 * @brief Calculate the squared norm of the denominator
	 * 
	 * Calculates the squared Length (magnitude) of the quaternion.
	 * @return float The squared Length of the quaternion.
	 */
	float LengthSquared() const;

	/**
	 * @brief Calculates the quaternion magnitude (length).
	 *
	 * Computes the Euclidean length (or magnitude) of the quaternion, defined as
	 * the square root of the sum of the squares of its components: sqrt{x^2 + y^2 + z^2 + w^2}.
	 * This operation is non-modifying and marked 'const'.
	 *
	 * @return float The magnitude (length) of the quaternion.
	 */
	float Length() const;

	/**
	 * @brief Calculates the conjugate of an input quaternion
	 * 
	 * Calculates the conjugate of the input quaternion 'quat' and assigns the result
	 * 
	 * @param quat The constant reference to the source quaternion.
	 * @return SQuaternion A modified SQuaternion object.
	 */
	SQuaternion GetConjugate(const SQuaternion& quat) const;

	/**
	 * @brief Calculates the conjugate of the current quaternion
	 *
	 * Calculates the conjugate of the current quaternion (*this).
	 * @return SQuaternion A modified SQuaternion object.
	 */
	SQuaternion GetConjugate() const;

	/**
	 * @brief Calculates the conjugate of the current quaternion (in-place modification).
	 *
	 * Calculates the conjugate of the current quaternion (*this).
	 * @return SQuaternion A modified SQuaternion object.
	 */
	SQuaternion& Conjugate();

	/**
	 * @brief Normalizes the Quaternion (in-place modification).
	 *
	 * Modifies the quaternion such that its length becomes 1.0, preserving its direction.
	 * If the current vector length is zero, the quaternion is left unchanged for safety.
	 *
	 * @return SQuaternion& A reference to the modified (normalized) SQuaternion object (*this).
	 */
	SQuaternion& Normalize();

	/**
	 * @brief Initializes the quaternion to the Identity quaternion.
	 *
	 * Sets all diagonal components (w) to 1.0f and all off-diagonal components to 0.0f.
	 */
	SQuaternion& Identity();

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
	SQuaternion Slerp(const SQuaternion& quat, float t) const;

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
	 * radians (`fAngle = ToRadian(fAngle);`). This suggests the function is
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
	 * *If `bRadian` is true, the angle is not negated internally, indicating a
	 * right-handed or normal rotation convention.*
	 */
	SQuaternion FromAxisAngle(float v3Axis[3], float fAngle, bool bRadian = true) const;
	SQuaternion FromAxisAngle(const SVector3Df& v3Axis, float fAngle, bool bRadian = true) const;
	SQuaternion FromAxisAngle(const glm::vec3& v3Axis, float fAngle, bool bRadian = true) const;

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
	float ToAxisAngle(float outAxis[3], bool bRadian = true) const;
	float ToAxisAngle(SVector3Df& outAxis, bool bRadian = true) const;
	float ToAxisAngle(glm::vec3& outAxis, bool bRadian = true) const;

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
	SQuaternion FromXRotation(float fAngle, bool bRadian = true) const;

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
	SQuaternion FromYRotation(float fAngle, bool bRadian = true) const;

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
	SQuaternion FromZRotation(float fAngle, bool bRadian = true) const;

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
	SQuaternion FromRotation(EAxis axis, float fAngle, bool bRadian = true) const;

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
	SQuaternion FromEulerZYX(float eulerRad[3]) const;
	SQuaternion FromEulerZYX(const SVector3Df& eulerRad) const;
	SQuaternion FromEulerZYX(const glm::vec3& eulerRad) const;

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
	SQuaternion FromEulerXYZ(float eulerRad[3]) const;
	SQuaternion FromEulerXYZ(const SVector3Df& eulerRad) const;
	SQuaternion FromEulerXYZ(const glm::vec3& eulerRad) const;

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
	void ToEulerZYX(float eulerRad[3]) const;
	SVector3Df ToEulerZYX() const;
	glm::vec3 ToEulerZYXGLM() const;

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
	SVector3Df RotateVec(const SVector3Df& vec3) const;
	glm::vec3 RotateVec(const glm::vec3& vec3) const;

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
	 * @note This method is faster than the standard 'RotateVec' but less readable.
	 * @pre The quaternion (*this) must be normalized (unit quaternion) for correct results.
	 */
	SVector3Df Rotate(const SVector3Df& vec3) const;
	glm::vec3 Rotate(const glm::vec3& vec3) const;

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
	SMatrix4x4 ToMatrix4() const;
	glm::mat4 ToMatrix4GLM() const;
};

using Quaternion = SQuaternion;