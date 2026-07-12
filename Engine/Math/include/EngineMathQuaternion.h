#pragma once

#include <glm/glm.hpp>
#include "MathUtils.h"

struct SVector2Df;
struct SVector3Df;
struct SVector4Df;

struct SMatrix2x2;
struct SMatrix3x3;
struct SMatrix4x4;

struct SQuaternion;
struct SEulerAngles;

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
	 * @brief Calculates the product of two quaternions.
	 *
	 * Computes the Hamilton product of q1 multiplied by q2, and returns the result
	 * in a new quaternion instance. This is equivalent to q1 * q2.
	 *
	 * @param q1 The constant reference to the left-hand side quaternion (first operand).
	 * @param q2 The constant reference to the right-hand side quaternion (second operand).
	 * @return SQuaternion A new quaternion instance resulting from the multiplication.
	 */
	SQuaternion Multiply(const SQuaternion& q1, const SQuaternion& q2);

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
	float LengthSquared(const SQuaternion& quat);

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
	float Length(const SQuaternion& quat);

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
	SQuaternion Conjugate(const SQuaternion& quat);

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
	SQuaternion Normalize(const SQuaternion& quat);

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
	SQuaternion Identity();

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
	SQuaternion Slerp(const SQuaternion& quat1, const SQuaternion& quat2, float t);

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
	SQuaternion FromAxisAngle(float v3Axis[3], float fAngle, bool bRadian = true);
	SQuaternion FromAxisAngle(const SVector3Df& v3Axis, float fAngle, bool bRadian = true);
	SQuaternion FromAxisAngle(const glm::vec3& v3Axis, float fAngle, bool bRadian = true);

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
	float ToAxisAngle(const SQuaternion& quat, float outAxis[3], bool bRadian = true);
	float ToAxisAngle(const SQuaternion& quat, SVector3Df& outAxis, bool bRadian = true);
	float ToAxisAngle(const SQuaternion& quat, glm::vec3& outAxis, bool bRadian = true);

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
	SQuaternion FromXRotation(float fAngle, bool bRadian = true);

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
	SQuaternion FromYRotation(float fAngle, bool bRadian = true);

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
	SQuaternion FromZRotation(float fAngle, bool bRadian = true);

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
	SQuaternion FromRotation(EAxis axis, float fAngle, bool bRadian = true);

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
	SQuaternion FromEulerZYX(float eulerRad[3]);
	SQuaternion FromEulerZYX(const SVector3Df& eulerRad);
	SQuaternion FromEulerZYX(const glm::vec3& eulerRad);

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
	SQuaternion FromEulerXYZ(float eulerRad[3]);
	SQuaternion FromEulerXYZ(const SVector3Df& eulerRad);
	SQuaternion FromEulerXYZ(const glm::vec3& eulerRad);

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
	void ToEulerZYX(const SQuaternion& quat, float eulerRad[3]);
	SVector3Df ToEulerZYX(const SQuaternion& quat);
	glm::vec3 ToEulerZYXGLM(const SQuaternion& quat);

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
	SVector3Df RotateVec(const SQuaternion& quat, const SVector3Df& vec3);
	glm::vec3 RotateVec(const SQuaternion& quat, const glm::vec3& vec3);

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
	SVector3Df Rotate(const SQuaternion& quat, const SVector3Df& vec3);
	glm::vec3 Rotate(const SQuaternion& quat, const glm::vec3& vec3);

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
	SMatrix4x4 ToMatrix4(const SQuaternion& quat);
	glm::mat4 ToMatrix4GLM(const SQuaternion& quat);

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
	SQuaternion FromMatrix4(const SMatrix4x4& mat);
	SQuaternion FromMatrix4(const glm::mat4& mat);


	/**
	 * @brief Creates a quaternion from a 3x3 rotation matrix.
	 *
	 * This function extracts the quaternion representation from a given
	 * 3x3 rotation matrix. The input matrix is assumed to be a valid rotation
	 * matrix (orthonormal with a determinant of +1).
	 *
	 * The conversion uses the standard algorithm for extracting quaternions
	 * from rotation matrices, ensuring numerical stability and correctness.
	 *
	 * @param mat [in] The SMatrix3x3 rotation matrix from which to extract the quaternion.
	 * @returns SQuaternion A new quaternion representing the same rotation as the input matrix.
	 *
	 * @note The resulting quaternion will be a unit quaternion if the input
	 *       matrix is a proper rotation matrix.
	 */
	SQuaternion FromMatrix4(const SMatrix3x3& mat);
	SQuaternion FromMatrix4(const glm::mat3& mat);

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
	SEulerAngles ToEulerAnglesDegrees(const SQuaternion& quat);

	SQuaternion ExtractRotation(const Matrix4& m);
	Matrix4 ComposeTransform(const Vector3D& pos, const SQuaternion& rot, const Vector3D& scale);

} // namespace EngineMath - Quaternion Part