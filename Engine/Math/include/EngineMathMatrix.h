#pragma once

struct SVector2Df;
struct SVector3Df;
struct SVector4Df;

struct SMatrix2x2;
struct SMatrix3x3;
struct SMatrix4x4;

struct SQuaternion;

struct SPersProjInfo;
struct SOrthoProjInfo;
struct STransform;

#define MAT_FORCE_DEPTH_ZERO_TO_ONE

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
	 * @brief Initializes the matrix as an Identity matrix.
	 *
	 * This function returns a new matrix where all diagonal elements are set to 1.0f
	 * and all off-diagonal elements are set to 0.0f.
	 *
	 * @param mat4 The matrix to be converted to identity (read-only).
	 * @return The identity 4x4 matrix.
	 */
	SMatrix4x4 IdentityMat4();

	/**
	 * Computes the inverse of a 4x4 matrix using the Adjugate/Cofactor method.
	 * * This implementation is optimized and unrolled, operating on a Column-Major
	 * matrix (OpenGL standard, M[col][row] indexing). The calculation is structured
	 * to directly produce the Column-Major transpose of the Adjugate matrix.
	 *
	 * @param mat4 The 4x4 matrix to invert (read-only).
	 * @return The inverted 4x4 matrix, or a zero matrix if the determinant is zero.
	 */
	SMatrix4x4 Inverse(const SMatrix4x4& mat4);

	/**
	 * @brief Transposes the matrix.
	 *
	 * This function returns a new matrix that is the transpose of the current matrix.
	 *
	 * @param mat the matrix to transpose
	 * @return The transposed matrix.
	 */
	SMatrix4x4 Transpose(const SMatrix4x4& mat);

	/**
	 * @brief Calculates the determinant of the 4x4 matrix.
	 *
	 * This function computes the determinant of the full 4x4 matrix using a highly
	 * optimized cofactor expansion (Laplace expansion) along the first row. The
	 * implementation is unrolled and optimized to minimize overhead, similar to
	 * high-performance math libraries (like GLM).
	 *
	 * @param mat the matrix to calculate it's determinant
	 * @return The determinant of the 4x4 matrix.
	 */
	float Determinant(const SMatrix4x4& mat);

	/**
	 * Constructs a view matrix that transforms world coordinates to camera space.
	 *
	 * @param v3Eye    The camera position in world space
	 * @param v3Center The point the camera is looking at
	 * @param v3Up     The up direction vector (typically (0,1,0))
	 * @return         A 4x4 view matrix in column-major order
	 */
	SMatrix4x4 LookAtRH(const SVector3Df& v3Eye, const SVector3Df& v3Center, const SVector3Df& v3Up);

	/**
	 * Constructs a perspective projection matrix for 3D rendering.
	 * Maps view space coordinates to normalized device coordinates (NDC).
	 *
	 * Uses a 45-degree field of view and clips geometry between near (0.1)
	 * and far (1000.0) planes. Compatible with OpenGL's coordinate system.
	 *
	 * @return 4x4 perspective projection matrix in column-major order
	 */
	SMatrix4x4 PerspectiveRH(const SPersProjInfo& persProj);

	/**
	 * @brief Constructs an orthographic projection matrix (Right-Handed, Z depth from -1 to 1).
	 *
	 * This function creates a matrix that maps a rectangular viewing volume
	 * [Left, Right], [Bottom, Top], [NearZ, FarZ] into the Normalized Device Coordinates (NDC) cube.
	 * Parallel lines remain parallel (no perspective distortion).
	 *
	 * @param orthoProj Structure containing the orthographic viewing volume parameters.
	 * @return SMatrix4x4 The resulting 4x4 orthographic projection matrix.
	 */
	SMatrix4x4 OrthographicRH(const SOrthoProjInfo& orthoProj);

	/**
	 * @brief Initializes the matrix for a rotation around the X-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the X-axis. The matrix can be configured for either left-handed
	 * or right-handed coordinate systems, based on the `bLeftHanded` parameter.
	 *
	 * @param fRotX: The angle of rotation in radians.
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	SMatrix4x4 RotateX(const float fRotX);

	/**
	 * @brief Initializes the matrix for a rotation around the Y-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the Y-axis. The matrix can be configured for either left-handed
	 * or right-handed coordinate systems, based on the `bLeftHanded` parameter.
	 *
	 * @param fRotY: The angle of rotation in radians.
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	SMatrix4x4 RotateY(const float fRotY);

	/**
	 * Initializes the matrix for a rotation around the Z-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the Z-axis.
	 *
	 * @param fRotZ: The angle of rotation in radians.
	 */
	SMatrix4x4 RotateZ(const float fRotZ);

	/**
	 * @brief Initializes the matrix for a rotation transformation using the ZYX Euler rotation sequence.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rz * ry * rx. This order applies the
	 * rotations in the sequence: X-axis, Y-axis, then Z-axis, relative to the
	 * local coordinate system.
	 *
	 * @param RotateX: The angle of rotation around the X-axis in degrees.
	 * @param RotateY: The angle of rotation around the Y-axis in degrees.
	 * @param RotateZ: The angle of rotation around the Z-axis in degrees.
	 */
	SMatrix4x4 RotateZYX(const float& fRotateX, const float& fRotateY, const float& fRotateZ);

	/**
	 * @brief Initializes the matrix for a rotation transformation using the ZYX Euler rotation sequence.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rz * ry * rx. This order applies the
	 * rotations in the sequence: X-axis, Y-axis, then Z-axis, relative to the
	 * local coordinate system.
	 *
	 * @param vRotation: The 3D Vector holding rotation degrees.
	 */
	SMatrix4x4 RotateZYX(const SVector3Df& vRotation);

	/**
	 * Initializes the matrix for a rotation transformation using the XYZ Euler rotation sequence.
	 *
	 * @param RotateX: The angle of rotation around the X-axis in degrees.
	 * @param RotateY: The angle of rotation around the Y-axis in degrees.
	 * @param RotateZ: The angle of rotation around the Z-axis in degrees.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rx * ry * rz. This order applies the
	 * rotations in the sequence: Z-axis, Y-axis, then X-axis, relative to the
	 * global coordinate system.
	 */
	SMatrix4x4 RotateXYZ(const float& fRotateX, const float& fRotateY, const float& fRotateZ);


	/**
	 * @brief Initializes the matrix for a rotation transformation using the XYZ Euler rotation sequence.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rx * ry * rz. This order applies the
	 * rotations in the sequence: Z-axis, Y-axis, then X-axis, relative to the
	 * local coordinate system.
	 *
	 * @param vRotation: The 3D Vector holding rotation degrees.
	 */
	SMatrix4x4 RotateXYZ(const SVector3Df& vRotation);

	/**
	 * @brief Applies a rotation of a given angle around an arbitrary axis to an existing 4x4 matrix.
	 *
	 * This function implements the rotation matrix derived from Rodrigues' formula for
	 * the axis-angle representation. It calculates the rotation matrix and then
	 * pre-multiplies the input matrix, effectively applying the rotation transformation
	 * in world space (assuming standard column-major conventions).
	 *
	 * @param mat The existing 4x4 matrix (e.g., Model matrix) to be rotated.
	 * @param fAngle The angle of rotation in degrees.
	 * @param v3Axis The 3D vector defining the axis of rotation (will be normalized internally).
	 * @return SMatrix4x4 The resulting 4x4 transformation matrix (mat * Rotation).
	 */
	SMatrix4x4 Rotate(const SMatrix4x4& mat, float fAngle, const SVector3Df& v3Axis);

	/**
	 * @brief Applies a rotation of a given angle around an arbitrary axis to an existing 4x4 matrix.
	 *
	 * This function implements the rotation matrix derived from Rodrigues' formula for
	 * the axis-angle representation. It calculates the rotation matrix and then
	 * pre-multiplies the input matrix, effectively applying the rotation transformation
	 * in world space (assuming standard column-major conventions).
	 *
	 * @param mat The existing 4x4 matrix (e.g., Model matrix) to be rotated.
	 * @param fAngle The angle of rotation in degrees.
	 * @param fRotX The value of X Axis we will make our axis vector with (will be normalized internally).
	 * @param fRotY The value of Y Axis we will make our axis vector with (will be normalized internally).
	 * @param fRotZ The value of Z Axis we will make our axis vector with (will be normalized internally).
	 * @return SMatrix4x4 The resulting 4x4 transformation matrix (mat * Rotation).
	 */
	SMatrix4x4 Rotate(const SMatrix4x4& mat, float fAngle, const float fRotX, const float fRotY, const float fRotZ);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param fScaleX: Scaling factor along the X-axis.
	 * @param fScaleY: Scaling factor along the Y-axis.
	 * @param fScaleZ: Scaling factor along the Z-axis.
	 */
	SMatrix4x4 Scale(const float fScaleX, const float fScaleY, const float fScaleZ);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param v3Scale: Scaling vector along the X, Y, Z axis.
	 * @return the scaled matrix
	 */
	SMatrix4x4 Scale(const SVector3Df& v3Scale);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param fScale: Scaling factor along the Axis.
	 * @return the scaled matrix
	 */
	SMatrix4x4 Scale(const float fScale);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param mat: the matrix to apply scale on
	 * @param fScaleX: Scaling factor along the X-axis.
	 * @param fScaleY: Scaling factor along the Y-axis.
	 * @param fScaleZ: Scaling factor along the Z-axis.
	 * @return the scaled matrix
	 */
	SMatrix4x4 Scale(const SMatrix4x4& mat, const float fScaleX, const float fScaleY, const float fScaleZ);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param mat: the matrix to apply scale on
	 * @param fScale: Scaling factor along the Axis.
	 * @return the scaled matrix
	 */
	SMatrix4x4 Scale(const SMatrix4x4& mat, const float fScale);

	/**
	 * @brief Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param mat: the matrix to apply scale on
	 * @param v3Scale: Scaling factor along the X, Y, Z axis.
	 * @return the scaled matrix
	 */
	SMatrix4x4 Scale(const SMatrix4x4& mat, const SVector3Df& v3Scale);

	/**
	 * @brief Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param fX: The translation offset along the X-axis.
	 * @param fY: The translation offset along the Y-axis.
	 * @param fZ: The translation offset along the Z-axis.
	 * @return the translated matrix
	 */
	SMatrix4x4 Translate(const float fX, const float fY, const float fZ);

	/**
	 * @brief Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param v3Translate The translation offset along the X, Y, Z axis.
	 * @return the translated matrix
	 */
	SMatrix4x4 Translate(const SVector3Df& v3Translate);

	/**
	 * @brief Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param mat: the matrix to apply translate on.
	 * @param fX: The translation offset along the X-axis.
	 * @param fY: The translation offset along the Y-axis.
	 * @param fZ: The translation offset along the Z-axis.
	 * @return the translated matrix
	 */
	SMatrix4x4 Translate(const SMatrix4x4& mat, const float fX, const float fY, const float fZ);

	/**
	 * @brief Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param v3Translate The translation offset along the X, Y, Z axis.
	 * @return the translated matrix
	 */
	SMatrix4x4 Translate(const SMatrix4x4& mat, const SVector3Df& v3Translate);

	/**
	 * @brief Converts a custom 4x4 matrix to a GLM matrix.
	 *
	 * This function takes a custom SMatrix4x4 and converts it into a glm::mat4,
	 * allowing for interoperability between the custom math library and GLM.
	 *
	 * @param Mat The custom 4x4 matrix to convert.
	 * @return glm::mat4 The resulting GLM 4x4 matrix.
	 */
	glm::mat4 Mat4ToGLM(const SMatrix4x4& Mat);

	/**
	 * @brief Transforms a 3D point using the matrix.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * including translation, rotation, and scaling. The input point is treated
	 * as a homogeneous coordinate with w=1.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point.
	 */
	Vector3D TransformDirection(const Matrix4& mat, const Vector3D& dir);

	/**
	 * @brief Transforms a 3D point using the matrix, including translation.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * treating the point as a homogeneous coordinate with w=1. This includes
	 * translation, rotation, and scaling.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point.
	 */
	Vector3D TransformPoint(const Matrix4& mat, const Vector3D& point);

	/**
	 * @brief Transforms a 3D point using the matrix with perspective division.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * treating the point as a homogeneous coordinate with w=1. After the
	 * transformation, it performs perspective division by the computed W component.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point after perspective division.
	 */
	Vector3D TransformPointPerspective(const Matrix4& mat, const Vector3D& point);

	/**
	 * @brief Calculates the six clipping planes from a combined view-projection matrix.
	 *
	 * This function extracts the six clipping planes (left, right, bottom, top,
	 * near, far) from a combined view-projection matrix using the Gribb-Hartmann
	 * method. The resulting planes are represented in the form Ax + By + Cz + D = 0.
	 *
	 * @param matViewProj The combined view-projection matrix.
	 * @param vLeft Output parameter for the left clipping plane.
	 * @param vRight Output parameter for the right clipping plane.
	 * @param vTop Output parameter for the top clipping plane.
	 * @param vBottom Output parameter for the bottom clipping plane.
	 * @param vNear Output parameter for the near clipping plane.
	 * @param vFar Output parameter for the far clipping plane.
	 */
	void CalculateClipPlanes(const Matrix4& mat, Vector4D& vLeft, Vector4D& vRight, Vector4D& vTop, Vector4D& vBottom, Vector4D& vNear, Vector4D& vFar);

	/**
	 * Calculates the model transformation matrix (TRS).
	 * Computes the world transformation matrix by multiplying
	 * Translation, Rotation (ZYX order), and Scale matrices. This follows
	 * the column-major order convention (T * R * S).
	 * @param v3Position: The translation vector (x, y, z).
	 * @param v3Rotation: The rotation angles in degrees or radians (Pitch, Yaw, Roll).
	 * @param v3Scale: The scaling factors for each axis.
	 *
	 * @return The final 4x4 model-to-world transformation matrix.
	 */
	Matrix4 GetModelMatrix(const Vector3D& v3Position, const Vector3D& v3Rotation, const Vector3D& v3Scale);
	Matrix4 GetModelMatrix(const Vector3D& v3Position, const SQuaternion& qRotation, const Vector3D& v3Scale);

	Vector3D ExtractTranslation(const Matrix4& m);
	Vector3D ExtractScale(const Matrix4& m);

	float GetMaxAxisScale(const Matrix4& m);

	Matrix4 BuildSocketAttachmentWorld(const Matrix4& actorWorld, const Matrix4& boneModel, const STransform& localOffset);

} // namespace EngineMath - Matrix Part