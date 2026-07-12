#include "MathPCH.h"
#include "Transform.h"
#include "EngineMathMatrix.h"
#include "EngineMathVectors.h"
#include "EngineMathQuaternion.h"
#include "PrespectiveProjection.h"
#include "OrthographicProjection.h"

/**
 * @brief Initializes the matrix as an Identity matrix.
 *
 * This function returns a new matrix where all diagonal elements are set to 1.0f
 * and all off-diagonal elements are set to 0.0f.
 *
 * @param mat4 The matrix to be converted to identity (read-only).
 * @return The identity 4x4 matrix.
 */
SMatrix4x4 EngineMath::IdentityMat4()
{
	return SMatrix4x4(1.0f);
}

/**
 * Computes the inverse of a 4x4 matrix using the Adjugate/Cofactor method.
 * * This implementation is optimized and unrolled, operating on a Column-Major
 * matrix (OpenGL standard, M[col][row] indexing). The calculation is structured
 * to directly produce the Column-Major transpose of the Adjugate matrix.
 *
 * @param mat4 The 4x4 matrix to invert (read-only).
 * @return The inverted 4x4 matrix, or a zero matrix if the determinant is zero.
 */
SMatrix4x4 EngineMath::Inverse(const SMatrix4x4& mat4)
{
	// 1. --- CALCULATE 2x2 DETERMINANTS (COFACTORS / MINORS) ---
	// These 12 coefficients (Coef00 through Coef23) are the 2x2 determinants 
	// of the submatrices formed by rows/columns 1, 2, and 3.
	// These are reused multiple times in the 3x3 cofactor calculations.

	// Coefs for 3x3 minors missing column 0
	const float Coef00 = mat4[2][2] * mat4[3][3] - mat4[3][2] * mat4[2][3];
	const float Coef02 = mat4[1][2] * mat4[3][3] - mat4[3][2] * mat4[1][3];
	const float Coef03 = mat4[1][2] * mat4[2][3] - mat4[2][2] * mat4[1][3];

	const float Coef04 = mat4[2][1] * mat4[3][3] - mat4[3][1] * mat4[2][3];
	const float Coef06 = mat4[1][1] * mat4[3][3] - mat4[3][1] * mat4[1][3];
	const float Coef07 = mat4[1][1] * mat4[2][3] - mat4[2][1] * mat4[1][3];

	const float Coef08 = mat4[2][1] * mat4[3][2] - mat4[3][1] * mat4[2][3];
	const float Coef10 = mat4[1][1] * mat4[3][2] - mat4[3][1] * mat4[1][2];
	const float Coef11 = mat4[1][1] * mat4[2][2] - mat4[2][1] * mat4[1][2];

	const float Coef12 = mat4[2][0] * mat4[3][3] - mat4[3][0] * mat4[2][3];
	const float Coef14 = mat4[1][0] * mat4[3][3] - mat4[3][0] * mat4[1][3];
	const float Coef15 = mat4[1][0] * mat4[2][3] - mat4[2][0] * mat4[1][3];

	const float Coef16 = mat4[2][0] * mat4[3][2] - mat4[3][0] * mat4[2][2];
	const float Coef18 = mat4[1][0] * mat4[3][2] - mat4[3][0] * mat4[1][2];
	const float Coef19 = mat4[1][0] * mat4[2][2] - mat4[2][0] * mat4[1][2];

	const float Coef20 = mat4[2][0] * mat4[3][1] - mat4[3][0] * mat4[2][1];
	const float Coef22 = mat4[1][0] * mat4[3][1] - mat4[3][0] * mat4[1][1];
	const float Coef23 = mat4[1][0] * mat4[2][1] - mat4[2][0] * mat4[1][1];

	const SVector4Df Fac0(Coef00, Coef00, Coef02, Coef03);
	const SVector4Df Fac1(Coef04, Coef04, Coef06, Coef07);
	const SVector4Df Fac2(Coef08, Coef08, Coef10, Coef11);
	const SVector4Df Fac3(Coef12, Coef12, Coef14, Coef15);
	const SVector4Df Fac4(Coef16, Coef16, Coef18, Coef19);
	const SVector4Df Fac5(Coef20, Coef20, Coef22, Coef23);

	const SVector4Df Vec0(mat4[1][0], mat4[0][0], mat4[0][0], mat4[0][0]);
	const SVector4Df Vec1(mat4[1][1], mat4[0][1], mat4[0][1], mat4[0][1]);
	const SVector4Df Vec2(mat4[1][2], mat4[0][2], mat4[0][2], mat4[0][2]);
	const SVector4Df Vec3(mat4[1][3], mat4[0][3], mat4[0][3], mat4[0][3]);

	const SVector4Df Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	const SVector4Df Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	const SVector4Df Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	const SVector4Df Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	const SVector4Df SignA(+1.0f, -1.0f, +1.0f, -1.0f);
	const SVector4Df SignB(-1.0f, +1.0f, -1.0f, +1.0f);

	SMatrix4x4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

	const SVector4Df Row0(Inverse[0][0], Inverse[1][0], Inverse[2][0], Inverse[3][0]);

	const SVector4Df Dot0 = SVector4Df(mat4[0]) * Row0;

	const float Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	const float OneOverDeterminant = 1.0f / Dot1;

	return Inverse * OneOverDeterminant;
}

/**
 * @brief Transposes the matrix.
 *
 * This function returns a new matrix that is the transpose of the current matrix.
 *
 * @param mat the matrix to transpose
 * @return The transposed matrix.
 */
SMatrix4x4 EngineMath::Transpose(const SMatrix4x4& mat)
{
	SMatrix4x4 transposedMat{ 0.0f };
	transposedMat[0][0] = mat[0][0];
	transposedMat[0][1] = mat[1][0];
	transposedMat[0][2] = mat[2][0];
	transposedMat[0][3] = mat[3][0];

	transposedMat[1][0] = mat[0][1];
	transposedMat[1][1] = mat[1][1];
	transposedMat[1][2] = mat[2][1];
	transposedMat[1][3] = mat[3][1];

	transposedMat[2][0] = mat[0][2];
	transposedMat[2][1] = mat[1][2];
	transposedMat[2][2] = mat[2][2];
	transposedMat[2][3] = mat[3][2];

	transposedMat[3][0] = mat[0][3];
	transposedMat[3][1] = mat[1][3];
	transposedMat[3][2] = mat[2][3];
	transposedMat[3][3] = mat[3][3];

	return transposedMat;
}

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
float EngineMath::Determinant(const SMatrix4x4& mat)
{
	const float SubFactor00 = mat[2][2] * mat[3][3] - mat[3][2] * mat[2][3];
	const float SubFactor01 = mat[2][1] * mat[3][3] - mat[3][1] * mat[2][3];
	const float SubFactor02 = mat[2][1] * mat[3][2] - mat[3][1] * mat[2][2];
	const float SubFactor03 = mat[2][0] * mat[3][3] - mat[3][0] * mat[2][3];
	const float SubFactor04 = mat[2][0] * mat[3][2] - mat[3][0] * mat[2][2];
	const float SubFactor05 = mat[2][0] * mat[3][1] - mat[3][0] * mat[2][1];

	SVector4Df DetCof(
		+(mat[1][1] * SubFactor00 - mat[1][2] * SubFactor01 + mat[1][3] * SubFactor02),
		-(mat[1][0] * SubFactor00 - mat[1][2] * SubFactor03 + mat[1][3] * SubFactor04),
		+(mat[1][0] * SubFactor01 - mat[1][1] * SubFactor03 + mat[1][3] * SubFactor05),
		-(mat[1][0] * SubFactor02 - mat[1][1] * SubFactor04 + mat[1][2] * SubFactor05)
	);

	return mat[0][0] * DetCof[0] + mat[0][1] * DetCof[1] + mat[0][2] * DetCof[2] + mat[0][3] * DetCof[3];
}

/**
 * Constructs a view matrix that transforms world coordinates to camera space.
 *
 * @param v3Eye    The camera position in world space
 * @param v3Center The point the camera is looking at
 * @param v3Up     The up direction vector (typically (0,1,0))
 * @return         A 4x4 view matrix in column-major order
 */
SMatrix4x4 EngineMath::LookAtRH(const SVector3Df& v3Eye, const SVector3Df& v3Center, const SVector3Df& v3Up)
{
	// Forward vector: f = (v_center - v_eye) / ||v_center - v_eye||
	SVector3Df f = v3Center - v3Eye;
	f.normalize();

	// Right vector: s = (f x v_up) / ||f x v_up||
	// where 'x' denotes cross product
	SVector3Df s = f.cross(v3Up);
	s.normalize();

	// Up vector: u = s x f
	// where 'x' denotes cross product
	SVector3Df u = s.cross(f);

	// Construct the view matrix M:
	//
	//     [  s_x    s_y    s_z   -s.p ]
	// M = [  u_x    u_y    u_z   -u.p ]
	//     [ -f_x   -f_y   -f_z    f.p ]
	//     [   0      0      0      1  ]
	//
	// where s = (s_x, s_y, s_z) is the right vector
	//       u = (u_x, u_y, u_z) is the up vector
	//       f = (f_x, f_y, f_z) is the forward vector
	//       p is the camera position (v3Eye)
	//       '.' denotes dot product
	//
	// Matrix is indexed as M[column][row] for OpenGL compatibility
	SMatrix4x4 viewMatrix{ 1.0f };

	// Rotation components (basis vectors)
	viewMatrix[0][0] = s.x;   viewMatrix[1][0] = s.y;   viewMatrix[2][0] = s.z;		// Row 0: right vector
	viewMatrix[0][1] = u.x;   viewMatrix[1][1] = u.y;   viewMatrix[2][1] = u.z;		// Row 1: up vector
	viewMatrix[0][2] = -f.x;  viewMatrix[1][2] = -f.y;  viewMatrix[2][2] = -f.z;	// Row 2: -forward vector

	// Translation components (negated dot products with camera position)
	viewMatrix[3][0] = -s.dot(v3Eye);	// -s.p
	viewMatrix[3][1] = -u.dot(v3Eye);	// -u.p
	viewMatrix[3][2] = f.dot(v3Eye);	//  f.p

	return (viewMatrix);
}

/**
 * Constructs a perspective projection matrix for 3D rendering.
 * Maps view space coordinates to normalized device coordinates (NDC).
 *
 * Uses a 45-degree field of view and clips geometry between near (0.1)
 * and far (1000.0) planes. Compatible with OpenGL's coordinate system.
 *
 * @return 4x4 perspective projection matrix in column-major order
 */
SMatrix4x4 EngineMath::PerspectiveRH(const SPersProjInfo& persProj)
{
	// Perspective projection parameters
	float Fov = persProj.FOV;  // Field of view in degrees
	float HalfTanFOV = std::tan(Fov / 2.0f);
	float AspectRatio = persProj.Width / persProj.Height;
	float NearZ = persProj.zNear;    // Near clipping plane
	float FarZ = persProj.zFar;  // Far clipping plane
	float ZRange = FarZ - NearZ;

	// Construct the perspective projection matrix P:
	//
	//     [ 1/(t*a)     0           0              0     ]
	// P = [    0      1/t           0              0     ]
	//     [    0       0     -(f+n)/(f-n)   -2fn/(f-n)   ]
	//     [    0       0          -1              0      ]
	//
	// where t = tan(fov/2)  (half tangent of field of view)
	//       a = aspect ratio (width/height)
	//       n = near clipping plane distance
	//       f = far clipping plane distance
	//
	// This maps view space to clip space with:
	//   X: [-aspect*tan(fov/2), aspect*tan(fov/2)] -> [-1, 1]
	//   Y: [-tan(fov/2), tan(fov/2)] -> [-1, 1]
	//   Z: [-n, -f] -> [-1, 1] (OpenGL depth range)

	SMatrix4x4 CameraProjectionMatrix{};

	// Column 0: X-axis scaling (accounts for aspect ratio)
	CameraProjectionMatrix[0][0] = 1.0f / (HalfTanFOV * AspectRatio);
	CameraProjectionMatrix[0][1] = 0.0f;
	CameraProjectionMatrix[0][2] = 0.0f;
	CameraProjectionMatrix[0][3] = 0.0f;

	// Column 1: Y-axis scaling (based on FOV)
	CameraProjectionMatrix[1][0] = 0.0f;
	CameraProjectionMatrix[1][1] = 1.0f / HalfTanFOV;
	CameraProjectionMatrix[1][2] = 0.0f;
	CameraProjectionMatrix[1][3] = 0.0f;

	// Column 2: Z-axis mapping and perspective division trigger
	CameraProjectionMatrix[2][0] = 0.0f;
	CameraProjectionMatrix[2][1] = 0.0f;
#if defined(MAT_FORCE_DEPTH_ZERO_TO_ONE)
	// Maps Z to [0, 1]
	CameraProjectionMatrix[2][2] = FarZ / (NearZ - FarZ);  // Z remapping
#else
	// Maps Z to [-1, 1]
	CameraProjectionMatrix[2][2] = -(FarZ + NearZ) / (FarZ - NearZ);  // Z remapping
#endif
	CameraProjectionMatrix[2][3] = -1.0f;  // Triggers perspective division (w = -z)

	// Column 3: Z translation component
	CameraProjectionMatrix[3][0] = 0.0f;
	CameraProjectionMatrix[3][1] = 0.0f;
#if defined(MAT_FORCE_DEPTH_ZERO_TO_ONE)
	// Maps Z to [0, 1]
	CameraProjectionMatrix[3][2] = -(FarZ * NearZ) / (FarZ - NearZ);  // Z offset
#else
	// Maps Z to [-1, 1]
	CameraProjectionMatrix[3][2] = -(2.0f * FarZ * NearZ) / (FarZ - NearZ);  // Z offset
#endif
	CameraProjectionMatrix[3][3] = 0.0f;

	return (CameraProjectionMatrix);
}

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
SMatrix4x4 EngineMath::OrthographicRH(const SOrthoProjInfo& orthoProj)
{
	// Orthographic projection parameters
	float Left = orthoProj.Left;
	float Right = orthoProj.Right;
	float Bottom = orthoProj.Bottom;
	float Top = orthoProj.Top;
	float NearZ = orthoProj.zNear;
	float FarZ = orthoProj.zFar;

	// Pre-calculate differences and sums for matrix elements
	float R_minus_L = Right - Left;
	float T_minus_B = Top - Bottom;
	float F_minus_N = FarZ - NearZ;

	// Construct the orthographic projection matrix P:
	//
	// 			[ 2/(r-l)       0             0        -(r+l)/(r-l) ]
	// 		P = [    0       2/(t-b)          0        -(t+b)/(t-b) ]
	// 			[    0           0        -2/(f-n)     -(f+n)/(f-n) ]
	// 			[    0           0             0             1      ]
	//
	// where l = Left boundary of the view volume
	// 		 r = Right boundary of the view volume
	// 		 t = Top boundary of the view volume
	// 		 b = Bottom boundary of the view volume
	// 		 n = Near clipping plane distance
	// 		 f = Far clipping plane distance
	//
	// This maps the view volume to the clip space NDC cube [-1, 1] with:
	// 	 X: [l, r] -> [-1, 1]
	// 	 Y: [b, t] -> [-1, 1]
	// 	 Z: [-n, -f] -> [-1, 1] (OpenGL depth range)

	// Explicitly clear matrix to 0 to avoid garbage or identity layout collision
	SMatrix4x4 CameraProjectionMatrix{ 0 };

	// Column 0: X scaling & translation handling
	CameraProjectionMatrix[0][0] = 2.0f / R_minus_L;
	CameraProjectionMatrix[0][1] = 0.0f;
	CameraProjectionMatrix[0][2] = 0.0f;
	CameraProjectionMatrix[0][3] = 0.0f;

	// Column 1: Y scaling & translation handling
	CameraProjectionMatrix[1][0] = 0.0f;
	CameraProjectionMatrix[1][1] = 2.0f / T_minus_B;
	CameraProjectionMatrix[1][2] = 0.0f;
	CameraProjectionMatrix[1][3] = 0.0f;

	// Column 2: Z scaling
	CameraProjectionMatrix[2][0] = 0.0f;
	CameraProjectionMatrix[2][1] = 0.0f;
#if defined(MAT_FORCE_DEPTH_ZERO_TO_ONE)
	CameraProjectionMatrix[2][2] = -1.0f / F_minus_N;
#else
	CameraProjectionMatrix[2][2] = -2.0f / F_minus_N;
#endif
	CameraProjectionMatrix[2][3] = 0.0f;

	// Column 3: Translation terms & W-component
	CameraProjectionMatrix[3][0] = -(Right + Left) / R_minus_L;
	CameraProjectionMatrix[3][1] = -(Top + Bottom) / T_minus_B;
#if defined(MAT_FORCE_DEPTH_ZERO_TO_ONE)
	CameraProjectionMatrix[3][2] = -(NearZ) / F_minus_N;
#else
	CameraProjectionMatrix[3][2] = -(FarZ + NearZ) / F_minus_N;
#endif
	CameraProjectionMatrix[3][3] = 1.0f; // Critical for Orthographic W-divide bypass

	return (CameraProjectionMatrix);
}

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
SMatrix4x4 EngineMath::RotateX(const float fRotX)
{
	SMatrix4x4 rotationMatrix{ 1 };

	// Column 1
	rotationMatrix[0][0] = 1.0f;
	rotationMatrix[0][1] = 0.0f;
	rotationMatrix[0][2] = 0.0f;
	rotationMatrix[0][3] = 0.0f;

	// Column 2
	rotationMatrix[1][0] = 0.0f;
	rotationMatrix[1][1] = std::cosf(fRotX);
	rotationMatrix[1][2] = std::sinf(fRotX);
	rotationMatrix[1][3] = 0.0f;

	// Column 3
	rotationMatrix[2][0] = 0.0f;
	rotationMatrix[2][1] = -std::sinf(fRotX);
	rotationMatrix[2][2] = std::cosf(fRotX);
	rotationMatrix[2][3] = 0.0f;

	// Column 4
	rotationMatrix[3][0] = 0.0f;
	rotationMatrix[3][1] = 0.0f;
	rotationMatrix[3][2] = 0.0f;
	rotationMatrix[3][3] = 1.0f;

	return (rotationMatrix);
}

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
SMatrix4x4 EngineMath::RotateY(const float fRotY)
{
	SMatrix4x4 rotationMatrix{ 1 };

	// Column 1
	rotationMatrix[0][0] = std::cosf(fRotY);
	rotationMatrix[0][1] = 0.0f;
	rotationMatrix[0][2] = -std::sinf(fRotY);
	rotationMatrix[0][3] = 0.0f;

	// Column 2
	rotationMatrix[1][0] = 0.0f;
	rotationMatrix[1][1] = 1.0f;
	rotationMatrix[1][2] = 0.0f;
	rotationMatrix[1][3] = 0.0f;

	// Column 3
	rotationMatrix[2][0] = std::sinf(fRotY);;
	rotationMatrix[2][1] = 0.0f;
	rotationMatrix[2][2] = std::cosf(fRotY);
	rotationMatrix[2][3] = 0.0f;

	// Column 4
	rotationMatrix[3][0] = 0.0f;
	rotationMatrix[3][1] = 0.0f;
	rotationMatrix[3][2] = 0.0f;
	rotationMatrix[3][3] = 1.0f;

	return (rotationMatrix);
}

/**
 * Initializes the matrix for a rotation around the Z-axis.
 *
 * This function sets up a 4x4 transformation matrix for a rotation
 * around the Z-axis.
 *
 * @param fRotZ: The angle of rotation in radians.
 */
SMatrix4x4 EngineMath::RotateZ(const float fRotZ)
{
	SMatrix4x4 rotationMatrix{ 1 };

	// Column 1
	rotationMatrix[0][0] = std::cosf(fRotZ);
	rotationMatrix[0][1] = std::sinf(fRotZ);
	rotationMatrix[0][2] = 0.0f;
	rotationMatrix[0][3] = 0.0f;

	// Column 2
	rotationMatrix[1][0] = -std::sinf(fRotZ);
	rotationMatrix[1][1] = std::cosf(fRotZ);
	rotationMatrix[1][2] = 0.0f;
	rotationMatrix[1][3] = 0.0f;

	// Column 3
	rotationMatrix[2][0] = 0.0f;
	rotationMatrix[2][1] = 0.0f;
	rotationMatrix[2][2] = 1.0f;
	rotationMatrix[2][3] = 0.0f;

	// Column 4
	rotationMatrix[3][0] = 0.0f;
	rotationMatrix[3][1] = 0.0f;
	rotationMatrix[3][2] = 0.0f;
	rotationMatrix[3][3] = 1.0f;

	return (rotationMatrix);
}

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
SMatrix4x4 EngineMath::RotateZYX(const float& fRotateX, const float& fRotateY, const float& fRotateZ)
{
	SMatrix4x4 matX{ 1 }, matY{ 1 }, matZ{ 1 }, finalRotation{ 1 };
	const float x = ToRadians(fRotateX);
	const float y = ToRadians(fRotateY);
	const float z = ToRadians(fRotateZ);

	matX = RotateX(x);
	matY = RotateY(y);
	matZ = RotateZ(z);

	/* ZYX Euler rotation sequence */
	finalRotation = matZ * matY * matX;

	return (finalRotation);
}

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
SMatrix4x4 EngineMath::RotateZYX(const SVector3Df& vRotation)
{
	return (RotateZYX(vRotation.x, vRotation.y, vRotation.z));
}

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
SMatrix4x4 EngineMath::RotateXYZ(const float& fRotateX, const float& fRotateY, const float& fRotateZ)
{
	SMatrix4x4 matX{ 1 }, matY{ 1 }, matZ{ 1 }, finalRotation{ 1 };
	const float x = ToRadians(fRotateX);
	const float y = ToRadians(fRotateY);
	const float z = ToRadians(fRotateZ);

	matX = RotateX(x);
	matY = RotateY(y);
	matZ = RotateZ(z);

	/* XYZ Euler rotation sequence */
	finalRotation = matX * matY * matZ;

	return (finalRotation);
}

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
SMatrix4x4 EngineMath::RotateXYZ(const SVector3Df& vRotation)
{
	return (RotateXYZ(vRotation.x, vRotation.y, vRotation.z));
}


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
SMatrix4x4 EngineMath::Rotate(const SMatrix4x4& mat, float fAngle, const SVector3Df& v3Axis)
{
	const float fA = ToRadians(fAngle);
	const float fC = std::cos(fA);
	const float fS = std::sin(fA);

	const SVector3Df vAxis = Normalize(v3Axis);
	const SVector3Df vTemp = SVector3Df((1.0f - fC) * vAxis);

	SMatrix4x4 rotateMat{ 0.0f };
	rotateMat[0][0] = fC + vTemp.x * vAxis.x;
	rotateMat[0][1] = vTemp.x * vAxis.y + fS * vAxis.z;
	rotateMat[0][2] = vTemp.x * vAxis.z - fS * vAxis.y;
	rotateMat[0][3] = 0.0f;

	rotateMat[1][0] = vTemp.y * vAxis.x - fS * vAxis.z;
	rotateMat[1][1] = fC + vTemp.y * vAxis.y;
	rotateMat[1][2] = vTemp.y * vAxis.z + fS * vAxis.x;
	rotateMat[1][3] = 0.0f;

	rotateMat[2][0] = vTemp.z * vAxis.x + fS * vAxis.y;
	rotateMat[2][1] = vTemp.z * vAxis.y - fS * vAxis.x;
	rotateMat[2][2] = fC + vTemp.z * vAxis.z;
	rotateMat[2][3] = 0.0f;

	SMatrix4x4 resultsMat{ 0.0f };
	resultsMat[0] = mat[0] * rotateMat[0][0] + mat[1] * rotateMat[0][1] + mat[2] * rotateMat[0][2];
	resultsMat[1] = mat[0] * rotateMat[1][0] + mat[1] * rotateMat[1][1] + mat[2] * rotateMat[1][2];
	resultsMat[2] = mat[0] * rotateMat[2][0] + mat[1] * rotateMat[2][1] + mat[2] * rotateMat[2][2];
	resultsMat[3] = mat[3];

	return (resultsMat);
}

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
SMatrix4x4 EngineMath::Rotate(const SMatrix4x4& mat, float fAngle, const float fRotX, const float fRotY, const float fRotZ)
{
	const float fA = ToRadians(fAngle);
	const float fC = std::cos(fA);
	const float fS = std::sin(fA);

	SVector3Df v3Axis(fRotX, fRotY, fRotZ);
	const SVector3Df vAxis = Normalize(v3Axis);
	const SVector3Df vTemp = SVector3Df((1.0f - fC) * vAxis);

	SMatrix4x4 rotateMat{ 0.0f };
	rotateMat[0][0] = fC + vTemp.x * vAxis.x;
	rotateMat[0][1] = vTemp.x * vAxis.y + fS * vAxis.z;
	rotateMat[0][2] = vTemp.x * vAxis.z - fS * vAxis.y;
	rotateMat[0][3] = 0.0f;

	rotateMat[1][0] = vTemp.y * vAxis.x - fS * vAxis.z;
	rotateMat[1][1] = fC + vTemp.y * vAxis.y;
	rotateMat[1][2] = vTemp.y * vAxis.z + fS * vAxis.x;
	rotateMat[1][3] = 0.0f;

	rotateMat[2][0] = vTemp.z * vAxis.x + fS * vAxis.y;
	rotateMat[2][1] = vTemp.z * vAxis.y - fS * vAxis.x;
	rotateMat[2][2] = fC + vTemp.z * vAxis.z;
	rotateMat[2][3] = 0.0f;

	SMatrix4x4 resultsMat{ 0.0f };
	resultsMat[0] = mat[0] * rotateMat[0][0] + mat[1] * rotateMat[0][1] + mat[2] * rotateMat[0][2];
	resultsMat[1] = mat[0] * rotateMat[1][0] + mat[1] * rotateMat[1][1] + mat[2] * rotateMat[1][2];
	resultsMat[2] = mat[0] * rotateMat[2][0] + mat[1] * rotateMat[2][1] + mat[2] * rotateMat[2][2];
	resultsMat[3] = mat[3];

	return (resultsMat);
}

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
SMatrix4x4 EngineMath::Scale(const float fScaleX, const float fScaleY, const float fScaleZ)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0][0] = fScaleX;
	scaleMatrix[0][1] = 0.0f;
	scaleMatrix[0][2] = 0.0f;
	scaleMatrix[0][3] = 0.0f;

	// Column 2
	scaleMatrix[1][0] = 0.0f;
	scaleMatrix[1][1] = fScaleY;
	scaleMatrix[1][2] = 0.0f;
	scaleMatrix[1][3] = 0.0f;

	// Column 3
	scaleMatrix[2][0] = 0.0f;
	scaleMatrix[2][1] = 0.0f;
	scaleMatrix[2][2] = fScaleZ;
	scaleMatrix[2][3] = 0.0f;

	// Column 4
	scaleMatrix[3][0] = 0.0f;
	scaleMatrix[3][1] = 0.0f;
	scaleMatrix[3][2] = 0.0f;
	scaleMatrix[3][3] = 1.0f;

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Scale(const SVector3Df& v3Scale)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0][0] = v3Scale.x;
	scaleMatrix[0][1] = 0.0f;
	scaleMatrix[0][2] = 0.0f;
	scaleMatrix[0][3] = 0.0f;

	// Column 2
	scaleMatrix[1][0] = 0.0f;
	scaleMatrix[1][1] = v3Scale.y;
	scaleMatrix[1][2] = 0.0f;
	scaleMatrix[1][3] = 0.0f;

	// Column 3
	scaleMatrix[2][0] = 0.0f;
	scaleMatrix[2][1] = 0.0f;
	scaleMatrix[2][2] = v3Scale.z;
	scaleMatrix[2][3] = 0.0f;

	// Column 4
	scaleMatrix[3][0] = 0.0f;
	scaleMatrix[3][1] = 0.0f;
	scaleMatrix[3][2] = 0.0f;
	scaleMatrix[3][3] = 1.0f;

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Scale(const float fScale)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0][0] = fScale;
	scaleMatrix[0][1] = 0.0f;
	scaleMatrix[0][2] = 0.0f;
	scaleMatrix[0][3] = 0.0f;

	// Column 2
	scaleMatrix[1][0] = 0.0f;
	scaleMatrix[1][1] = fScale;
	scaleMatrix[1][2] = 0.0f;
	scaleMatrix[1][3] = 0.0f;

	// Column 3
	scaleMatrix[2][0] = 0.0f;
	scaleMatrix[2][1] = 0.0f;
	scaleMatrix[2][2] = fScale;
	scaleMatrix[2][3] = 0.0f;

	// Column 4
	scaleMatrix[3][0] = 0.0f;
	scaleMatrix[3][1] = 0.0f;
	scaleMatrix[3][2] = 0.0f;
	scaleMatrix[3][3] = 1.0f;

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Scale(const SMatrix4x4& mat, const float fScaleX, const float fScaleY, const float fScaleZ)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0] = mat[0] * fScaleX;

	// Column 2
	scaleMatrix[1] = mat[1] * fScaleY;

	// Column 3
	scaleMatrix[2] = mat[2] * fScaleZ;

	// Column 4
	scaleMatrix[3] = mat[3];

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Scale(const SMatrix4x4& mat, const float fScale)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0] = mat[0] * fScale;

	// Column 2
	scaleMatrix[1] = mat[1] * fScale;

	// Column 3
	scaleMatrix[2] = mat[2] * fScale;

	// Column 4
	scaleMatrix[3] = mat[3];

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Scale(const SMatrix4x4& mat, const SVector3Df& v3Scale)
{
	SMatrix4x4 scaleMatrix{ 0.0f };

	// Column 1
	scaleMatrix[0] = mat[0] * v3Scale.x;

	// Column 2
	scaleMatrix[1] = mat[1] * v3Scale.y;

	// Column 3
	scaleMatrix[2] = mat[2] * v3Scale.z;

	// Column 4
	scaleMatrix[3] = mat[3];

	return (scaleMatrix);
}

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
SMatrix4x4 EngineMath::Translate(const float fX, const float fY, const float fZ)
{
	SMatrix4x4 translateMatrix = SMatrix4x4(1.0f);

	const SVector3Df v3Translate(fX, fY, fZ);
	translateMatrix[3] = translateMatrix[0] * v3Translate.x + translateMatrix[1] * v3Translate.y + translateMatrix[2] * v3Translate.z + translateMatrix[3];
	return (translateMatrix);
}

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
SMatrix4x4 EngineMath::Translate(const SVector3Df& v3Translate)
{
	SMatrix4x4 translateMatrix = SMatrix4x4(1.0f);
	translateMatrix[3] = translateMatrix[0] * v3Translate.x + translateMatrix[1] * v3Translate.y + translateMatrix[2] * v3Translate.z + translateMatrix[3];
	return (translateMatrix);
}

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
SMatrix4x4 EngineMath::Translate(const SMatrix4x4& mat, const float fX, const float fY, const float fZ)
{
	SMatrix4x4 translateMatrix(mat);
	const SVector3Df v3Translate(fX, fY, fZ);
	translateMatrix[3] = mat[0] * v3Translate.x + mat[1] * v3Translate.y + mat[2] * v3Translate.z + mat[3];
	return (translateMatrix);
}

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
SMatrix4x4 EngineMath::Translate(const SMatrix4x4& mat, const SVector3Df& v3Translate)
{
	SMatrix4x4 translateMatrix(mat);
	translateMatrix[3] = mat[0] * v3Translate.x + mat[1] * v3Translate.y + mat[2] * v3Translate.z + mat[3];
	return (translateMatrix);
}

/**
 * @brief Converts a custom 4x4 matrix to a GLM matrix.
 *
 * This function takes a custom SMatrix4x4 and converts it into a glm::mat4,
 * allowing for interoperability between the custom math library and GLM.
 *
 * @param Mat The custom 4x4 matrix to convert.
 * @return glm::mat4 The resulting GLM 4x4 matrix.
 */
glm::mat4 EngineMath::Mat4ToGLM(const SMatrix4x4& Mat)
{
	glm::mat4 value{};
	value[0][0] = Mat[0][0]; value[0][1] = Mat[0][1]; value[0][2] = Mat[0][2]; value[0][3] = Mat[0][3];
	value[1][0] = Mat[1][0]; value[1][1] = Mat[1][1]; value[1][2] = Mat[1][2]; value[1][3] = Mat[1][3];
	value[2][0] = Mat[2][0]; value[2][1] = Mat[2][1]; value[2][2] = Mat[2][2]; value[2][3] = Mat[2][3];
	value[3][0] = Mat[3][0]; value[3][1] = Mat[3][1]; value[3][2] = Mat[3][2]; value[3][3] = Mat[3][3];
	return (value);
}

/**
 * @brief Transforms a 3D point using the matrix.
 *
 * This function applies the full 4x4 transformation to a 3D point,
 * including translation, rotation, and scaling. The input point is treated
 * as a homogeneous coordinate with w=1.
 *
 * @param mat The transformation matrix.
 * @param point The 3D point to transform.
 * @return The transformed 3D point.
 */
Vector3D EngineMath::TransformDirection(const Matrix4& mat, const Vector3D& dir)
{
	// Column0 * dir.x + Column1 * dir.y + Column2 * dir.z
	return Vector3D(
		mat[0][0] * dir.x + mat[1][0] * dir.y + mat[2][0] * dir.z,
		mat[0][1] * dir.x + mat[1][1] * dir.y + mat[2][1] * dir.z,
		mat[0][2] * dir.x + mat[1][2] * dir.y + mat[2][2] * dir.z
	);
	// Notice we ignore m[3][0], m[3][1], m[3][2] (the translation)
}

/**
 * @brief Transforms a 3D point using the matrix, including translation.
 *
 * This function applies the full 4x4 transformation to a 3D point,
 * treating the point as a homogeneous coordinate with w=1. This includes
 * translation, rotation, and scaling.
 *
 * @param mat The transformation matrix.
 * @param point The 3D point to transform.
 * @return The transformed 3D point.
 */
Vector3D EngineMath::TransformPoint(const Matrix4& mat, const Vector3D& point)
{
	return Vector3D(
		(mat[0][0] * point.x + mat[1][0] * point.y + mat[2][0] * point.z) + mat[3][0],
		(mat[0][1] * point.x + mat[1][1] * point.y + mat[2][1] * point.z) + mat[3][1],
		(mat[0][2] * point.x + mat[1][2] * point.y + mat[2][2] * point.z) + mat[3][2]
	);
}

/**
 * @brief Transforms a 3D point using the matrix with perspective division.
 *
 * This function applies the full 4x4 transformation to a 3D point,
 * treating the point as a homogeneous coordinate with w=1. After the
 * transformation, it performs perspective division by the computed W component.
 *
 * @param mat The transformation matrix.
 * @param point The 3D point to transform.
 * @return The transformed 3D point after perspective division.
 */
Vector3D EngineMath::TransformPointPerspective(const Matrix4& mat, const Vector3D& point)
{
	// Calculate W component
	float w = mat[0][3] * point.x + mat[1][3] * point.y + mat[2][3] * point.z + mat[3][3];

	// Transform the point
	Vector3D result = EngineMath::TransformPoint(mat, point);

	// Perform perspective divide if W is not 1
	if (w != 1.0f && std::abs(w) > 0.000001f)
	{
		result /= w; // Divide by W to get back to Normalized Device Coordinates
	}
	return (result);
}

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
 * @param vBottom Output parameter for the bottom clipping plane.
 * @param vTop Output parameter for the top clipping plane.
 * @param vNear Output parameter for the near clipping plane.
 * @param vFar Output parameter for the far clipping plane.
 */
void EngineMath::CalculateClipPlanes(const Matrix4& matViewProj, Vector4D& vLeft, Vector4D& vRight, Vector4D& vTop, Vector4D& vBottom, Vector4D& vNear, Vector4D& vFar)
{
	// mat4[Column][Row]
	SVector4Df Row1(matViewProj[0][0], matViewProj[1][0], matViewProj[2][0], matViewProj[3][0]);
	SVector4Df Row2(matViewProj[0][1], matViewProj[1][1], matViewProj[2][1], matViewProj[3][1]);
	SVector4Df Row3(matViewProj[0][2], matViewProj[1][2], matViewProj[2][2], matViewProj[3][2]);
	SVector4Df Row4(matViewProj[0][3], matViewProj[1][3], matViewProj[2][3], matViewProj[3][3]);

	// The standard Gribb-Hartmann formulas for OpenGL:
	vLeft = Row4 + Row1;
	vRight = Row4 - Row1;
	vBottom = Row4 + Row2;
	vTop = Row4 - Row2;
	vNear = Row4 + Row3;
	vFar = Row4 - Row3;

	// Normalize the planes
	vLeft = EngineMath::Normalize(vLeft);
	vRight = EngineMath::Normalize(vRight);
	vBottom = EngineMath::Normalize(vBottom);
	vTop = EngineMath::Normalize(vTop);
	vNear = EngineMath::Normalize(vNear);
	vFar = EngineMath::Normalize(vFar);
}

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
Matrix4 EngineMath::GetModelMatrix(const Vector3D& v3Position, const Vector3D& v3Rotation, const Vector3D& v3Scale)
{
	Matrix4 matTranslation = EngineMath::Translate(v3Position);
	Matrix4 matRotation = EngineMath::RotateZYX(v3Rotation);
	Matrix4 matScale = EngineMath::Scale(v3Scale);
	
	// Standard Column-Major Order: T * R * S
	Matrix4 matWorldTransformation = matTranslation * matRotation * matScale;
	return (matWorldTransformation);
}

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
Matrix4 EngineMath::GetModelMatrix(const Vector3D& v3Position, const SQuaternion& qRotation, const Vector3D& v3Scale)
{
	Matrix4 matTranslation = EngineMath::Translate(v3Position);
	Matrix4 matRotation = EngineMath::ToMatrix4(qRotation);
	Matrix4 matScale = EngineMath::Scale(v3Scale);

	// Standard Column-Major Order: T * R * S
	Matrix4 matWorldTransformation = matTranslation * matRotation * matScale;
	return (matWorldTransformation);
}

Vector3D EngineMath::ExtractTranslation(const Matrix4& m)
{
	return Vector3D(m[3][0], m[3][1], m[3][2]);
}

Vector3D EngineMath::ExtractScale(const Matrix4& m)
{
	Vector3D xAxis(m[0][0], m[0][1], m[0][2]);
	Vector3D yAxis(m[1][0], m[1][1], m[1][2]);
	Vector3D zAxis(m[2][0], m[2][1], m[2][2]);

	return Vector3D(
		xAxis.length(),
		yAxis.length(),
		zAxis.length()
	);
}

float EngineMath::GetMaxAxisScale(const Matrix4& m)
{
	const float sx = std::sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]);
	const float sy = std::sqrt(m[0][1] * m[0][1] + m[1][1] * m[1][1] + m[2][1] * m[2][1]);
	const float sz = std::sqrt(m[0][2] * m[0][2] + m[1][2] * m[1][2] + m[2][2] * m[2][2]);

	return std::fmax(sx, std::fmax(sy, sz));
}

Matrix4 EngineMath::BuildSocketAttachmentWorld(const Matrix4& actorWorld, const Matrix4& boneModel, const STransform& localOffset)
{
	Matrix4 local = Matrix4(1.0f);
	local = EngineMath::Translate(local, localOffset.m_v3Position);
	local = local * EngineMath::ToMatrix4(EngineMath::Normalize(localOffset.m_qOrientation));
	local = EngineMath::Scale(local, localOffset.m_v3Scale);

	return actorWorld * boneModel * local;
}
