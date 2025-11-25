#include "Camera.h"
#include "Window.h"

CCamera::CCamera(CWindow* pWindow)
{
	m_v3CameraPosition = Vector3D(0.0f, 0.0f, 0.0f);
	m_v3Up = Vector3D(0.0f, 1.0f, 0.0f);
	m_v3Target = Vector3D(0.0f, 0.0f, 1.0f);

	m_pWindow = pWindow;
}

Matrix4 CCamera::GetMatrix() const
{
	Vector3D U = m_v3Up.cross(m_v3Target);
	U.normalize();

	Vector3D V = m_v3Target.cross(U);
	V.normalize();

	Vector3D N = m_v3Target;
	N.normalize();
	Matrix4 CameraRotationMatrix{};
	CameraRotationMatrix.mat[0][0] = U.x;
	CameraRotationMatrix.mat[0][1] = U.y;
	CameraRotationMatrix.mat[0][2] = U.z;
	CameraRotationMatrix.mat[0][3] = 0.0f;

	CameraRotationMatrix.mat[1][0] = V.x;
	CameraRotationMatrix.mat[1][1] = V.y;
	CameraRotationMatrix.mat[1][2] = V.z;
	CameraRotationMatrix.mat[1][3] = 0.0f;

	CameraRotationMatrix.mat[2][0] = N.x;
	CameraRotationMatrix.mat[2][1] = N.y;
	CameraRotationMatrix.mat[2][2] = N.z;
	CameraRotationMatrix.mat[2][3] = 0.0f;

	CameraRotationMatrix.mat[3][0] = 0.0f;
	CameraRotationMatrix.mat[3][1] = 0.0f;
	CameraRotationMatrix.mat[3][2] = 0.0f;
	CameraRotationMatrix.mat[3][3] = 1.0f;

	Matrix4 CameraTranslationMatrix{};
	CameraTranslationMatrix.InitIdentity();
	CameraTranslationMatrix.mat[0][3] = -m_v3CameraPosition.x;
	CameraTranslationMatrix.mat[1][3] = -m_v3CameraPosition.y;
	CameraTranslationMatrix.mat[2][3] = -m_v3CameraPosition.z;

	Matrix4 finalMat = CameraRotationMatrix * CameraTranslationMatrix;
	return (finalMat);
}

Matrix4 CCamera::GetProjectionMatrix() const
{
	// Prespective Projection
	GLfloat Fov = 45.0f; // 45 Degrees
	GLfloat HalfTanFOV = std::tan(ToRadian(Fov / 2.0f));
	GLfloat AspectRatio = m_pWindow->GetWidthF() / m_pWindow->GetHeightF();
	GLfloat NearZ = 0.1f;
	GLfloat FarZ = 1000.0f;
	GLfloat ZRange = FarZ - NearZ;

	Matrix4 CameraProjectionMatrix{};

	// Row 0 (X mapping)
	CameraProjectionMatrix.mat[0][0] = 1.0f / (HalfTanFOV * AspectRatio);
	CameraProjectionMatrix.mat[0][1] = 0.0f;
	CameraProjectionMatrix.mat[0][2] = 0.0f;
	CameraProjectionMatrix.mat[0][3] = 0.0f;

	// Row 1 (Y mapping)
	CameraProjectionMatrix.mat[1][0] = 0.0f;
	CameraProjectionMatrix.mat[1][1] = 1.0f / HalfTanFOV;
	CameraProjectionMatrix.mat[1][2] = 0.0f;
	CameraProjectionMatrix.mat[1][3] = 0.0f;

	// Row 2 (Z mapping: Z -> Z_ndc, maps [-n, -f] to [-1, 1])
	CameraProjectionMatrix.mat[2][0] = 0.0f;
	CameraProjectionMatrix.mat[2][1] = 0.0f;
	// (Far + Near) / (Far - Near)
	CameraProjectionMatrix.mat[2][2] = (FarZ + NearZ) / ZRange;;
	// -2 * Far * Near / (Far - Near)
	CameraProjectionMatrix.mat[2][3] = (-2.0f * FarZ * NearZ) / ZRange;

	// Row 3 (W component for perspective division)
	CameraProjectionMatrix.mat[3][0] = 0.0f;
	CameraProjectionMatrix.mat[3][1] = 0.0f;
	// -1.0f because we are projecting z-values that are negative in view space.
	CameraProjectionMatrix.mat[3][2] = -1.0f;
	CameraProjectionMatrix.mat[3][3] = 0.0f;

	return (CameraProjectionMatrix);
}

