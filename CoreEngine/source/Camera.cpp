#include "Camera.h"
#include "Window.h"

CCamera::CCamera(CWindow* pWindow)
{
	m_v3Position = Vector3D(0.0f, 0.0f, 0.0f);
	m_v3Front = Vector3D(0.0f, 0.0f, -1.0f);
	m_v3WorldUp = Vector3D(0.0f, 1.0f, 0.0f);
	
	m_v3Right = m_v3Front.cross(m_v3WorldUp);
	m_v3Right.normalize();

	m_v3Up = m_v3Right.cross(m_v3Front);
	m_v3Up.normalize();

	m_pWindow = pWindow;
}

/**
 * Constructs the view matrix for this camera.
 * Transforms world coordinates into camera (view) space using a right-handed
 * coordinate system where the camera looks down the negative Z-axis.
 *
 * @return 4x4 view matrix in column-major order
 */
Matrix4 CCamera::GetViewMatrix() const
{
	// Build view matrix using LookAt transformation:
	// - Eye position: current camera position (m_v3Position)
	// - Target: point along camera's forward direction (m_v3Position + m_v3Front)
	// - Up vector: camera's local up direction (m_v3Up)
	//
	// This creates a coordinate frame with:
	//   Right = normalize(Front x Up)
	//   Up    = normalize(Right x Front)
	//   Front = camera's forward direction
	Matrix4 viewMatrix{};
	return viewMatrix.LookAtRH(m_v3Position, m_v3Position + m_v3Front, m_v3Up);
}

/**
 * Constructs the perspective projection matrix for this camera.
 * Maps view space coordinates to normalized device coordinates (NDC) using
 * a right-handed perspective projection with symmetric frustum.
 *
 * Uses fixed parameters:
 *   - 45-degree field of view
 *   - Window aspect ratio (width/height)
 *   - Near plane at 0.1 units
 *   - Far plane at 1000.0 units
 *
 * @return 4x4 projection matrix in column-major order (OpenGL compatible)
 */
Matrix4 CCamera::GetProjectionMatrix() const
{
	// Configure perspective projection parameters
	SPersProjInfo persProj{};
	persProj.FOV = 45.0f;                        // Field of view in degrees
	persProj.Width = m_pWindow->GetWidthF();     // Viewport width
	persProj.Height = m_pWindow->GetHeightF();   // Viewport height
	persProj.zNear = 0.1f;                       // Near clipping plane
	persProj.zFar = 10000.0f;                     // Far clipping plane

	// Build perspective projection matrix:
	//
	//     [ 1/(t*a)     0           0              0     ]
	// P = [    0      1/t           0              0     ]
	//     [    0       0     -(f+n)/(f-n)   -2fn/(f-n)   ]
	//     [    0       0          -1              0      ]
	//
	// where t = tan(fov/2), a = width/height, n = zNear, f = zFar
	Matrix4 projectionMatrix{};
	return projectionMatrix.PerspectiveRH(persProj);
}


Matrix4 CCamera::GetViewProjectionMatrix() const
{
	return (GetProjectionMatrix() * GetViewMatrix());
}
