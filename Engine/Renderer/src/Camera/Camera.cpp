#include "Camera/Camera.h"
#include "Logging/LogManager.h"
#include "EngineMath.h"
#include "EngineMathVectors.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

CCamera::CCamera()
{
	m_v3Position = Vector3D(-300.0f, 100.0f, 230.0f);
	m_v3Front = Vector3D(0.0f, 0.0f, -1.0f);

	m_v3WorldRight = Vector3D(1.0f, 0.0f, 0.0f);
	m_v3WorldUp = Vector3D(0.0f, 1.0f, 0.0f);
	m_v3WorldView = Vector3D(0.0f, 0.0f, -1.0f);

	m_v3Right = m_v3WorldView.cross(m_v3WorldUp);
	m_v3Right.normalize();

	m_v3Up = m_v3Right.cross(m_v3Front);
	m_v3Up.normalize();

	m_v2MousePos = Vector2D(0.0f);

	m_fYaw = 0.0f;
	m_fPitch = 0.0f;
	m_fRoll = 0.0f;

	m_fCameraZoom = 45.0f;

	m_fCameraSpeed = 100.0f;
	m_fSensitivity = 0.1f;

	m_iCameraType = CAMERA_PERSPECTIVE;

	// Camera View Matrix
	m_matView = {};

	// Camera Projection Matrix;
	m_matProjection = {};

	// Camera View-Projection Matrix
	m_matViewProjection = {};

	// Camera Billboard Matrix
	m_matBillBoard = {};

	m_qOrientation = SQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

	m_bViewDirty = true; 
	m_bProjectionDirty = true;
	m_bViewProjDirty = true;
	m_bBillboardDirty = true;

	m_bUseQuaternion = true;

	m_persProjInfo = {};
	m_orthoProjInfo = {};

	m_iViewPortWidth = 1440;
	m_iViewPortHeight = 960;
}

CCamera::~CCamera()
{
}

/**
 * @brief Sets up the camera's projection parameters and GPU memory.
 *
 * Configures both perspective and orthographic projection settings using
 * window dimensions and zoom levels. Also initializes the Uniform Buffer
 * Object (UBO) to facilitate matrix sharing with shaders.
 */
void CCamera::Initialize(int32_t iCameraType, int32_t iWidth, int32_t iHeight)
{
	m_iCameraType = iCameraType;

	// Configure perspective projection parameters
	m_persProjInfo.FOV = EngineMath::ToRadians(m_fCameraZoom);		// Field of view in degrees
	m_persProjInfo.Width = static_cast<float>(m_iViewPortWidth);			// Viewport width
	m_persProjInfo.Height = static_cast<float>(m_iViewPortHeight);			// Viewport height
	m_persProjInfo.zNear = 0.1f;										// Near clipping plane
	m_persProjInfo.zFar = 1000.0f;									// Far clipping plane

	// Configure orthographic projection parameters
	const float fAspetRatio = static_cast<float>(m_iViewPortWidth) / static_cast<float>(m_iViewPortHeight);
	m_orthoProjInfo;
	m_orthoProjInfo.Left = -m_fCameraZoom * fAspetRatio;			/** The left plane of the orthogonal frustum */
	m_orthoProjInfo.Right = m_fCameraZoom * fAspetRatio;			/** The right plane of the orthogonal frustum */
	m_orthoProjInfo.Bottom = -m_fCameraZoom;						/** The bottom plane of the orthogonal frustum */
	m_orthoProjInfo.Top = m_fCameraZoom;							/** The top plane of the orthogonal frustum */
	m_orthoProjInfo.zNear = 0.1f;									/** The near clipping plane distance */
	m_orthoProjInfo.zFar = 1000.0f;									/** The far clipping plane distance */
	m_orthoProjInfo.Width = static_cast<float>(m_iViewPortWidth);		/** The width of the orthogonal view in world-space units */
	m_orthoProjInfo.Height = static_cast<float>(m_iViewPortHeight);		/** The height of the orthogonal view in world-space units */
}

/**
 * @brief Renders the camera's user interface (UI) elements.
 *
 * This function integrates with the UI system to display camera-related
 * information and controls, such as position, orientation, and settings.
 */
void CCamera::RenderCameraUI()
{
	/*
	// 1) Build model matrix for camera visualization
	SEulerAngles angles = EngineMath::ToEulerAnglesDegrees(m_qOrientation);

	// 2) ImGui window
	ImGui::Begin("Camera UI");

	// Define the rotation increment (e.g., 10 degrees per click)
	const GLfloat fDeltaTime = static_cast<GLfloat>(CTimerManager::Instance().GetDeltaTime());
	// Define the rotation speed in DEGREES PER SECOND (much better than fixed steps)
	const GLfloat ROTATION_SPEED_DEG_PER_SEC = 60.0f;

	// Calculate the actual rotation step for this frame
	const GLfloat FRAME_ROTATION_STEP = ROTATION_SPEED_DEG_PER_SEC * fDeltaTime;

	// -------------------------------------------------------------------
	// ROTATION CONTROL SECTION
	// -------------------------------------------------------------------
	ImGui::Separator();
	ImGui::Text("Manual Rotation (%.1f deg):", FRAME_ROTATION_STEP);

	// A. World Yaw Rotation (Around the Global Y-axis)
	ImGui::Button("Yaw Left (World)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the World Up Vector (0, 1, 0) Left
		RotateAroundAxis(Vector3D(0.0f, 1.0f, 0.0f), FRAME_ROTATION_STEP);
	}

	ImGui::SameLine();
	ImGui::Button("Yaw Right (World)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the World Up Vector (0, 1, 0) Right
		RotateAroundAxis(Vector3D(0.0f, 1.0f, 0.0f), -FRAME_ROTATION_STEP);
	}

	ImGui::NewLine();
	// B. Local Pitch Rotation (Around the Camera's Local Right Vector)
	ImGui::Button("Pitch Up (Local)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the camera's local Right Vector
		RotateAroundAxis(m_v3Right, FRAME_ROTATION_STEP); // Assuming m_v3Right holds the camera's current right vector
	}

	ImGui::SameLine();
	ImGui::Button("Pitch Down (Local)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the camera's local Right Vector
		RotateAroundAxis(m_v3Right, -FRAME_ROTATION_STEP); // Assuming m_v3Right holds the camera's current right vector
	}

	ImGui::NewLine();
	ImGui::Button("Roll CW (Local)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the camera's local Front Vector
		RotateAroundAxis(m_v3Front, -FRAME_ROTATION_STEP); // Assuming m_v3Front is the camera's forward vector
	}

	ImGui::SameLine();
	ImGui::Button("Roll CCW (Local)");
	if (ImGui::IsItemActive())
	{
		// Rotate around the camera's local Front Vector
		RotateAroundAxis(m_v3Front, FRAME_ROTATION_STEP); // Assuming m_v3Front is the camera's forward vector
	}


	// 3) Text info in same window
	ImGui::Separator();
	ImGui::Text("Camera Position:");
	ImGui::Text("X: %.2f", m_v3Position.x);
	ImGui::Text("Y: %.2f", m_v3Position.y);
	ImGui::Text("Z: %.2f", m_v3Position.z);

	ImGui::Separator();
	ImGui::Text("Camera Orientation:");
	ImGui::Text("Yaw:   %.2f", angles.fYaw);
	ImGui::Text("Pitch: %.2f", angles.fPitch);
	ImGui::Text("Roll:  %.2f", angles.fRoll);

	ImGui::Separator();
	ImGui::Text("Camera Zoom (FOV): %.2f", m_fCameraZoom);
	ImGui::Separator();
	ImGui::Text("Compass Direction: %s", GetCurrentCompassDirection().c_str());

	ImGui::NewLine();
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

	ImGui::End();
	*/
}

/**
* @brief Constructs the View Matrix using our Right-Handed LookAt function.
*
* * The View Matrix (V) is the inverse of the camera's World Matrix. It transforms
* vertices from **World Space** to **View Space** (also known as Camera Space).
*
* * **LookAtRH Logic:**
* - `v3Pos`: The eye position (where the camera is). * - `v3Pos + v3Front`: The center point (where the camera is looking).
* - `v3Up`: The world up direction (usually (0, 1, 0)).
*
* * The function effectively builds a rotation matrix from the camera's
* orthogonal basis vectors (Right, Up, Forward) and incorporates the
* negated translation (-s.p, -u.p, f.p) into the final column.
*
* @return The 4x4 View Matrix in OpenGL column-major format.
*/
Matrix4 CCamera::GetViewMatrix()
{
	if (!m_bViewDirty)
	{
		return (m_matView);
	}

	// Build view matrix using LookAt transformation:
	// - Eye position: current camera position (m_v3Position)
	// - Target: point along camera's forward direction (m_v3Position + m_v3Front)
	// - Up vector: camera's local up direction (m_v3Up)
	//
	// This creates a coordinate frame with:
	//   Right = normalize(Front x Up)
	//   Up    = normalize(Right x Front)
	//   Front = camera's forward direction
	m_matView = EngineMath::LookAtRH(m_v3Position, m_v3Position + m_v3Front, m_v3Up);

	// m_bViewDirty = false; // Mark view matrix as up-to-date
	return m_matView;
}

/**
 * @brief Constructs a perspective projection matrix using our PerspectiveRH.
 * * The projection matrix maps the 3D viewing frustum into a normalized cube
 * (Normalized Device Coordinates / NDC). This is essential for converting
 * view-space coordinates into clip-space coordinates by triggering
 * the perspective division (w-component).
 * * Note: This uses a Right-Handed (RH) perspective, compatible with OpenGL's
 * default depth range of [-1, 1].
 *
 * @return 4x4 projection matrix in column-major order (OpenGL compatible)
 */
Matrix4 CCamera::GetProjectionMatrix()
{
	if (!m_bProjectionDirty)
	{
		return m_matProjection;
	}

	if (m_iCameraType == CAMERA_PERSPECTIVE)
	{
		// Build perspective projection matrix:
		//
		//     [ 1/(t*a)     0           0              0     ]
		// P = [    0      1/t           0              0     ]
		//     [    0       0     -(f+n)/(f-n)   -2fn/(f-n)   ]
		//     [    0       0          -1              0      ]
		//
		// where t = tan(fov/2), a = width/height, n = zNear, f = zFar

		m_persProjInfo.FOV = EngineMath::ToRadians(m_fCameraZoom);
		m_matProjection = EngineMath::PerspectiveRH(m_persProjInfo);
	}
	else if (m_iCameraType == CAMERA_ORTHOGRAPHIC)
	{
		// Build the orthographic projection matrix.
		// The matrix maps the rectangular viewing volume [left, right], [bottom, top], [zNear, zFar] 
		// into the normalized device coordinates (NDC) cube [-1, 1].
		//
		// 			[ 2/(r-l)       0             0        -(r+l)/(r-l) ]
		// 		P = [    0       2/(t-b)          0        -(t+b)/(t-b) ]
		// 			[    0           0        -2/(f-n)     -(f+n)/(f-n) ]
		// 			[    0           0             0             1      ]

		const float fAspetRatio = static_cast<float>(m_iViewPortWidth) / static_cast<float>(m_iViewPortHeight);
		m_orthoProjInfo;
		m_orthoProjInfo.Left = -m_fCameraZoom * fAspetRatio;			/** The left plane of the orthogonal frustum */
		m_orthoProjInfo.Right = m_fCameraZoom * fAspetRatio;			/** The right plane of the orthogonal frustum */
		m_orthoProjInfo.Bottom = -m_fCameraZoom;						/** The bottom plane of the orthogonal frustum */
		m_orthoProjInfo.Top = m_fCameraZoom;							/** The top plane of the orthogonal frustum */

		m_matProjection = EngineMath::OrthographicRH(m_orthoProjInfo);
	}

	// m_bProjectionDirty = false; // Mark projection matrix as up-to-date
	return m_matProjection;
}

/**
 * @brief Computes the combined View-Projection Matrix.
 * * This matrix transforms vertices from World Space directly into Clip Space (ready for the GPU).
 * It is the final transformation applied in the vertex shader: P * V.
 * * Multiplication order is crucial for column-major matrices:
 * [Projection Matrix] * [View Matrix].
 *
 * @return The 4x4 View-Projection Matrix in custom column-major format.
 */
Matrix4 CCamera::GetViewProjectionMatrix()
{
	// If either piece changed, we must re-multiply
	if (m_bViewDirty || m_bProjectionDirty || m_bViewProjDirty)
	{
		// Note: Calling the getters here will trigger their internal 
		// recalculations only if THEIR specific dirty flags are set.
		m_matViewProjection = GetProjectionMatrix() * GetViewMatrix();
		m_bViewDirty = false;
		m_bProjectionDirty = false;
		m_bViewProjDirty = false;
	}
	return m_matViewProjection;
}

/**
 * @brief Retrieves the billboard matrix for this camera.
 * Billboards are objects that always face the camera, commonly used for sprites and particles.
 *
 * @return 4x4 billboard matrix in column-major order
 */
Matrix4 CCamera::GetBillBoardMatrix()
{
	if (m_bBillboardDirty)
	{
		UpdateBillBoardMatrix();
	}

	return (m_matBillBoard);
}

/**
 * @brief Constructs the View Matrix using GLM's Right-Handed LookAt function.
 *
 * * The View Matrix (V) is the inverse of the camera's World Matrix. It transforms
 * vertices from **World Space** to **View Space** (also known as Camera Space).
 *
 * * **GLM LookAtRH Logic:**
 * - `v3Pos`: The eye position (where the camera is).
 * - `v3Pos + v3Front`: The center point (where the camera is looking).
 * - `v3Up`: The world up direction (usually (0, 1, 0)).
 *
 * * The function effectively builds a rotation matrix from the camera's
 * orthogonal basis vectors (Right, Up, Forward) and incorporates the
 * negated translation (-s.p, -u.p, f.p) into the final column.
 *
 * @return The 4x4 View Matrix in GLM's column-major format.
 */
glm::mat4 CCamera::GetViewMatrixGLM()
{
	if (!m_bViewDirty)
	{
		return EngineMath::Mat4ToGLM(m_matView);
	}

	glm::vec3 v3Pos = m_v3Position.ToGLM();
	glm::vec3 v3Front = m_v3Front.ToGLM();
	glm::vec3 v3Up = m_v3Up.ToGLM();
	m_matView = glm::lookAtRH(v3Pos, v3Pos + v3Front, v3Up);

	m_bViewDirty = false; // Mark view matrix as up-to-date
	return (EngineMath::Mat4ToGLM(m_matView));
}

/**
 * @brief Constructs a perspective projection matrix using GLM.
 * The projection matrix maps the 3D viewing frustum into a normalized cube
 * (Normalized Device Coordinates / NDC). This is essential for converting
 * view-space coordinates into clip-space coordinates by triggering
 * the perspective division (w-component).
 * * Note: This uses a Right-Handed (RH) perspective, compatible with OpenGL's
 * default depth range of [-1, 1].
 * 
 * @return glm 4x4 projection matrix in column-major order (OpenGL compatible)
 */
glm::mat4 CCamera::GetProjectionMatrixGLM()
{
	if (!m_bProjectionDirty)
	{
		return EngineMath::Mat4ToGLM(m_matProjection);
	}

	if (m_iCameraType == CAMERA_PERSPECTIVE)
	{
		// Build perspective projection matrix:
		//
		//     [ 1/(t*a)     0           0              0     ]
		// P = [    0      1/t           0              0     ]
		//     [    0       0     -(f+n)/(f-n)   -2fn/(f-n)   ]
		//     [    0       0          -1              0      ]
		//
		// where t = tan(fov/2), a = width/height, n = zNear, f = zFar
		m_persProjInfo.FOV = EngineMath::ToRadians(m_fCameraZoom);
		m_matProjection = (glm::perspectiveRH(m_persProjInfo.FOV, m_persProjInfo.Width / m_persProjInfo.Height, m_persProjInfo.zNear, m_persProjInfo.zFar));
	}
	else if (m_iCameraType == CAMERA_ORTHOGRAPHIC)
	{
		// Build the orthographic projection matrix.
		// The matrix maps the rectangular viewing volume [left, right], [bottom, top], [zNear, zFar] 
		// into the normalized device coordinates (NDC) cube [-1, 1].
		//
		// 			[ 2/(r-l)       0             0        -(r+l)/(r-l) ]
		// 		P = [    0       2/(t-b)          0        -(t+b)/(t-b) ]
		// 			[    0           0        -2/(f-n)     -(f+n)/(f-n) ]
		// 			[    0           0             0             1      ]

		const float fAspetRatio = static_cast<float>(m_iViewPortWidth) / static_cast<float>(m_iViewPortHeight);
		m_orthoProjInfo;
		m_orthoProjInfo.Left = -m_fCameraZoom * fAspetRatio;			/** The left plane of the orthogonal frustum */
		m_orthoProjInfo.Right = m_fCameraZoom * fAspetRatio;			/** The right plane of the orthogonal frustum */
		m_orthoProjInfo.Bottom = -m_fCameraZoom;						/** The bottom plane of the orthogonal frustum */
		m_orthoProjInfo.Top = m_fCameraZoom;							/** The top plane of the orthogonal frustum */

		m_matProjection = glm::orthoRH(m_orthoProjInfo.Left, m_orthoProjInfo.Right, m_orthoProjInfo.Bottom, m_orthoProjInfo.Top, m_orthoProjInfo.zNear, m_orthoProjInfo.zFar);
	}

	m_bProjectionDirty = false; // Mark projection matrix as up-to-date
	return EngineMath::Mat4ToGLM(m_matProjection);
}

/**
 * @brief Computes the combined View-Projection Matrix using GLM functions.
 * This is the concatenation of the projection (P) and view (V) matrices: P * V.
 * The result is a single matrix used to transform world coordinates into clip space
 * for rendering.
 * 
 * @return The final 4x4 View-Projection Matrix in GLM's column-major format.
 */
glm::mat4 CCamera::GetViewProjectionMatrixGLM()
{
	// If either piece changed, we must re-multiply
	if (m_bViewDirty || m_bProjectionDirty || m_bViewProjDirty)
	{
		// Note: Calling the getters here will trigger their internal 
		// recalculations only if THEIR specific dirty flags are set.
		m_matViewProjection = GetProjectionMatrixGLM() * GetViewMatrixGLM();
		m_bViewProjDirty = false;
	}
	return EngineMath::Mat4ToGLM(m_matViewProjection);
}

/**
 * @brief Processes keyboard input for camera movement (strafe and forward/back).
 *
 * * Movement is implemented using **speed-based translation**, scaled by the
 * time elapsed since the last frame (fDeltaTime) to ensure **frame-rate independence**.
 *
 * @param camDirection The direction of movement (e.g., DIRECTION_FORWARD).
 * @param fDeltaTime The time passed since the last frame, in seconds.
 */
void CCamera::ProcessKeyboard(int32_t camDirection, double dDeltaTime)
{
	const float fVelocity = m_fCameraSpeed * static_cast<float>(dDeltaTime);
	switch (camDirection)
	{
	case DIRECTION_FORWARD:
		// Translate position along the current m_v3Front vector.
		m_v3Position += m_v3Front * fVelocity;
		break;

	case DIRECTION_BACKWARD:
		// Translate position in the opposite direction of the m_v3Front vector.
		m_v3Position -= m_v3Front * fVelocity;
		break;

	case DIRECTION_RIGHT:
		// Translate position along the current m_v3Right vector (strafe right).
		m_v3Position += m_v3Right * fVelocity;
		break;

	case DIRECTION_LEFT:
		// Translate position against the current m_v3Right vector (strafe left).
		m_v3Position -= m_v3Right * fVelocity;
		break;
	}

	m_bViewDirty = true;
	m_bBillboardDirty = true;
}

/**
 * @brief Processes mouse movement for camera rotation (Pitch and Yaw).
 *
 * * Calculates rotation delta since the last frame and accumulates it into the
 * m_fPitch and m_fYaw angles, ensuring continuous rotation.
 *
 * @param m_v2MouseXY The current mouse cursor position (X and Y coordinates).
 */
void CCamera::ProcessMouse(Vector2D m_v2MouseXY)
{
	// Calculate change since last frame
	// v2MouseDelta.y is inverted (m_v2MousePos.y - m_v2MouseXY.y) for typical FPS control,
	// where moving the mouse up decreases screen Y but should increase Pitch (look up).
	Vector2D v2MouseDelta = Vector2D(m_v2MouseXY.x - m_v2MousePos.x, m_v2MousePos.y - m_v2MouseXY.y); // Invert Y for typical FPS controls

	// Update stored position for the next frame
	m_v2MousePos = m_v2MouseXY;

	// Accumulate the rotation changes
	if (m_bUseQuaternion == false)
	{
		m_fYaw += v2MouseDelta.x * m_fSensitivity;
		m_fPitch += v2MouseDelta.y * m_fSensitivity;

		// Clamp the pitch angle to [-89.0f, 89.0f] degrees to prevent the camera from 
		// flipping over (looking directly above 90 or below -90 degrees).
		m_fPitch = std::clamp(m_fPitch, -89.0f, 89.0f);
	}
	else
	{
		// Quaternion-based rotation accumulation
		float fDeltaYaw = -v2MouseDelta.x * m_fSensitivity;
		float fDeltaPitch = v2MouseDelta.y * m_fSensitivity;

		// Compute right axis from current orientation
		SVector3Df localRight = EngineMath::Rotate(m_qOrientation, Vector3D(1.0f, 0.0f, 0.0f));

		// Create rotation quaternions
		SQuaternion pitchQuat = EngineMath::FromAxisAngle(localRight, fDeltaPitch);
		SQuaternion yawQuat = EngineMath::FromAxisAngle(m_v3WorldUp, fDeltaYaw);

		// Accumulate orientation
		m_qOrientation = yawQuat * pitchQuat * m_qOrientation;
		m_qOrientation = EngineMath::Normalize(m_qOrientation);
	}

	UpdateCameraVectors();
}

/**
 * @brief Processes roll input to rotate the camera around its forward axis.
 *
 * Roll rotation tilts the camera left or right, affecting the horizon line.
 * This function updates the m_fRoll angle and recalculates the camera's basis vectors.
 *
 * @param fDeltaRoll The change in roll angle (in degrees).
 */
void CCamera::ProcessRoll(float fDeltaRoll)
{
	// If we are using Euler angles (m_bUseQuaternion == false)
	if (m_bUseQuaternion == false)
	{
		m_fRoll += fDeltaRoll * m_fSensitivity;
		// Unlike Pitch, Roll is usually not clamped.
	}
	else
	{
		// The rotation axis for Roll is the camera's current Front vector (Z-axis).
		// This vector is calculated and stored in m_v3Front by UpdateCameraVectors().
		// We use m_v3Front to define the axis of rotation.

		// 1. Create the Roll rotation quaternion
		SQuaternion rollQuat = EngineMath::FromAxisAngle(m_v3Front, fDeltaRoll * m_fSensitivity);

		// 2. Accumulate orientation by pre-multiplying the roll: 
		// New Rotation = Roll * Current Orientation
		// (Pre-multiplying applies the rotation in the local space of the camera)
		m_qOrientation = rollQuat * m_qOrientation;
		m_qOrientation = EngineMath::Normalize(m_qOrientation);
	}

	UpdateCameraVectors();
}

/**
 * @brief Processes mouse scroll input to adjust the camera's zoom level (FOV).
 *
 * Scrolling changes the Field of View (m_fCameraZoom), directly affecting the
 * Projection Matrix (a smaller FOV = more zoom).
 *
 * @param fMouseScrollY The vertical scroll offset (e.g., +1.0f or -1.0f).
 */
void CCamera::ProcessMouseScroll(float fMouseScrollY)
{
	m_fCameraZoom -= fMouseScrollY;

	// Clamp the zoom (FOV) to a sensible range (e.g., 1.0 degrees to 90.0 degrees) 
	// to prevent visual distortion or matrix errors (singularity).
	m_fCameraZoom = std::clamp(m_fCameraZoom, 1.0f, 90.0f);

	// Mark projection matrix as dirty to recalculate with new FOV
	m_bProjectionDirty = true;

	UpdateCameraVectors();
}

/**
 * UpdateCameraVectors - Recalculates the camera coordinate system
 *
 * This function derives the orthonormal basis vectors (Front, Right, Up)
 * from the current orientation. It supports two mathematical paths:
 * 1) Quaternion-based: Rotates world axes by the orientation quaternion.
 * 2) Euler-based: Uses spherical coordinates (Yaw/Pitch) to compute the
 * Front vector and derives Right/Up via cross products.
 *
 * Sets the view dirty flag to trigger a matrix recalculation.
 */
void CCamera::UpdateCameraVectors()
{	
	if (m_bUseQuaternion)
	{
		// Rotate canonical axes
		m_v3Front = EngineMath::Rotate(m_qOrientation, m_v3WorldView);	// Forward vector
		m_v3Right = EngineMath::Rotate(m_qOrientation, m_v3WorldRight);	// Right vector
		m_v3Up = EngineMath::Rotate(m_qOrientation, m_v3WorldUp);		// Up vector

		// Normalize
		//m_v3Front.normalize();
		//m_v3Right.normalize();
		//m_v3Up.normalize();
	}
	else
	{
		// Non-quaternion approach (Euler angles):
		// Calculate the new Front vector based on updated Yaw and Pitch angles.
		m_fPitch = std::clamp(m_fPitch, -89.0f, 89.0f); // Prevent gimbal lock

		// calculate the new Front vector (derived from spherical coordinates)
		// X component: cos(Yaw) * cos(Pitch)
		m_v3Front.x = std::cos(EngineMath::ToRadians(m_fYaw)) * std::cos(EngineMath::ToRadians(m_fPitch));

		// Y component: sin(Pitch) (Vertical direction)
		m_v3Front.y = std::sin(EngineMath::ToRadians(m_fPitch));

		// Z component: sin(Yaw) * cos(Pitch)
		m_v3Front.z = std::sin(EngineMath::ToRadians(m_fYaw)) * std::cos(EngineMath::ToRadians(m_fPitch));

		// Normalize the vector to maintain unit length
		m_v3Front.normalize();

		// also re-calculate the Right vector (s) using the Cross Product of Front and World Up.
		// This ensures the Right vector is perpendicular to both.
		SVector3Df tempRight = m_v3Front.cross(m_v3WorldUp);
		tempRight.normalize(); // Normalize Right vector

		// Recalculate the true Up vector (u) using the Cross Product of Right and Front.
		// This corrects for any minor tilt introduced when Yaw/Pitch are updated, ensuring 
		// m_v3Up is perfectly perpendicular to the m_v3Front/m_v3Right plane.
		SVector3Df tempUp = tempRight.cross(m_v3Front);
		tempUp.normalize(); // Normalize Up vector

		// Apply Roll, We create a rotation matrix (or use a helper) to rotate around the Front vector
		float rollRad = EngineMath::ToRadians(m_fRoll);
		float cosR = std::cos(rollRad);
		float sinR = std::sin(rollRad);

		// Rodrigues' Rotation Formula logic or manual basis rotation:
		// NewRight = tempRight * cos(roll) + tempUp * sin(roll)
		// NewUp    = tempUp * cos(roll) - tempRight * sin(roll)
		m_v3Right = (tempRight * cosR) + (tempUp * sinR);
		m_v3Up = (tempUp * cosR) - (tempRight * sinR);

		m_v3Right.normalize();
		m_v3Up.normalize();
	}

	m_bViewDirty = true;
	m_bBillboardDirty = true;
}

/**
 * @brief Recomputes the camera's billboard matrix.
 *
 * * This function updates the m_matBillboard matrix to ensure that
 * billboards always face the camera, regardless of its orientation.
 */
void CCamera::UpdateBillBoardMatrix()
{
	// 1. Get the current View Matrix
	// 2. Invert the rotation part of the View Matrix to get the Billboard Matrix
	// Copy the view matrix to billboard matrix
	m_matBillBoard = m_matView;

	// Zero out the translation part (4th column)
	m_matBillBoard[3][0] = 0.0f;
	m_matBillBoard[3][1] = 0.0f;
	m_matBillBoard[3][2] = 0.0f;

	// Ensure the bottom right component is 1.0 (W_w)
	m_matBillBoard[3][3] = 1.0f;

	// Transpose the upper-left 3x3 rotation part to invert it
	for (int32_t i = 0; i < 3; ++i)
	{
		for (int32_t j = i + 1; j < 3; ++j)
		{
			std::swap(m_matBillBoard[i][j], m_matBillBoard[j][i]);
		}
	}

	m_bBillboardDirty = false;	// Set to true when view matrix changes
}

/**
 * @brief Rotates the camera around a specified axis by a given angle.
 *
 * This function creates a rotation quaternion based on the provided axis
 * and angle, then applies it to the camera's current orientation.
 *
 * @param v3Axis The axis to rotate around (should be a normalized vector).
 * @param fAngleDegrees The angle to rotate by, in degrees.
 */
void CCamera::RotateAroundAxis(const Vector3D& v3Axis, float fAngleDegrees)
{
	// 2. Create the rotation quaternion for the desired spin
	// This function must exist in your EngineMath library.
	SQuaternion rotationQuat = EngineMath::FromAxisAngle(v3Axis, fAngleDegrees);
	rotationQuat = EngineMath::Normalize(rotationQuat);

	// 3. Apply the new rotation to the camera's current orientation (quaternion multiplication)
	// Pre-multiplication (rotationQuat * m_qOrientation) applies rotation relative to the World frame.
	m_qOrientation = rotationQuat * m_qOrientation;

	// 4. Normalize the result (crucial to maintain a unit quaternion)
	m_qOrientation = EngineMath::Normalize(m_qOrientation);

	// 5. Update the camera's basis vectors to reflect the new orientation
	UpdateCameraVectors();
}

/**
 * @brief Translates the camera's internal Yaw angle into a human-readable
 * cardinal or intercardinal direction (e.g., "North," "South-East").
 *
 * This function relies on the convention that the camera's **North** direction
 * is aligned with the **Negative Z-axis** of the world, which corresponds to
 * a Yaw angle of -90.0 degrees (or 270.0 degrees normalized).
 *
 * @return A string representing the current direction (8-point compass).
 */
std::string CCamera::GetCurrentCompassDirection() const
{
	SEulerAngles angles = EngineMath::ToEulerAnglesDegrees(m_qOrientation);

	// Note: It's assumed that m_fYaw is handled to be within the 0 to 360 range 
	// before this function is called To be safe, we perform local wrapping if m_fYaw isn't always positive.
	float normalizedYaw = angles.fYaw;

	// Shift the zero point: 0 degrees on a compass should be North.
	// Since the trigonometric function used for Yaw treats 0 degrees as (+X / East), 
	// and North is at -90 degrees, we add 90 degrees to align North with the 0-degree mark.
	float compassAngle = normalizedYaw + 90.0f;

	// Wrap the resulting compass angle to ensure it stays within [0.0f, 360.0f).
	if (compassAngle >= 360.0f)
	{
		compassAngle -= 360.0f;
	}
	else if (compassAngle < 0.0f) // Handle negative initial Yaw before the +90 shift
	{
		compassAngle += 360.0f;
	}

	// Determine the cardinal direction by splitting the 360 degrees into 8 sectors
	// of 45 degrees each, centered on the direction (e.g., North is 337.5 to 22.5).

	// North Sector (centered at 0/360)
	if (compassAngle < 22.5f || compassAngle >= 337.5f)
	{
		return "North";
	}

	// North-East Sector (centered at 45)
	if (compassAngle < 67.5f)
	{
		return "North-East";
	}

	// East Sector (centered at 90)
	if (compassAngle < 112.5f)
	{
		return "East";
	}

	// South-East Sector (centered at 135)
	if (compassAngle < 157.5f)
	{
		return "South-East";
	}

	// South Sector (centered at 180)
	if (compassAngle < 202.5f)
	{
		return "South";
	}
	// South-West Sector (centered at 225)
	if (compassAngle < 247.5f)
	{
		return "South-West";
	}

	// West Sector (centered at 270)
	if (compassAngle < 292.5f)
	{
		return "West";
	}

	// North-West Sector (centered at 315)
	return "North-West";
}

/**
 * @brief Updates the camera's internal state and GPU data.
 *
 * Triggers the recalculation of directional vectors, updates the
 * Uniform Buffer Object matrices, and refreshes the billboard matrix.
 */
void CCamera::Update()
{
}

/** 
 * @brief Retrieves the camera's current position in world space.
 *
 * @return The 3D position vector of the camera.
 */
Vector3D CCamera::GetPosition() const
{
	return m_v3Position;
}

/** 
 * @brief Retrieves the camera's current front (forward) direction vector.
 *
 * @return The normalized front direction vector of the camera.
 */
Vector3D CCamera::GetFront() const
{
	return m_v3Front;
}

/** 
 * @brief Retrieves the camera's current right direction vector.
 *
 * @return The normalized right direction vector of the camera.
 */
Vector3D CCamera::GetRight() const
{
	return (m_v3Right);
}

/** 
 * @brief Retrieves the camera's current up direction vector.
 *
 * @return The normalized up direction vector of the camera.
 */
Vector3D CCamera::GetUp() const
{
	return (m_v3Up);
}

float CCamera::GetFOV() const
{
	return (EngineMath::ToRadians(m_fCameraZoom));
}

SPersProjInfo CCamera::GetPerspectiveProjectionInfo() const
{
	return (m_persProjInfo);
}

float CCamera::GetAspectRatio() const
{
	return (m_persProjInfo.Width / m_persProjInfo.Height);
}

float CCamera::GetNearPlane() const
{
	return (m_persProjInfo.zNear);
}

float CCamera::GetFarPlane() const
{
	return (m_persProjInfo.zFar);
}