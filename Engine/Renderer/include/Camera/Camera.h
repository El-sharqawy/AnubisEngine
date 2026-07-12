#pragma once

#include "TypeVector2.h"
#include "Quaternion.h"
#include "PrespectiveProjection.h"
#include "OrthographicProjection.h"

#include <string>

class CWindowManager;
struct SRay;

enum ECameraDirections : int32_t
{
	DIRECTION_FORWARD,
	DIRECTION_BACKWARD,
	DIRECTION_RIGHT,
	DIRECTION_LEFT,
};

enum ECameraType : int32_t
{
	CAMERA_PERSPECTIVE,
	CAMERA_ORTHOGRAPHIC,
}; 

class CCamera
{
public:
	/**
	 * @brief Constructor for the CCamera class.
	 * 
	 * Initializes default camera parameters including position, orientation,
	 */
	CCamera();

	/**
	 * @brief Destructor for the CCamera class.
	 *
	 * Cleans up any allocated resources associated with the camera instance.
	 */
	~CCamera();

	/**
	 * @brief Sets up the camera's projection parameters and GPU memory.
	 *
	 * Configures both perspective and orthographic projection settings using
	 * window dimensions and zoom levels. Also initializes the Uniform Buffer
	 * Object (UBO) to facilitate matrix sharing with shaders.
	 */
	void Initialize(int32_t iCameraType, int32_t iWidth = 1440, int32_t iHeight = 960);

	/**
	 * @brief Renders the camera's user interface (UI) elements.
	 *
	 * This function integrates with the UI system to display camera-related
	 * information and controls, such as position, orientation, and settings.
	 */
	void RenderCameraUI();

	/**
	 * Constructs the view matrix for this camera.
	 * Transforms world coordinates into camera (view) space using a right-handed
	 * coordinate system where the camera looks down the negative Z-axis.
	 *
	 * @return 4x4 view matrix in column-major order
	 */
	Matrix4 GetViewMatrix();

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
	Matrix4 GetProjectionMatrix();

	/**
	 * @brief Computes the combined View-Projection Matrix.
	 * * This matrix transforms vertices from World Space directly into Clip Space (ready for the GPU).
	 * It is the final transformation applied in the vertex shader: P * V.
	 * * Multiplication order is crucial for column-major matrices:
	 * [Projection Matrix] * [View Matrix].
	 *
	 * @return The 4x4 View-Projection Matrix in custom column-major format.
	 */
	Matrix4 GetViewProjectionMatrix();

	/**
	 * @brief Retrieves the billboard matrix for this camera.
	 * Billboards are objects that always face the camera, commonly used for sprites and particles.
	 *
	 * @return 4x4 billboard matrix in column-major order
	 */
	Matrix4 GetBillBoardMatrix();

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
	glm::mat4 GetViewMatrixGLM();

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
	glm::mat4 GetProjectionMatrixGLM();

	/**
	 * @brief Computes the combined View-Projection Matrix using GLM functions.
	 * This is the concatenation of the projection (P) and view (V) matrices: P * V.
	 * The result is a single matrix used to transform world coordinates into clip space
	 * for rendering.
	 *
	 * @return The final 4x4 View-Projection Matrix in GLM's column-major format.
	 */
	glm::mat4 GetViewProjectionMatrixGLM();

	/**
	 * @brief Processes keyboard input for camera movement (strafe and forward/back).
	 *
	 * * Movement is implemented using **speed-based translation**, scaled by the
	 * time elapsed since the last frame (dDeltaTime) to ensure **frame-rate independence**.
	 *
	 * @param camDirection The direction of movement (e.g., DIRECTION_FORWARD).
	 * @param dDeltaTime The time passed since the last frame, in seconds.
	 */
	void ProcessKeyboard(int32_t camDirection, double dDeltaTime);

	/**
	 * @brief Processes mouse movement for camera rotation (Pitch and Yaw).
	 *
	 * * Calculates rotation delta since the last frame and accumulates it into the
	 * m_fPitch and m_fYaw angles, ensuring continuous rotation.
	 *
	 * @param m_v2MouseXY The current mouse cursor position (X and Y coordinates).
	 */
	void ProcessMouse(Vector2D m_v2MouseXY);

	/**
	 * @brief Processes roll input to rotate the camera around its forward axis.
	 *
	 * Roll rotation tilts the camera left or right, affecting the horizon line.
	 * This function updates the m_fRoll angle and recalculates the camera's basis vectors.
	 *
	 * @param fDeltaRoll The change in roll angle (in degrees).
	 */
	void ProcessRoll(float fDeltaRoll);

	/**
	 * @brief Processes mouse scroll input to adjust the camera's zoom level (FOV).
	 *
	 * Scrolling changes the Field of View (m_fCameraZoom), directly affecting the
	 * Projection Matrix (a smaller FOV = more zoom).
	 *
	 * @param fMouseScrollY The vertical scroll offset (e.g., +1.0f or -1.0f).
	 */
	void ProcessMouseScroll(float fMouseScrollY);

protected:

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
	void UpdateCameraVectors();

	/**
	 * @brief Recomputes the camera's billboard matrix.
	 *
	 * * This function updates the m_matBillboard matrix to ensure that
	 * billboards always face the camera, regardless of its orientation.
	 */
	void UpdateBillBoardMatrix();

	/**
	 * @brief Updates the camera's uniform buffer object (UBO) with the latest
	 * view, projection, and view-projection matrices.
	 *
	 * This function is called whenever the camera's matrices are marked dirty,
	 * ensuring that the GPU has the most up-to-date data for rendering.
	 */
	void UpdateUniformBufferObject();

	/**
	 * @brief Rotates the camera around a specified axis by a given angle.
	 *
	 * This function creates a rotation quaternion based on the provided axis
	 * and angle, then applies it to the camera's current orientation.
	 *
	 * @param v3Axis The axis to rotate around (should be a normalized vector).
	 * @param fAngleDegrees The angle to rotate by, in degrees.
	 */
	void RotateAroundAxis(const Vector3D& v3Axis, float fAngleDegrees);

public:
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
	std::string GetCurrentCompassDirection() const;

	/**
	 * @brief Updates the camera's internal state and GPU data.
	 *
	 * Triggers the recalculation of directional vectors, updates the
	 * Uniform Buffer Object matrices, and refreshes the billboard matrix.
	 */
	void Update();

	// Accessors
	/**
	 * @brief Retrieves the camera's current position in world space.
	 *
	 * @return The 3D position vector of the camera.
	 */
	Vector3D GetPosition() const;

	/**
	 * @brief Retrieves the camera's current front (forward) direction vector.
	 *
	 * @return The normalized front direction vector of the camera.
	 */
	Vector3D GetFront() const;

	/**
	 * @brief Retrieves the camera's current right direction vector.
	 *
	 * @return The normalized right direction vector of the camera.
	 */
	Vector3D GetRight() const;

	/**
	 * @brief Retrieves the camera's current up direction vector.
	 *
	 * @return The normalized up direction vector of the camera.
	 */
	Vector3D GetUp() const;

	void SetViewDirty(bool bDirty) { m_bViewDirty = bDirty; }
	void SetProjectionDirty(bool bDirty) { m_bProjectionDirty = bDirty; }
	void SetViewProjDirty(bool bDirty) { m_bViewProjDirty = bDirty; }

	float GetFOV() const;

	SPersProjInfo GetPerspectiveProjectionInfo() const;

	float GetAspectRatio() const;
	float GetNearPlane() const;
	float GetFarPlane() const;

private:
	// --- Camera Transform and Basis Vectors ---

	// The world coordinates (X, Y, Z) of the camera's current location.
	Vector3D m_v3Position;

	// The camera's local Forward vector (F). Where the camera is currently looking. 
	// Used for calculating the View Matrix and for forward/backward movement (W/S).
	Vector3D m_v3Front;

	// The camera's local Right vector (R). Perpendicular to m_v3Front and m_v3Up. 
	// Used for constructing the View Matrix and for strafing (A/D) movement.
	Vector3D m_v3Right;

	// The camera's local Up vector (U). Used for constructing the View Matrix 
	// and ensuring the camera remains upright (no roll).
	Vector3D m_v3Up;

	// --- Input State ---

	// Stores the mouse cursor's last known X and Y screen coordinates.
	// Used to calculate the mouse delta for rotation (Yaw and Pitch).
	Vector2D m_v2MousePos;

	// --- Rotation Angles ---

	// The accumulated horizontal rotation angle (in degrees). Rotation around the local Up axis.
	float m_fYaw;

	// The accumulated vertical rotation angle (in degrees). Rotation around the local Right axis.
	float m_fPitch;

	// The accumulated roll rotation angle (in degrees). Rotation around the local Front axis.
	float m_fRoll;

	// The field-of-view (FOV) value used for zooming in/out (often controlled by the scroll wheel).
	float m_fCameraZoom;

	// --- Camera Properties ---

	// The base speed (units per second) at which the camera moves (e.g., 5.0f).
	float m_fCameraSpeed;

	// The multiplier applied to mouse delta to control the speed of rotation (e.g., 0.1f).
	float m_fSensitivity;

	// The camera Type (PRE
	int32_t m_iCameraType;

	// Camera View Matrix
	Matrix4 m_matView;

	// Camera Projection Matrix;
	Matrix4 m_matProjection;

	// Camera View-Projection Matrix
	Matrix4 m_matViewProjection;

	// Camera Billboard Matrix
	Matrix4 m_matBillBoard;

	// The global World Up vector (U_World), typically (0, 1, 0). 
	// Used as the fixed axis for Yaw (horizontal) rotation.
	Vector3D m_v3WorldUp;	// View along Y Axis (0,1,0)

	// The global World Up vector (R_World), typically (1, 0, 0). 
	// Used as the fixed axis for Roll (vertical) rotation.
	Vector3D m_v3WorldRight;	// View along X Axis (1,0,0)

	// The initial or temporary view vector used during the quaternion rotation calculations 
	// before being assigned to m_v3Front. (Often initialized to (0, 0, 1) or (0, 0, -1)).
	Vector3D m_v3WorldView; // View along Z Axis (0,0,-1)

	// --- Quaternion Orientation ---
	// Accumulates all Yaw and Pitch rotations into a single quaternion.
	SQuaternion m_qOrientation;

	// Camera Matrix State Flags
	bool m_bViewDirty = true;       // Set to true when position/rotation changes
	bool m_bProjectionDirty = true; // Set to true when zoom/window size changes
	bool m_bViewProjDirty = true;	// Set to true when either view or projection is dirty
	bool m_bBillboardDirty = true;	// Set to true when view matrix changes

	// Use Quaternion to calculate camera rotations
	bool m_bUseQuaternion;

	// Camera Projections
	SPersProjInfo m_persProjInfo;
	SOrthoProjInfo m_orthoProjInfo;

	int32_t m_iViewPortWidth;
	int32_t m_iViewPortHeight;
};