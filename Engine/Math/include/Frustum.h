#pragma once

/**
 * @brief Represents a view frustum defined by its eight corner points.
 *
 * This structure encapsulates the eight corners of a view frustum:
 * four corners for the near plane and four corners for the far plane.
 * It provides methods to initialize and transform the frustum corners
 * using an inverse view-projection matrix.
 */
struct SFrustum
{
	SVector4Df v4NearTopLeft;
	SVector4Df v4NearBottomLeft;
	SVector4Df v4NearTopRight;
	SVector4Df v4NearBottomRight;

	SVector4Df v4FarTopLeft;
	SVector4Df v4FarBottomLeft;
	SVector4Df v4FarTopRight;
	SVector4Df v4FarBottomRight;

	SFrustum() = default;
	~SFrustum() = default;

	/**
	 * @brief Initializes the frustum corners from the inverse view-projection matrix.
	 *
	 * This function calculates the eight corners of the frustum in world space
	 * by transforming the normalized device coordinates (NDC) corners using the
	 * provided inverse view-projection matrix. It performs a perspective divide
	 * to convert from homogeneous coordinates to 3D coordinates.
	 *
	 * @param matInvViewProj The inverse view-projection matrix.
	 */
	void InitializeFromMatrix(const Matrix4& matInvViewProj);

	/**
	 * @brief Transforms the frustum corners using the inverse view-projection matrix.
	 *
	 * This function applies the provided inverse view-projection matrix to each
	 * corner of the frustum, transforming them from clip space to world space.
	 * It also performs a perspective divide to convert from homogeneous coordinates
	 * to 3D coordinates.
	 *
	 * @param matInvViewProj The inverse view-projection matrix.
	 */
	void Transform(const Matrix4& matInvViewProj);
};

/**
 * @brief Represents a frustum culling object for view frustum clipping.
 *
 * This structure encapsulates the six clipping planes of a view frustum:
 * left, right, top, bottom, near, and far. It provides methods to update
 * the planes from a view-projection matrix and to test whether points
 * are inside the frustum.
 */
struct SFrustumCulling
{
	SVector4Df v4LeftClipPlane;
	SVector4Df v4RightClipPlane;
	SVector4Df v4TopClipPlane;
	SVector4Df v4BottomClipPlane;
	SVector4Df v4NearClipPlane;
	SVector4Df v4FarClipPlane;

	SFrustumCulling() = default;
	~SFrustumCulling() = default;

	/**
	 * @brief Constructs a frustum culling object from the view-projection matrix.
	 *
	 * This constructor initializes the frustum clipping planes by calling
	 * the Update method with the provided view-projection matrix.
	 *
	 * @param matViewProj The combined view-projection matrix.
	 */
	SFrustumCulling(const Matrix4& matViewProj);

	/**
	 * @brief Updates the frustum clipping planes from the view-projection matrix.
	 *
	 * This function recalculates the six clipping planes (left, right, top,
	 * bottom, near, far) of the frustum using the provided view-projection matrix.
	 * It utilizes the EngineMath::CalculateClipPlanes function to perform the
	 * extraction of the planes.
	 *
	 * @param matViewProj The combined view-projection matrix.
	 */
	void Update(const Matrix4& matViewProj);

	/**
	 * @brief Checks if a 3D point is inside the view frustum.
	 *
	 * This function tests whether the given 3D point lies within the six
	 * clipping planes of the view frustum by converting it to homogeneous
	 * coordinates and performing dot product tests.
	 *
	 * @param v3Point The 3D point to test.
	 * @return true if the point is inside the frustum; false otherwise.
	 */
	bool IsPointInsideViewFrustum(const SVector3Df& v3Point) const;

	/**
	 * @brief Checks if a 4D point is inside the view frustum.
	 *
	 * This function tests whether the given 4D point (in homogeneous coordinates)
	 * lies within the six clipping planes of the view frustum. It uses a small
	 * negative epsilon to create a safety buffer, preventing flickering at the edges.
	 *
	 * @param v4Point The 4D point to test.
	 * @return true if the point is inside the frustum; false otherwise.
	 */
	bool IsPointInsideViewFrustum(const SVector4Df& v4Point) const;

	bool IsSphereInsideViewFrustum(const SVector3Df& center, float radius) const;
	bool IsAABBInsideViewFrustum(const Vector3D& minB, const Vector3D& maxB) const;

};