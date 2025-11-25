#pragma once

#include <maths.h>

class CWindow;

class CCamera
{
public:
	CCamera(CWindow* pWindow);
	~CCamera() = default;

	Matrix4 GetMatrix() const;
	Matrix4 GetProjectionMatrix() const;

private:
	Vector3D m_v3CameraPosition;
	Vector3D m_v3Up;		// V
	Vector3D m_v3Target;	// N

	CWindow* m_pWindow;
};