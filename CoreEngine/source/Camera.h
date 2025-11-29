#pragma once

#include <maths.h>

class CWindow;

class CCamera
{
public:
	CCamera(CWindow* pWindow);
	~CCamera() = default;

	Matrix4 GetViewMatrix() const;
	Matrix4 GetProjectionMatrix() const;
	Matrix4 GetViewProjectionMatrix() const;

private:
	Vector3D m_v3Position;
	Vector3D m_v3Front;
	Vector3D m_v3WorldUp;
	Vector3D m_v3Right;
	Vector3D m_v3Up;

	CWindow* m_pWindow;
};