#pragma once

#include "API/Entity.h"
#include "Transform.h"

class CTransformComponent : public IComponent
{
public:
    CTransformComponent() = default;
    ~CTransformComponent() = default;

	// Accessors
	const SVector3Df& GetPosition() const;
	void SetPosition(const SVector3Df& v3Pos);

	const SQuaternion& GetRotation() const;
	void SetRotation(const SQuaternion& v3Rot);

	const SVector3Df& GetScale() const;
	void SetScale(const SVector3Df& v3Scale);

	const Matrix4& GetWorldMatrix();

    STransform& GetTransform() { return m_sTransform; }
private:
    STransform m_sTransform;
	bool m_bIsTransformDirty;
};
