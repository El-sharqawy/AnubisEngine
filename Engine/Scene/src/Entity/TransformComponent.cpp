#include "Stdafx.h"
#include "Entity/TransformComponent.h"
#include "EngineMathMatrix.h"

const SVector3Df& CTransformComponent::GetPosition() const
{
	return (m_sTransform.m_v3Position);
}

void CTransformComponent::SetPosition(const SVector3Df& v3Pos)
{
	m_sTransform.m_v3Position = v3Pos;

	// Mark as dirty
	m_bIsTransformDirty = true;
}

const SQuaternion& CTransformComponent::GetRotation() const
{
	return (m_sTransform.m_qOrientation);
}

void CTransformComponent::SetRotation(const SQuaternion& v3Rot)
{
	m_sTransform.m_qOrientation = v3Rot;

	// Mark as dirty
	m_bIsTransformDirty = true;
}

const SVector3Df& CTransformComponent::GetScale() const
{
	return (m_sTransform.m_v3Scale);
}

void CTransformComponent::SetScale(const SVector3Df& v3Scale)
{
	m_sTransform.m_v3Scale = v3Scale;

	// Mark as dirty
	m_bIsTransformDirty = true;
}

const Matrix4& CTransformComponent::GetWorldMatrix()
{
	// Check if the transform data has changed
	if (m_bIsTransformDirty)
	{
		// Store the new, correct matrix
		m_sTransform.m_CachedMatrix = EngineMath::GetModelMatrix(GetPosition(), GetRotation(), GetScale());

		// 2. Clear the flag, as the matrix is now up-to-date
		m_bIsTransformDirty = false;
	}

	// 3. Return the cached matrix, either the one that is just built or the old one
	return m_sTransform.m_CachedMatrix;
}
