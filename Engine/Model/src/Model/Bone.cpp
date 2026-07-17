#include "Model/Bone.h"
#include "EngineMathMatrix.h"
#include "EngineMathVectors.h"
#include "EngineMathQuaternion.h"

CBone::CBone(const std::string& stName, int32_t iBoneID, const aiNodeAnim* channel)
	: m_stName(stName)
	, m_iBoneID(iBoneID)
	, m_matLocalTransform(1.0f)
{
	m_iNumPositions = channel->mNumPositionKeys;

	for (int32_t positionIndex = 0; positionIndex < m_iNumPositions; ++positionIndex)
	{
		aiVector3D aiPosition = channel->mPositionKeys[positionIndex].mValue;
		float timeStamp = channel->mPositionKeys[positionIndex].mTime;

		SKeyPosition data = {};
		data.position = Vector3D(aiPosition.x, aiPosition.y, aiPosition.z);
		data.timeStamp = timeStamp;
		m_vPositions.push_back(data);
	}

	m_iNumRotations = channel->mNumRotationKeys;
	for (int32_t rotationIndex = 0; rotationIndex < m_iNumRotations; ++rotationIndex)
	{
		aiQuaternion aiOrientation = channel->mRotationKeys[rotationIndex].mValue;
		float timeStamp = channel->mRotationKeys[rotationIndex].mTime;
		SKeyRotation data = {};
		data.orientation = { aiOrientation.w, aiOrientation.x, aiOrientation.y, aiOrientation.z };
		data.timeStamp = timeStamp;
		m_vRotations.push_back(data);
	}

	m_iNumScalings = channel->mNumScalingKeys;
	for (int keyIndex = 0; keyIndex < m_iNumScalings; ++keyIndex)
	{
		aiVector3D aiScale = channel->mScalingKeys[keyIndex].mValue;
		float timeStamp = channel->mScalingKeys[keyIndex].mTime;
		SKeyScale data;
		data.scale = Vector3D(aiScale.x, aiScale.y, aiScale.z);
		data.timeStamp = timeStamp;
		m_vScales.push_back(data);
	}
}

/*interpolates  b/w positions,rotations & scaling keys based on the curren time of
the animation and prepares the local transformation matrix by combining all keys
tranformations*/
void CBone::Update(float animationTime)
{
	Matrix4 translation = InterpolatePosition(animationTime);
	Matrix4 rotation = InterpolateRotation(animationTime);
	Matrix4 scale = InterpolateScaling(animationTime);
	m_matLocalTransform = translation * rotation * scale;
}

/* Gets the current index on mKeyPositions to interpolate to based on the current animation time*/
int32_t CBone::GetPositionIndex(float animationTime) const
{
	// Handle loop-back / scrub
	if (animationTime < m_vPositions[m_iLastPositionIndex].timeStamp)
	{
		m_iLastPositionIndex = 0;
	}

	for (int32_t index = 0; index < m_iNumPositions - 1; index++)
	{
		if (animationTime < m_vPositions[index + 1].timeStamp)
		{
			m_iLastPositionIndex = index;
			return index;
		}
	}

	return (m_iNumPositions - 1 > 0) ? m_iNumPositions - 2 : 0;
}

/* Gets the current index on mKeyRotations to interpolate to based on the current animation time*/
int32_t CBone::GetRotationIndex(float animationTime) const
{
	// Handle loop-back / scrub
	if (animationTime < m_vRotations[m_iLastRotationIndex].timeStamp)
	{
		m_iLastRotationIndex = 0;
	}

	for (int32_t index = 0; index < m_iNumRotations - 1; index++)
	{
		if (animationTime < m_vRotations[index + 1].timeStamp)
		{
			m_iLastRotationIndex = index;
			return index;
		}
	}

	return (m_iNumRotations - 1 > 0) ? m_iNumRotations - 2 : 0;
}

/* Gets the current index on mKeyScalings to interpolate to based on the current animation time */
int32_t CBone::GetScaleIndex(float animationTime) const
{
	// Handle loop-back / scrub
	if (animationTime < m_vScales[m_iLastScaleIndex].timeStamp)
	{
		m_iLastScaleIndex = 0;
	}

	for (int32_t index = 0; index < m_iNumScalings - 1; index++)
	{
		if (animationTime < m_vScales[index + 1].timeStamp)
		{
			m_iLastScaleIndex = index;
			return index;
		}
	}

	return (m_iNumScalings - 1 > 0) ? m_iNumScalings - 2 : 0;
}

/* Gets normalized value for Lerp & Slerp*/
float CBone::GetScaleFactor(float lastTimeStamp, float nextTimeStamp, float animationTime) const
{
	float scaleFactor = 0.0f;
	float midWayLength = animationTime - lastTimeStamp;
	float framesDiff = nextTimeStamp - lastTimeStamp;
	scaleFactor = midWayLength / framesDiff;
	return scaleFactor;
}

/*figures out which position keys to interpolate b/w and performs the interpolation and returns the translation matrix*/
Matrix4 CBone::InterpolatePosition(float animationTime)
{
	if (1 == m_iNumPositions)
	{
		return EngineMath::Translate(Matrix4(1.0f), m_vPositions[0].position);
	}

	int32_t p0Index = GetPositionIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vPositions[p0Index].timeStamp, m_vPositions[p1Index].timeStamp, animationTime);
	Vector3D finalPosition = EngineMath::Mix(m_vPositions[p0Index].position, m_vPositions[p1Index].position, scaleFactor);
	return EngineMath::Translate(Matrix4(1.0f), finalPosition);
}

Matrix4 CBone::InterpolateRotation(float animationTime)
{
	if (1 == m_iNumRotations)
	{
		Quaternion rotation = EngineMath::Normalize(m_vRotations[0].orientation);
		return EngineMath::ToMatrix4(rotation);
	}

	int32_t p0Index = GetRotationIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vRotations[p0Index].timeStamp, m_vRotations[p1Index].timeStamp, animationTime);
	Quaternion finalRotation = EngineMath::Slerp(m_vRotations[p0Index].orientation, m_vRotations[p1Index].orientation, scaleFactor);
	finalRotation = EngineMath::Normalize(finalRotation);
	return EngineMath::ToMatrix4(finalRotation);
}

Matrix4 CBone::InterpolateScaling(float animationTime)
{
	if (1 == m_iNumScalings)
	{
		return EngineMath::Scale(Matrix4(1.0f), m_vScales[0].scale);
	}

	int32_t p0Index = GetScaleIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vScales[p0Index].timeStamp, m_vScales[p1Index].timeStamp, animationTime);
	Vector3D finalScale = EngineMath::Mix(m_vScales[p0Index].scale, m_vScales[p1Index].scale, scaleFactor);
	return EngineMath::Scale(Matrix4(1.0f), finalScale);
}

Vector3D CBone::GetInterpolatedPosition(float animationTime) const
{
	if (1 == m_iNumPositions)
	{
		return (m_vPositions[0].position);
	}

	int32_t p0Index = GetPositionIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vPositions[p0Index].timeStamp, m_vPositions[p1Index].timeStamp, animationTime);
	Vector3D finalPosition = EngineMath::Mix(m_vPositions[p0Index].position, m_vPositions[p1Index].position, scaleFactor);
	return (finalPosition);
}

SQuaternion CBone::GetInterpolatedRotation(float animationTime) const
{
	if (1 == m_iNumRotations)
	{
		Quaternion rotation = EngineMath::Normalize(m_vRotations[0].orientation);
		return (rotation);
	}

	int32_t p0Index = GetRotationIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vRotations[p0Index].timeStamp, m_vRotations[p1Index].timeStamp, animationTime);
	Quaternion finalRotation = EngineMath::Slerp(m_vRotations[p0Index].orientation, m_vRotations[p1Index].orientation, scaleFactor);
	finalRotation = EngineMath::Normalize(finalRotation);
	return (finalRotation);
}

Vector3D CBone::GetInterpolatedScaling(float animationTime) const
{
	if (1 == m_iNumScalings)
	{
		return (m_vScales[0].scale);
	}

	int32_t p0Index = GetScaleIndex(animationTime);
	int32_t p1Index = p0Index + 1;
	float scaleFactor = GetScaleFactor(m_vScales[p0Index].timeStamp, m_vScales[p1Index].timeStamp, animationTime);
	Vector3D finalScale = EngineMath::Mix(m_vScales[p0Index].scale, m_vScales[p1Index].scale, scaleFactor);
	return (finalScale);
}
