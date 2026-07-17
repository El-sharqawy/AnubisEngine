#pragma once

#include "TypeMatrix4.h"
#include "Quaternion.h"
#include <string>
#include <assimp/anim.h>

struct SKeyPosition
{
    Vector3D position;
    float timeStamp;
};

struct SKeyRotation
{
    Quaternion orientation;
    float timeStamp;
};

struct SKeyScale
{
    Vector3D scale;
    float timeStamp;
};

struct SBoneInfo
{
    std::string stName;
    int32_t iBoneID = -1;
    Matrix4 matOffset;          // aiBone::mOffsetMatrix
};

class CBone
{
public:
    /*reads keyframes from aiNodeAnim*/
    CBone(const std::string& stName, int32_t iBoneID, const aiNodeAnim* channel);

    void Update(float animationTime);

    Matrix4 GetLocalTransform() const { return m_matLocalTransform; }
    std::string GetBoneName() const { return m_stName; }
    int32_t GetBoneID() const { return m_iBoneID; }

    int32_t GetPositionIndex(float animationTime) const;
    int32_t GetRotationIndex(float animationTime) const;
    int32_t GetScaleIndex(float animationTime) const;

    Vector3D GetInterpolatedPosition(float animationTime) const;
    SQuaternion GetInterpolatedRotation(float animationTime) const;
    Vector3D GetInterpolatedScaling(float animationTime) const;

protected:
    /* Gets normalized value for Lerp & Slerp*/
    float GetScaleFactor(float lastTimeStamp, float nextTimeStamp, float animationTime) const;

    /*figures out which position keys to interpolate b/w and performs the interpolation and returns the translation matrix*/
    Matrix4 InterpolatePosition(float animationTime);

    /*figures out which rotations keys to interpolate b/w and performs the interpolation and returns the rotation matrix*/
    Matrix4 InterpolateRotation(float animationTime);

    /*figures out which scaling keys to interpolate b/w and performs the interpolation and returns the scale matrix*/
    Matrix4 InterpolateScaling(float animationTime);

private:
    std::vector<SKeyPosition> m_vPositions;
    std::vector<SKeyRotation> m_vRotations;
    std::vector<SKeyScale> m_vScales;
    int32_t m_iNumPositions;
    int32_t m_iNumRotations;
    int32_t m_iNumScalings;

    Matrix4 m_matLocalTransform;
    std::string m_stName;
    int32_t m_iBoneID;

    mutable int32_t m_iLastPositionIndex = 0;
    mutable int32_t m_iLastRotationIndex = 0;
    mutable int32_t m_iLastScaleIndex = 0;
};