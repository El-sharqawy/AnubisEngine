#pragma once

#include "Model/Bone.h"
#include <map>
#include <unordered_map>
#include <assimp/scene.h>

class CSkeleton;

struct SAssimpNodeData
{
    Matrix4 matTransformation;
    std::string stName;
    int32_t iChildrenCount;
    std::vector<SAssimpNodeData> vChildren;
};

enum class EAnimationMode : uint32_t
{
    ANIMATION_MODE_GENERAL, // No Weapon
    ANIMATION_MODE_ONEHAND_SWORD,
    ANIMATION_MODE_TWOHANDED_SWORD,
};

enum class EAnimationsTypes : uint32_t
{
    ANIMATION_WAIT,
    ANIMATION_WALK,
    ANIMATION_RUN,
};

class CAnimation
{
public:
    CAnimation() = default;
    ~CAnimation() = default;

    // Returns false on failure — caller decides what to do (skip, log, fallback)
    bool LoadFromFile(const std::string& stAnimationFile, std::shared_ptr<CSkeleton> pAnimationSkeleton);
    bool IsValid() const { return m_bValid; }

    CBone* FindBone(const std::string& name);

    float GetTicksPerSecond() const { return m_iTicksPerSecond; }
    float GetDuration() const { return m_fDuration; }
    const SAssimpNodeData& GetRootNode() const { return m_sRootNode; }
    const std::map<std::string, SBoneInfo>& GetBoneIDMap() const { return m_mBoneInfoMap; }
    int32_t FindBoneTrackIndex(const std::string& name) const;
    CBone& GetBoneByIndex(int32_t index);
    const CBone& GetBoneByIndex(int32_t index) const;

    EAnimationMode GetAnimatinoMode() const;
    void SetAnimationMode(EAnimationMode animMode);
    EAnimationsTypes GetAnimationType() const;
    void SetAnimationType(EAnimationsTypes animType);

protected:
    void ReadMissingBones(const aiAnimation* animation, std::shared_ptr<CSkeleton> pAnimationSkeleton);
    void ReadHeirarchyData(SAssimpNodeData& dest, const aiNode* src);

private:
    float m_fDuration = 0.0f;
    int32_t m_iTicksPerSecond = 0;
    std::vector<CBone> m_vBones = {};
    SAssimpNodeData m_sRootNode = {};
    std::map<std::string, SBoneInfo> m_mBoneInfoMap = {};
    std::unordered_map<std::string, int32_t> m_mBoneTrackIndex = {};

    // Animation Data
    EAnimationMode m_eAnimationMode = EAnimationMode::ANIMATION_MODE_GENERAL;
    EAnimationsTypes m_eAnimationType = EAnimationsTypes::ANIMATION_WAIT;

    bool m_bValid = false;
};