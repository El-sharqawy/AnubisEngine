#pragma once

#include "Model/Animation.h"
#include <queue>

static constexpr uint32_t MAX_BONES_PER_DRAW = 128;

struct SPlaybackState
{
    std::shared_ptr<CAnimation> pCurrentAnimation;
    float fCurrentTime = 0.0f;
    bool bLoop = true;
};

struct SQueuedAnimation
{
    std::string stName;
    bool bLoop = false;
};

struct SRuntimeAnimNode
{
    const SAssimpNodeData* pSourceNode = nullptr;

    int32_t iCurrentTrackIndex = -1;
    int32_t iNextTrackIndex = -1;      // can be filled later per blend, or rebuilt per clip pair
    int32_t iSkeletonBoneIndex = -1;

    Vector3D vDefaultPos;
    Vector3D vDefaultScale;
    SQuaternion qDefaultRot;

    Matrix4 matOffset = Matrix4(1.0f);

    std::vector<int32_t> vChildren;
};
 
class CSkeleton;
class CAnimator
{
public:
    CAnimator();
    void InitializeForSkeleton(const CSkeleton& skeleton);
    void SetAnimationLibrary(const std::unordered_map<std::string, std::shared_ptr<CAnimation>>& Animations);

    void AddAnimation(const std::string& stAnimationName, std::shared_ptr<CAnimation> pAnimation);
    bool PlayAnimation(const std::string& stAnimationName, bool bLoopAnimation = true, float fBlendTime = 0.2f);
    bool PlayAnimation(const std::shared_ptr<CAnimation>& pAnimation, bool bLoopAnimation = true, float fBlendTime = 0.2f);
    bool QueueAnimation(const std::string& stAnimationName, bool bLoopAnimation);

    void UpdateAnimation(float deltatTime);
    void CalculateBoneTransform(const SAssimpNodeData* node, const Matrix4& parentTransform, const CSkeleton* pSkeleton);
    const std::vector<Matrix4>& GetFinalBoneMatrices() const;

    const Matrix4& GetBoneModelTransform(uint32_t index) const;
    const Matrix4& GetBoneModelTransform(const std::string& boneName) const;

    void SetBlendingEnabled(bool bEnabled);

    bool IsUpdating() const;
    void SetUpdating(bool bEnabled);

    int32_t BuildRuntimeNodeRecursive(
        const SAssimpNodeData* pNode,
        const std::shared_ptr<CAnimation>& pAnimation,
        const CSkeleton* pSkeleton);
    void BuildRuntimeNodes();
    void CalculateBlendedBoneTransformCached(
        int32_t nodeIndex,
        const Matrix4& parentTransform,
        float fBlendFactor);
    void PrepareBlendRuntimeData();

protected:
    void AdvancePlayback(SPlaybackState& state, float deltaTime);
    void StartAnimation(const std::shared_ptr<CAnimation>& pAnimation, bool bLoopAnimation);
    bool PlayNextQueuedAnimation();
    void CalculateBlendedBoneTransform(
        const SAssimpNodeData* pNode,
        const Matrix4& parentTransform,
        const CSkeleton* pSkeleton,
        float fBlendFactor);

private:
    std::vector<Matrix4> m_vFinalBoneMatrices;      // skinning matrices
    std::vector<Matrix4> m_vBoneModelTransforms;    // animated bone transforms, no offset

    SPlaybackState m_sCurrentState;
    SPlaybackState m_sNextState;

    std::unordered_map<std::string, std::shared_ptr<CAnimation>> m_mAnimations;
    std::queue<SQueuedAnimation> m_qAnimationQueue;
    std::vector<SRuntimeAnimNode> m_vRuntimeNodes;

    float m_fDeltaTime = 0.0f;

    float m_fBlendTime = 0.0f;
    float m_fBlendTimer = 0.0f;
    bool  m_bBlending = false;
    bool m_bUpdating = true;
    const CSkeleton* m_pSkeleton = nullptr;
};