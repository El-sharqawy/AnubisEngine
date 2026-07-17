#include "Stdafx.h"
#include "Model/Animator.h"
#include "Model/Skeleton.h"
#include "Logging/LogManager.h"
#include "EngineMath.h"
#include "EngineMathVectors.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"

CAnimator::CAnimator()
{
    m_vFinalBoneMatrices.resize(MAX_BONES_PER_DRAW, Matrix4(1.0f));
}

void CAnimator::InitializeForSkeleton(const CSkeleton& skeleton)
{
    const uint32_t boneCount = skeleton.GetBoneCount();

    m_pSkeleton = &skeleton;

    m_vRuntimeNodes.clear();

    assert(boneCount <= MAX_BONES_PER_DRAW && "Skeleton bone count exceeds renderer max bones per draw");

    m_vFinalBoneMatrices.resize(boneCount, Matrix4(1.0f));
    m_vBoneModelTransforms.resize(boneCount, Matrix4(1.0f));
}

void CAnimator::SetAnimationLibrary(const std::unordered_map<std::string, std::shared_ptr<CAnimation>>& Animations)
{
    m_mAnimations = Animations;
}

void CAnimator::AddAnimation(const std::string& stAnimationName, std::shared_ptr<CAnimation> pAnimation)
{
    if (!pAnimation)
    {
        return;
    }

    syslog("Added Animation {}", stAnimationName);
    m_mAnimations[stAnimationName] = std::move(pAnimation);
}

void CAnimator::UpdateAnimation(float deltaTime)
{
    m_fDeltaTime = deltaTime;

    if (!m_sCurrentState.pCurrentAnimation || !m_pSkeleton)
    {
        return;
    }

    AdvancePlayback(m_sCurrentState, deltaTime);

    if (m_bBlending && m_sNextState.pCurrentAnimation)
    {
        AdvancePlayback(m_sNextState, deltaTime);

        m_fBlendTimer += deltaTime;
        float alpha = std::clamp(m_fBlendTimer / m_fBlendTime, 0.0f, 1.0f);

        Matrix4 rootTransform = Matrix4(1.0f);

        if (!m_vRuntimeNodes.empty())
        {
            CalculateBlendedBoneTransformCached(0, rootTransform, alpha);
        }

        if (alpha >= 1.0f)
        {
            m_sCurrentState = m_sNextState;
            m_sNextState = {};
            m_bBlending = false;
            m_fBlendTimer = 0.0f;
            m_fBlendTime = 0.0f;

            BuildRuntimeNodes();
        }
    }
    else
    {
        CalculateBoneTransform(&m_sCurrentState.pCurrentAnimation->GetRootNode(), Matrix4(1.0f), m_pSkeleton);
    }
}

void CAnimator::AdvancePlayback(SPlaybackState& state, float deltaTime)
{
    if (!state.pCurrentAnimation)
        return;

    float tps = state.pCurrentAnimation->GetTicksPerSecond();
    if (tps <= 0.0f)
        tps = 25.0f;

    const float duration = state.pCurrentAnimation->GetDuration();
    if (duration <= 0.0f)
        return;

    state.fCurrentTime += tps * deltaTime;

    if (state.bLoop)
    {
        state.fCurrentTime = fmod(state.fCurrentTime, duration);
        if (state.fCurrentTime < 0.0f)
            state.fCurrentTime += duration;
    }
    else
    {
        if (state.fCurrentTime > duration)
            state.fCurrentTime = duration;
    }
}

void CAnimator::CalculateBlendedBoneTransform(
    const SAssimpNodeData* pNode,
    const Matrix4& parentTransform,
    const CSkeleton* pSkeleton,
    float fBlendFactor)
{
    if (!pNode || !pSkeleton) return;
    if (!m_sCurrentState.pCurrentAnimation || !m_sNextState.pCurrentAnimation) return;

    const std::string strNodeName = pNode->stName;

    // FALLBACK DEFAULTS (If bone isn't in an animation channel) 
    // Extract safely by stripping scale components away sequentially
    Vector3D currentPos = EngineMath::ExtractTranslation(pNode->matTransformation);
    Vector3D currentScale = EngineMath::ExtractScale(pNode->matTransformation);
    // Crucial: You must normalize the extraction matrix or scale will corrupt the quaternion
    SQuaternion currentRot = EngineMath::ExtractRotation(pNode->matTransformation);

    Vector3D nextPos = currentPos;
    Vector3D nextScale = currentScale;
    SQuaternion nextRot = currentRot;

    // STATE A: Current Animation Data
    if (CBone* pCurrentBone = m_sCurrentState.pCurrentAnimation->FindBone(strNodeName))
    {
        // Get raw un-baked components directly from keyframes
        currentPos = pCurrentBone->GetInterpolatedPosition(m_sCurrentState.fCurrentTime);
        currentRot = pCurrentBone->GetInterpolatedRotation(m_sCurrentState.fCurrentTime);
        currentScale = pCurrentBone->GetInterpolatedScaling(m_sCurrentState.fCurrentTime);
    }

    // STATE B: Next Animation Data
    if (CBone* pNextBone = m_sNextState.pCurrentAnimation->FindBone(strNodeName))
    {
        // Get raw un-baked components directly from keyframes
        nextPos = pNextBone->GetInterpolatedPosition(m_sNextState.fCurrentTime);
        nextRot = pNextBone->GetInterpolatedRotation(m_sNextState.fCurrentTime);
        nextScale = pNextBone->GetInterpolatedScaling(m_sNextState.fCurrentTime);
    }

    // BLEND CHANNELS
    Vector3D blendedPos = EngineMath::Lerp(currentPos, nextPos, fBlendFactor);
    Vector3D blendedScale = EngineMath::Lerp(currentScale, nextScale, fBlendFactor);
    SQuaternion blendedRot = EngineMath::Slerp(currentRot, nextRot, fBlendFactor);

    // COMPOSE SINGLE FINAL MATRIX
    Matrix4 blendedLocalTransform = EngineMath::ComposeTransform(blendedPos, blendedRot, blendedScale);
    Matrix4 globalTransformation = parentTransform * blendedLocalTransform;

    // SAVE TO FINALS
    const auto& boneInfoMap = pSkeleton->GetBoneInfoMap();
    auto itBoneInfo = boneInfoMap.find(strNodeName);
    if (itBoneInfo != boneInfoMap.end())
    {
        const int iBoneIndex = itBoneInfo->second.iBoneID;
        const Matrix4& matOffset = itBoneInfo->second.matOffset;

        if (iBoneIndex >= 0 && iBoneIndex < static_cast<int>(m_vFinalBoneMatrices.size()))
        {
            m_vBoneModelTransforms[iBoneIndex] = globalTransformation;
            m_vFinalBoneMatrices[iBoneIndex] = globalTransformation * matOffset;
        }
    }

    // --- HIERARCHY RECURSION ---
    for (uint32_t i = 0; i < pNode->iChildrenCount; ++i)
    {
        CalculateBlendedBoneTransform(
            &pNode->vChildren[i],
            globalTransformation,
            pSkeleton,
            fBlendFactor);
    }
}

bool CAnimator::PlayAnimation(const std::string& stAnimationName, bool bLoopAnimation, float fBlendTime)
{
    auto it = m_mAnimations.find(stAnimationName);
    if (it == m_mAnimations.end())
    {
        syserr("No animation called {}", stAnimationName);
        return false;
    }

    const auto& pNewAnimation = it->second;
    if (!pNewAnimation)
    {
        return false;
    }

    if (m_sCurrentState.pCurrentAnimation == pNewAnimation)
    {
        return true;
    }

    while (!m_qAnimationQueue.empty())
    {
        m_qAnimationQueue.pop();
    }

    if (!m_sCurrentState.pCurrentAnimation || fBlendTime <= 0.0f)
    {
        m_sCurrentState.pCurrentAnimation = pNewAnimation;
        m_sCurrentState.fCurrentTime = 0.0f;
        m_sCurrentState.bLoop = bLoopAnimation;

        m_bBlending = false;
        m_fBlendTimer = 0.0f;
        m_fBlendTime = 0.0f;
        BuildRuntimeNodes();
        return true;
    }

    m_sNextState.pCurrentAnimation = pNewAnimation;
    m_sNextState.fCurrentTime = 0.0f;
    m_sNextState.bLoop = bLoopAnimation;

    m_fBlendTime = fBlendTime;
    m_fBlendTimer = 0.0f;
    m_bBlending = true;
    PrepareBlendRuntimeData();
    return true;
}

bool CAnimator::PlayAnimation(const std::shared_ptr<CAnimation>& pAnimation, bool bLoopAnimation, float fBlendTime)
{
    if (pAnimation == nullptr)
    {
        syserr("No Animation Pointer Given");
        return (false);
    }

    if (m_sCurrentState.pCurrentAnimation == pAnimation)
    {
        return true;
    }

    while (!m_qAnimationQueue.empty())
    {
        m_qAnimationQueue.pop();
    }

    if (!m_sCurrentState.pCurrentAnimation || fBlendTime <= 0.0f)
    {
        m_sCurrentState.pCurrentAnimation = pAnimation;
        m_sCurrentState.fCurrentTime = 0.0f;
        m_sCurrentState.bLoop = bLoopAnimation;

        m_bBlending = false;
        m_fBlendTimer = 0.0f;
        m_fBlendTime = 0.0f;
        BuildRuntimeNodes();
        return true;
    }

    m_sNextState.pCurrentAnimation = pAnimation;
    m_sNextState.fCurrentTime = 0.0f;
    m_sNextState.bLoop = bLoopAnimation;

    m_fBlendTime = fBlendTime;
    m_fBlendTimer = 0.0f;
    m_bBlending = true;
    PrepareBlendRuntimeData();
    return true;
}

bool CAnimator::QueueAnimation(const std::string& stAnimationName, bool bLoopAnimation)
{
    auto it = m_mAnimations.find(stAnimationName);
    if (it == m_mAnimations.end())
    {
        return false;
    }

    if (!m_sCurrentState.pCurrentAnimation)
    {
        StartAnimation(it->second, bLoopAnimation);
        return true;
    }

    m_qAnimationQueue.push({ stAnimationName, bLoopAnimation });
    return true;
}

void CAnimator::StartAnimation(const std::shared_ptr<CAnimation>& pAnimation, bool bLoopAnimation)
{
    m_sCurrentState.pCurrentAnimation = pAnimation;
    m_sCurrentState.fCurrentTime = 0.0f;
    m_sCurrentState.bLoop = bLoopAnimation;

    BuildRuntimeNodes();
}

bool CAnimator::PlayNextQueuedAnimation()
{
    while (!m_qAnimationQueue.empty())
    {
        SQueuedAnimation nextAnimation = m_qAnimationQueue.front();
        m_qAnimationQueue.pop();

        auto it = m_mAnimations.find(nextAnimation.stName);
        if (it == m_mAnimations.end())
        {
            continue; // skip missing clip names
        }

        syslog("Added Animation: {} - Loop: {}", nextAnimation.stName, nextAnimation.bLoop);
        StartAnimation(it->second, nextAnimation.bLoop);
        return true;
    }

    return false;
}

void CAnimator::CalculateBoneTransform(const SAssimpNodeData* node, const Matrix4& parentTransform, const CSkeleton* pSkeleton)
{
    std::string nodeName = node->stName;
    Matrix4 nodeTransform = node->matTransformation;
    CBone* pBone = m_sCurrentState.pCurrentAnimation->FindBone(nodeName);

    if (pBone)
    {
        pBone->Update(m_sCurrentState.fCurrentTime);
        nodeTransform = pBone->GetLocalTransform();
    }

    Matrix4 matGlobalTransform = parentTransform * nodeTransform;

    const auto& boneMapInfo = m_sCurrentState.pCurrentAnimation->GetBoneIDMap();
    auto it = boneMapInfo.find(nodeName);

    if (it != boneMapInfo.end())
    {
        int32_t index = it->second.iBoneID;
        Matrix4 matOffset = it->second.matOffset;

        if (index >= 0 && index < static_cast<int32_t>(m_vFinalBoneMatrices.size()))
        {
            Matrix4 matBoneModel = pSkeleton->GetGlobalInverseTransform() * matGlobalTransform;

            m_vBoneModelTransforms[index] = matBoneModel;
            m_vFinalBoneMatrices[index] = matBoneModel * matOffset;
        }
    }

    for (int i = 0; i < node->iChildrenCount; i++)
    {
        CalculateBoneTransform(&node->vChildren[i], matGlobalTransform, pSkeleton);
    }
}

const std::vector<Matrix4>& CAnimator::GetFinalBoneMatrices() const
{
    return m_vFinalBoneMatrices;
}

const Matrix4& CAnimator::GetBoneModelTransform(uint32_t index) const
{
    assert(index < m_vBoneModelTransforms.size());
    return m_vBoneModelTransforms[index];
}

const Matrix4& CAnimator::GetBoneModelTransform(const std::string& boneName) const
{
    assert(m_pSkeleton);

    const auto& boneInfoMap = m_pSkeleton->GetBoneInfoMap();
    auto it = boneInfoMap.find(boneName);

    static Matrix4 s_Matrix4 = Matrix4(1.0f);
    if (it == boneInfoMap.end())
    {
        syserr("Bone {} not found", boneName);
        return s_Matrix4;
    }

    // assert(it != boneInfoMap.end() && "Bone name not found");
    return m_vBoneModelTransforms[it->second.iBoneID];
}

void CAnimator::SetBlendingEnabled(bool bEnabled)
{
    m_bBlending = bEnabled;
}

int32_t CAnimator::BuildRuntimeNodeRecursive(
    const SAssimpNodeData* pNode,
    const std::shared_ptr<CAnimation>& pAnimation,
    const CSkeleton* pSkeleton)
{
    if (!pNode)
    {
        return -1;
    }

    const int32_t nodeIndex = static_cast<int32_t>(m_vRuntimeNodes.size());
    m_vRuntimeNodes.emplace_back();

    m_vRuntimeNodes[nodeIndex].pSourceNode = pNode;
    m_vRuntimeNodes[nodeIndex].vDefaultPos = EngineMath::ExtractTranslation(pNode->matTransformation);
    m_vRuntimeNodes[nodeIndex].vDefaultScale = EngineMath::ExtractScale(pNode->matTransformation);
    m_vRuntimeNodes[nodeIndex].qDefaultRot = EngineMath::ExtractRotation(pNode->matTransformation);

    const std::string& nodeName = pNode->stName;

    if (pAnimation)
        m_vRuntimeNodes[nodeIndex].iCurrentTrackIndex = pAnimation->FindBoneTrackIndex(nodeName);

    const auto& boneInfoMap = pSkeleton->GetBoneInfoMap();
    auto itBone = boneInfoMap.find(nodeName);
    if (itBone != boneInfoMap.end())
    {
        m_vRuntimeNodes[nodeIndex].iSkeletonBoneIndex = itBone->second.iBoneID;
        m_vRuntimeNodes[nodeIndex].matOffset = itBone->second.matOffset;
    }

    m_vRuntimeNodes[nodeIndex].vChildren.reserve(pNode->iChildrenCount);

    for (uint32_t i = 0; i < pNode->iChildrenCount; ++i)
    {
        int32_t childIndex = BuildRuntimeNodeRecursive(&pNode->vChildren[i], pAnimation, pSkeleton);
        if (childIndex >= 0)
        {
            m_vRuntimeNodes[nodeIndex].vChildren.push_back(childIndex);
        }
    }

    return nodeIndex;
}


void CAnimator::BuildRuntimeNodes()
{
    m_vRuntimeNodes.clear();

    if (!m_sCurrentState.pCurrentAnimation || !m_pSkeleton)
        return;

    const SAssimpNodeData& rootNode = m_sCurrentState.pCurrentAnimation->GetRootNode();

    m_vRuntimeNodes.reserve(128); // or estimated node count
    BuildRuntimeNodeRecursive(&rootNode, m_sCurrentState.pCurrentAnimation, m_pSkeleton);
}

void CAnimator::CalculateBlendedBoneTransformCached(
    int32_t nodeIndex,
    const Matrix4& parentTransform,
    float fBlendFactor)
{
    const SRuntimeAnimNode& node = m_vRuntimeNodes[nodeIndex];

    Vector3D currentPos = node.vDefaultPos;
    Vector3D currentScale = node.vDefaultScale;
    SQuaternion currentRot = node.qDefaultRot;

    Vector3D nextPos = node.vDefaultPos;
    Vector3D nextScale = node.vDefaultScale;
    SQuaternion nextRot = node.qDefaultRot;

    if (node.iCurrentTrackIndex >= 0)
    {
        const CBone& bone = m_sCurrentState.pCurrentAnimation->GetBoneByIndex(node.iCurrentTrackIndex);
        currentPos = bone.GetInterpolatedPosition(m_sCurrentState.fCurrentTime);
        currentRot = bone.GetInterpolatedRotation(m_sCurrentState.fCurrentTime);
        currentScale = bone.GetInterpolatedScaling(m_sCurrentState.fCurrentTime);
    }

    if (node.iNextTrackIndex >= 0)
    {
        const CBone& bone = m_sNextState.pCurrentAnimation->GetBoneByIndex(node.iNextTrackIndex);
        nextPos = bone.GetInterpolatedPosition(m_sNextState.fCurrentTime);
        nextRot = bone.GetInterpolatedRotation(m_sNextState.fCurrentTime);
        nextScale = bone.GetInterpolatedScaling(m_sNextState.fCurrentTime);
    }

    Matrix4 localTransform = EngineMath::ComposeTransform(
        EngineMath::Lerp(currentPos, nextPos, fBlendFactor),
        EngineMath::Slerp(currentRot, nextRot, fBlendFactor),
        EngineMath::Lerp(currentScale, nextScale, fBlendFactor));

    Matrix4 globalTransform = parentTransform * localTransform;
    Matrix4 boneModelSpace = m_pSkeleton->GetGlobalInverseTransform() * globalTransform;

    if (node.iSkeletonBoneIndex >= 0 &&
        node.iSkeletonBoneIndex < static_cast<int32_t>(m_vFinalBoneMatrices.size()))
    {
        m_vBoneModelTransforms[node.iSkeletonBoneIndex] = boneModelSpace;
        m_vFinalBoneMatrices[node.iSkeletonBoneIndex] = boneModelSpace * node.matOffset;
    }

    for (int32_t childIndex : node.vChildren)
    {
        CalculateBlendedBoneTransformCached(childIndex, globalTransform, fBlendFactor);
    }
}

void CAnimator::PrepareBlendRuntimeData()
{
    if (!m_sNextState.pCurrentAnimation)
        return;

    for (SRuntimeAnimNode& node : m_vRuntimeNodes)
    {
        node.iNextTrackIndex = -1;

        if (!node.pSourceNode)
            continue;

        node.iNextTrackIndex =
            m_sNextState.pCurrentAnimation->FindBoneTrackIndex(node.pSourceNode->stName);
    }
}
