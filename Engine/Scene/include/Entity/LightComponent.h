#pragma once

#include "Entity.h"
#include "TypeVector3.h"

enum class ELightType
{
    Directional,
    Point,
    Spot
};

class CFrameBuffer;

class CLightComponent : public CIComponent
{
public:
    CLightComponent() = default;
    ~CLightComponent() = default;

    ELightType GetType() const { return m_eType; }
    void SetType(ELightType t) { m_eType = t; }

    const Vector3D& GetColor() const { return m_v3Color; }
    void SetColor(const Vector3D& c) { m_v3Color = c; }

    float GetIntensity() const { return m_fIntensity; }
    void SetIntensity(float v) { m_fIntensity = v; }

    float GetRange() const { return m_fRange; }
    void SetRange(float v) { m_fRange = v; }

    float GetInnerCone() const { return m_fInnerCone; }
    void SetInnerCone(float v) { m_fInnerCone = v; }

    float GetOuterCone() const { return m_fOuterCone; }
    void SetOuterCone(float v) { m_fOuterCone = v; }

    bool CastsShadows() const { return m_bCastShadows; }
    void SetCastShadows(bool v) { m_bCastShadows = v; }

private:
    ELightType m_eType = ELightType::Point;

    Vector3D m_v3Color = Vector3D(1.0f);
    float m_fIntensity = 1.0f;
    float m_fRange = 10.0f;

    float m_fInnerCone = 0.8f;
    float m_fOuterCone = 1.0f;

    bool m_bCastShadows = false;
};
