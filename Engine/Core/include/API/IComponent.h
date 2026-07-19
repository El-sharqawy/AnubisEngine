#pragma once

class CEntity;

class IComponent
{
public:
    virtual ~IComponent() = default;

protected:
    CEntity* m_pOwnerEntity;
};
