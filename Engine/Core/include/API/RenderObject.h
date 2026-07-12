#pragma once

#include "API/Pipeline.h"
#include "API/Material.h"
#include "API/Buffer.h"
#include "TypeMatrix4.h"

class CVertexArray;

struct SRenderItem
{
    IPipeline* pPipeline = nullptr;
    IMaterial* pMaterial = nullptr;
    IBuffer* pVertexBuffer = nullptr;
    IBuffer* pIndexBuffer = nullptr;

    CVertexArray* pVertexArray = nullptr;

    uint32_t    indexCount = 0;
    uint32_t    firstIndex = 0;

    Matrix4   modelMatrix{ 1.0f };

    uint64_t    sortKey = 0;
};

class IRenderObject
{
public:
    virtual ~IRenderObject() = default;
};