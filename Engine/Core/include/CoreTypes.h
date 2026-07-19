#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>
#include <filesystem>
#include "MathPCH.h"

struct SViewport
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
    float minDepth = 0.f;
    float maxDepth = 1.f;
};

struct SRect2D
{
    int32_t x = 0;
    int32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

struct SUniformBufferBlock
{
    Matrix4 matView;
    Matrix4 matProjection;
    Matrix4 matViewProjection;
};

struct SUniformBufferBlockModel
{
    Matrix4 matModel;
    uint32_t skinPaletteFirstMatrix;
    uint32_t skinMatrixCount;
    uint32_t flags = 0;
    uint32_t padding[1];               // 6468 to 6479 (12 bytes) - Perfect 16-byte alignment
};
