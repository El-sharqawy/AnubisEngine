#pragma once

#include "API/Buffer.h"
#include <string>

class COpenGLBuffer : public IBuffer
{
public:
    COpenGLBuffer() = default;
    ~COpenGLBuffer() = default;

    // Non-copyable, movable
    COpenGLBuffer(const COpenGLBuffer&) = delete;
    COpenGLBuffer& operator=(const COpenGLBuffer&) = delete;
    COpenGLBuffer(COpenGLBuffer&& other) noexcept;
    COpenGLBuffer& operator=(COpenGLBuffer&& other) noexcept;

    void Clear();
    void UpdateBufferData(const SBufferDesc& bufferDesc, uint32_t uiBufferID);

    // OpenGL Only Data
    uint32_t GetBufferID() const { return (m_uiBufferID); }
    void SetBindingPoint(uint32_t uiBindingPt);
    bool IsBoundToBase() const;
    void UpdateBufferID(uint32_t uiNewBufferID);
    void UpdateBufferSize(uint64_t uiNewBufferSize);

    // Buffer Properties
    const std::string& GetName() const override { return (m_stName); }
    EBufferType GetType() const override { return (m_eType); }
    EBufferMemoryType GetMemoryType() const override { return (m_eMemoryType); }
    EBindingLayoutSetsPoints GetBindingLayoutSetsPoint() const override { return (m_sBindingSets.bindingSet); }
    uint32_t GetBindingPoint() const override { return (m_sBindingSets.bindingPoint); }
    uint64_t GetSize() const override { return (m_uiSize); }
    bool IsValid() const override { return (m_bIsValid); }

private:
    uint32_t m_uiBufferID = 0;
};
