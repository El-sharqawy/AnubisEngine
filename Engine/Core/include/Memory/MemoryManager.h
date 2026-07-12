#pragma once

#include "ServiceLocator.h"
#include "CorePlatform.h"
#include "MemoryTags.h"
#include <atomic>
#include <mutex>

// Default alloc    -> 16  bytes  (Vec4, Mat4, general objects)
// Terrain buffers  -> 16  bytes(heightmap float arrays, SIMD loops)
constexpr size_t ANUBIS_DEFAULT_ALIGN = 16;		// general + SIMD SSE

// SIMD - heavy math  -> 32  bytes(AVX Mat4 batch transforms)
constexpr size_t ANUBIS_SIMD_ALIGN = 32;		// AVX/AVX2
constexpr size_t ANUBIS_ALIGN_AVX = 32;

// Cache - sensitive  -> 64  bytes(hot structs read every frame)
constexpr size_t ANUBIS_CACHE_ALIGN = 64;		// cache line isolation

// UBO / SSBO data  -> 256 bytes(OpenGL requires this for binding offsets)
constexpr size_t ANUBIS_GPU_ALIGN = 256;		// UBO/SSBO (GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT)

constexpr size_t ANUBIS_ALLOCATE_MAGIC_NUM = 0xDEADBEEF;
constexpr size_t ANUBIS_DEALLOCATE_MAGIC_NUM = 0xBAADF00D;

typedef struct alignas(16) SMemoryBlockHeader
{
	size_t size;
	size_t align;
	size_t adjustment;
	uint32_t magic;   // To detect corruption (e.g., 0xDEADBEEF)
	uint32_t line;
	const char* file;
	const char* typeName;
	const char* function;

	struct SMemoryBlockHeader* next;
	struct SMemoryBlockHeader* prev;

	EMemoryTags tag;
	// Current size (x64): 8+4+4+8+8+8+8 = 48 bytes.
	// 48 is a multiple of 16, so no extra padding is strictly needed,
	// but we can keep a small buffer for future safety.
} TMemoryBlockHeader;

// Service Locator, Memory Manager
class CMemoryManager
{
public:
	// Now = default truly works because all members handle their own init
	CMemoryManager() = default;
	~CMemoryManager();

	// Disable copying - You don't want two managers managing the same list!
	CMemoryManager(const CMemoryManager&) = delete;
	CMemoryManager& operator=(const CMemoryManager&) = delete;

	void* TrackedMalloc(const char* func, const char* file, int line, const char* typeName, size_t size, EMemoryTags tag, size_t align);
	void* TrackedCalloc(const char* func, const char* file, int line, const char* typeName, size_t count, size_t size, EMemoryTags tag, size_t align);
	void* TrackedRealloc(const char* func, const char* file, int line, const char* typeName, void* pUserPtr, size_t nsize);
	void* TrackedStrdup(const char* func, const char* file, int line, const char* typeName, const char* szSource, EMemoryTags tag);
	void TrackedFree(const char* func, const char* file, int line, void* pUserPtr);

	template<typename T, typename... Args>
	T* TrackedNew(const char* func, const char* file, int line, EMemoryTags tag, Args&&... args)
	{
		void* mem = TrackedMalloc(func, file, line, typeid(T).name(), sizeof(T), tag, alignof(T));
		return new (mem) T(std::forward<Args>(args)...);  // placement new — calls constructor
	}

	template<typename T>
	void TrackedDelete(const char* func, const char* file, int line, T* ptr)
	{
		if (!ptr) return;
		ptr->~T();                    // explicit destructor call
		TrackedFree(func, file, line, ptr);       // then free the memory
	}

	// --- Array variants ---
	template<typename T>
	T* TrackedNewArray(const char* func, const char* file, int line, size_t count, EMemoryTags tag)
	{
		void* mem = TrackedMalloc(func, file, line, typeid(T).name(), sizeof(T) * count, tag, alignof(T));
		T* arr = static_cast<T*>(mem);
		for (size_t i = 0; i < count; i++)
			new (&arr[i]) T();        // default-construct each element
		return arr;
	}

	template<typename T>
	void TrackedDeleteArray(const char* func, const char* file, int line, T* ptr, size_t count)
	{
		if (!ptr) return;
		for (size_t i = count; i > 0; i--)
			ptr[i - 1].~T();          // destruct in reverse order (matches C++ standard)
		TrackedFree(func, file, line, ptr);
	}

	// Print Data
	bool Validate();
	void DumpLeaks();
	void PrintData();

	// Get Data
	uint64_t GetTotalAllocatedRaw() const;
	uint64_t GetTotalAllocatedUser() const;
	uint64_t GetTotalFreed() const;
	uint64_t GetPeakUsage() const;
	uint64_t GetCurrentUsage() const;
	uint64_t GetAllocationCount() const;
protected:
	const char* FormatMemorySize(uint64_t bytes);
	void FormatMemorySizeThreadSafe(uint64_t bytes, char* out_buf, size_t buf_size);
	std::string FormatBytes(uint64_t bytes);

private:
	std::atomic<uint64_t> totalAllocatedRaw{ 0 };		// total allocated memory in bytes
	std::atomic<uint64_t> totalAllocatedUser{ 0 };		// total allocated memory in bytes
	std::atomic<uint64_t> totalFreed{ 0 };				// total freed memory in bytes
	std::atomic<uint64_t> peakUsage{ 0 };				// peak memory usage in bytes
	std::atomic<uint64_t> currentUsage{ 0 };			// current memory usage in bytes
	std::atomic<uint64_t> allocationCount{ 0 };			// number of allocations

	size_t usageByTag[MEM_TAG_COUNT]{};					// number of used bytes by each Tag

	SMemoryBlockHeader* head = nullptr;					// Head of the "live" allocations list

	// Use C++ standard library for cross-platform locking
	std::mutex m_lock;

	bool isInitialized = true;
};
