#include "Stdafx.h"
#include "Memory/MemoryManager.h"
#include "Logging/LogManager.h"
#include <memory>
#include <cstdio>
#include <cstdlib>

// #define ENABLE_MEMORY_LOGS

CMemoryManager::~CMemoryManager()
{

}

void* CMemoryManager::TrackedMalloc(const char* func, const char* file, int line, const char* typeName, size_t size, EMemoryTags tag, size_t align)
{
	// 1. Adjustment must be at least header_size so the header fits before user ptr.
    // Also must satisfy the requested alignment.

	size_t header_size = sizeof(SMemoryBlockHeader);
	size_t adjustment = (align > header_size) ? align : header_size;
	if (adjustment % align != 0)
	{
		adjustment += align - (adjustment % align);
	}

	size_t total_size = size + adjustment;

	// 2. Use _aligned_malloc or ensure malloc gives us 16-byte alignment
	// On most 64-bit systems, malloc is 16-byte aligned by default.
	// size_t* raw_ptr = (size_t*)malloc(total_size); // allocate with total size
	void* raw_ptr = _mm_malloc(total_size, align); // Ensure we get a 16-byte aligned block from the OS

	if (!raw_ptr)
	{
		return (nullptr);
	}

	// Place the header at the END of the adjustment zone,
	// just before where the user pointer begins.
	//
	//  raw_ptr                         user_ptr
	//    |<------- adjustment -------->|<--- size --->|
	//    |  [padding][SMemoryBlockHeader]              |
	//
	uint8_t* pUserPtr = static_cast<uint8_t*>(raw_ptr) + adjustment;

	// 3. Fill the header — now Free() can recover everything from user_ptr alone
	SMemoryBlockHeader* header = reinterpret_cast<SMemoryBlockHeader*>(pUserPtr) - 1; // 1 = sizeof(type)

	header->size = size;
	header->align = align;
	header->adjustment = adjustment;
	header->magic = ANUBIS_ALLOCATE_MAGIC_NUM;
	header->file = file;
	header->line = line;
	header->function = func;
	header->typeName = typeName;
	header->tag = tag;

	// 4. Thread-Safe Linked List Insertion
	{
		std::lock_guard<std::mutex> lock(m_lock);
		header->next = head;
		header->prev = nullptr;
		if (head)
		{
			head->prev = header;
		}
		head = header;

		currentUsage += total_size;
		totalAllocatedRaw += total_size;
		totalAllocatedUser += size;
		allocationCount++;
		if (currentUsage > peakUsage)
		{
			peakUsage = currentUsage.load();
		}
		usageByTag[tag] += size;
	}

#if defined(_DEBUG)
	if ((reinterpret_cast<uintptr_t>(pUserPtr) % align) != 0)
	{
		syserrc("Alignment Error! Requested: %zu, Got: %p", align, pUserPtr);
	}
#endif

#if defined(ENABLE_MEMORY_LOGS)
	// Use the manager's current usage for the log
	syslogc("allocated: %zu bytes (total: %zu). Total system usage: %llu (%s:%d)", size, total_size, currentUsage.load(), file, line);
#endif

	// Return pointer after the header
	return pUserPtr;
}

void* CMemoryManager::TrackedCalloc(const char* func, const char* file, int line, const char* typeName, size_t count, size_t size, EMemoryTags tag, size_t align)
{
	size_t total_size = count * size; // actual size

	// We still need our header for the tracker!
	void* ptr = TrackedMalloc(func, file, line, typeName, total_size, tag, align);
	if (ptr)
	{
		memset(ptr, 0, total_size);
	}

	return (ptr);
}

void* CMemoryManager::TrackedRealloc(const char* func, const char* file, int line, const char* typeName, void* pUserPtr, size_t nsize)
{
	// If size is 0, it's just a free
	if (nsize == 0)
	{
		TrackedFree(func, file, line, pUserPtr);
		return nullptr;
	}

	// If ptr is NULL, it's just a malloc
	if (pUserPtr == nullptr)
	{
		return TrackedMalloc(func, file, line, typeName ? typeName : nullptr, nsize, MEM_TAG_NONE, ANUBIS_DEFAULT_ALIGN);
	}

	// Move the pointer back to find the header
	SMemoryBlockHeader* pOldHeader = reinterpret_cast<SMemoryBlockHeader*>(pUserPtr) - 1; // 1 = sizeof(type)

	// Safety check: Validate the magic number before doing anything
	if (pOldHeader->magic != ANUBIS_ALLOCATE_MAGIC_NUM)
	{
		syserr("Critical: realloc on invalid/corrupt pointer!");
		__debugbreak();
		return (nullptr);
	}

	// Allocate the NEW block
	const char* finalTypeName = (typeName != nullptr) ? typeName : pOldHeader->typeName;

	void* new_ptr = TrackedMalloc(func, file, line, finalTypeName, nsize, pOldHeader->tag, ANUBIS_DEFAULT_ALIGN);
	if (!new_ptr)
	{
		// Recovery: If realloc fails, the old pointer is still valid
		// but our stats are now wrong, We should handle this!
		syserr("Realloc failed!");
		return pUserPtr;
	}

	// Update header and stats for the new size
	size_t copy_size = (pOldHeader->size < nsize) ? pOldHeader->size : nsize;
	memcpy(new_ptr, pUserPtr, copy_size);

#if defined(ENABLE_MEMORY_LOGS)
	syslogc("Reallocated: %zu bytes (Old: %zu)", nsize, pOldHeader->size);
#endif

	// Free the old block
	TrackedFree(func, file, line, pUserPtr);

	return new_ptr;
}

void* CMemoryManager::TrackedStrdup(const char* func, const char* file, int line, const char* typeName, const char* szSource, EMemoryTags tag)
{
	if (!szSource)
	{
		char* emptyStr = (char*)TrackedMalloc(func, file, line, typeName ? typeName : "char*", 1, tag, ANUBIS_DEFAULT_ALIGN);
		if (emptyStr)
		{
			emptyStr[0] = '\0';
		}
		return emptyStr;
	}

	size_t len = strlen(szSource) + 1; // +1 for the null terminator '\0'
	char* newStr = (char*)TrackedMalloc(func, file, line, typeName ? typeName : "char*", len, tag, ANUBIS_DEFAULT_ALIGN);

	if (newStr)
	{
		memcpy(newStr, szSource, len);
	}

	return (newStr);
}

void CMemoryManager::TrackedFree(const char* func, const char* file, int line, void* pUserPtr)
{
	if (pUserPtr == nullptr)
	{
		return;
	}

	// 1. Move the pointer back to find the header
	// Shift back by struct size to find the real start
	SMemoryBlockHeader* header = reinterpret_cast<SMemoryBlockHeader*>(pUserPtr) - 1;

	// 2. Validation Check
	if (header->magic != ANUBIS_ALLOCATE_MAGIC_NUM)
	{
		fprintf(stderr, "MEMORY CORRUPTION! \n");
		// fprintf(stderr, "Attempted free at: %s:%d\n", get_filename(file), line);

		if (header->magic == ANUBIS_DEALLOCATE_MAGIC_NUM)
		{
			fprintf(stderr, "Error: DOUBLE FREE detected! (Already freed elsewhere) (%s, %s, %d)\n", func, file, line);
		}
		else
		{
			fprintf(stderr, "Error: Pointer was never allocated or is corrupted. (%s, %s, %d)\n", func, file, line);
		}
		return;
	}

	assert(header->adjustment >= sizeof(SMemoryBlockHeader) && "TrackedFree: adjustment is invalid — header likely corrupted!");

	size_t total_size = header->size + header->adjustment;

	// 3. Thread-Safe Unlinking
	{
		std::lock_guard<std::mutex> lock(m_lock);
		if (header->prev)
		{
			// Not the head: point the previous node to our next node
			header->prev->next = header->next;
		}
		else
		{
			// This IS the head: move the head pointer to our next node
			head = header->next;
		}

		if (header->next)
		{
			// Not the tail: point the next node back to our previous node
			header->next->prev = header->prev;
		}

		// 3.1 Update Stats
		currentUsage -= total_size;
		totalFreed += total_size;
		allocationCount--;
		// Update tags
		usageByTag[header->tag] -= header->size;
	}

#if defined(ENABLE_MEMORY_LOGS)
	syslogc("Automatically detected and will free: %zu (total: %zu) bytes (%s:%d)", header->size, total_size, Anubis::GetFileName(file), line);
#endif

	// 5. Clean up the evidence (Defensive Programming)
	header->magic = ANUBIS_DEALLOCATE_MAGIC_NUM; // Custom "Already Freed" magic

	// Fill user memory with a garbage pattern to catch "use-after-free"
	memset(pUserPtr, 0xFE, header->size); // Easy to track use-after-free bugs
	
	// Recover the original raw_ptr using the stored adjustment
	void* raw_ptr = static_cast<uint8_t*>(pUserPtr) - header->adjustment;

	_mm_free(raw_ptr);
}

bool CMemoryManager::Validate()
{
	SMemoryBlockHeader* curr = head;
	int32_t index = 0;
	bool is_corrupt = false;

	std::lock_guard<std::mutex> lock(m_lock);
	while (curr)
	{
		// 1. Check Magic Number
		if (curr->magic != ANUBIS_ALLOCATE_MAGIC_NUM)
		{
			syserrc("CRITICAL: Memory Corruption detected at block %d!", index);
			syserrc("Block allocated at %s:%d (Type: %s)", curr->file, curr->line, curr->typeName ? curr->typeName : "Unknown");

			// If the magic is 0xBAADF00D, we found a node that stayed in the list after free
			if (curr->magic == ANUBIS_DEALLOCATE_MAGIC_NUM)
			{
				syserr("Error: Node is marked as FREED but still exists in the live list!");
			}

			is_corrupt = true;
			// We stop here because if the header is corrupt, the 'next' pointer might be garbage
			break;
		}

		// 2. Cross-link validation (The "Perfect" Check)
		// If I have a next, its 'prev' MUST be me.
		if (curr->next && curr->next->prev != curr)
		{
			syserrc("CRITICAL: Linked List pointer corruption at %s:%d (Type: %s)", curr->file, curr->line, curr->typeName ? curr->typeName : "UnKnown");
			is_corrupt = true;
			break;
		}

		curr = curr->next;
		index++;
	}

	if (is_corrupt)
	{
		// Force a crash so you can see the callstack in the debugger
		syserr("CRITICAL: Pointer corruption...");
#ifdef _MSC_VER
		__debugbreak(); // This stops the code in Visual Studio right on the line!
#endif
		assert(!is_corrupt);
	}

	return !is_corrupt;
}

void CMemoryManager::DumpLeaks()
{
	std::lock_guard<std::mutex> lock(m_lock);

	SMemoryBlockHeader* curr = head;
	
	if (!curr)
	{
		syslog("No leaks detected! Great job.");
	}
	else
	{
		syslog("--- MEMORY LEAK REPORT ---");
		while (curr)
		{
			syslogc("Leak: %zu bytes allocated at %s:%d", curr->size, curr->file, curr->line);
			curr = curr->next;
		}
	}
}

void CMemoryManager::PrintData()
{
	std::lock_guard<std::mutex> lock(m_lock);

	SMemoryBlockHeader* curr = head;

	if (!curr)
	{
		syslogc("No Current Active Elements.");
	}
	else
	{
		syslogc("--- MEMORY MANAGER REPORT ---");
		syslogc("Allocation Count: %llu", allocationCount.load());

		std::string totalAllocatedRawString = FormatBytes(totalAllocatedRaw.load());
		std::string totalAllocatedUserString = FormatBytes(totalAllocatedUser.load());
		std::string currentAllocatedString = FormatBytes(currentUsage.load());
		std::string totalFreedString = FormatBytes(totalFreed.load());
		std::string peakString = FormatBytes(peakUsage.load());

		syslogc("Total Allocated Raw: %s", totalAllocatedRawString.c_str());
		syslogc("Total Allocated User: %s", totalAllocatedUserString.c_str());
		syslogc("Current Usage: %s", currentAllocatedString.c_str());
		syslogc("Current Total Freed: %s", totalFreedString.c_str());
		syslogc("Peak Usage: %s", peakString.c_str());

		while (curr)
		{
			if (curr->typeName != NULL)
			{
				syslogc("Object Allocated At %s:%d with size: %s, Type: %s", curr->file, curr->line, FormatBytes(curr->size).c_str(), curr->typeName);
			}
			else
			{
				syslogc("Object Allocated At %s:%d with size: %s", curr->file, curr->line, FormatBytes(curr->size).c_str());
			}
			curr = curr->next;
		}
	}
}

uint64_t CMemoryManager::GetTotalAllocatedRaw() const
{
	return totalAllocatedRaw.load();
}

uint64_t CMemoryManager::GetTotalAllocatedUser() const
{
	return totalAllocatedUser.load();
}

uint64_t CMemoryManager::GetTotalFreed() const
{
	return totalFreed.load();
}

uint64_t CMemoryManager::GetPeakUsage() const
{
	return peakUsage.load();
}

uint64_t CMemoryManager::GetCurrentUsage() const
{
	return currentUsage.load();
}

uint64_t CMemoryManager::GetAllocationCount() const
{
	return allocationCount.load();
}

const char* CMemoryManager::FormatMemorySize(uint64_t bytes)
{
	static char buffer[32]; // Static buffer for quick logging (not thread-safe!)
	const char* units[] = { "Bytes", "KB", "MB", "GB", "TB" };
	int32_t unit_index = 0;
	double size = (double)bytes;

	while (size >= 1024 && unit_index < 4) {
		size /= 1024;
		unit_index++;
	}

	// Format to 2 decimal places
	snprintf(buffer, sizeof(buffer), "%.0f %s", size, units[unit_index]);
	return buffer;
}

void CMemoryManager::FormatMemorySizeThreadSafe(uint64_t bytes, char* out_buf, size_t buf_size)
{
	if (!out_buf || buf_size == 0)
	{
		return;
	}

	const char* units[] = { "Bytes", "KB", "MB", "GB", "TB" };
	int32_t unit_index = 0;
	double size = static_cast<double>(bytes);

	// Using 1024.0 to ensure double precision math
	while (size >= 1024.0 && unit_index < 4)
	{
		size /= 1024.0;
		unit_index++;
	}

	snprintf(out_buf, buf_size, "%.2f %s", size, units[unit_index]);
}

std::string CMemoryManager::FormatBytes(uint64_t bytes)
{
	static const std::vector<std::string> units = { "Bytes", "KB", "MB", "GB", "TB" };
	double size = static_cast<double>(bytes);
	size_t unit_index = 0;

	while (size >= 1024.0 && unit_index < units.size() - 1)
	{
		size /= 1024.0;
		unit_index++;
	}

	// std::stringstream ss;
	// ss << std::fixed << std::setprecision(2) << size << " " << units[unit_index];
	// Using snprintf is often faster than stringstream for simple formatting
	char buf[32];
	snprintf(buf, sizeof(buf), "%.2f %s", size, units[unit_index].c_str());
	return std::string(buf);
}