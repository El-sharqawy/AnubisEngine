#pragma once

#include <cstdint>
#include <string>

// The CRC table must be accessible. It's often declared as a const array.
extern const uint32_t CRCTable[256];

// Modern C++ CRC32
uint32_t GetCRC32(const std::string& buf);
uint32_t GetCRC32(const std::string& buf, const std::string& name);

uint32_t GetCaseCRC32(const std::string& buf);
uint32_t GetCaseCRC32(const std::string& buf, const std::string& name);
