// AnubisAssert.h
#pragma once

#include <cstdlib>      // std::abort
#include <cstdio>       // std::fprintf

// ----------------------------------------------------------------
//  Platform: break into debugger before abort so you get a stack
//  trace in Visual Studio / RenderDoc instead of a crash dialog
// ----------------------------------------------------------------
#if defined(_MSC_VER)
#define ANUBIS_DEBUG_BREAK()  __debugbreak()
#elif defined(__GNUC__) || defined(__clang__)
#define ANUBIS_DEBUG_BREAK()  __builtin_trap()
#else
#define ANUBIS_DEBUG_BREAK()  std::abort()
#endif

// ----------------------------------------------------------------
//  Core assert — always compiled in (even Release)
//  Use for invariants that must NEVER be violated
// ----------------------------------------------------------------
#define ANUBIS_ASSERT(expr, ...)                                                \
    do {                                                                        \
        if (!(expr)) [[unlikely]]                                               \
        {                                                                       \
            std::fprintf(stderr,                                                \
                "\n[ANUBIS] Assertion failed!\n"                               \
                "  Expression : %s\n"                                           \
                "  File       : %s\n"                                           \
                "  Line       : %d\n"                                           \
                "  Function   : %s\n",                                          \
                #expr, __FILE__, __LINE__, __FUNCTION__);                       \
            std::fprintf(stderr, "  Message    : " __VA_ARGS__);                \
            std::fprintf(stderr, "\n\n");                                       \
            ANUBIS_DEBUG_BREAK();                                               \
            std::abort();                                                       \
        }                                                                       \
    } while(0)
// ----------------------------------------------------------------
//  Assert with message
// ----------------------------------------------------------------
#define ANUBIS_ASSERT_MSG(expr, msg)                                            \
    do {                                                                        \
        if (!(expr)) [[unlikely]]                                               \
        {                                                                       \
            std::fprintf(stderr,                                                \
                "\n[ANUBIS] Assertion failed!\n"                               \
                "  Expression : %s\n"                                           \
                "  Message    : %s\n"                                           \
                "  File       : %s\n"                                           \
                "  Line       : %d\n"                                           \
                "  Function   : %s\n\n",                                        \
                #expr, msg, __FILE__, __LINE__, __FUNCTION__);                  \
            ANUBIS_DEBUG_BREAK();                                               \
            std::abort();                                                       \
        }                                                                       \
    } while(0)

// ----------------------------------------------------------------
//  Debug-only assert — stripped completely in Release
//  Use for expensive checks (loop invariants, pointer walks)
// ----------------------------------------------------------------
#if defined(_DEBUG) || defined(ANUBIS_ENABLE_DEBUG_ASSERTS)
#define ANUBIS_DASSERT(expr)          ANUBIS_ASSERT(expr)
#define ANUBIS_DASSERT_MSG(expr, msg) ANUBIS_ASSERT_MSG(expr, msg)
#else
#define ANUBIS_DASSERT(expr)          do { (void)(expr); } while(0)
#define ANUBIS_DASSERT_MSG(expr, msg) do { (void)(expr); } while(0)
#endif

// ----------------------------------------------------------------
//  Static (compile-time) assert — clean alias
// ----------------------------------------------------------------
#define ANUBIS_STATIC_ASSERT(expr, msg)   static_assert(expr, msg)

// ----------------------------------------------------------------
//  Unreachable code marker
// ----------------------------------------------------------------
#define ANUBIS_UNREACHABLE()                                                    \
    do {                                                                        \
        std::fprintf(stderr,                                                    \
            "\n[ANUBIS] Unreachable code reached!\n"                           \
            "  File     : %s\n"                                                 \
            "  Line     : %d\n"                                                 \
            "  Function : %s\n\n",                                              \
            __FILE__, __LINE__, __FUNCTION__);                                  \
        ANUBIS_DEBUG_BREAK();                                                   \
        std::abort();                                                           \
    } while(0)