#pragma once

#include <cstdio>
#include <glad/glad.h>

#ifdef _MSC_VER
// Microsoft Visual C++ uses __FUNCTION__ as a non-standard alternative
#define FUNCTION_NAME __FUNCTION__
#elif defined(__GNUC__) || defined(__clang__)
// GCC and Clang use the standard __func__
#define FUNCTION_NAME __func__
#else
// Fallback for other compilers, might not be as useful
#define FUNCTION_NAME "UnknownFunction"
#endif

#define syserr(...) fprintf(stderr, "%s: ", FUNCTION_NAME); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");
#define syslog(...) fprintf(stdout, "%s: ", FUNCTION_NAME); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n");

#if !defined(EXIT_SUCCESS)
#define EXIT_SUCCESS 0
#endif

#if !defined(EXIT_FAILURE)
#define EXIT_FAILURE 1
#endif

inline bool IsGLVersionHigher(int MajorVer, int MinorVer)
{
    static int glMajorVersion = 0;
    static int glMinorVersion = 0;

    glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
    glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);

    sys_log("IsGLVersionHigher: v%d.%d", glMajorVersion, glMinorVersion);

    return ((glMajorVersion >= MajorVer) && (glMinorVersion >= MinorVer));
}

