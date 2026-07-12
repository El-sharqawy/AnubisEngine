// stdafx.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef STDAFX_MATH_H
#define STDAFX_MATH_H

// add headers that you want to pre-compile here
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

#include <cmath>

#include <string>		// Fixes 'string' is not a member of 'std'
#include <fstream>		// Fixes 'ofstream' is not a member of 'std'
#include <mutex>		// Fixes 'mutex' is not a member of 'std'
#include <vector>		// Needed for your SMeshData2D and SFontInfo
#include <algorithm>
#include <cassert>

#include "MathUtils.h"
#include "EngineMath.h"
#include "Quaternion.h"
#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include "TypeMatrix2.h"
#include "TypeMatrix3.h"
#include "TypeMatrix4.h"
#include "Grid.h"

#endif //STDAFX_MATH_H