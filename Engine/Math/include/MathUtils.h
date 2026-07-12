#pragma once

#include <cmath>
#include <cstdint>

#undef min
#undef max
#undef minmax

#if defined(_WIN32) || defined(_WIN64)
#define M_E        2.71828182845904523536   // e
#define M_LOG2E    1.44269504088896340736   // log2(e)
#define M_LOG10E   0.434294481903251827651  // log10(e)
#define M_LN2      0.693147180559945309417  // ln(2)
#define M_LN10     2.30258509299404568402   // ln(10)
#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
#define M_PI_4     0.785398163397448309616  // pi/4
#define M_1_PI     0.318309886183790671538  // 1/pi
#define M_2_PI     0.636619772367581343076  // 2/pi
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)
#endif

#define powi(base, exp) (GLint)std::powf((GLfloat)(base), (GLfloat)(exp))

constexpr float VECTOR_EPSILON = 1e-6f;
constexpr double VECTOR_EPSILON_SQ = VECTOR_EPSILON * VECTOR_EPSILON; // Squared epsilon for double precision

struct SEulerAngles
{
	float fYaw;
	float fPitch;
	float fRoll;
};

enum class EAngleUnit
{
	ANGLE_DEGREE,
	ANGLE_RADIAN
};

enum class EMatrixLayout
{
	MATRIX_ROW_MAJOR,
	MATRIX_COLUMN_MAJOR
};

enum class EAxis
{
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_UNDEFINED
};

enum class EQuaternion
{
	QUATERNION_LEFT_HANDED,
	QUATERNION_RIGHT_HANDED
};

enum class EEulerAngleOrder
{
	EULER_ANGLE_ORDER_XYZ,
	EULER_ANGLE_ORDER_XZY,
	EULER_ANGLE_ORDER_YXZ,
	EULER_ANGLE_ORDER_YZX,
	EULER_ANGLE_ORDER_ZXY,
	EULER_ANGLE_ORDER_ZYX
};

namespace EngineMath
{
	int32_t min(int32_t a, int32_t b);
	int32_t max(int32_t a, int32_t b);
	int32_t minmax(int32_t min, int32_t value, int32_t max);

	size_t mins(size_t a, size_t b);
	size_t maxs(size_t a, size_t b);
	size_t minmaxs(size_t min, size_t value, size_t max);

	int64_t minll(int64_t a, int64_t b);
	int64_t maxll(int64_t a, int64_t b);
	int64_t minmaxll(int64_t min, int64_t value, int64_t max);

	float minf(float a, float b);
	float maxf(float a, float b);
	float minmaxf(float min, float value, float max);
}