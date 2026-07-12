#include "MathUtils.h"

int32_t EngineMath::min(int32_t a, int32_t b)
{
	return a < b ? a : b;
}

int32_t EngineMath::max(int32_t a, int32_t b)
{
	return a > b ? a : b;
}

int32_t EngineMath::minmax(int32_t min, int32_t value, int32_t max)
{
	if (max < min)
	{
		return EngineMath::max(min, value);
	}

	int tv;
	tv = (min > value ? min : value);
	return (max < tv) ? max : tv;
}

size_t EngineMath::mins(size_t a, size_t b)
{
	return a < b ? a : b;
}

size_t EngineMath::maxs(size_t a, size_t b)
{
	return a > b ? a : b;
}

size_t EngineMath::minmaxs(size_t min, size_t value, size_t max)
{
	if (max < min)
	{
		return EngineMath::maxs(min, value);
	}

	int tv;
	tv = (min > value ? min : value);
	return (max < tv) ? max : tv;
}
int64_t EngineMath::minll(int64_t a, int64_t b)
{
	return a < b ? a : b;
}

int64_t EngineMath::maxll(int64_t a, int64_t b)
{
	return a > b ? a : b;
}

int64_t EngineMath::minmaxll(int64_t min, int64_t value, int64_t max)
{
	if (max < min)
	{
		return EngineMath::maxll(min, value);
	}

	int tv;
	tv = (min > value ? min : value);
	return (max < tv) ? max : tv;
}

float EngineMath::minf(float a, float b)
{
	return a < b ? a : b;
}

float EngineMath::maxf(float a, float b)
{
	return a > b ? a : b;
}

float EngineMath::minmaxf(float min, float value, float max)
{
	if (max < min)
	{
		return maxf(min, value);
	}

	float tv;

	tv = (min > value ? min : value);
	return (max < tv) ? max : tv;
}
