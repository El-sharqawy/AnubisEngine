#include "Stdafx.h"
#include "Time/TimerManager.h"

CTimerManager::CTimerManager()
{
	Initialize();
}

/**
 * @brief Initializes the timer, setting start and last time points.
 */
void CTimerManager::Initialize()
{
	m_tStartTime = TimePoint::clock::now();
	m_tLastTime = TimePoint::clock::now();

	m_dDeltaTime = 0.0;
	m_dElapsedTime = 0.0;

	m_dDeltaTimeMilli = 0.0;
	m_dElapsedTimeMilli = 0.0;

	// FPS calculation
	m_iFrameCount = 0;
	m_dFpsTimeAccum = 0.0;
	m_dFPS = 0.0;
}

void CTimerManager::Destroy()
{
	// No dynamic resources to clean up in this implementation
}

/**
 * @brief Updates the timer, calculating delta time and elapsed time.
 * in both seconds and milliseconds.
 */
void CTimerManager::Update()
{
	m_bFpsUpdatedThisFrame = false;

	TimePoint now = Clock::now();
	m_tFrameStart = now;

	// --- DELTA TIME CALCULATION ---
	std::chrono::duration<double> dt = now - m_tLastTime;
	m_dDeltaTime = dt.count(); // Delta time in seconds

	// Calculate Delta Time in Milliseconds
	// We cast the duration to milliseconds and then use count()
	std::chrono::duration<double, std::milli> deltaTimeMilli = now - m_tLastTime;
	m_dDeltaTimeMilli = deltaTimeMilli.count(); // Delta time in milliseconds

	// --- ELAPSED TIME CALCULATION ---
	std::chrono::duration<double> elapsed = now - m_tStartTime;
	m_dElapsedTime = elapsed.count(); // Total elapsed time in seconds

	// Calculate Elapsed Time in Milliseconds
	// We cast the duration to milliseconds and then use count()
	std::chrono::duration<double, std::milli> elapsedMilli = now - m_tStartTime;
	m_dElapsedTimeMilli = elapsedMilli.count(); // Total elapsed time in milliseconds

	m_tLastTime = now;

	// FPS calculation (uses m_dDeltaTime in seconds)
	m_iFrameCount++;
	m_dFpsTimeAccum += m_dDeltaTime;

	if (m_dFpsTimeAccum >= 1.0)
	{
		m_dFPS = static_cast<double>(m_iFrameCount) / m_dFpsTimeAccum;
		m_iFrameCount = 0;
		m_dFpsTimeAccum = 0.0;
		m_bFpsUpdatedThisFrame = true;
	}
}

void CTimerManager::LimitFrameRate(int32_t iTargetFPS)
{
	if (iTargetFPS <= 0)
	{
		return; // No limit
	}

	// 1. Calculate how long the frame SHOULD take
	double targetFrameTime = 1.0 / static_cast<double>(iTargetFPS);

	// 2. Calculate how long the frame ACTUALLY took
	// m_tFrameStart should be set at the very beginning of the loop
	auto currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = currentTime - m_tFrameStart;
	double timeUsed = elapsed.count();

	// 3. If we are too fast, wait
	if (timeUsed < targetFrameTime)
	{
		double timeToWait = targetFrameTime - timeUsed;

		// --- Hybrid Sleep Strategy ---

		// 2. Hybrid Sleep (Saves CPU, maintains precision)
		if (timeToWait > 0.002) // If more than 2ms left, sleep
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>((timeToWait - 0.001) * 1000)));
		}

		// B. Spin-lock for the final micro-precision
		// This burns a tiny bit of CPU to ensure we hit the exact microsecond
		while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - m_tFrameStart).count() < targetFrameTime)
		{
			// Busy-wait .. do nothing
		}
	}
}

/**
 * @brief Accessor for delta time in seconds.
 */
double CTimerManager::GetDeltaTime() const
{
	return (m_dDeltaTime);
}

/**
 * @brief Accessor for elapsed time in seconds.
 */
double CTimerManager::GetElapsedTime() const
{
	return (m_dElapsedTime);
}

/**
 * @brief Accessor for delta time in milliseconds.
 */
double CTimerManager::GetDeltaTimeMS() const
{
	return (m_dDeltaTimeMilli);
}

/**
 * @brief Accessor for elapsed time in milliseconds.
 */
double CTimerManager::GetElapsedTimeMS() const
{
	return (m_dElapsedTimeMilli);
}

/**
 * @brief Accessor for FPS value.
 */
double CTimerManager::GetFPS() const
{
	return (m_dFPS);
}

/**
 * @brief Accessor for delta time in seconds in float type.
 */
float CTimerManager::GetDeltaTimeF() const
{
	return static_cast<float>(m_dDeltaTime);

}

/**
 * @brief Accessor for elapsed time in seconds in float type.
 */
float CTimerManager::GetElapsedTimeF() const
{
	return static_cast<float>(m_dElapsedTime);
}

/**
 * @brief Accessor for delta time in milliseconds in float type.
 */
float CTimerManager::GetDeltaTimeMSF() const
{
	return static_cast<float>(m_dDeltaTimeMilli);
}

/**
 * @brief Accessor for elapsed time in milliseconds in float type.
 */
float CTimerManager::GetElapsedTimeMSF() const
{
	return static_cast<float>(m_dElapsedTimeMilli);
}

/**
 * @brief Accessor for FPS value in float type.
 */
float CTimerManager::GetFPSF() const
{
	return static_cast<float>(m_dFPS);
}
