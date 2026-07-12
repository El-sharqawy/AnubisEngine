#pragma once

#include "CorePlatform.h"
#include <chrono>

/**
 * @brief CTimerManager: A service locator class for managing time in the engine.
 *
 * This class provides functionality to track delta time, elapsed time,
 * and frames per second (FPS) using high-resolution clocks. It is designed
 * as a service locator to ensure a single instance throughout the application.
 *
 * Key Features:
 * - Delta Time Calculation: Time between the current and last frame.
 * - Elapsed Time Calculation: Total time since the timer started.
 * - FPS Calculation: Frames rendered per second.
 *
 * Usage:
 * - Call Update() each frame to refresh time values.
 * - Access time values via GetDeltaTime(), GetElapsedTime(), and GetFPS().
 */
class CTimerManager
{
public:
	/**
	 * @brief Constructor initializes the timer manager.
	 */
	CTimerManager();

	/** 
	 * @brief Destructor.
	 */
	~CTimerManager() = default;

	/**
	 * @brief Initializes the timer, setting start and last time points.
	 */
	void Initialize();

	/**
	 * @brief Cleans up any resources used by the timer manager.
	 */
	void Destroy();

	/**
	 * @brief Updates the timer, calculating delta time and elapsed time.
	 * in both seconds and milliseconds.
	 */
	void Update();

	void LimitFrameRate(int32_t iTargetFPS);

	/**
	 * @brief Accessor for delta time in seconds.
	 */
	double GetDeltaTime() const;

	/**
	 * @brief Accessor for elapsed time in seconds.
	 */
	double GetElapsedTime() const;

	/**
	 * @brief Accessor for delta time in milliseconds.
	 */
	double GetDeltaTimeMS() const;

	/**
	 * @brief Accessor for elapsed time in milliseconds.
	 */
	double GetElapsedTimeMS() const;

	/**
	 * @brief Accessor for FPS value.
	 */
	double GetFPS() const;

	/**
	 * @brief Accessor for delta time in seconds in float type.
	 */
	float GetDeltaTimeF() const;

	/**
	 * @brief Accessor for elapsed time in seconds in float type.
	 */
	float GetElapsedTimeF() const;

	/**
	 * @brief Accessor for delta time in milliseconds in float type.
	 */
	float GetDeltaTimeMSF() const;

	/**
	 * @brief Accessor for elapsed time in milliseconds in float type.
	 */
	float GetElapsedTimeMSF() const;

	/**
	 * @brief Accessor for FPS value in float type.
	 */
	float GetFPSF() const;

	bool HasFPSUpdatedThisFrame() const { return m_bFpsUpdatedThisFrame; }

private:      
	// Using high_resolution_clock for best precision
	using Clock = std::chrono::high_resolution_clock;	// Type alias for clock
	using TimePoint = std::chrono::time_point<Clock>;	// Type alias for time point

	TimePoint m_tStartTime;			// Time when the timer started
	TimePoint m_tLastTime;			// Time at the last update

	double m_dDeltaTime;			// Delta time in seconds
	double m_dElapsedTime;		// Elapsed time in seconds
	double m_dDeltaTimeMilli;		// Delta time in milliseconds
	double m_dElapsedTimeMilli;	// Elapsed time in milliseconds

	// FPS calculation
	int32_t m_iFrameCount;			// Frame count within the current second
	double m_dFpsTimeAccum;		// Accumulated time for FPS calculation
	double m_dFPS;				// Current FPS value

	// FPS Calculation helpers
	std::chrono::high_resolution_clock::time_point m_tFrameStart;
	bool m_bFpsUpdatedThisFrame = false;
};