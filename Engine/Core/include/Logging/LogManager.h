#pragma once

#include "Utils/CoreUtils.h"
#include "CoreDefines.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <format>
#include <iomanip>
#include <string>
#include <chrono>
#include <mutex>

class CLogManager
{
public:
	/**
	 * @brief Constructor: Opens the log file immediately (RAII).
	 * * The file handle is managed by std::ofstream.
	 */
	CLogManager() = default;

	/**
	 * @brief Destructor: Automatically closes the log file (RAII).
	 * * The file will automatically close when m_file is destroyed.
	 */
	~CLogManager() = default;

	/**
	 * @brief Initializes the log manager by opening the log files.
	 *
	 * @param logFilePath Path to the general log file.
	 * @param errorLogFilePath Path to the error log file.
	 */
	void Initialize(const std::string& logFilePath = LOG_PATH, const std::string& errorLogFilePath = ERROR_LOG_PATH);

	/**
	 * @brief Closes the log files and cleans up resources.
	 */
	void Destroy();

	/**
	 * @brief Writes a formatted message along with a timestamp to the log file.
	 *
	 * @param msg The message string to write.
	 */
	void Write(const std::string& msg);

	/**
	 * @brief Writes a formatted error message to the error log file with context.
	 *
	 * @param fileName The name of the source file where the error occurred.
	 * @param funcName The name of the function where the error occurred.
	 * @param lineNum The line number in the source file where the error occurred.
	 * @param c_szFormat The format string for the error message.
	 * @param ... Additional arguments for the format string.
	 */
	void SysErrorC(const char* fileName, const char* funcName, int lineNum, const char* c_szFormat, ...);

	/**
	 * @brief Writes a formatted error message to the error log file with context.
	 *
	 * @param fileName The name of the source file where the error occurred.
	 * @param funcName The name of the function where the error occurred.
	 * @param lineNum The line number in the source file where the error occurred.
	 * @param szFormat The format string for the error message.
	 * @param ... Additional arguments for the format string.
	 */
	template <typename... Args>
	void SysError(const char* fileName, const char* funcName, int lineNum, const std::string& szFormat, Args... args)
	{
		// --- 1. Thread Safety and File Check ---
		if (!m_gSysErr.is_open())
		{
			std::cerr << "Error: Log file not open!" << std::endl;
			return;
		}

		// --- 2. Format the message using std::format ---
		std::string formatted;
		try
		{
			formatted = std::vformat(szFormat, std::make_format_args(args...));
		}
		catch (const std::exception& e)
		{
			formatted = "SYS_ERR_ERROR (Invalid Format): " + std::string(e.what());
		}

		// --- 3. Time Setup and Thread-Safe Population ---
		std::lock_guard<std::mutex> lock(m_writeMutex);

		auto now_tp = std::chrono::system_clock::now();
		auto now_time = std::chrono::system_clock::to_time_t(now_tp);
		std::tm localTimeStruct{}; // Initialized to zero

		// Use platform-specific thread-safe time function to POPULATE localTimeStruct
#if defined(_WIN32) || defined(_WIN64)
		if (localtime_s(&localTimeStruct, &now_time) != 0)
		{
			return; // Check for failure
		}
#elif defined(__GNUC__) || defined(__clang__) // POSIX systems
		if (localtime_r(&now_time, &localTimeStruct) == nullptr)
		{
			return; // Check for failure
		}
#else
	// Fallback if no thread-safe function is known
		localTimeStruct = *std::localtime(&now_time);
#endif

		// Calculate milliseconds
		// Note: This calculates MS relative to epoch, not within the current minute/second
		int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp.time_since_epoch()).count();
		ms = ms % 60000; // Time within the current minute, in milliseconds

		// --- 4. Build Output String with Function/Line Info ---
		std::stringstream ss;

		std::string stFileName = Anubis::GetFileNameModern(fileName);
		// Output Format: SYSERR: %m%d %H:%M:%S%ms [func:line] :: message
		ss << "SYSERR: "
			// << std::put_time(&localTimeStruct, "%m%d %H:%M");
			<< std::put_time(&localTimeStruct, "%H:%M:")
			<< std::setw(5) << std::setfill('0') << ms
			// << " [" << stFileName << ":" << funcName << ":" << lineNum << "] :: "
			<< " [" << funcName << "]:: "
			<< formatted;

		// --- 5. Write to Both Destinations ---
		std::string final_log_entry = ss.str();

		// Destination A: Write to the external log file
		m_gSysErr << final_log_entry << std::endl; // Use flush for immediate disk write

		// Destination B: Write to the console/debug output
		std::cerr << final_log_entry << std::endl;
	}

	/**
	 * @brief Writes a formatted log message to the log file with context.
	 *
	 * @param fileName The name of the source file where the log is generated.
	 * @param funcName The name of the function where the log is generated.
	 * @param lineNum The line number in the source file where the log is generated.
	 * @param c_szFormat The format string for the log message.
	 * @param ... Additional arguments for the format string.
	 */
	void SysLogC(const char* fileName, const char* funcName, int lineNum, const char* c_szFormat, ...);

	/**
	 * @brief Writes a formatted log message to the log file with context.
	 *
	 * @param fileName The name of the source file where the log is generated.
	 * @param funcName The name of the function where the log is generated.
	 * @param lineNum The line number in the source file where the log is generated.
	 * @param szFormat The format string for the log message.
	 * @param ... Additional arguments for the format string.
	 */
	template <typename... Args>
	void SysLog(const char* fileName, const char* funcName, int lineNum, const std::string& szFormat, Args... args)
	{
		// --- 1. Thread Safety and File Check ---
		if (!m_gSysLog.is_open())
		{
			std::cerr << "Error: Log file not open!" << std::endl;
			return;
		}

		// --- 2. Format the message using std::format ---
		std::string formatted;
		try
		{
			formatted = std::vformat(szFormat, std::make_format_args(args...));
		}
		catch (const std::exception& e)
		{
			formatted = "SYS_LOG_ERROR (Invalid Format): " + std::string(e.what());
		}

		// --- 3. Time Setup and Thread-Safe Population ---
		std::lock_guard<std::mutex> lock(m_writeMutex);

		auto now_tp = std::chrono::system_clock::now();
		auto now_time = std::chrono::system_clock::to_time_t(now_tp);
		std::tm localTimeStruct{}; // Initialized to zero

		// Use platform-specific thread-safe time function to POPULATE localTimeStruct
#if defined(_WIN32) || defined(_WIN64)
		if (localtime_s(&localTimeStruct, &now_time) != 0)
		{
			return; // Check for failure
		}
#elif defined(__GNUC__) || defined(__clang__) // POSIX systems
		if (localtime_r(&now_time, &localTimeStruct) == nullptr)
		{
			return; // Check for failure
		}
#else
	// Fallback if no thread-safe function is known
		localTimeStruct = *std::localtime(&now_time);
#endif

		// Calculate milliseconds
		// Note: This calculates MS relative to epoch, not within the current minute/second
		int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp.time_since_epoch()).count();
		ms = ms % 60000; // Time within the current minute, in milliseconds

		// --- 4. Build Output String with Function/Line Info ---
		std::stringstream ss;

		std::string stFileName = Anubis::GetFileNameModern(fileName);
		// Output Format: SYSLOG: %m%d %H:%M:%S%ms [func:line] :: message
		ss << "SYSLOG: "
			// << std::put_time(&localTimeStruct, "%m%d %H:%M");
			<< std::put_time(&localTimeStruct, "%H:%M:")
			<< std::setw(5) << std::setfill('0') << ms
			// << " [" << stFileName << ":" << funcName << ":" << lineNum << "] :: "
			<< " [" << funcName << "]:: "
			<< formatted;

		// --- 5. Write to Both Destinations ---
		std::string final_log_entry = ss.str();

		// Destination A: Write to the external log file
		m_gSysLog << final_log_entry << std::endl; // Use flush for immediate disk write

		// Destination B: Write to the console/debug output
		std::cout << final_log_entry << std::endl;
	}

	/**
	 * @brief Writes a formatted log message to the log file.
	 *
	 * @param c_szFormat The format string for the log message.
	 * @param ... Additional arguments for the format string.
	 */
	void Log(const char* c_szFormat, ...);

	template<typename... Args>
	void SysWarn(const char* file, const char* func, int line,
		std::string_view fmt, Args&&... args)
	{
		const std::string msg = std::vformat(fmt, std::make_format_args(args...));
		// Yellow color code — distinguishable from red error and white info
		std::fprintf(stdout, "\033[33m[WARN]  %s\n  >> %s (%s:%d)\033[0m\n", msg.c_str(), func, file, line);
	}

private:
	// --- Members ---
	std::ofstream m_gSysLog;
	std::ofstream m_gSysErr;
	std::mutex m_writeMutex; // Mutex for thread-safe writing
};

#if defined(_WIN32) || defined(_WIN64)
#define syserrc(fmt, ...) CServiceLocator::Get<CLogManager>().SysErrorC(__FILE__,__FUNCTION__, __LINE__, fmt, __VA_ARGS__)
#define syslogc(fmt, ...) CServiceLocator::Get<CLogManager>().SysLogC(__FILE__,__FUNCTION__, __LINE__, fmt, __VA_ARGS__)
#define syserr(fmt, ...) CServiceLocator::Get<CLogManager>().SysError(__FILE__,__FUNCTION__, __LINE__, fmt, __VA_ARGS__)
#define syslog(fmt, ...) CServiceLocator::Get<CLogManager>().SysLog(__FILE__,__FUNCTION__, __LINE__, fmt, __VA_ARGS__)
#define syswarn(fmt, ...) CServiceLocator::Get<CLogManager>().SysWarn  (__FILE__, __FUNCTION__, __LINE__, fmt, __VA_ARGS__)
#define WriteLog(fmt, ...) CServiceLocator::Get<CLogManager>().Log(fmt, __VA_ARGS__)
#else
#define syserrc(fmt, args...) CServiceLocator::Get<CLogManager>().SysErrorC(__FILE__,__FUNCTION__, __LINE__, fmt, ##args)
#define syslogc(fmt, args...) CServiceLocator::Get<CLogManager>().SysLogC(__FILE__,__FUNCTION__, __LINE__, fmt, ##args)
#define syserr(fmt, args...) CServiceLocator::Get<CLogManager>().SysError(__FILE__,__FUNCTION__, __LINE__, fmt, ##args)
#define syslog(fmt, args...) CServiceLocator::Get<CLogManager>().SysLog(__FILE__,__FUNCTION__, __LINE__, fmt, ##args)
#define syswarn(fmt, args...) CServiceLocator::Get<CLogManager>().SysWarn  (__FILE__, __FUNCTION__, __LINE__, fmt, ##args)
#define log(fmt, args...) CServiceLocator::Get<CLogManager>().Log(fmt, ##args)
#endif	// __WIN32__
