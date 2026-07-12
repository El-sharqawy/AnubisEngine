#include "Stdafx.h"
#include "Logging/LogManager.h"

/**
 * @brief Initializes the log manager by opening the log files.
 *
 * @param logFilePath Path to the general log file.
 * @param errorLogFilePath Path to the error log file.
 */
void CLogManager::Initialize(const std::string& logFilePath, const std::string& errorLogFilePath)
{
    // Creates "Assets/Logs/" if it doesn't exist — no-op if it does
    if (!std::filesystem::exists(logFilePath))
    {
        std::filesystem::create_directories(std::filesystem::path(logFilePath).parent_path());  // Creates parent directories too
    }

    if (!std::filesystem::exists(errorLogFilePath))
    {
        std::filesystem::create_directories(std::filesystem::path(errorLogFilePath).parent_path());  // Creates parent directories too
    }

    // std::ofstream handles the file opening and closing.
    // std::ios::out is the default for ofstream, but using it explicitly is clear.
    m_gSysLog.open(LOG_PATH, std::ios::out);
    m_gSysErr.open(ERROR_LOG_PATH, std::ios::out);
}

/**
 * @brief Closes the log files and cleans up resources.
 */
void CLogManager::Destroy()
{
    if (m_gSysLog.is_open())
    {
        m_gSysLog.flush();
        m_gSysLog.close();
    }

    if (m_gSysErr.is_open())
    {
        m_gSysErr.flush();
        m_gSysErr.close();
    }
}

/**
 * @brief Writes a formatted message along with a timestamp to the log file.
 *
 * @param msg The message string to write.
 */
void CLogManager::Write(const std::string& msg)
{
    if (!m_gSysLog.is_open())
    {
        // Optional: Log an error to stderr if the file isn't open
        std::cerr << "Error: Log file not open!" << std::endl;
        return;
    }

    // Lock to ensure thread safety during the write operation
    std::lock_guard<std::mutex> lock(m_writeMutex);

    // 1. Get current time (using std::chrono)
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm localTimeStruct; // Buffer is now on the stack (thread-local)

    // 2. Build the formatted string once
    std::stringstream ss;

    // 2. Format the date/time string
    // Using std::put_time for modern, safer formatting
#if defined(_WIN32) || defined(_WIN64)
    if (localtime_s(&localTimeStruct, &now) == 0)
#else
    if (std::tm* timePtr = std::localtime(&now))
#endif
    {
        ss << std::put_time(&localTimeStruct, "SYSLOG: %H:%M:%S") << " :: " << msg;
    }
    else
    {
        ss << "TimeError :: " << msg;
    }

    // 3. Output to BOTH destinations:

    // Destination A: Write to the external log file
    m_gSysLog << ss.str() << std::endl;

    // Destination B: Write to the console/debug output
    std::cout << ss.str() << std::endl; // Use std::cout or std::clog

    // Note: std::endl automatically flushes the stream, replacing fflush().
}

/**
 * @brief Writes a formatted error message to the error log file with context.
 *
 * @param fileName The name of the source file where the error occurred.
 * @param funcName The name of the function where the error occurred.
 * @param lineNum The line number in the source file where the error occurred.
 * @param c_szFormat The format string for the error message.
 * @param ... Additional arguments for the format string.
 */
void CLogManager::SysErrorC(const char* fileName, const char* funcName, int lineNum, const char* c_szFormat, ...)
{
    // --- 1. Thread Safety and File Check ---
    if (!m_gSysErr.is_open())
    {
        std::cerr << "Error: Log file not open!" << std::endl;
        return;
    }

    // --- 2. Determine required buffer size (a common C++ trick to avoid fixed buffers) ---
    va_list args;
    va_start(args, c_szFormat);
    // We try to estimate the required size for the formatted message
    int32_t message_len = std::vsnprintf(nullptr, 0, c_szFormat, args);
    va_end(args);

    if (message_len < 0)
    {
        // Handle error in format string
        return;
    }

    const int32_t total_len = message_len + 1; // +1 for null terminator
    // Use a dynamically sized buffer (modern C++ prefers std::vector<char>)
    std::vector<char> szBuf(total_len);

    // Format the user's message into the buffer after the prefix
    va_start(args, c_szFormat);
    std::vsnprintf(szBuf.data(), message_len + 1, c_szFormat, args);
    va_end(args);

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
        << szBuf.data();

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
void CLogManager::SysLogC(const char* fileName, const char* funcName, int lineNum, const char* c_szFormat, ...)
{
    // --- 1. Thread Safety and File Check ---
    if (!m_gSysLog.is_open())
    {
        std::cerr << "Error: Log file not open!" << std::endl;
        return;
    }

    // --- 2. Determine required buffer size (a common C++ trick to avoid fixed buffers) ---
    va_list args;
    va_start(args, c_szFormat);
    // We try to estimate the required size for the formatted message
    int32_t message_len = std::vsnprintf(nullptr, 0, c_szFormat, args);
    va_end(args);

    if (message_len < 0)
    {
        // Handle error in format string
        return;
    }

    const int32_t total_len = message_len + 1; // +1 for null terminator
    // Use a dynamically sized buffer (modern C++ prefers std::vector<char>)
    std::vector<char> szBuf(total_len);

    // Format the user's message into the buffer after the prefix
    va_start(args, c_szFormat);
    std::vsnprintf(szBuf.data(), message_len + 1, c_szFormat, args);
    va_end(args);

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
        << szBuf.data();

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
void CLogManager::Log(const char* c_szFormat, ...)
{
    // --- 1. Thread Safety and File Check ---
    if (!m_gSysLog.is_open())
    {
        std::cerr << "Error: Log file not open!" << std::endl;
        return;
    }

    // --- 2. Determine required buffer size (a common C++ trick to avoid fixed buffers) ---
    va_list args;
    va_start(args, c_szFormat);
    // We try to estimate the required size for the formatted message
    int32_t message_len = std::vsnprintf(nullptr, 0, c_szFormat, args);
    va_end(args);

    if (message_len < 0)
    {
        // Handle error in format string
        return;
    }

    const int32_t total_len = message_len + 1; // +1 for null terminator
    // Use a dynamically sized buffer (modern C++ prefers std::vector<char>)
    std::vector<char> szBuf(total_len);

    // Format the user's message into the buffer after the prefix
    va_start(args, c_szFormat);
    std::vsnprintf(szBuf.data(), message_len + 1, c_szFormat, args);
    va_end(args);

    // --- 3. Time Setup and Thread-Safe Population ---
    std::lock_guard<std::mutex> lock(m_writeMutex);

    // --- 4. Build Output String with Function/Line Info ---
    std::stringstream ss;

    // Output Format: SYSLOG: %m%d %H:%M:%S%ms [func:line] :: message
    ss << szBuf.data();

    // --- 5. Write to Both Destinations ---
    std::string final_log_entry = ss.str();

    // Destination A: Write to the external log file
    m_gSysLog << final_log_entry << std::endl; // Use flush for immediate disk write

    // Destination B: Write to the console/debug output
    std::cout << final_log_entry << std::endl;
}
