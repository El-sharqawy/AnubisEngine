#include "Stdafx.h"
#include "Config/ConfigManager.h"
#include "Logging/LogManager.h"
/**
 * @brief Loads and parses the configuration file, populating the internal data structure.
 * @param configFileName The path to the configuration file (e.g., "config.ini" or "settings.json").
 * @return true if loading and parsing succeeded, false otherwise.
 */
bool CConfigManager::Initialize(const std::string& configFileName)
{
	// Creates "Assets/Config/" if it doesn't exist — no-op if it does
	if (!std::filesystem::exists(configFileName))
	{
		std::filesystem::create_directories(std::filesystem::path(configFileName).parent_path());  // Creates parent directories too
	}

	std::fstream fileStream(configFileName, std::ios::in);
	if (!fileStream.is_open())
	{
		syserr("Failed to Load default engine configuration file: {}", configFileName);
		return (false);
	}

	try
	{
		nlohmann::json jsonData;
		try
		{
			fileStream >> jsonData;
		}
		catch (const nlohmann::json::parse_error& e)
		{
			syserr("JSON parse error for File {}, error: {}", configFileName, e.what());
			return (false);
		}

		// --- Validation Checks ---
		if (!jsonData.contains("window") || !jsonData["window"].is_array() || jsonData["window"].empty())
		{
			syserr("JSON parse error for File {}, error: 'window' section missing or invalid format.", configFileName);
			return (false);
		}

		// 1. Map 'window' section (which is an array, we access the first element [0])
		const auto& windowObject = jsonData["window"][0];

		// Must convert all types (int, float, bool) to std::string before storing in ConfigMap
		try
		{
			m_gConfigData["window"]["title"] = windowObject["title"].get<std::string>();

			// Convert integer/number types to string for storage in ConfigMap
			m_gConfigData["window"]["width"] = std::to_string(windowObject["width"].get<int32_t>());
			m_gConfigData["window"]["height"] = std::to_string(windowObject["height"].get<int32_t>());
			m_gConfigData["window"]["fullscreen"] = windowObject["fullscreen"].get<bool>() ? "True" : "False";
			m_gConfigData["window"]["config_resolution"] = windowObject["config_resolution"].get<bool>() ? "True" : "False";
		}
		catch (const std::exception& e)
		{
			syserr("JSON data conversion error in 'window' section: {}", e.what());
			return (false);
		}


		if (!jsonData.contains("graphics") || !jsonData["graphics"].is_array() || jsonData["graphics"].empty())
		{
			syserr("JSON parse error for File {}, error: 'window' section missing or invalid format.", configFileName.c_str());
			return (false);
		}

		// 2. Map 'graphics' section (assuming it's a standard object)
		const auto& graphicsObject = jsonData["graphics"][0];
		try
		{
			m_gConfigData["graphics"]["fov"] = std::to_string(graphicsObject["fov"].get<float>());
			m_gConfigData["graphics"]["target_framerate"] = std::to_string(graphicsObject["target_framerate"].get<int32_t>());
			m_gConfigData["graphics"]["limit_framerate"] = graphicsObject["limit_framerate"].get<bool>() ? "True" : "False";
		}
		catch (const std::exception& e)
		{
			syserr("JSON data conversion error in 'graphics' section: {}", e.what());
			// Continue parsing other sections if this one fails (optional, based on tolerance)
		}

#if defined(ENABLE_DEBUG_LOGS)
		syslog("Config file loaded successfully.");
#endif
		return (true);
	}
	catch (nlohmann::json::parse_error& e)
	{
		syserr("JSON Parse Error: {}", e.what());
		return (false);
	}
	catch (nlohmann::json::type_error& e)
	{
		syserr("JSON Type Error (Wrong data format): {}", e.what());
		return (false);
	}

	return (false);
}

/**
 * @brief Saves the current config data and Cleans up all resources used by the configuration manager.
 */
void CConfigManager::Destroy()
{
	SaveConfig();
	Clear();
}

/**
 * @brief Clears all loaded configuration data from memory.
 */
void CConfigManager::Clear()
{
	m_gConfigData.clear();
	m_stringCache.clear();
	m_intCache.clear();
	m_floatCache.clear();
	m_boolCache.clear();
}

/**
 * @brief Clears all cached parsed values.
 */
void CConfigManager::ClearCache()
{
	m_stringCache.clear();
	m_intCache.clear();
	m_floatCache.clear();
	m_boolCache.clear();
}

/**
 * @brief Writes the current in-memory configuration data back to a file.
 * @param filename The path to save the configuration file to.
 * @return true if saving succeeded, false otherwise.
 */
bool CConfigManager::SaveConfig(const std::string& fileName) const
{
	std::ofstream file(fileName);
	if (!file.is_open())
	{
		syserr("Failed to open file for writing config: {}", fileName);
		return (false);
	}

	nlohmann::json jsonConfigRoot = nlohmann::json::object();
	// --- 1. Handle 'window' section (Array structure) ---

	jsonConfigRoot["window"] = SerializeWindowSettings();

	// --- 2. Handle 'graphics' section (Object structure) ---
	jsonConfigRoot["graphics"] = SerializeGraphicsSettings();

	// --- 3. Write to file ---
	// Use std::setw(4) to format the JSON output with 4 spaces indentation for readability
	file << std::setw(2) << jsonConfigRoot << std::endl;

	file.close();
	syslog("Config file saved successfully to {}", fileName);
	return (true);
}

/**
 * @brief Retrieves a raw string value safely
 *
 * @param section The section / group name(e.g., "Renderer").
 * @param key The key name(e.g., "Resolution").
 * @param defaultValue The value to return if the section or key is not found.
 * @return The configuration value as a string.
 */
std::string CConfigManager::GetRawValue(const std::string& section, const std::string& key) const
{
	auto itSection = m_gConfigData.find(section);
	if (itSection == m_gConfigData.end())
	{
		syserr("Failed to find the key section {}", section);
		return ""; // Section not found
	}

	const SettingMap& sectionMap = itSection->second;
	auto keyIt = sectionMap.find(key);
	if (keyIt == sectionMap.end())
	{
		syserr("Failed to find the key {}", key);
		return ""; // Key not found
	}

	return keyIt->second;
}

/**
 * @brief Retrieves a string value from the configuration data.
 *
 * @param section The section/group name (e.g., "Renderer").
 * @param key The key name (e.g., "Resolution").
 * @param defaultValue The value to return if the section or key is not found.
 * @return The configuration value as a string.
 */
const std::string& CConfigManager::GetString(const std::string& section, const std::string& key, const std::string& defaultValue) const
{
	// 1. Create a unique cache key (e.g., "graphics.shadows")
	std::string cacheKey = section + "." + key;

	// 2. Check if we've already parsed this
	auto it = m_stringCache.find(cacheKey);
	if (it != m_stringCache.end())
	{
		return it->second;
	}

	// 3. Not in cache, do the "expensive" work once
	std::string rawValue = GetRawValue(section, key);
	if (!rawValue.empty())
	{
		m_stringCache[cacheKey] = rawValue;
	}
	else
	{
		m_stringCache[cacheKey] = defaultValue;
	}

	// 4. Return the reference to the string inside the map
	return m_stringCache[cacheKey];
}

/**
 * @brief Retrieves an integer value from the configuration data, converting the string.
 *
 * @param section The section/group name (e.g., "Audio").
 * @param key The key name (e.g., "VolumeLevel").
 * @param defaultValue The value to return if the key is not found or conversion fails.
 * @return The configuration value as an integer.
 */
int32_t CConfigManager::GetInteger(const std::string& section, const std::string& key, int32_t defaultValue) const
{
	// 1. Create a unique cache key (e.g., "graphics.shadows")
	std::string cacheKey = section + "." + key;

	// 2. Check if we've already parsed this
	auto it = m_intCache.find(cacheKey);
	if (it != m_intCache.end())
	{
		return it->second;
	}

	// 3. Not in cache, do the "expensive" work once
	std::string rawValue = GetRawValue(section, key);
	int32_t iValue = defaultValue;
	if (!rawValue.empty())
	{
		auto [ptr, ec] = std::from_chars(rawValue.data(), rawValue.data() + rawValue.size(), iValue);

		if (ec != std::errc())
		{
			iValue = defaultValue;
			syserr("Error: Failed to convert key: {} (Value: {}) to Integer. Returning default", key.c_str(), rawValue.c_str());
		}
	}

	// 4. Store in cache for next time
	m_intCache[cacheKey] = iValue;
	return (iValue);
}

/**
 * @brief Retrieves a floating-point value from the configuration data, converting the string.
 *
 * @param section The section/group name (e.g., "Physics").
 * @param key The key name (e.g., "Gravity").
 * @param defaultValue The value to return if the key is not found or conversion fails.
 * @return The configuration value as a float.
 */
float CConfigManager::GetFloat(const std::string& section, const std::string& key, float defaultValue) const
{
	// 1. Create a unique cache key (e.g., "graphics.shadows")
	std::string cacheKey = section + "." + key;

	// 2. Check if we've already parsed this
	auto it = m_floatCache.find(cacheKey);
	if (it != m_floatCache.end())
	{
		return it->second;
	}

	// 3. Not in cache, do the "expensive" work once
	std::string rawValue = GetRawValue(section, key);
	float fValue = defaultValue;

	if (!rawValue.empty())
	{
		auto [ptr, ec] = std::from_chars(rawValue.data(), rawValue.data() + rawValue.size(), fValue);

		if (ec != std::errc())
		{
			syserr("Error: Failed to convert key: {} (Value: {}) to Integer. Returning default", key.c_str(), rawValue.c_str());
			fValue = defaultValue;
		}
	}

	// 4. Store in cache for next time
	m_floatCache[cacheKey] = fValue;
	return (fValue);
}

/**
 * @brief Retrieves a boolean value from the configuration data, based on common string values ("true", "1", "false", "0").
 *
 * @param section The section/group name (e.g., "Debug").
 * @param key The key name (e.g., "DrawWireframe").
 * @param defaultValue The value to return if the key is not found or conversion fails.
 * @return The configuration value as a boolean.
 */
bool CConfigManager::GetBoolean(const std::string& section, const std::string& key, bool defaultValue) const
{
	// 1. Create a unique cache key (e.g., "graphics.shadows")
	std::string cacheKey = section + "." + key;

	// 2. Check if we've already parsed this
	auto it = m_boolCache.find(cacheKey);
	if (it != m_boolCache.end())
	{
		return it->second;
	}

	// 3. Not in cache, do the "expensive" work once
	std::string rawValue = GetRawValue(section, key);

	bool result = defaultValue;
	if (!rawValue.empty())
	{
		// Inline lowercase check without full transform for speed
		if (rawValue[0] == '1' || rawValue[0] == 't' || rawValue[0] == 'T')
		{
			result = true;
		}
		else if (rawValue[0] == '0' || rawValue[0] == 'f' || rawValue[0] == 'F')
		{
			result = false;
		}
		return (result);
	}

	// 4. Store in cache for next time
	m_boolCache[cacheKey] = result;
	return result;
}

/**
 * @brief Sets or updates a configuration value in the internal data structure.
 *
 * @param section The section/group name (e.g., "Renderer").
 * @param key The key name (e.g., "Resolution").
 * @param value The value to set for the specified key.
 */
void CConfigManager::SetValue(const std::string& section, const std::string& key, const std::string& value)
{
	m_gConfigData[section][key] = value;
}

/**
 * @brief Serializes the 'window' settings from the internal data structure into a JSON object.
 *
 * @return A nlohmann::json object representing the 'window' settings.
 */
nlohmann::json CConfigManager::SerializeWindowSettings() const
{
	// 1. Retrieve values from ConfigMap
	std::string titleStr = GetRawValue("window", "title");
	int32_t width = GetInteger("window", "width");
	int32_t height = GetInteger("window", "height");
	bool full_Screen = GetBoolean("window", "fullscreen");
	bool config_Resolution = GetBoolean("window", "config_resolution");

	// 2. Apply Defaults if values are missing/invalid
	if (titleStr.empty())
	{
		titleStr = "Anubis-Engine";
	}
	if (width == 0)
	{
		width = 1280;
	}
	if (height == 0)
	{
		height = 720;
	}

	// 3. Construct the JSON object
	nlohmann::json windowEntry{};
	windowEntry["title"] = titleStr;
	windowEntry["width"] = width;
	windowEntry["height"] = height;
	windowEntry["fullscreen"] = full_Screen; // Load Fullscreen from config file
	windowEntry["config_resolution"] = config_Resolution; // Load Resolution from config file

	// 4. Return as an array (matches our JSON structure)
	return nlohmann::json::array({ windowEntry });
}

/**
 * @brief Serializes the 'graphics' settings from the internal data structure into a JSON object.
 *
 * @return A nlohmann::json object representing the 'graphics' settings.
 */
nlohmann::json CConfigManager::SerializeGraphicsSettings() const
{
	// 1. Retrieve values from ConfigMap
	float fCameraFov = GetFloat("graphics", "fov");
	int32_t iMaxFPS = GetInteger("graphics", "target_framerate");
	bool bLimitFrameRate = GetBoolean("graphics", "limit_framerate");
	
	// 2. Apply Defaults if values are missing/invalid
	if (fCameraFov == 0.0f)
	{
		fCameraFov = 45.0f;
	}
	if (iMaxFPS == 0)
	{
		iMaxFPS = 240;
	}

	// 3. Construct the JSON object
	nlohmann::json graphicsEntry{};
	graphicsEntry["fov"] = fCameraFov;
	graphicsEntry["target_framerate"] = iMaxFPS;
	graphicsEntry["limit_framerate"] = bLimitFrameRate;

	// 4. Return as an array (matches our JSON structure)
	return nlohmann::json::array({ graphicsEntry });
}

/**
 * @brief Reloads the configuration data from the file, discarding any unsaved changes.
 */
void CConfigManager::Reload()
{
	// Clear the internal typed caches so fresh data is parsed
	m_boolCache.clear();
	m_intCache.clear();
	m_stringCache.clear();

	// Wipe the main data map and reload from disk
	m_gConfigData.clear();
	Initialize(CONFIG_PATH);
}