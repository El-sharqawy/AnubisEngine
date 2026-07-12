#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "CoreDefines.h"

class CConfigManager
{
public:
	/**
	 * @brief Constructor: Initializes the configuration manager.
	 */
	CConfigManager() = default;

	/**
	 * @brief Destructor: Cleans up resources.
	 */
	~CConfigManager() = default;

	/**
	 * @brief Writes the current in-memory configuration data back to a file.
	 * @param filename The path to save the configuration file to.
	 * @return true if saving succeeded, false otherwise.
	 */
	bool Initialize(const std::string& fileName = CONFIG_PATH);

	/**
	 * @brief Saves the current config data and Cleans up all resources used by the configuration manager.
	 */
	void Destroy();

	/**
	 * @brief Clears all loaded configuration data from memory.
	 */
	void Clear();

	/**
	 * @brief Clears all cached parsed values.
	 */
	void ClearCache();

	/**
	 * @brief Writes the current in-memory configuration data back to a file.
	 * @param filename The path to save the configuration file to.
	 * @return true if saving succeeded, false otherwise.
	 */
	bool SaveConfig(const std::string& fileName = CONFIG_PATH) const;

	// Typed Access Interface

	/**
	 * @brief Retrieves a raw string value safely
	 *
	 * @param section The section / group name(e.g., "Renderer").
	 * @param key The key name(e.g., "Resolution").
	 * @param defaultValue The value to return if the section or key is not found.
	 * @return The configuration value as a string.
	 */
	std::string GetRawValue(const std::string& section, const std::string& key) const;

	/**
	 * @brief Retrieves a string value from the configuration data.
	 *
	 * @param section The section/group name (e.g., "Renderer").
	 * @param key The key name (e.g., "Resolution").
	 * @param defaultValue The value to return if the section or key is not found.
	 * @return The configuration value as a string.
	 */
	const std::string& GetString(const std::string& section, const std::string& key, const std::string& defaultValue = "") const;

	/**
	 * @brief Retrieves an integer value from the configuration data, converting the string.
	 *
	 * @param section The section/group name (e.g., "Audio").
	 * @param key The key name (e.g., "VolumeLevel").
	 * @param defaultValue The value to return if the key is not found or conversion fails.
	 * @return The configuration value as an integer.
	 */
	int32_t GetInteger(const std::string& section, const std::string& key, int32_t defaultValue = 0) const;

	/**
	 * @brief Retrieves a floating-point value from the configuration data, converting the string.
	 *
	 * @param section The section/group name (e.g., "Physics").
	 * @param key The key name (e.g., "Gravity").
	 * @param defaultValue The value to return if the key is not found or conversion fails.
	 * @return The configuration value as a float.
	 */
	float GetFloat(const std::string& section, const std::string& key, float defaultValue = 0.0f) const;

	/**
	 * @brief Retrieves a boolean value from the configuration data, based on common string values ("true", "1", "false", "0").
	 *
	 * @param section The section/group name (e.g., "Debug").
	 * @param key The key name (e.g., "DrawWireframe").
	 * @param defaultValue The value to return if the key is not found or conversion fails.
	 * @return The configuration value as a boolean.
	 */
	bool GetBoolean(const std::string& section, const std::string& key, bool defaultValue = false) const;

	/**
	 * @brief Sets or updates a configuration value in memory.
	 *
	 * @param section The section/group name (e.g., "Audio").
	 * @param key The key name (e.g., "VolumeLevel").
	 * @param value The string value to set.
	 */
	void SetValue(const std::string& section, const std::string& key, const std::string& value);

	/**
	 * @brief Serializes the 'window' settings from the internal data structure into a JSON object.
	 *
	 * @return A nlohmann::json object representing the 'window' settings.
	 */
	nlohmann::json SerializeWindowSettings() const;

	/**
	 * @brief Serializes the 'graphics' settings from the internal data structure into a JSON object.
	 *
	 * @return A nlohmann::json object representing the 'graphics' settings.
	 */
	nlohmann::json SerializeGraphicsSettings() const;

	/**
	 * @brief Reloads the configuration data from the file, discarding any unsaved changes.
	 */
	void Reload();

private:
	// Alias for readability
	// Using std::string for keys and values ensures maximum flexibility.
	using SettingMap = std::unordered_map<std::string, std::string>;
	using ConfigMap = std::unordered_map<std::string, SettingMap>;

	// Internal storage
	ConfigMap m_gConfigData;

	// Caches for parsed values to avoid repeated conversions
	mutable std::unordered_map<std::string, std::string> m_stringCache;
	mutable std::unordered_map<std::string, int32_t> m_intCache;
	mutable std::unordered_map<std::string, float> m_floatCache;
	mutable std::unordered_map<std::string, bool> m_boolCache;
};