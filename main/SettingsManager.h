#pragma once
#include <string>
#include <type_traits>
#include <vector>
#include <utility>
#include "esp_log.h"
#include "NvsStorageManager.h"
#include "JsonWrapper.h"

class SettingsManager {
    NvsStorageManager nvs;

public:
    using ChangeList = std::vector<std::pair<std::string, std::string>>;

    SettingsManager(NvsStorageManager& nvsManager)
        : nvs(nvsManager) {
        loadSettings();
    }

    std::string tz = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    std::string ntpServer = "time.google.com";
    int frequency = 1000;       // e.g., for PWM freq
    float duty = 10.0f;         // e.g., duty cycle
    bool invert = false;        // new invert logic
    float max_speed = 10000.0f; // example

    void loadSettings() {
        std::string value;

        nvs.retrieve("tz", tz);
        nvs.retrieve("ntpServer", ntpServer);

        // optional frequency
        if (nvs.retrieve("frequency", value)) {
            frequency = std::stoi(value);
        }

        // optional duty
        if (nvs.retrieve("duty", value)) {
            duty = std::stof(value);
        }

        // optional invert
        if (nvs.retrieve("invert", value)) {
            // store true/false as "t" or "f", for example
            invert = (value == "t");
        }

        // optional max_speed
        if (nvs.retrieve("max_speed", value)) {
            max_speed = std::stof(value);
        }
    }

    std::string toJson() const {
        JsonWrapper json;
        json.AddItem("tz", tz);
        json.AddItem("ntpServer", ntpServer);
        json.AddItem("frequency", std::to_string(frequency));
        json.AddItem("duty", std::to_string(duty));
        json.AddItem("invert", invert ? "true" : "false");
        json.AddItem("max_speed", std::to_string(max_speed));
        return json.ToString();
    }

    void Store(const std::string& key, const std::string& value) {
        nvs.store(key, value);
    }

    ChangeList updateFromJson(const std::string& jsonString) {
        ChangeList changes;
        JsonWrapper json = JsonWrapper::Parse(jsonString);

        // Update each field if present
        updateFieldIfChanged(json, "tz", tz, changes);
        updateFieldIfChanged(json, "ntpServer", ntpServer, changes);
        updateFieldIfChanged(json, "frequency", frequency, changes);
        updateFieldIfChanged(json, "duty", duty, changes);
        updateFieldIfChanged(json, "max_speed", max_speed, changes);

        // specialized template for bool invert
        updateFieldIfChanged(json, "invert", invert, changes);

        // Now store any changes that occurred
        for (const auto& [key, val] : changes) {
            nvs.store(key, val);
        }

        return changes;
    }

    // Helper that returns a JSON representation of the changes
    std::string convertChangesToJson(const SettingsManager::ChangeList& changes) {
        cJSON *root = cJSON_CreateObject();
        for (const auto& [key, value] : changes) {
            cJSON_AddStringToObject(root, key.c_str(), value.c_str());
        }
        char* rawJson = cJSON_Print(root);
        std::string jsonResponse(rawJson);
        cJSON_Delete(root);
        free(rawJson);
        return jsonResponse;
    }

private:
    template <typename T>
    void updateFieldIfChanged(JsonWrapper& json, const std::string& key, T& field, ChangeList& changes) {
        // Check if the JSON has this field
        if (json.ContainsField(key)) {
            T newValue;
            if (json.GetField(key, newValue)) {
                // If the value is different, update it
                if (newValue != field) {
                    field = newValue;
                    if constexpr (std::is_same_v<T, std::string>) {
                        changes.emplace_back(key, field);
                    } else {
                        changes.emplace_back(key, std::to_string(field));
                    }
                }
            } else {
                ESP_LOGE("SettingsManager", "Failed to retrieve new value for %s", key.c_str());
            }
        }
    }

    // Specialized version for bool fields
    void updateFieldIfChanged(JsonWrapper& json, const std::string& key, bool& field, ChangeList& changes) {
        if (json.ContainsField(key)) {
            std::string boolStr;
            if (json.GetField(key, boolStr)) {
                bool newValue = (boolStr == "true");
                if (newValue != field) {
                    field = newValue;
                    // Store 't' or 'f' in NVS, for example
                    changes.emplace_back(key, field ? "t" : "f");
                }
            } else {
                ESP_LOGE("SettingsManager", "Failed to retrieve new bool for %s", key.c_str());
            }
        }
    }
};

