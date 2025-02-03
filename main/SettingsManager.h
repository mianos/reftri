#pragma once
#include <string>
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
    float duty = 10.0f;
    bool invert = false;
    float max_speed = 10000.0f;

    void loadSettings() {
        std::string storedValue;
        nvs.retrieve("tz", tz);
        nvs.retrieve("ntpServer", ntpServer);

        if (nvs.retrieve("duty", storedValue)) {
            duty = std::stof(storedValue);
        }

        if (nvs.retrieve("invert", storedValue)) {
            invert = (storedValue == "true");
        }

        if (nvs.retrieve("max_speed", storedValue)) {
            max_speed = std::stof(storedValue);
        }
    }

    std::string toJson() const {
        JsonWrapper json;
        json.AddItem("tz", tz);
        json.AddItem("ntpServer", ntpServer);
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
        JsonWrapper parsed = JsonWrapper::Parse(jsonString);

        updateFieldIfChanged(parsed, "tz", tz, changes);
        updateFieldIfChanged(parsed, "ntpServer", ntpServer, changes);
        updateFieldIfChanged(parsed, "duty", duty, changes);
        updateFieldIfChanged(parsed, "max_speed", max_speed, changes);
        updateFieldIfChanged(parsed, "invert", invert, changes);

        for (const auto& [key, val] : changes) {
            nvs.store(key, val);
        }
        return changes;
    }

    std::string convertChangesToJson(const ChangeList& changes) {
        cJSON* root = cJSON_CreateObject();
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
    void updateFieldIfChanged(JsonWrapper& json,
                              const std::string& key,
                              T& field,
                              ChangeList& changes) {
        if (json.ContainsField(key)) {
            T newValue;
            if (json.GetField(key, newValue)) {
                if (newValue != field) {
                    field = newValue;
                    changes.emplace_back(key, toString(field));
                }
            } else {
                ESP_LOGE("SettingsManager", "Failed to retrieve new value for %s", key.c_str());
            }
        }
    }

    void updateFieldIfChanged(JsonWrapper& json,
                              const std::string& key,
                              bool& field,
                              ChangeList& changes) {
        if (json.ContainsField(key)) {
            std::string newValueStr;
            if (json.GetField(key, newValueStr)) {
                bool newVal = (newValueStr == "true");
                if (newVal != field) {
                    field = newVal;
                    changes.emplace_back(key, newVal ? "true" : "false");
                }
            } else {
                ESP_LOGE("SettingsManager", "Failed to parse bool for %s", key.c_str());
            }
        }
    }

    template <typename U>
    std::string toString(const U& val) {
        if constexpr (std::is_same_v<U, std::string>) {
            return val;
        } else {
            return std::to_string(val);
        }
    }
};

