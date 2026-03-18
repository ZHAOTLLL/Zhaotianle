/**
 * airspace_resource_manager.hpp - 空域资源（YAML 配置）
 */
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace resources {

class AirspaceResourceManager {
public:
    struct Location {
        std::string name;
        std::string code;
        std::string description;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        std::map<std::string, std::string> attributes;
        std::vector<std::string> access_restrictions;
    };

    bool loadLocationsFromConfig(const std::string& config_file);
    std::vector<Location> getAllLocations() const;
    std::optional<Location> getLocationByCode(const std::string& code) const;
    std::optional<Location> getLocationByName(const std::string& name) const;

private:
    std::vector<Location> locations_;

    static std::string trim(const std::string& s);
    static bool parseKeyValue(const std::string& line, std::string& key, std::string& value);
};

} // namespace resources
