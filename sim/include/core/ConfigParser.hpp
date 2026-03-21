#pragma once

#include "core/SimConfig.hpp"
#include <string>

namespace sim::core {

/**
 * @brief XML configuration file parser.
 *
 * Reads a simulation configuration XML file and returns a
 * fully populated SimConfig struct. Unknown elements are
 * silently ignored for forward compatibility.
 *
 * Usage:
 *   auto cfg = ConfigParser::load("config/scenario.xml");
 *   // cfg now contains all simulation parameters
 *
 * Also provides a writer for generating template configs:
 *   ConfigParser::save_template("config/default.xml");
 */
class ConfigParser {
public:
    /**
     * @brief Parse an XML configuration file.
     * @param filepath  Path to the XML file
     * @return Fully populated SimConfig
     * @throws std::runtime_error if file cannot be opened or parsed
     */
    [[nodiscard]] static SimConfig load(const std::string& filepath);

    /**
     * @brief Write a SimConfig to an XML file.
     *
     * Serializes all fields of the config, including optional GNC
     * models and all targets. The output XML is loadable by load().
     *
     * @param cfg       Configuration to serialize
     * @param filepath  Output path for the XML file
     */
    static void save(const SimConfig& cfg, const std::string& filepath);

    /**
     * @brief Write a default configuration template to disk.
     * @param filepath  Output path for the XML file
     */
    static void save_template(const std::string& filepath);
};

} // namespace sim::core