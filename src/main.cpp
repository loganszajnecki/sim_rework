#include <iostream>
#include <string>

#include "core/Logger.hpp"
#include "core/SimConfig.hpp"
#include "core/ConfigParser.hpp"
#include "core/DataRecorder.hpp"
#include "core/SimRunner.hpp"

using namespace sim::core;

static void print_usage(const char* prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " <config.xml>            Run simulation\n"
              << "  " << prog << " --generate-template     Write default config\n";
}

int main(int argc, char* argv[]) {
    Logger::init("missile_sim.log", spdlog::level::info);

    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string arg = argv[1];

    if (arg == "--generate-template") {
        ConfigParser::save_template("default.xml");
        std::cout << "Template configuration written to default.xml\n";
        return 0;
    }

    // Load configuration
    SIM_INFO("===============================================");
    SIM_INFO("  6-DOF Missile Simulation");
    SIM_INFO("===============================================");

    SimConfig cfg;
    try {
        cfg = ConfigParser::load(arg);
    } catch (const std::exception& e) {
        SIM_ERROR("Failed to load config: {}", e.what());
        return 1;
    }

    // Set up data recorder
    DataRecorder::Config rec_cfg;
    rec_cfg.filepath = cfg.recorder.filepath;
    rec_cfg.record_interval = cfg.recorder.interval;
    DataRecorder recorder(rec_cfg);

    // Run simulation
    SimRunner runner;
    SimResult result = runner.run(cfg, &recorder, 0);

    // Print summary
    auto reason_str = [](TerminationReason r) -> const char* {
        switch (r) {
            case TerminationReason::closest_approach: return "Closest approach";
            case TerminationReason::ground_impact:    return "Ground impact";
            case TerminationReason::timeout:          return "Timeout";
            default:                                  return "Unknown";
        }
    };

    std::cout << "\n  === Summary ===============================\n"
              << "  Termination:    " << reason_str(result.termination) << "\n"
              << "  Flight time:    " << result.final_time << " s\n"
              << "  Miss distance:  " << result.miss_distance << " m\n"
              << "  POCA time:      " << result.miss_time << " s\n"
              << "  Impact speed:   " << result.impact_speed << " m/s\n"
              << "  Max altitude:   " << result.max_altitude << " m\n"
              << "  Max speed:      " << result.max_speed << " m/s\n"
              << "  Max Mach:       " << result.max_mach << "\n"
              << std::endl;

    SIM_INFO("Simulation complete. Data written to {}", cfg.recorder.filepath);
    return 0;

}