#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <memory>
#include <string>
#include <vector>

namespace sim::core
{
    /**
     * @brief Centralized simulation logger.
     * 
     * This wrapper around spdlog providing dual-sink logging to both
     * console (colored) and file. Initialize once at program startup
     * via Logger::init(), then use the SIM_* macros throughout the 
     * codebase.
     * Log levels (in order of increasing severity):
     *   trace    — per-timestep diagnostics, state dumps
     *   debug    — model entry/exit, intermediate computati\ons
     *   info     — simulation milestones (burnout, impact, apogee)
     *   warn     — recoverable issues (clamped values, near-singular matrices)
     *   error    — serious problems (negative mass, failed convergence)
     *   critical — unrecoverable state (NaN in state vector)
     *  
     * Usage:
     *   Logger::init("sim.log", spdlog::level::info);
     *   SIM_INFO("Launch at t={:.3f}s, pitch={:.1f} deg", t, pitch_deg);
     *   SIM_WARN("Atmosphere altitude clamped to {:.0f} m", max_alt);
     */
    class Logger
    {
    public:
        /**
         * @brief Initialize the logging system.
         *
         * Creates a multi-sink logger with console output (colored, stderr)
         * and file output. Call once before any SIM_* macro usage.
         *
         * @param log_file   Path to the log file
         * @param level      Minimum log level (default: info)
         */
        static void init(const std::string& log_file = "sim.log",
                         spdlog::level::level_enum level = spdlog::level::info)
        {
            // Console sink: colored output to stderr
            auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
            console_sink->set_level(level);

            // File sink: full detail to the disk
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);            
            file_sink->set_level(spdlog::level::trace); // file always gets everything

            // Multi-sink logger
            std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
            logger_ = std::make_shared<spdlog::logger>("sim", sinks.begin(), sinks.end());
            logger_->set_level(spdlog::level::trace);  // let sinks decide what to show
            logger_->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

            spdlog::set_default_logger(logger_);
        }

        /**
         * @brief Get the logger instance.
         * @return Shared pointer to the spdlog logger.
         */
        static std::shared_ptr<spdlog::logger>& get() { return logger_; }
    
    private:
        static inline std::shared_ptr<spdlog::logger> logger_ = spdlog::default_logger();
    };
} // namespace sim::core

// Convenience macros
// These forward to spdlog with format string support.
// Usage: SIM_INFO("Speed = {:.2f} m/s", speed);
#define SIM_TRACE(...)    SPDLOG_LOGGER_TRACE(sim::core::Logger::get(), __VA_ARGS__)
#define SIM_DEBUG(...)    SPDLOG_LOGGER_DEBUG(sim::core::Logger::get(), __VA_ARGS__)
#define SIM_INFO(...)     SPDLOG_LOGGER_INFO(sim::core::Logger::get(), __VA_ARGS__)
#define SIM_WARN(...)     SPDLOG_LOGGER_WARN(sim::core::Logger::get(), __VA_ARGS__)
#define SIM_ERROR(...)    SPDLOG_LOGGER_ERROR(sim::core::Logger::get(), __VA_ARGS__)
#define SIM_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(sim::core::Logger::get(), __VA_ARGS__)