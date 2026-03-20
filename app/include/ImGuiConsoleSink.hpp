#pragma once
 
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
 
#include <mutex>
#include <string>
#include <vector>
#include <deque>

namespace app {

    /**
     * @brief A log entry with level and formatted text.
     */
    struct ConsoleEntry {
        spdlog::level::level_enum level = spdlog::level::info;
        std::string text;
    };

    /**
     * @brief Custom spdlog sink that captures messages for ImGui display. 
     * 
     * This sink is added to the sim's Logger alongside the existing
     * console and file sinks. Every SIM_INFO, SIM_WARN, etc. call 
     * automatically feeds into this buffer.
     * 
     * The WorkspaceScreen drains the buffer each frame via drain()
     * and renders the entries in the Console panel with color-coded
     * log levels.
     * 
     * Thread safety:
     *   - sink_it_() and flush_() are called under spdlog's internal mutex.
     *   - drain() locks a separate mutex to swap the pending buffer.
     *   - No contention between the sim thread and the render thread
     *     beyond the brief swap.
     * 
     * Usage:
     *   auto sink = std::make_shared<ImGuiConsoleSink>();
     *   sim::core::Logger::get()->sinks().push_back(sink);
     * 
     *   // Each frame in the workspace:
     *   auto entries = sink->drain();
     *   for (auto& e : entries) { ... }
     */
    class ImGuiConsoleSink : public spdlog::sinks::base_sink<std::mutex>
    {
    public:
        /// Maximum number of entries retained in the display buffer.
        static constexpr size_t kMaxEntries = 5000;

        ImGuiConsoleSink() = default;

        /**
         * @brief Drain all pending entries since the last drain.
         * 
         * Moves pending entries into the caller's vector. This is
         * called once per frame from the render thread. The internal
         * pending buffer is cleared after the call. 
         */
        std::vector<ConsoleEntry> drain() {
            std::lock_guard<std::mutex> lock(drainMutex_);
            std::vector<ConsoleEntry> result;
            result.swap(pending_);
            return result;
        }

        /// Clear the entire display history.
        void clearHistory() {
            std::lock_guard<std::mutex> lock(drainMutex_);
            pending_.clear();
        }

    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override {
            // Format the message using spdlog's formatter
            spdlog::memory_buf_t formatted;
            this->formatter_->format(msg, formatted);

            ConsoleEntry entry;
            entry.level = msg.level;
            entry.text  = std::string(formatted.data(), formatted.size());

            // String trailing newline if present
            if (!entry.text.empty() && entry.text.back() == '\n') {
                entry.text.pop_back();
            }

            std::lock_guard<std::mutex> lock(drainMutex_);
            pending_.push_back(std::move(entry));
        }

        void flush_() override {
            // Nothing to flush - entries are consumed by drain().
        }
    
    private:
        std::mutex              drainMutex_;
        std::vector<ConsoleEntry> pending_;
    };

} // namespace app