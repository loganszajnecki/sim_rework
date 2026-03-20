#include "Application.hpp"
#include "screens/MainMenuScreen.hpp"
#include "screens/WorkspaceScreen.hpp"
#include "screens/PlaceholderScreen.hpp"

#include "core/Logger.hpp"    // sim logger — must init before screens
#include <iostream>
#include <filesystem>
#include <spdlog/spdlog.h>

/**
 * @brief Locate the resource directories relative to the executable.
 *
 * Looks for shaders/ and res/ in the following order:
 *   1. Current working directory.
 *   2. Executable's directory (for out-of-source builds).
 *   3. Source tree (for in-source builds).
 */
struct AppPaths {
    std::string shaders;
    std::string res;
    bool valid = false;

    static AppPaths find() {
        namespace fs = std::filesystem;
        AppPaths p;

        // Try paths relative to the current working directory and
        // common build layouts.
        std::vector<std::string> candidates = {
            ".",                          // running from app/
            "..",                         // running from app/build/
            "../..",                      // running from app/build/Debug/
            "app",                        // running from project root
        };

        for (const auto& base : candidates) {
            fs::path shaderDir = fs::path(base) / "shaders";
            fs::path resDir    = fs::path(base) / "res";

            if (fs::exists(shaderDir / "entity.vert")) {
                p.shaders = shaderDir.string();
                p.res     = resDir.string();
                p.valid   = true;
                return p;
            }
        }

        std::cerr << "Warning: could not locate shaders/ directory.\n"
                  << "Make sure you run from the app/ directory or set paths.\n";
        p.shaders = "shaders";
        p.res     = "res";
        p.valid   = false;
        return p;
    }
};

int main(int /*argc*/, char* /*argv*/[]) {
    // Locate resources
    auto paths = AppPaths::find();
    if (!paths.valid) {
        std::cerr << "Continuing with default paths (may fail to load shaders).\n";
    }

    // Create application
    app::Application application;

    app::Application::Config cfg;
    cfg.width  = 1920;
    cfg.height = 1080;
    cfg.vsync  = true;
    cfg.title  = "Missile Flight Simulator";

    if (!application.init(cfg)) {
        std::cerr << "Failed to initialize application.\n";
        return 1;
    }

    // Initialize sim logger
    // Must happen before screens attach sinks. Console sink (stderr)
    // and file sink are set up here; the workspace adds its ImGui
    // sink on enter().
    sim::core::Logger::init("missile_app.log", spdlog::level::info);
    application.registerScreen("main_menu",
        std::make_unique<app::MainMenuScreen>(paths.shaders, paths.res));

    application.registerScreen("config",
        std::make_unique<app::WorkspaceScreen>(paths.shaders, paths.res));
    application.registerScreen("viewer",
        std::make_unique<app::PlaceholderScreen>("TRAJECTORY VIEWER"));
    application.registerScreen("settings",
        std::make_unique<app::PlaceholderScreen>("SETTINGS"));

    // Start on the main menu
    application.switchTo("main_menu");

    // Run
    application.run();

    return 0;
}