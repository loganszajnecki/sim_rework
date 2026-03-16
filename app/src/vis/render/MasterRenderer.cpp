#include "vis/render/MasterRenderer.hpp"

namespace vis {

    MasterRenderer::MasterRenderer(const Camera& cam,
                                const std::string& vertPath,
                                const std::string& fragPath)
        : shader_(vertPath, fragPath)
        , renderer_(shader_)
    {
        // Load projection matrix once (reload if FOV/aspect changes).
        shader_.start();
        shader_.loadProjectionMatrix(cam);
        shader_.stop();

        // Default GL state for 3D rendering.
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }

    void MasterRenderer::processEntity(Entity& e) {
        auto it = entities_.find(e.model);
        if (it != entities_.end()) {
            it->second.push_back(&e);
        } else {
            entities_[e.model] = {&e};
        }
    }

    void MasterRenderer::render(const Light& sun, const Camera& cam) {
        prepare();

        shader_.start();
        shader_.loadSkyColor(skyColor_);
        shader_.loadLight(sun);
        shader_.loadViewMatrix(cam);
        renderer_.render(entities_);
        shader_.stop();

        entities_.clear();
    }

    void MasterRenderer::prepare() {
        glClearColor(skyColor_.r, skyColor_.g, skyColor_.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

} // namespace vis