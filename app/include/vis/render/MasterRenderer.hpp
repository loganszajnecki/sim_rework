#pragma once
 
#include <unordered_map>
#include <vector>
 
#include <glm/glm.hpp>

#include "vis/shaders/EntityShader.hpp"
#include "vis/render/EntityRenderer.hpp"
#include "vis/entities/Entity.hpp"
#include "vis/entities/Light.hpp"
#include "vis/Camera.hpp"

namespace vis {

    /**
     * @brief High-level entity rendering orchestrator.
     * 
     * Collects entities into per-model batches each frame, sets up common
     * shader state (projection, view, light), and delegates drawing to 
     * EntityRenderer. Clears the batch map after each render() call.
     */
    class MasterRenderer
    {
    public:
        using BatchMap = EntityRenderer::BatchMap;

    	MasterRenderer(const std::string& vertPath,
                       const std::string& fragPath);
        
        MasterRenderer(const MasterRenderer&)            = delete;
        MasterRenderer& operator=(const MasterRenderer&) = delete;
        MasterRenderer(MasterRenderer&&)                 = default;
        MasterRenderer& operator=(MasterRenderer&&)      = default;

        /// Queue an entity for rendering this frame (non-owning pointer).
        void processEntity(Entity& e);

        /// Render all queued entities, then clear the batch map.
        void render(const Light& sun, const Camera& cam);

        void setSkyColor(const glm::vec3& rgb) { skyColor_ = rgb; }
    
    private:

        glm::vec3 skyColor_{0.01f, 0.01f, 0.02f};
        
        EntityShader   shader_;
        EntityRenderer renderer_;
        BatchMap       entities_;
    };

} // namespace vis