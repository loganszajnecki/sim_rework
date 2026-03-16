#pragma once

#include <unordered_map>
#include <vector>

#include <glad/glad.h>

#include "vis/shaders/EntityShader.hpp"
#include "vis/models/TexturedModel.hpp"
#include "vis/entities/Entity.hpp"

namespace vis {

    /**
     * @brief Low-level renderer for batched textured entities.
     * 
     * Given a batch map (TexturedModel* -> list<Entity*>), this class:
     *   1. Binds each model's VAO + texture.
     *   2. Configures culling based on texture transparency.
     *   3. Uploads per-instance transform matrices.
     *   4. Issues glDrawElements calls.
     * 
     * The called (MasterRenderer) must have already started the shader
     * and loaded projection/view/light uniforms.
     */
    class EntityRenderer
    {
    public:
        using BatchMap = std::unordered_map<TexturedModel*, std::vector<Entity*>>;

        explicit EntityRenderer(EntityShader& shader);

        void render(const BatchMap& entities);
    
    private:
        void prepareTexturedModel(TexturedModel* model);
        void unbindTexturedModel();
        void prepareInstance(const Entity& entity);

        EntityShader& shader_;
    };

} // namespace vis 