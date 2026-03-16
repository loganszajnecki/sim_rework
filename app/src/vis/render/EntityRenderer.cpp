#include "vis/render/EntityRenderer.hpp"
 
#include <glm/gtc/matrix_transform.hpp>
 
namespace vis {
 
    EntityRenderer::EntityRenderer(EntityShader& shader)
        : shader_(shader) {}

    void EntityRenderer::render(const BatchMap& entities) {
        for (auto& [model, instances] : entities) {
            prepareTexturedModel(model);
            for (auto* entity : instances) {
                prepareInstance(*entity);
                glDrawElements(GL_TRIANGLES, model->raw.indexCount,
                               GL_UNSIGNED_INT, nullptr);
            }
            unbindTexturedModel();
        }
    }

    void EntityRenderer::prepareTexturedModel(TexturedModel* model) {
        // Bind VAO and enable attribute arrays.
        glBindVertexArray(model->raw.vao);
        glEnableVertexAttribArray(0);  // position
        glEnableVertexAttribArray(1);  // texCoords
        glEnableVertexAttribArray(2);  // normal
    
        // Material uniforms.
        const auto& tex = model->texture;
        shader_.loadShine(tex.shineDamper, tex.reflectivity);
        shader_.loadFakeLighting(tex.useFakeLighting);
    
        // Transparency → disable back-face culling.
        if (tex.hasTransparency) {
            glDisable(GL_CULL_FACE);
        }
    
        // Bind texture to unit 0.
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex.id);
    }
    
    void EntityRenderer::unbindTexturedModel() {
        glEnable(GL_CULL_FACE);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glBindVertexArray(0);
    }
    
    void EntityRenderer::prepareInstance(const Entity& entity) {
        // Build the model matrix: translate → rotateX → rotateY → rotateZ → scale.
        glm::mat4 m(1.0f);
        m = glm::translate(m, entity.position);
        m = glm::rotate(m, glm::radians(entity.rotation.x), glm::vec3(1, 0, 0));
        m = glm::rotate(m, glm::radians(entity.rotation.y), glm::vec3(0, 1, 0));
        m = glm::rotate(m, glm::radians(entity.rotation.z), glm::vec3(0, 0, 1));
        m = glm::scale(m, glm::vec3(entity.scale));
    
        shader_.loadTransformation(m);
    }

} // namespace vis