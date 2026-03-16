#pragma once

#include "vis/shaders/ShaderProgram.hpp"
#include "vis/Camera.hpp"
#include "vis/entities/Light.hpp"

#include <glm/glm.hpp>

namespace vis {

    /**
     * @brief Shader for textured entites with Phong lighting. 
     * 
     * Phong Lighting: Ambient + Diffuse + Specular
     * 
     * Uniforms:
     *   - transformationMatrix : model matrix (per-entity)
     *   - projectionMatrix     : camera projection
     *   - viewMatrix           : camera view
     *   - lightPosition        : world-space light position
     *   - lightColor           : light RGB
     *   - shineDamper          : specular exponent
     *   - reflectivity         : specular intensity
     *   - useFakeLighting      : skip normal-based lighting
     *   - skyColor             : background color (for fog, if desired)
     * 
     * Vertex Attrributes:
     *   0 - position (vec3)
     *   1 - texCoords (vec2)
     *   2 - normal (vec3)
     */
    class EntityShader : public ShaderProgram
    {
    public:
        EntityShader(const std::string& vertPath,
                     const std::string& fragPath);

        // Per-frame uniforms
        void loadProjectionMatrix(const Camera& cam);
        void loadViewMatrix(const Camera& cam);
        void loadLight(const Light& light);
        void loadSkyColor(const glm::vec3& rgb);

        // Per-entity uniforms
        void loadTransformation(const glm::mat4& m);
        void loadShine(float damper, float reflectivity);
        void loadFakeLighting(bool useFake);

    protected:
        void bindAttributes() override;
        void getAllUniformLocations() override;

    private:
        GLint loc_transformation_  = -1;
        GLint loc_projection_      = -1;
        GLint loc_view_            = -1;
        GLint loc_lightPos_        = -1;
        GLint loc_lightColor_      = -1;
        GLint loc_shineDamper_     = -1;
        GLint loc_reflectivity_    = -1;
        GLint loc_useFakeLighting_ = -1;
        GLint loc_skyColor_        = -1;
        GLint loc_texSampler_      = -1;
    };

} // namespace vis