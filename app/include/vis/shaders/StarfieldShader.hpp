#pragma once

#include "vis/shaders/ShaderProgram.hpp"
#include "vis/Camera.hpp"

namespace vis {

    /**
     * @brief Shader for rendering point-sprite stars.
     *
     * Uniforms:
     *   - projectionMatrix : camera projection
     *   - viewMatrix       : camera view
     *   - basePointSize    : base size for star points
     *
     * Vertex attributes:
     *   0 — position (vec3)
     *   1 — brightness (float)
     */
    class StarfieldShader : public ShaderProgram
    {
    public:
        StarfieldShader(const std::string& vertPath,
                        const std::string& fragPath);

        // Load uniforms
        void loadProjectionMatrix(const Camera& cam);
        void loadViewMatrix(const Camera& cam);
        void loadBasePointSize(float size);

    protected:
        void bindAttributes() override;
        void getAllUniformLocations() override;

    private:
        GLint loc_projection_ = -1;
        GLint loc_view_       = -1;
        GLint loc_pointSize_  = -1;
    };

} // namespace vis