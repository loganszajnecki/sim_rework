#pragma once

#include "vis/shaders/ShaderProgram.hpp"
#include "vis/Camera.hpp"

#include <glm/glm.hpp>

namespace vis {

    /**
     * @brief Shader for rendering colored lines (grid, axes, trajectories).
     *
     * Uniforms:
     *   - viewProjectionMatrix : combined VP matrix
     *   - lineColor            : RGB color for the current batch
     *
     * Vertex attributes:
     *   0 — position (vec3)
     */
    class LineShader : public ShaderProgram
    {
    public:
        LineShader(const std::string& vertPath,
                const std::string& fragPath);
    
        void loadViewProjection(const Camera& cam);
        void loadColor(const glm::vec3& color);
    
    protected:
        void bindAttributes() override;
        void getAllUniformLocations() override;
    
    private:
        GLint loc_vp_    = -1;
        GLint loc_color_ = -1;
    };

} // namespace vis