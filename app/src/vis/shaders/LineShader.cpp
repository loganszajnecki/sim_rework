#include "vis/shaders/LineShader.hpp"

namespace vis {

    LineShader::LineShader(const std::string& vertPath,
                        const std::string& fragPath)
        : ShaderProgram(vertPath, fragPath)
    {
        bindAttributes();
        link();
        getAllUniformLocations();
    }

    void LineShader::bindAttributes() {
        bindAttribute(0, "position");
    }

    void LineShader::getAllUniformLocations() {
        loc_vp_    = getUniformLocation("viewProjectionMatrix");
        loc_color_ = getUniformLocation("lineColor");
    }

    void LineShader::loadViewProjection(const Camera& cam) {
        loadMat4(loc_vp_, cam.projectionMatrix() * cam.viewMatrix());
    }

    void LineShader::loadColor(const glm::vec3& color) {
        loadVec3(loc_color_, color);
    }

} // namespace vis