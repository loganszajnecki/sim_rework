#include "vis/shaders/StarfieldShader.hpp"

namespace vis {

    StarfieldShader::StarfieldShader(const std::string& vertPath,
                                     const std::string& fragPath)
        : ShaderProgram(vertPath, fragPath)
    {
        bindAttributes();
        link();
        getAllUniformLocations();
    }

    void StarfieldShader::bindAttributes() {
        bindAttribute(0, "position");
        bindAttribute(1, "brightness");
    }

    void StarfieldShader::getAllUniformLocations() {
        loc_projection_ = getUniformLocation("projectionMatrix");
        loc_view_       = getUniformLocation("viewMatrix");
        loc_pointSize_  = getUniformLocation("basePointSize");
    }

    void StarfieldShader::loadProjectionMatrix(const Camera& cam) {
        loadMat4(loc_projection_, cam.projectionMatrix());
    }

        void StarfieldShader::loadViewMatrix(const Camera& cam) {
        loadMat4(loc_view_, cam.viewMatrix());
    }
    
    void StarfieldShader::loadBasePointSize(float size) {
        loadFloat(loc_pointSize_, size);
    }

} // namespace vis