#include "vis/shaders/EntityShader.hpp"

#include <glm/gtc/type_ptr.hpp>

namespace vis {

    EntityShader::EntityShader(const std::string& vertPath,
                            const std::string& fragPath)
        : ShaderProgram(vertPath, fragPath)
    {
        bindAttributes();
        link();
        getAllUniformLocations();
    }

    void EntityShader::bindAttributes() {
        bindAttribute(0, "position");
        bindAttribute(1, "texCoords");
        bindAttribute(2, "normal");
    }

    void EntityShader::getAllUniformLocations() {
        loc_transformation_  = getUniformLocation("transformationMatrix");
        loc_projection_      = getUniformLocation("projectionMatrix");
        loc_view_            = getUniformLocation("viewMatrix");
        loc_lightPos_        = getUniformLocation("lightPosition");
        loc_lightColor_      = getUniformLocation("lightColor");
        loc_shineDamper_     = getUniformLocation("shineDamper");
        loc_reflectivity_    = getUniformLocation("reflectivity");
        loc_useFakeLighting_ = getUniformLocation("useFakeLighting");
        loc_skyColor_        = getUniformLocation("skyColor");
        loc_texSampler_      = getUniformLocation("textureSampler");
    }

    // Per-frame uniforms

    void EntityShader::loadProjectionMatrix(const Camera& cam) {
        loadMat4(loc_projection_, cam.projectionMatrix());
    }

    void EntityShader::loadViewMatrix(const Camera& cam) {
        loadMat4(loc_view_, cam.viewMatrix());
    }

    void EntityShader::loadLight(const Light& light) {
        loadVec3(loc_lightPos_, light.position);
        loadVec3(loc_lightColor_, light.color);
    }

    void EntityShader::loadSkyColor(const glm::vec3& rgb) {
        loadVec3(loc_skyColor_, rgb);
    }

    // Per-entity uniforms

    void EntityShader::loadTransformation(const glm::mat4& m) {
        loadMat4(loc_transformation_, m);
    }

    void EntityShader::loadShine(float damper, float reflectivity) {
        loadFloat(loc_shineDamper_, damper);
        loadFloat(loc_reflectivity_, reflectivity);
    }

    void EntityShader::loadFakeLighting(bool useFake) {
        loadBool(loc_useFakeLighting_, useFake);
    }

} // namespace vis