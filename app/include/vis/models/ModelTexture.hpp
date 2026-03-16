#pragma once

#include <glad/glad.h>

namespace vis {

    /**
     * @brief Material and texture descriptor for a model. 
     * 
     * Combines an OpenGL texture ID with material properties that
     * control how the surface is lit and rendered. 
     * 
     * Lifetime:
     *   This struct does not manage the GL texture object.
     *   The Loader is responsible for glDeleteTextures.
     */
    struct ModelTexture {
        GLuint id = 0;

        float shineDamper     = 1.0f;   ///< Specular highlight tightness (higher = tighter).
        float reflectivity    = 0.0f;   ///< Specular intensity.
        bool  hasTransparency = false;  ///< Disable back-face culling for this model.
        bool  useFakeLighting = false;  ///< Flat shading (ignore normals).

        ModelTexture() = default;
        explicit ModelTexture(GLuint texId) : id(texId) {}
    };

} // namespace vis