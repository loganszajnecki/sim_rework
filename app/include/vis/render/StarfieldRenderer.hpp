#pragma once

#include <string>
#include <glad/glad.h>
 
#include "vis/shaders/StarfieldShader.hpp"
#include "vis/Camera.hpp"

namespace vis {

    /**
     * @brief Renders a procedural starfield background.
     *
     * Generates a fixed number of stars at random positions on a large
     * sphere around the origin. Each star has a random brightness that
     * controls its size and color temperature (bright = blue-white,
     * dim = warm yellow).
     *
     * The starfield is rendered as GL_POINTS with point sprites before
     * the main entity pass (no depth write, so entities draw on top).
     *
     * Usage:
     *   StarfieldRenderer stars("shaders", 3000);
     *   // in render loop:
     *   stars.render(camera);
     */
    class StarfieldRenderer 
    {
    public:
        /**
         * @param shaderDir   Path to directory containing starfield.vert/frag.
         * @param numStars    Number of stars to generate.
         * @param radius      Radius of the star sphere (should be < camera far plane).
         */
        StarfieldRenderer(const std::string& shaderDir,
                          int numStars, float radius);

        ~StarfieldRenderer();

        StarfieldRenderer(const StarfieldRenderer&)            = delete;
        StarfieldRenderer& operator=(const StarfieldRenderer&) = delete;
        StarfieldRenderer(StarfieldRenderer&&)                 = delete;
        StarfieldRenderer& operator=(StarfieldRenderer&&)      = delete;
    
        /// Render the starfield. Call before entity rendering.
        void render(const Camera& cam);
    
    private:
        StarfieldShader shader_;
        GLuint vao_       = 0;
        GLuint vboPos_    = 0;
        GLuint vboBright_ = 0;
        int    numStars_  = 0;
    };

} // namespace vis