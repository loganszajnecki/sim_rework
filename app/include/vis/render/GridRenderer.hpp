#pragma once

#include <string>
#include <glad/glad.h>

#include "vis/shaders/LineShader.hpp"
#include "vis/Camera.hpp"

namespace vis {

    /**
     * @brief Renders a ground grid and coordinate axes.
     *
     * Draws a grid on the XY plane (Z=0, Z-up world) and colored
     * axes: X=red, Y=green, Z=blue. Used in the workspace viewport
     * to provide spatial reference.
     *
     * The grid is centered at the origin with configurable extent
     * and spacing.
     */
    class GridRenderer {
    public:
        /**
         * @param shaderDir   Path to shaders/ directory.
         * @param extent      Grid half-size in meters (grid goes -extent to +extent).
         * @param spacing     Distance between grid lines in meters.
         */
        GridRenderer(const std::string& shaderDir,
                    float extent, float spacing);
        ~GridRenderer();

        GridRenderer(const GridRenderer&)            = delete;
        GridRenderer& operator=(const GridRenderer&) = delete;
        GridRenderer(GridRenderer&&)                 = delete;
        GridRenderer& operator=(GridRenderer&&)      = delete;

        /// Render the grid and axes.
        void render(const Camera& cam);

    private:
        LineShader shader_;

        GLuint vaoGrid_  = 0;
        GLuint vboGrid_  = 0;
        int    gridCount_ = 0;    // number of vertices in the grid

        GLuint vaoAxes_  = 0;
        GLuint vboAxes_  = 0;
    };

} // namespace vis