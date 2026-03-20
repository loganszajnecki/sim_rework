#include "vis/render/GridRenderer.hpp"

#include <vector>
#include <cmath>

namespace vis {

    GridRenderer::GridRenderer(const std::string& shaderDir,
                            float extent, float spacing)
        : shader_(shaderDir + "/line.vert", shaderDir + "/line.frag")
    {
        // Generate grid lines on the XY plane (Z=0)
        std::vector<float> gridVerts;

        int lines = static_cast<int>(extent / spacing);
        for (int i = -lines; i <= lines; ++i) {
            float pos = static_cast<float>(i) * spacing;

            // Line parallel to Y axis (varies X).
            gridVerts.push_back(pos);
            gridVerts.push_back(-extent);
            gridVerts.push_back(0.0f);

            gridVerts.push_back(pos);
            gridVerts.push_back(extent);
            gridVerts.push_back(0.0f);

            // Line parallel to X axis (varies Y).
            gridVerts.push_back(-extent);
            gridVerts.push_back(pos);
            gridVerts.push_back(0.0f);

            gridVerts.push_back(extent);
            gridVerts.push_back(pos);
            gridVerts.push_back(0.0f);
        }

        gridCount_ = static_cast<int>(gridVerts.size()) / 3;

        glGenVertexArrays(1, &vaoGrid_);
        glBindVertexArray(vaoGrid_);
        glGenBuffers(1, &vboGrid_);
        glBindBuffer(GL_ARRAY_BUFFER, vboGrid_);
        glBufferData(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(gridVerts.size() * sizeof(float)),
                    gridVerts.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        // Generate axis lines
        // Each axis: origin to +extent along that axis. 6 vertices total.
        float axLen = extent * 0.6f;
        float axisVerts[] = {
            // X axis (red)
            0.0f, 0.0f, 0.0f,    axLen, 0.0f,  0.0f,
            // Y axis (green)
            0.0f, 0.0f, 0.0f,    0.0f,  axLen, 0.0f,
            // Z axis (blue)
            0.0f, 0.0f, 0.0f,    0.0f,  0.0f,  axLen,
        };

        glGenVertexArrays(1, &vaoAxes_);
        glBindVertexArray(vaoAxes_);
        glGenBuffers(1, &vboAxes_);
        glBindBuffer(GL_ARRAY_BUFFER, vboAxes_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(axisVerts), axisVerts, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    GridRenderer::~GridRenderer() {
        if (vboGrid_) { glDeleteBuffers(1, &vboGrid_); }
        if (vaoGrid_) { glDeleteVertexArrays(1, &vaoGrid_); }
        if (vboAxes_) { glDeleteBuffers(1, &vboAxes_); }
        if (vaoAxes_) { glDeleteVertexArrays(1, &vaoAxes_); }
    }

    void GridRenderer::render(const Camera& cam) {
        shader_.start();
        shader_.loadViewProjection(cam);

        // Grid (subtle dark lines)
        shader_.loadColor(glm::vec3(0.25f, 0.25f, 0.30f));
        glBindVertexArray(vaoGrid_);
        glDrawArrays(GL_LINES, 0, gridCount_);
        glBindVertexArray(0);

        // Axes (colored, slightly thicker)
        glLineWidth(2.0f);

        // X = red
        shader_.loadColor(glm::vec3(0.9f, 0.2f, 0.2f));
        glBindVertexArray(vaoAxes_);
        glDrawArrays(GL_LINES, 0, 2);

        // Y = green
        shader_.loadColor(glm::vec3(0.2f, 0.9f, 0.2f));
        glDrawArrays(GL_LINES, 2, 2);

        // Z = blue
        shader_.loadColor(glm::vec3(0.2f, 0.4f, 0.9f));
        glDrawArrays(GL_LINES, 4, 2);

        glBindVertexArray(0);
        glLineWidth(1.0f);

        shader_.stop();
    }

} // namespace vis