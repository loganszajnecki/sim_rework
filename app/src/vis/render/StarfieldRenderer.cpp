#include "vis/render/StarfieldRenderer.hpp"

#include <cmath>
#include <random>
#include <vector>

namespace vis {

    // Constructor
    StarfieldRenderer::StarfieldRenderer(const std::string& shaderDir,
                                         int numStars, float radius)
        : shader_(shaderDir + "/starfield.vert",
                  shaderDir + "/starfield.frag")
        , numStars_(numStars)
    {
        // Generate random star positions on a sphere
        std::mt19937 rng(42); // fixed seed for deterministic starfield
        std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

        std::vector<float> positions;
        std::vector<float> brightness;
        positions.reserve(static_cast<size_t>(numStars) * 3);
        brightness.reserve(static_cast<size_t>(numStars));

        constexpr float PI = 3.14159265358979323846f;

        for (int i = 0; i < numStars; ++i) {
            // Uniform distribution on a sphere via spherical coords. 
            float theta = 2.0f * PI * dist01(rng);              // azimuth [0, 2pi]
            float phi   = std::acos(1.0f - 2.0f * dist01(rng)); // polar [0, pi]

            float x = radius * std::sin(phi) * std::cos(theta);
            float y = radius * std::sin(phi) * std::sin(theta);
            float z = radius * std::cos(phi); 

            positions.push_back(x);
            positions.push_back(y);
            positions.push_back(z);

            // Power curve b = raw^3 gives mostly dim stars with rare bright ones. 
            float raw = dist01(rng);
            brightness.push_back(raw*raw*raw*0.85f + 0.15f);
        }

        // Upload to GPU
        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);

        // Position buffer (attrib 0).
        glGenBuffers(1, &vboPos_);
        glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
        glBufferData(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(positions.size() * sizeof(float)),
                    positions.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    
        // Brightness buffer (attrib 1).
        glGenBuffers(1, &vboBright_);
        glBindBuffer(GL_ARRAY_BUFFER, vboBright_);
        glBufferData(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(brightness.size() * sizeof(float)),
                    brightness.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
    
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    // Destructor
    StarfieldRenderer::~StarfieldRenderer() {
        if (vboPos_)    { glDeleteBuffers(1, &vboPos_); }
        if (vboBright_) { glDeleteBuffers(1, &vboBright_); }
        if (vao_)       { glDeleteVertexArrays(1, &vao_); }
        // shader_ cleans itself up via ShaderProgram RAII.
    }

    // Render
    void StarfieldRenderer::render(const Camera& cam) {
        // Stars render behind everything: disable depth write.
        glDepthMask(GL_FALSE);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);   // additive blend for glow
    
        shader_.start();
        shader_.loadProjectionMatrix(cam);
        shader_.loadViewMatrix(cam);
        shader_.loadBasePointSize(2.5f);
    
        glBindVertexArray(vao_);
        glDrawArrays(GL_POINTS, 0, numStars_);
        glBindVertexArray(0);
    
        shader_.stop();
    
        // Restore default state.
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_TRUE);
        glDisable(GL_PROGRAM_POINT_SIZE);
    }

} // namespace vis