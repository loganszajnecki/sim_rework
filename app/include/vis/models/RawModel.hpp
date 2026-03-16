#pragma once

#include <glad/glad.h>

namespace vis {

    /**
     * @brief Handle to an OpenGL mesh (VAO + index count)
     * 
     * RawModel is a lightweight value type that identifies a loaded mesh
     * on the GPU. It does not own the GL resources - the Loader class
     * manages creation and cleanup of VAOs and VBOs.
     * 
     * Fields:
     *   vao        - vertex array object name.
     *   indexCount - number of indices in the element buffer (for glDrawElements).
     */
    struct RawModel {
        GLuint vao = 0;
        GLsizei indexCount = 0;

        RawModel() = default;
        RawModel(GLuint vao,  GLsizei count)
            : vao(vao), indexCount(count) {}
    };

} // namespace vis