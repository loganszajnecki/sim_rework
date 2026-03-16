#include "vis/Loader.hpp"

#include <iostream>
#include <stdexcept>

#include <stb_image.h>

namespace vis {

    Loader::~Loader() {
        cleanup();
    }

    void Loader::cleanup() {
        if (!vaos_.empty()) {
            glDeleteVertexArrays(static_cast<GLsizei>(vaos_.size()), vaos_.data());
            vaos_.clear();
        }
        if (!vbos_.empty()) {
            glDeleteBuffers(static_cast<GLsizei>(vbos_.size()), vbos_.data());
            vbos_.clear();
        }
        if (!textures_.empty()) {
            glDeleteTextures(static_cast<GLsizei>(textures_.size()), textures_.data());
            textures_.clear();
        }
    }

    // ── Mesh loading ─────────────────────────────────────────────

    RawModel Loader::loadToVAO(const std::vector<float>& positions,
                                const std::vector<float>& texCoords,
                                const std::vector<float>& normals,
                                const std::vector<unsigned int>& indices)
    {
        GLuint vao = createVAO();
        bindIndicesBuffer(indices);
        storeDataInAttributeList(0, 3, positions);   // position: vec3
        storeDataInAttributeList(1, 2, texCoords);   // texCoords: vec2
        storeDataInAttributeList(2, 3, normals);     // normal: vec3
        unbindVAO();
        return RawModel(vao, static_cast<GLsizei>(indices.size()));
    }

    GLuint Loader::createVAO() {
        GLuint vao = 0;
        glGenVertexArrays(1, &vao);
        vaos_.push_back(vao);
        glBindVertexArray(vao);
        return vao;
    }

    void Loader::storeDataInAttributeList(GLuint attrib, int coordSize,
                                        const std::vector<float>& data)
    {
        GLuint vbo = 0;
        glGenBuffers(1, &vbo);
        vbos_.push_back(vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(data.size() * sizeof(float)),
                    data.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(attrib, coordSize, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(attrib);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void Loader::bindIndicesBuffer(const std::vector<unsigned int>& indices) {
        GLuint ebo = 0;
        glGenBuffers(1, &ebo);
        vbos_.push_back(ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                    indices.data(), GL_STATIC_DRAW);
        // Note: do NOT unbind GL_ELEMENT_ARRAY_BUFFER while VAO is bound.
    }

    void Loader::unbindVAO() {
        glBindVertexArray(0);
    }

    // ── Texture loading ──────────────────────────────────────────

    GLuint Loader::loadTexture(const std::string& filepath) {
        int width = 0, height = 0, channels = 0;
        stbi_set_flip_vertically_on_load(true);
        unsigned char* data = stbi_load(filepath.c_str(), &width, &height,
                                        &channels, 4);
        if (!data) {
            std::cerr << "Loader: failed to load texture: " << filepath << "\n";
            return 0;
        }

        GLuint texId = 0;
        glGenTextures(1, &texId);
        textures_.push_back(texId);
        glBindTexture(GL_TEXTURE_2D, texId);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // Anisotropic filtering if available (EXT extension, not core in GL 4.1).
    #ifdef GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT
        float maxAniso = 0.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
        if (maxAniso > 1.0f) {
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                            std::min(maxAniso, 8.0f));
        }
    #endif

        glBindTexture(GL_TEXTURE_2D, 0);
        stbi_image_free(data);
        return texId;
    }

    GLuint Loader::createSolidTexture(unsigned char r, unsigned char g,
                                    unsigned char b, unsigned char a)
    {
        unsigned char pixel[4] = {r, g, b, a};

        GLuint texId = 0;
        glGenTextures(1, &texId);
        textures_.push_back(texId);
        glBindTexture(GL_TEXTURE_2D, texId);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, pixel);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        glBindTexture(GL_TEXTURE_2D, 0);
        return texId;
    }

} // namespace vis