#include "vis/shaders/ShaderProgram.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <glm/gtc/type_ptr.hpp>

namespace vis {

    // File I/O

    std::string ShaderProgram::readFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("ShaderProgram: cannot open file: " + path);
        }
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }

    // Shader compilation

    GLuint ShaderProgram::loadShaderFromFile(const std::string& path,
                                            GLenum type) {
        std::string source = readFile(path);
        const char* src = source.c_str();

        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char log[1024];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            glDeleteShader(shader);
            throw std::runtime_error(
                "Shader compile error (" + path + "):\n" + log);
        }
        return shader;
    }

    // Constructor / destructor

    ShaderProgram::ShaderProgram(const std::string& vertPath,
                                const std::string& fragPath)
    {
        vert_ = loadShaderFromFile(vertPath, GL_VERTEX_SHADER);
        frag_ = loadShaderFromFile(fragPath, GL_FRAGMENT_SHADER);

        program_ = glCreateProgram();
        glAttachShader(program_, vert_);
        glAttachShader(program_, frag_);
        // Derived class calls bindAttributes() then link().
    }

    ShaderProgram::~ShaderProgram() {
        destroy();
    }

    ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
        : program_(other.program_), vert_(other.vert_), frag_(other.frag_)
    {
        other.program_ = 0;
        other.vert_    = 0;
        other.frag_    = 0;
    }

    ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
        if (this != &other) {
            destroy();
            program_ = other.program_;
            vert_    = other.vert_;
            frag_    = other.frag_;
            other.program_ = 0;
            other.vert_    = 0;
            other.frag_    = 0;
        }
        return *this;
    }

    // Link

    void ShaderProgram::link() {
        glLinkProgram(program_);

        GLint success = 0;
        glGetProgramiv(program_, GL_LINK_STATUS, &success);
        if (!success) {
            char log[1024];
            glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
            throw std::runtime_error(std::string("Shader link error:\n") + log);
        }

        glValidateProgram(program_);
    }

    // Start / stop / destroy

    void ShaderProgram::start() const {
        glUseProgram(program_);
    }

    void ShaderProgram::stop() const {
        glUseProgram(0);
    }

    void ShaderProgram::destroy() {
        if (program_ != 0) {
            stop();
            if (vert_ != 0) { glDetachShader(program_, vert_); glDeleteShader(vert_); vert_ = 0; }
            if (frag_ != 0) { glDetachShader(program_, frag_); glDeleteShader(frag_); frag_ = 0; }
            glDeleteProgram(program_);
            program_ = 0;
        }
    }

    // Uniform helpers

    GLint ShaderProgram::getUniformLocation(const char* name) const {
        return glGetUniformLocation(program_, name);
    }

    void ShaderProgram::loadFloat(GLint loc, float v) const {
        glUniform1f(loc, v);
    }

    void ShaderProgram::loadInt(GLint loc, int v) const {
        glUniform1i(loc, v);
    }

    void ShaderProgram::loadBool(GLint loc, bool v) const {
        glUniform1i(loc, v ? 1 : 0);
    }

    void ShaderProgram::loadVec3(GLint loc, const glm::vec3& v) const {
        glUniform3f(loc, v.x, v.y, v.z);
    }

    void ShaderProgram::loadVec4(GLint loc, const glm::vec4& v) const {
        glUniform4f(loc, v.x, v.y, v.z, v.w);
    }

    void ShaderProgram::loadMat4(GLint loc, const glm::mat4& m) const {
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
    }

    void ShaderProgram::bindAttribute(GLuint index, const char* name) {
        glBindAttribLocation(program_, index, name);
    }

} // namespace vis