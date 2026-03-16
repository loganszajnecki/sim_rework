#pragma once

#include <string>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace vis {

    /**
     * @brief RAII wrapper around an OpenGL shader program.
     * 
     * Loads vertex + fragment shader source from disk, compiles them,
     * and links them into a program. Derived classes bind attributes 
     * and fetch uniform locations. 
     * 
     * Lifetime:
     *   - Constructor compiles + attaches shaders and creates a program.
     *   - Derived class constructor calls: bindAttributes(), link(), getAllUniformLocations().
     *   - Destructor calls destroy() (detach, delete shaders, delete program).
     *   - Move safe: move ctor/assignment transfer ownership. 
     * 
     * All GL calls require a current OpenGL context.
     */
    class ShaderProgram 
    {
    public:
        ShaderProgram(const std::string& vertPath,
                      const std::string& fragPath);
        virtual ~ShaderProgram();

        ShaderProgram(const ShaderProgram&)            = delete;
        ShaderProgram& operator=(const ShaderProgram&) = delete;
        ShaderProgram(ShaderProgram&& other) noexcept;
        ShaderProgram& operator=(ShaderProgram&& other) noexcept;

        /// Bind this program (glUseProgram).
        void start() const;
    
        /// Unbind (glUseProgram(0)).
        void stop() const;
    
        /// Explicit teardown; safe and idempotent.
        void destroy();
    
    protected:
        /// Link the program after attributes are bound. Call from derived ctor.
        void link();

        /// Derived classes bind vertex attribute indices to names.
        virtual void bindAttributes() = 0;

        /// Derived classes fetch uniform locations.
        virtual void getAllUniformLocations() = 0;

        GLint getUniformLocation(const char* name) const;

        // Uniform upload helpers
        void loadFloat(GLint loc, float v) const;
        void loadInt(GLint loc, int v) const;
        void loadBool(GLint loc, bool v) const;
        void loadVec3(GLint loc, const glm::vec3& v) const;
        void loadVec4(GLint loc, const glm::vec4& v) const;
        void loadMat4(GLint loc, const glm::mat4& m) const;

        /// Bind a vertex attribute index to a named attribute.
        void bindAttribute(GLuint index, const char* name);
    
    protected:
        GLuint program_ = 0;
        GLuint vert_    = 0;
        GLuint frag_    = 0;
    
    private:
        static GLuint loadShaderFromFile(const std::string& path, GLenum type);
        static std::string readFile(const std::string& path);
    };

} // namespace vis