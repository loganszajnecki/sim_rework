#pragma once

#include <string>
#include <vector>

#include <glad/glad.h>

#include "vis/models/RawModel.hpp"

namespace vis {

    /**
     * @brief Creates and tracks OpenGL resources (VAOs, VBOs, textures).
     * 
     * All GL objects created thru Loader are tracked and cleaned up
     * in the destructor. This is the single point of resource management
     * for the rendering layer.
     * 
     * RAII: safe and idempotent cleanup on destruction
     */
    class Loader 
    {
    public:
        Loader() = default;
        ~Loader();

        Loader(const Loader&)            = delete;
        Loader& operator=(const Loader&) = delete;
        Loader(Loader&&)                 = default;
        Loader& operator=(Loader&&)      = default;

        /**
         * @brief Load a mesh from vertex data into a VAO.
         * 
         * @param positions   Flat array of vertex positions (x,y,z, x,y,z, ...).
         * @param texCoords   Flat array of texture coordinates (u, v, u, v, ...).
         * @param normals     Flat array of normals (nx, ny, nz, nx, ny, nz, ...).
         * @param indices     Index buffer for glDrawElements
         * @return RawModel handle to the loaded mesh.  
         */
        RawModel loadToVAO(const std::vector<float>& positions,
                    const std::vector<float>& texCoords,
                    const std::vector<float>& normals,
                    const std::vector<unsigned int>& indices);
        
        /**
         * @brief Load a 2D texture from a file (PNG, JPG, etc via stb_image)
         * 
         * @param filepath   Path to the texture file. 
         * @return GLuint texture object name (0 on failure).
         */
        GLuint loadTexture(const std::string& filepath);

        /**
         * @brief Create a 1x1 solid color texture (useful as fallback).
         */
        GLuint createSolidTexture(unsigned char r, unsigned char g,
                                  unsigned char b, unsigned char a = 255);
        
        /// Destroy all tracked GL resources. Safe, idempotent.
        void cleanup();
    
    private:
        GLuint createVAO();
        void storeDataInAttributeList(GLuint attrib, int coordSize,
                                    const std::vector<float>& data);
        void bindIndicesBuffer(const std::vector<unsigned int>& indices);
        void unbindVAO();
    
        std::vector<GLuint> vaos_;
        std::vector<GLuint> vbos_;
        std::vector<GLuint> textures_;
    };

} // namespace vis