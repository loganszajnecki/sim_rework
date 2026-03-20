#pragma once
 
#include <glad/glad.h>

namespace vis {

    /**
     * RAII wrapper around an OpenGL framebuffer object (FBO)
     * 
     * Creates a framebuffer with a color texture attachment and a depth
     * renderbuffer. The color texture can be passsed to ImGui::Image()
     * to display a rendered scene inside a dockable panel.
     * 
     * Usage:
     *   Framebuffer fbo(800, 600);
     *   fbo.bind();
     *   // render 3D scene...
     *   fbo.unbind();
     *   ImGui::Image((ImTextureID)(intptr_t)fbo.colorTexture(), size);
     * 
     * Call resize() when the viewport panel changes size. This recreates
     * the attachments at the new resolution.
     */
    class Framebuffer
    {
    public:

        Framebuffer() = default;
        Framebuffer(int width, int height);
        ~Framebuffer();

        Framebuffer(const Framebuffer&)            = delete;
        Framebuffer& operator=(const Framebuffer&) = delete;
        Framebuffer(Framebuffer&& other) noexcept;
        Framebuffer& operator=(Framebuffer&& other) noexcept;

        /// Bind this FBO as the render target.
        void bind() const;
    
        /// Unbind (return to the default framebuffer).
        void unbind() const;

        /// Resize the FBO attachments. Safe to call every frame if needed.
        void resize(int width, int height);
    
        /// The color texture ID (for ImGui::Image).
        [[nodiscard]] GLuint colorTexture() const { return colorTex_; }
    
        [[nodiscard]] int width()  const { return width_; }
        [[nodiscard]] int height() const { return height_; }
        [[nodiscard]] bool valid() const { return fbo_ != 0; }

    private:
        void create_();
        void destroy_();
    
        GLuint fbo_      = 0;
        GLuint colorTex_ = 0;
        GLuint depthRbo_ = 0;
        int    width_    = 0;
        int    height_   = 0;
    };

} // namespace vis