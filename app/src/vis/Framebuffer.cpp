#include "vis/Framebuffer.hpp"

#include <iostream>

namespace vis {

    Framebuffer::Framebuffer(int width, int height)
        : width_(width), height_(height)
    {
        create_();
    }

    Framebuffer::~Framebuffer() {
        destroy_();
    }

    Framebuffer::Framebuffer(Framebuffer&& other) noexcept
        : fbo_(other.fbo_)
        , colorTex_(other.colorTex_)
        , depthRbo_(other.depthRbo_)
        , width_(other.width_)
        , height_(other.height_)
    {
        other.fbo_      = 0;
        other.colorTex_ = 0;
        other.depthRbo_ = 0;
        other.width_    = 0;
        other.height_   = 0;
    }

    Framebuffer& Framebuffer::operator=(Framebuffer&& other) noexcept {
        if (this != &other) {
            destroy_();
            fbo_      = other.fbo_;
            colorTex_ = other.colorTex_;
            depthRbo_ = other.depthRbo_;
            width_    = other.width_;
            height_   = other.height_;
            other.fbo_      = 0;
            other.colorTex_ = 0;
            other.depthRbo_ = 0;
            other.width_    = 0;
            other.height_   = 0;
        }
        return *this;
    }

    void Framebuffer::bind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glViewport(0, 0, width_, height_);
    }

    void Framebuffer::unbind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Framebuffer::resize(int width, int height) {
        if (width == width_ && height == height_) return;
        if (width <= 0 || height <= 0) return;

        destroy_();
        width_  = width;
        height_ = height;
        create_();
    }

    void Framebuffer::create_() {
        if (width_ <= 0 || height_ <= 0) return;

        // ── Color texture ────────────────────────────────────────
        glGenTextures(1, &colorTex_);
        glBindTexture(GL_TEXTURE_2D, colorTex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);

        // ── Depth renderbuffer ───────────────────────────────────
        glGenRenderbuffers(1, &depthRbo_);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRbo_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8,
                            width_, height_);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        // ── Framebuffer ──────────────────────────────────────────
        glGenFramebuffers(1, &fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_TEXTURE_2D, colorTex_, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                                GL_RENDERBUFFER, depthRbo_);

        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "Framebuffer incomplete: 0x" << std::hex
                    << status << std::dec << "\n";
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Framebuffer::destroy_() {
        if (colorTex_) { glDeleteTextures(1, &colorTex_);       colorTex_ = 0; }
        if (depthRbo_) { glDeleteRenderbuffers(1, &depthRbo_);  depthRbo_ = 0; }
        if (fbo_)      { glDeleteFramebuffers(1, &fbo_);        fbo_      = 0; }
    }

} // namespace vis