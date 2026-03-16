#pragma once

#include "vis/models/RawModel.hpp"
#include "vis/models/ModelTexture.hpp"

namespace vis {

    /**
     * @brief Pairs a mesh (RawModel) with its materia/texture (ModelTexture)
     * 
     * A TexturedModel is a chared renderable asset. Multiple Entity instances
     * can reference the same TexturedModel, drawing many copies of the same 
     * mesh with the same texture (instancing by pointer).
     * 
     * Typically contructred once and then references by Entity objects.
     */
    struct TexturedModel {
        RawModel raw;
        ModelTexture texture;

        TexturedModel() = default;
        TexturedModel(const RawModel& r, const ModelTexture& t)
            : raw(r), texture(t) {} // lol rawr
    };

} // namespace vis 