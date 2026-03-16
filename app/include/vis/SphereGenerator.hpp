#pragma once

#include <vector>

namespace vis {

    /**
     * @brief Generates UV sphere mesh data for loading into a VAO.
     *
     * Creates a sphere centered at the origin with the given radius.
     * The mesh has proper texture coordinates (equirectangular mapping)
     * and smooth normals for Phong shading.
     *
     * Usage:
     *   auto sphere = SphereGenerator::generate(1.0f, 64, 32);
     *   RawModel model = loader.loadToVAO(sphere.positions, sphere.texCoords,
     *                                      sphere.normals, sphere.indices);
     */
    struct SphereGenerator {

        struct SphereData {
            std::vector<float>        positions;
            std::vector<float>        texCoords;
            std::vector<float>        normals;
            std::vector<unsigned int> indices;
        };

        /**
         * @brief Generate a UV sphere.
         *
         * @param radius     Sphere radius.
         * @param sectors    Longitude slices (higher = smoother).
         * @param stacks     Latitude slices (higher = smoother).
         * @return SphereData with flat arrays ready for loadToVAO.
         */
        static SphereData generate(float radius, int sectors, int stacks);
    };

} // namespace vis