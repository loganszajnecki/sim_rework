#include "vis/SphereGenerator.hpp"

#include <cmath>

namespace vis {

    SphereGenerator::SphereData
    SphereGenerator::generate(float radius, int sectors, int stacks) {
        SphereData data;

        const float PI = 3.14159265358979323846f;

        // Vertices
        // Walk from top pole (stack=0) to bottom pole (stack=stacks).
        // At each latitude ring, walk around the longitude (sectors).

        for (int i = 0; i <= stacks; ++i) {
            float stackAngle = PI / 2.0f - PI * static_cast<float>(i) / static_cast<float>(stacks);
            float xz = radius * std::cos(stackAngle);   // radius of ring at this latitude
            float y  = radius * std::sin(stackAngle);   // height (will become Z in Z-up)

            for (int j = 0; j <= sectors; ++j) {
                float sectorAngle = 2.0f * PI * static_cast<float>(j) / static_cast<float>(sectors);

                float x = xz * std::cos(sectorAngle);
                float z = xz * std::sin(sectorAngle);

                // Position (Z-up: swap y<->z from standard sphere formulation).
                data.positions.push_back(x);
                data.positions.push_back(z);     // OpenGL Z-up: y-axis is "into screen" or "forward"
                data.positions.push_back(y);     // Z is up

                // Normal (unit sphere → normal = position/radius).
                float nx = x / radius;
                float ny = z / radius;
                float nz = y / radius;
                data.normals.push_back(nx);
                data.normals.push_back(ny);
                data.normals.push_back(nz);

                // Texture coordinates (equirectangular).
                float u = static_cast<float>(j) / static_cast<float>(sectors);
                float v = 1.0f - static_cast<float>(i) / static_cast<float>(stacks);
                data.texCoords.push_back(u);
                data.texCoords.push_back(v);
            }
        }

        // Indices
        // Two triangles per quad between adjacent latitude rings.

        for (int i = 0; i < stacks; ++i) {
            int k1 = i * (sectors + 1);       // start of current ring
            int k2 = k1 + (sectors + 1);      // start of next ring

            for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
                // Skip degenerate triangles at the poles.
                if (i != 0) {
                    data.indices.push_back(static_cast<unsigned int>(k1));
                    data.indices.push_back(static_cast<unsigned int>(k2));
                    data.indices.push_back(static_cast<unsigned int>(k1 + 1));
                }
                if (i != (stacks - 1)) {
                    data.indices.push_back(static_cast<unsigned int>(k1 + 1));
                    data.indices.push_back(static_cast<unsigned int>(k2));
                    data.indices.push_back(static_cast<unsigned int>(k2 + 1));
                }
            }
        }

        return data;
    }

} // namespace vis