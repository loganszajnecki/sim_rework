#version 410 core

// Vertex attributes
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoords;
layout(location = 2) in vec3 normal;

// Uniforms
uniform mat4 transformationMatrix;
uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform vec3 lightPosition;
uniform bool useFakeLighting;

// Outputs to fragment shader
out vec2 pass_texCoords;
out vec3 surfaceNormal;
out vec3 toLightVector;
out vec3 toCameraVector;

void main() {
    vec4 worldPosition = transformationMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * viewMatrix * worldPosition;

    pass_texCoords = texCoords;

    // Normal: use actual normal or fake (always pointing outward).
    vec3 actualNormal = useFakeLighting ? vec3(0.0, 0.0, 1.0) : normal;
    surfaceNormal = (transformationMatrix * vec4(actualNormal, 0.0)).xyz;

    // Vectors for Phong shading.
    toLightVector = lightPosition - worldPosition.xyz;

    // Camera position is the inverse of the view matrix translation.
    vec3 cameraPos = (inverse(viewMatrix) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    toCameraVector = cameraPos - worldPosition.xyz;
}