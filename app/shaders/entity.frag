#version 410 core

// Inputs from vertex shader
in vec2 pass_texCoords;
in vec3 surfaceNormal;
in vec3 toLightVector;
in vec3 toCameraVector;

// Uniforms
uniform sampler2D textureSampler;
uniform vec3  lightColor;
uniform float shineDamper;
uniform float reflectivity;
uniform vec3  skyColor;

// Output
out vec4 out_Color;

void main() {
    // Normalize interpolated vectors.
    vec3 unitNormal   = normalize(surfaceNormal);
    vec3 unitToLight  = normalize(toLightVector);
    vec3 unitToCamera = normalize(toCameraVector);

    // Diffuse
    float nDotL = dot(unitNormal, unitToLight);
    // Ambient minimum so the dark side of Earth isn't pure black.
    float brightness = max(nDotL, 0.08);
    vec3 diffuse = brightness * lightColor;

    // Specular (Phong)
    vec3 lightDir  = -unitToLight;
    vec3 reflected = reflect(lightDir, unitNormal);
    float specFactor = dot(reflected, unitToCamera);
    specFactor = max(specFactor, 0.0);
    float dampedFactor = pow(specFactor, shineDamper);
    vec3 specular = dampedFactor * reflectivity * lightColor;

    // Texture sample
    vec4 texColor = texture(textureSampler, pass_texCoords);

    // Discard fully transparent fragments (for transparent textures).
    if (texColor.a < 0.1) {
        discard;
    }

    out_Color = vec4(diffuse, 1.0) * texColor + vec4(specular, 0.0);
}