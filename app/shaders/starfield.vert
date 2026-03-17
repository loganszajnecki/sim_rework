#version 410 core

layout(location = 0) in vec3 position;
layout(location = 1) in float brightness;

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform float basePointSize;

out float pass_brightness;

void main() {
    gl_Position = projectionMatrix * viewMatrix * vec4(position, 1.0);
    pass_brightness = brightness;

    // Vary point size with brightness - brighter stars appear larger.
    gl_PointSize = basePointSize * (0.5 + brightness * 1.5);
}