#version 410 core

layout(location = 0) in vec3 position;

uniform mat4 viewProjectionMatrix;
uniform vec3 lineColor;

out vec3 pass_color;

void main() {
    gl_Position = viewProjectionMatrix * vec4(position, 1.0);
    pass_color = lineColor;
}