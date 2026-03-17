#version 410 core

in float pass_brightness;

out vec4 out_Color;

void main() {
    // Soft circular point: distance from center of the point sprite.
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = length(coord);

    // discard pizels outside a soft circle
    if (dist > 0.5) {
        discard;
    }

    // Soft falloff from center
    float alpha = 1.0 - smoothstep(0.0, 0.5, dist);

    // Base white with subtle warm/cool tiny based on brightness.
    // Bright stars skew slightly blue-white, dim stars skew war,.
    vec3 coolTint = vec3(0.85, 0.9, 1.0); // blue-white
    vec3 warmTint = vec3(1.0, 0.92, 0.8); // warm yellow
    vec3 starColor = mix(warmTint, coolTint, pass_brightness);

    out_Color = vec4(starColor * pass_brightness, alpha * pass_brightness);
}