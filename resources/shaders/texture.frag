#version 330 core
out vec4 fragColor;

in vec3 normal_cameraSpace;
in vec2 uv;

uniform sampler2D tex;

void main() {
    // Do lighting in camera space
    vec3 lightDir = normalize(vec3(0, 0.5, 1));
    float c = clamp(dot(normal_cameraSpace, lightDir), 0, 1);

    fragColor = texture(tex, uv);
}
