#version 330 core

layout(location = 0) in vec3 position; // Position of the vertex
layout(location = 1) in vec3 normal;   // Normal of the vertex
layout(location = 3) in vec2 in_uv;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

uniform mat3 inverseTransposeModel;

out vec3 normal_cameraSpace;
out vec2 uv;

void main() {
    normal_cameraSpace = normalize(inverse(transpose(mat3(view))) * inverseTransposeModel * normal);
    uv = in_uv;

    gl_Position = proj * view * model * vec4(position, 1);
}
