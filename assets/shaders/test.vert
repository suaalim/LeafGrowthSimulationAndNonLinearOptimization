#version 460 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 color;

uniform mat4 viewProj;

out vec3 fragColor;

void main() {
	gl_Position = viewProj * vec4(pos, 1.0);
	fragColor = color;
}
