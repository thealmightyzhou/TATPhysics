#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texcoordinate;

out vec3 onormal;
out vec3 ofragPos;
out vec2 texcoordinate0;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * model * vec4(position, 1.0f);
    ofragPos = vec3(model * vec4(position, 1.0f));
    onormal = mat3(transpose(inverse(model))) * normal;  
	texcoordinate0 = texcoordinate;
}