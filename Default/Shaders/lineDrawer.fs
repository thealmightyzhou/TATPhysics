#version 330 core
out vec4 color;
in vec3 ocolour;

void main()
{
  	color = vec4(ocolour,1.0f);
} 
