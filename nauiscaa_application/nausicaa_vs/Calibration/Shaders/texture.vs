#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
layout (location = 1) in vec2  aTC; 
out vec2 vTC;

uniform mat4 pm,mm;

void main()
{
    gl_Position   =  pm*mm* vec4(aPos, 1.0);
    vTC  = aTC;
}
