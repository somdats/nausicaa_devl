#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0

uniform mat4 mm;
uniform mat4 pm;
out float d;

void main()
{
    gl_Position   =  pm*mm*vec4(aPos, 1.0);
    d = length(aPos);
}
