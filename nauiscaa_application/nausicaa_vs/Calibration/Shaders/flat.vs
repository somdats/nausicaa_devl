#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
out vec3 posVS;

uniform mat4 mm;
uniform mat4 pm;

void main()
{
    posVS         = (mm * vec4(aPos, 1.0)).xyz;
    gl_Position   =  pm * mm * vec4(aPos, 1.0);
}
