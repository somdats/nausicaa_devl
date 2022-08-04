#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
layout (location = 1) in float d;    // the distance from the LIDAR


out VS_OUT {
    vec3 posWS;
    float d;
} vs_out;

uniform mat4 mm;
uniform mat4 pm;

void main()
{
    gl_Position   =  pm * mm * vec4(aPos, 1.0);

    vs_out.posWS  =  aPos;
    vs_out.d = d;
}
