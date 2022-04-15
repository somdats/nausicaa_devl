#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
layout (location = 1) in float d;    // the distance from the LIDAR

out vec4 vertexColor; // specify a color output to the fragment shader
//out vec4 textureCoord;// texture coordinate for camera 0

out VS_OUT {
    vec4 textureCoord;
    float d;
} vs_out;

uniform mat4 mm;
uniform mat4 pm;
uniform mat4 toCam;

void main()
{
    vec4 pos = pm * mm * vec4(aPos, 1.0);
    gl_Position   = pos;
    vs_out.textureCoord  =  toCam * vec4(aPos, 1.0);
    vs_out.d = d;
}
