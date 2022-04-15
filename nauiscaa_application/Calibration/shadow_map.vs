#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
layout (location = 1) in float d;    // the distance from the LIDAR

out vec4 vertexColor; // specify a color output to the fragment shader
//out vec4 textureCoord;// texture coordinate for camera 0

out VS_OUT {
    float d;
} vs_out;


uniform mat4 toCam;

void main()
{
    vec4 pos = toCam * vec4(aPos, 1.0);
    gl_Position   = pos;
    vs_out.d = d;
}
