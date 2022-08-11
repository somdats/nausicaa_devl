#version 330 core
layout (location = 0) in vec3  aPos; // the position variable has attribute position 0
layout (location = 1) in float d;    // the distance from the LIDAR

out vec4 vertexColor; // specify a color output to the fragment shader
//out vec4 textureCoord;// texture coordinate for camera 0

out VS_OUT {
    vec4 textureCoord[6];
    float d;
} vs_out;

uniform mat4 lidarToWorld;
uniform mat4 mm;
uniform mat4 pm;
uniform mat4 toCam[6];
uniform int numcam;

void main()
{
    vec4 pos = pm * mm * vec4(aPos, 1.0);
    gl_Position   = pos;
    for(int i= 0;i < 6; ++i)
        vs_out.textureCoord[i]  =  toCam[i] * lidarToWorld * vec4(aPos, 1.0);
    vs_out.d = d;
}
