#version 330 core
layout (points) in;
layout (points, max_vertices = 1) out;

//in vec4 textureCoord;// texture coordinate for camera 0
out vec4 textureCoordFS[6];// texture coordinate for camera 0
uniform int numcam;

in VS_OUT {
    vec4 textureCoord[6];
} gs_in[];

void main() {
    for(int i= 0;i < 6; ++i)
     textureCoordFS[i] = gs_in[0].textureCoord[i];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    EndPrimitive();
}
