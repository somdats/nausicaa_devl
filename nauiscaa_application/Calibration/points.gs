#version 330 core
layout (points) in;
layout (points, max_vertices = 1) out;

//in vec4 textureCoord;// texture coordinate for camera 0
out vec4 textureCoordFS;// texture coordinate for camera 0

in VS_OUT {
    vec4 textureCoord;
} gs_in[];

void main() {
    textureCoordFS = gs_in[0].textureCoord;
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    EndPrimitive();
}
