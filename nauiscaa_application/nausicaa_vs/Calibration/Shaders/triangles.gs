#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

out vec4 textureCoordFS[6];// texture coordinate for camera 0
//out vec4 color;

in VS_OUT {
    vec4 textureCoord[6];
    float d;
} gs_in[];

vec4 ramp(float v){
    if(v < 0.25) return mix(vec4(1.0,0.0,0.0,1.0),vec4(0.0,1.0,0.0,1.0),v/0.25);
    if(v < 0.5) return mix(vec4(0.0,1.0,0.0,1.0),vec4(1.0,1.0,0.0,1.0),(v-0.25)/0.25);
    if(v < 0.75) return mix(vec4(1.0,1.0,0.0,1.0),vec4(1.0,0.0,1.0,1.0),(v-0.5)/0.25);
    return mix(vec4(1.0,0.0,1.0,1.0),vec4(0.0,0.0,1.0,1.0),(v-0.75)/0.25);
}

void main() {
    float mind = 1000.0;
    float maxd = 0.0;

    if( abs(gs_in[0].d - gs_in[1].d) > 0.5 ) return;
    if( abs(gs_in[1].d - gs_in[2].d) > 0.5 ) return;
    if( abs(gs_in[0].d - gs_in[2].d) > 0.5 ) return;

    for(int i = 0;i < 6; ++i)
        textureCoordFS[i] = gs_in[0].textureCoord[i];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();


    for(int i = 0;i < 6; ++i)
        textureCoordFS[i] = gs_in[1].textureCoord[i];
    gl_Position = gl_in[1].gl_Position;
    EmitVertex();


    for(int i = 0;i < 6; ++i)
        textureCoordFS[i] = gs_in[2].textureCoord[i];
    gl_Position = gl_in[2].gl_Position;
    EmitVertex();

    EndPrimitive();
}
