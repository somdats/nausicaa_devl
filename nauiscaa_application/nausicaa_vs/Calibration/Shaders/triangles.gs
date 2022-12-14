#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

out vec4 textureCoordFS[6];// texture coordinate for camera 0
//out vec4 color;

in VS_OUT {
    vec3 pos_vs;
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

    vec3 n;
    vec3 n01 = normalize(gs_in[1].pos_vs -gs_in[0].pos_vs);
    vec3 n02 = normalize(gs_in[2].pos_vs -gs_in[0].pos_vs);
    n = normalize(cross(n01,n02));

    if(n.z < 0.8 || 
    gs_in[0].pos_vs.z > -1.0 || 
    gs_in[1].pos_vs.z > -1.0 || 
    gs_in[2].pos_vs.z > -1.0  
    ){ 
        if( abs(gs_in[0].d - gs_in[1].d) > 0.5 ) return;
        if( abs(gs_in[1].d - gs_in[2].d) > 0.5 ) return;
        if( abs(gs_in[0].d - gs_in[2].d) > 0.5 ) return;
        }

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
