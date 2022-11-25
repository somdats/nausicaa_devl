#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

out vec4 p;// texture coordinate for camera 0
//out vec4 color;


uniform mat4 toCam;

in VS_OUT {
    float d;
} gs_in[];

vec4 ramp(float v){
    if(v < 0.25) return mix(vec4(1.0,0.0,0.0,1.0),vec4(0.0,1.0,0.0,1.0),v/0.25);
    if(v < 0.5) return mix(vec4(0.0,1.0,0.0,1.0),vec4(1.0,1.0,0.0,1.0),(v-0.25)/0.25);
    if(v < 0.75) return mix(vec4(1.0,1.0,0.0,1.0),vec4(1.0,0.0,1.0,1.0),(v-0.5)/0.25);
    return mix(vec4(1.0,0.0,1.0,1.0),vec4(0.0,0.0,1.0,1.0),(v-0.75)/0.25);
}

void main() {
    if( abs(gs_in[0].d - gs_in[1].d) > 0.5 ) return;
    if( abs(gs_in[1].d - gs_in[2].d) > 0.5 ) return;
    if( abs(gs_in[0].d - gs_in[2].d) > 0.5 ) return;

    vec4 bary = (gl_in[0].gl_Position +  gl_in[1].gl_Position + gl_in[2].gl_Position )/3.0;

    vec4 pos;

    pos = gl_in[0].gl_Position;
    pos.xy = pos.xy+ (pos.xy-bary.xy);
    gl_Position = toCam*pos;
    p = gl_in[0].gl_Position;
    EmitVertex();


    pos = gl_in[1].gl_Position;
    pos.xy = pos.xy+ (pos.xy-bary.xy);
    gl_Position = toCam*pos;
    p = gl_in[1].gl_Position;
    EmitVertex();


    pos = gl_in[2].gl_Position;
    pos.xy = pos.xy+(pos.xy-bary.xy);
    gl_Position = toCam*pos;
    p = gl_in[2].gl_Position;
    EmitVertex();

    EndPrimitive();
}
