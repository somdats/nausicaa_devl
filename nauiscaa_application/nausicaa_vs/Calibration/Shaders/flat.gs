#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

out vec3 posWS;
out float d;

in VS_OUT {
    vec3 posWS;
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

   float disc = 0;
//    if ( ( abs(gs_in[0].d - gs_in[1].d) > 0.5 )  || 
//      ( abs(gs_in[1].d - gs_in[2].d) > 0.5 )   ||
//      ( abs(gs_in[0].d - gs_in[2].d) > 0.5 ) )
//      disc = 1.0; 


    gl_Position = gl_in[0].gl_Position;
    posWS = gs_in[0].posWS;
    d = disc;
    EmitVertex();


    gl_Position = gl_in[1].gl_Position;
    posWS = gs_in[1].posWS;
    d = disc;
    EmitVertex();


    gl_Position = gl_in[2].gl_Position;
    posWS = gs_in[2].posWS;
    d = disc;
    EmitVertex();

    EndPrimitive();
}
