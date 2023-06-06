#version 330 core  
out vec4 color; 

in vec2 vTexCoord;

uniform sampler2D uTextureRef;
uniform sampler2D uTextureCam;

// this produce the Hue for v:0..1 (for debug purposes)
vec3 hsv2rgb(float  v)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(vec3(v,v,v) + K.xyz) * 6.0 - K.www);
    return   mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0),1.0);
}

float error_naive(vec3 c0,vec3 c1){
    if(c0==vec3(0,0,0))
     return 1.0;
    return length(c0-c1);
}

void main(void) 
{ 
    color = (texture(uTextureRef,vTexCoord)+texture(uTextureCam,vec2(vTexCoord.x,1.0-vTexCoord.y)))*0.5;

    vec3 c0 = texture(uTextureRef,vTexCoord).xyz;

    // lazy color coding. BLack means there was no textured geometry  from texture Ref 
    if(c0==vec3(0,0,0))
        discard;

    vec3 c1 = texture(uTextureCam,vec2(vTexCoord.x,1.0-vTexCoord.y)).xyz;
    color = vec4((error_naive(c0,c1)),0.0,0.0,1.0);

} 