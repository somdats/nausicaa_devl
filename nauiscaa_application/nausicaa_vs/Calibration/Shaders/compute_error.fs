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

void main(void) 
{ 
    color = (texture(uTextureRef,vTexCoord)+texture(uTextureCam,vec2(vTexCoord.x,1.0-vTexCoord.y)))*0.5;
} 