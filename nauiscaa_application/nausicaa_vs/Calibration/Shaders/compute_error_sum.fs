#version 460 core  
 

out vec4 color; 


uniform sampler2D uTextureRef;
uniform sampler2D uTextureCam;

uniform int uSize_x;
uniform int uSize_y;

int zeros;
int nanni_0,nanni_1,nanni;
// this produce the Hue for v:0..1 (for debug purposes)
vec3 hsv2rgb(float  v)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(vec3(v,v,v) + K.xyz) * 6.0 - K.www);
    return   mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0),1.0);
}

highp  double error_naive(vec3 c0,vec3 c1){
    if(c0==vec3(0,0,0)){ 
     return 1.73;
    }
    return length(c0-c1);
}

bool isnanvec3(vec3 v){
    return (isnan(v.x) || isnan(v.y) || isnan(v.z));
}

void main(void) 
{ 
    highp  double err = 0.0;
    color = vec4(0,0,0,1);
    vec3 c0,c1;

    for(int i = 0; i < uSize_x;++i)
        for(int j = 0; j < uSize_y;++j){   
            c0 = texelFetch(uTextureRef,ivec2(i,j),0).xyz;
            c1 = texelFetch(uTextureCam,ivec2(i,uSize_y-j),0).xyz;
            err += error_naive(c0,c1) ;
        }
   color.x = float(err);
 } 