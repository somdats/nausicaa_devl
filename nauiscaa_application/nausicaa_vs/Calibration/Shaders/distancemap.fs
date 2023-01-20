
#version 330 core

uniform float maxdist;

in float d;
out vec4 FragColor;

vec3 ramp(  float  minf,  float   maxf ,float v )
    {
         
        float step=(maxf-minf)/4.0;
        if(v <  minf ) {  return vec3(1.0,0.0,0.0); }
        v-=minf;
        if(v<step) { return mix(vec3(1.0,0.0,0.0),vec3(1.0,1.0,0.0),v/step); }
        v-=step;
        if(v<step) {return mix(vec3(1.0,1.0,0.0),vec3(0.0,1.0,0.0),v/step); }
        v-=step;
        if(v<step)  {return mix(vec3(0.0,1.0,0.0),vec3(0.0,1.0,1.0),v/step); }
        v-=step;
        if(v<step)  {return mix(vec3(0.0,1.0,1.0),vec3(0.0,0.0,1.0),v/step); }

        return vec3(0.0,0.0,1.0);
    }

void main()
{
    FragColor  = vec4(ramp(1.0,maxdist,d),1.0);
 }
