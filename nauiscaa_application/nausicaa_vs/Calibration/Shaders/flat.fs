
#version 330 core
in vec3 posWS;
in float d;
out vec4 FragColor;


void main()
{
   vec3 N =  normalize(cross(dFdx(posWS),dFdy(posWS)));
   float l = abs(dot(N,vec3(0.0,1.0,0.0)));
   if(gl_FrontFacing)
       FragColor =  vec4(vec3(l),1.0);
       else
       FragColor =  vec4(vec3(l)*vec3(0.8,0.8,1.0),1.0);
  if(d>0.5)
    FragColor.yz = vec2(0.0,0.0);
 }
