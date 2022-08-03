
#version 330 core
in vec3 posVS;
out vec4 FragColor;


void main()
{
   vec3 N =  normalize(cross(dFdx(posVS),dFdy(posVS)));
   float l = dot(N,vec3(0.0,1.0,0.0));
   FragColor =  vec4(vec3(l),1.0);
 }
