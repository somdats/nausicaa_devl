
#version 330 core
out vec4 FragColor;
in vec4  p;

void main()
{
 FragColor = vec4( gl_FragCoord.z,gl_FragCoord.x,gl_FragCoord.y,1.0);
 }
