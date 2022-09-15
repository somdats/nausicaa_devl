
#version 330 core
out vec4 FragColor;
in vec2 vTC;

uniform   sampler2D uTexture;
 

void main()
{
    FragColor =texture2D(uTexture,vTC);
}
