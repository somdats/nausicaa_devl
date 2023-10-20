#version 460 core  
out vec4 color; 

in vec2 vTexCoord;

uniform sampler2D uTexture;
uniform int uErode_dilate;

float is_border( vec2 p)
{
    int i = int( p.x*1948.0);
    int j = int( p.y*1096.0);

    if( i < 3 || i> 1945 || j < 3 || j> 1095)
     return 0.0;

    if(uErode_dilate == 0) // erode
    { 
        for (int ii = i-8; ii < i+8; ++ii)
            for (int jj = j-8; jj < j+8; ++jj)
               if( (texelFetch(uTexture,ivec2(ii,jj),0).xyz == vec3(0,0,0)) )
                    return 1.0;
    }else{
        for (int ii = i-50; ii < i+50; ++ii)
            for (int jj = j-50; jj < j+50; ++jj)
               if( (texelFetch(uTexture,ivec2(ii,jj),0).xyz != vec3(0,0,0)) )
                    return 0.0;
        return 1.0;
    }


     return 0.0;
}

void main(void) 
{ 
    float v = is_border(vTexCoord);
    color = vec4(vec3(1.0-v),1.0);
} 
