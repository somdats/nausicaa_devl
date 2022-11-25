
#version 330 core
out vec4 FragColor;

//in vec4 vertexColor; // the input variable from the vertex shader (same name and same type)
in vec4 textureCoordFS[6];
//in vec4 color;

uniform   sampler2D camTex[6];
uniform   sampler2D camDepth[6];
uniform   int      aligned[6];
uniform   int      used[6];

float weight[6];

void main()
{   
    vec4 tCoord[6];
    float total_weigth= 0.0;
    for(int ic = 0; ic < 4; ++ic)
        if(aligned[ic] == 1 && used[ic] == 1)
        { 
            tCoord[ic]  = textureCoordFS[ic] / textureCoordFS[ic].w; //(for opengl)
         
            tCoord[ic] = (tCoord[ic]+1.0)*0.5;
            tCoord[ic].y = 1.0 - tCoord[ic].y; // REVERSE Y FOR OPENCVV
             weight[ic] = 0.0;
             if( !( textureCoordFS[ic].z < 0.0 ||   tCoord[ic].x < 0.0 ||  tCoord[ic].x > 1.0 || tCoord[ic].y < 0.0 ||  tCoord[ic].y > 1.0))
                  {   
                    vec4 dep  = texture2D(camDepth[ic],vec2(tCoord[ic].x,1.0-tCoord[ic].y),1.0);
                    vec4 col;
                    if ( ( tCoord[ic].z   < dep.x + 0.01)){ 
                       weight[ic] = max( length(dFdx(tCoord[ic]).xy)  , length(dFdy(tCoord[ic]).xy) );
                       total_weigth = total_weigth + weight[ic];
                    }
                  }
        }

     for(int ic = 0; ic < 4; ++ic)
         weight[ic] = weight[ic] / total_weigth;

    // opengl matrices
    FragColor = vec4(0.0,0.0,0.0,1.0);
    for(int ic = 0; ic < 4; ++ic)
        FragColor += weight[ic]*texture2D(camTex[ic],tCoord[ic].xy,1.0);
 }
