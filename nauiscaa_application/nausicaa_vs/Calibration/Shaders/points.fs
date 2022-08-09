
#version 330 core
out vec4 FragColor;

//in vec4 vertexColor; // the input variable from the vertex shader (same name and same type)
in vec4 textureCoordFS[6];
//in vec4 color;

uniform   sampler2D camTex[6];
uniform   sampler2D camDepth[6];
uniform   int      aligned[6];


float weight[6];

void main()
{   
    float total_weigth= 0.0;
    for(int ic = 0; ic < 4; ++ic)
        if(aligned[ic] == 1)
        { 
            vec4 tCoord  = textureCoordFS[ic] / textureCoordFS[ic].w; //(for opengl)
         
            tCoord = (tCoord+1.0)*0.5;
            tCoord.y = 1.0 - tCoord.y; // REVERSE Y FOR OPENCVV

             if(!( textureCoordFS[ic].z < 0.0 ||   tCoord.x < 0.0 ||  tCoord.x > 1.0 || tCoord.y < 0.0 ||  tCoord.y > 1.0))
                  {   
                    weight[ic] = max( length(dFdx(tCoord).xy)  , length(dFdy(tCoord).xy) );
                    total_weigth = total_weigth + weight[ic];
                  }
              else
                  weight[ic] = 0.0;
        }

     for(int ic = 0; ic < 4; ++ic)
         weight[ic] = weight[ic] / total_weigth;

    // opengl matrices
    FragColor = vec4(0.0,0.0,0.0,1.0);
    for(int ic = 0; ic < 4; ++ic)
        if(aligned[ic] == 1)
            if(weight[ic]>0.0)
                { 
                    vec4 tCoord  = textureCoordFS[ic] / textureCoordFS[ic].w; //(for opengl)
                    tCoord = (tCoord+1.0)*0.5;
                    tCoord.y = 1.0 - tCoord.y; // REVERSE Y FOR OPENCVV

                     if(!( textureCoordFS[ic].z < 0.0 ||   tCoord.x < 0.0 ||  tCoord.x > 1.0 || tCoord.y < 0.0 ||  tCoord.y > 1.0))
                     {
                        vec4 dep  = texture2D(camDepth[ic],vec2(tCoord.x,1.0-tCoord.y),1.0);
                         vec4 col;
                 //         if (tCoord.z   > dep.x)
                 //            FragColor += vec4(1.0,1.0,1.0,1.0);
               //            FragColor = vec4( 0.0,dep.x,tCoord.z,1.0);
            //            FragColor =  col;
                         FragColor += weight[ic]*texture2D(camTex[ic],tCoord.xy,1.0);
                   }
              } 
 }
