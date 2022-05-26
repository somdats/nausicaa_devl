
#version 330 core
out vec4 FragColor;

//in vec4 vertexColor; // the input variable from the vertex shader (same name and same type)
in vec4 textureCoordFS;
//in vec4 color;

uniform   sampler2D camTex;
uniform   sampler2D camDepth;

void main()
{
    // opencv matrices
//    vec4 tCoord  = textureCoordFS / textureCoordFS.z; //(for opencv)
//    tCoord /= vec4(1948.0,1096.0,1.0,1.0);

    // opengl matrices
    vec4 tCoord  = textureCoordFS / textureCoordFS.q; //(for opengl)
    tCoord = (tCoord+1.0)*0.5;
    tCoord.y = 1.0 - tCoord.y; // REVERSE Y FOR OPENCVV

    if( textureCoordFS.z < 0.0 ||   tCoord.x < 0.0 ||  tCoord.x > 1.0 || tCoord.y < 0.0 ||  tCoord.y > 1.0)
            FragColor = vec4(1.0,0.0,0.0,1.0);
        else{
            vec4 dep  = texture2D(camDepth,tCoord.xy,1.0);
//            vec4 col;
//             if (tCoord.z   > dep.x+0.1)
//                col = vec4(abs(dep.xyz-vec3(tCoord.z)),1.0);
//                  else
//                col  = texture2D(camTex,tCoord.xy,1.0);
//            FragColor =  col;
            FragColor =  texture2D(camTex,tCoord.xy,1.0);

            }

//FragColor = color;

    // FragColor = vec4(tCoord.xy,0.0,1.0);
    // FragColor = vec4(1.0,0.5,0.0,1.0);
 }
