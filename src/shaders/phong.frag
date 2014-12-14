/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
//#version 130
//#extension GL_ARB_draw_buffers : enable
//#extension GL_ARB_compatibility : enable
//#extension GL_EXT_gpu_shader4 : enable

varying in vec3 halfVec;
varying in vec3 normal;
varying in vec3 lightVec;
varying in vec3 tangent;
varying in vec2 texCoord0;

#ifdef TEXTURE_KD
uniform sampler2D map_Kd;
#endif

#ifdef TEXTURE_BUMP
uniform sampler2D map_Bump;
#endif

void main(void)
{
    vec3 lightVec_fin = normalize(lightVec);
    vec3 normal_fin   = normalize(normal);
    vec3 halfVec_fin  = normalize(halfVec);

    #ifdef TEXTURE_BUMP
    normal_fin = normalize( texture2D(map_Bump, texCoord0).xyz * 2.0 - 1.0);
    #endif

    //	Computing Ambiant color
    vec4 ambientColor = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;

    // compute the diffuse coefficient of the blinn-phong model
    float diffuseCoeff = clamp(dot(lightVec_fin, normal_fin), 0., 1.);

    /*
    if( diffuseCoeff < 0.3 ) diffuseCoeff = 0.;
    else if( diffuseCoeff < 0.7) diffuseCoeff = 0.3;
    else diffuseCoeff = 0.7;
    */

    // Apparemment gl_FrontMaterial.diffuse ne marche pas avec glColorPointer ..
    #ifdef TEXTURE_KD
    vec4 diffuseColor = texture2D(map_Kd, texCoord0) * gl_Color * gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    #else
    vec4 diffuseColor = gl_Color * gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    #endif

    // 	Specular color
    vec4 specularColor = gl_FrontMaterial.specular * gl_LightSource[0].specular;

    // compute the specular component of tje Blinn-Phong model, using the material's shininess
    float specularCoeff = pow(clamp(dot(normal_fin, halfVec_fin),0.,1.), gl_FrontMaterial.shininess);

    /*
    specularCoeff = specularCoeff-0.2;
    if( specularCoeff < 0.3 ) specularCoeff = 0.;
    else if( specularCoeff < 0.7) specularCoeff = 0.3;
    else specularCoeff = 0.7;
    */

    // lighting coefficient (SPOT)
    float spot = 0.0; //Assume no lighting
    //if fragment get some light
    //if its a spot, compute the angle between light direction and view direction. otherwise view direction is assumed to be the same as light direction
    float angleCos = (gl_LightSource[0].spotCosCutoff > 0.0) ? dot (-lightVec_fin, normalize(gl_LightSource[0].spotDirection)) : 1.0;
    if (angleCos > gl_LightSource[0].spotCosCutoff)
        spot = pow (angleCos, gl_LightSource[0].spotExponent);


    float distance = length(lightVec);
    spot /= (gl_LightSource[0].constantAttenuation +
             gl_LightSource[0].linearAttenuation * distance +
             gl_LightSource[0].quadraticAttenuation * distance * distance);

    // Couleur finale du fragment
    // compute the final fragment color by putting together the elements of the model (ambiant, diffuse, shadows, specular and reflection)
    gl_FragColor.rgb = (ambientColor + (spot * (diffuseCoeff * diffuseColor + specularCoeff * specularColor)) ).rgb;

//debuggl_FragColor.rgb = texture2D(map_Kd, texCoord0);
    // Partie alpha
    gl_FragColor.a = diffuseColor.a;


    // DEBUG ----------------------------------------------------------
    // For sliced ARMADILLO
#if 0

    diffuseColor = vec4(0.9, 0.9, 0.9, 1.);
    vec3 cl = (ambientColor + (diffuseCoeff * diffuseColor)).rgb;
    gl_FragColor.rgb = cl;

    float fac = 0.05;
    if(gl_FragCoord.z < fac)
        gl_FragColor.rgb = /*vec3( 0.1,  0.1, 0.8)*/ vec3( 1.,  1., 1.) * (1. - (1./fac)*gl_FragCoord.z) + cl * (1. - (1. - (1./fac)*gl_FragCoord.z));
#endif

    // Interior shape is lit only with diffuse and colored brown
    float cosl = dot(lightVec_fin, normal_fin);
    if(cosl < 0.f)
        gl_FragColor.rgb =  vec3( 0.2f, 0.2f , 0.4f ) /*vec3( 0.6f, 0.4f , 0.2f )*/ * vec3( abs(cosl) );


    /*
    gl_FragColor.rgb = vec3(spot);// ----
    gl_FragColor.rgb = vec3(diffuseCoeff)+ambientColor.rgb;//=
    */
    //gl_FragColor.rgb = normalize(position_mv) * 0.5 + 0.5;
    //gl_FragColor = vec4(0.f, 1.f, 0.f, 1.f);
    //    #ifdef TEST
    //    #endif
    //    gl_FragColor = gl_FrontMaterial.diffuse;
    //gl_FragColor.rgb = tangent*0.5 + 0.5;//diffuseColor;
    //gl_FragColor.rgb = normal_fin * 0.5f + 0.5f;

    //gl_FragColor.rg = texCoord0;
    //gl_FragColor.rgb = lightVec_fin * 0.5f + 0.5f;
    }
