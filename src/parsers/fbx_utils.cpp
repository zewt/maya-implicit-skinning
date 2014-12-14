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
#include "fbx_utils.hpp"

#include <iostream>

#ifdef IOS_REF
#undef  IOS_REF
#define IOS_REF (*(g_manager->GetIOSettings()))
#endif

// =============================================================================
namespace Fbx_utils {
// =============================================================================

void print_ver(int* v)
{
    std::cout << v[0] << "." << v[1] << "." << v[2] << std::endl;
}

// -----------------------------------------------------------------------------

bool load_scene(const std::string& filename,
                KFbxScene* fbx_scene,
                KFbxSdkManager* g_manager)
{
    const char* cfile = filename.c_str();
    int file_ver[3];
    int sdk_ver[3];

    // Initialize Fbx loader

    // Get the file version number generate by the FBX SDK.
    KFbxSdkManager::GetFileFormatVersion(sdk_ver[0], sdk_ver[1], sdk_ver[2]);
    std::cout << "FBX version number for this FBX SDK is "; print_ver(sdk_ver);

    // Create an importer.
    KFbxImporter* importer = KFbxImporter::Create( g_manager,"");

    // Initialize the importer by providing a filename.
    const bool import_state = importer->Initialize(cfile, -1, &(IOS_REF));

    // Get file version for the loaded file
    importer->GetFileVersion(file_ver[0], file_ver[1], file_ver[2]);
    std::cout <<  "FBX version number for file " << cfile << " is ";
    print_ver(file_ver);

    if( !import_state )
    {
        std::cout << "Call to KFbxImporter::Initialize() failed.\n";
        std::cout << "Error returned: " << importer->GetLastErrorString();
        std::cout << "\n\n";
        return false;
    }

    if(importer->IsFBX())
    {
        // From this point, it is possible to access animation stack information
        // without the expense of loading the entire file.

        std::cout << "Animation Stack Information\n";
        int stack_len = importer->GetAnimStackCount();
        std::cout << "    Number of Animation Stacks: " << stack_len << "\n";
        std::cout << "    Current Animation Stack: \"";
        std::cout << importer->GetActiveAnimStackName().Buffer() << "\"\n";
        std::cout << std::endl;
        for(int i = 0; i < stack_len; i++)
        {
            KFbxTakeInfo* info = importer->GetTakeInfo(i);
            std::cout << "    Animation Stack " << i << "\n";
            std::cout << "         Name: \"" << info->mName.Buffer() << "\"\n";
            std::cout << "         Description: \"" << info->mDescription.Buffer();
            std::cout << "\"\n";

            // Change the value of the import name if the animation stack should
            // be imported under a different name.

            std::cout << "         Import Name: \"" << info->mImportName.Buffer();
            std::cout << "\"\n";

            // Set the value of the import state to false if the animation stack
            // should be not be imported.
            const char* tmp = info->mSelect ? "true" : "false";
            std::cout << "         Import State: " << tmp << "\n";
            std::cout << std::endl;
        }
        // Set the import states. By default, the import states are always set to
        // true. The code below shows how to change these states.
        IOS_REF.SetBoolProp(IMP_FBX_MATERIAL,        true);
        IOS_REF.SetBoolProp(IMP_FBX_TEXTURE,         true);
        IOS_REF.SetBoolProp(IMP_FBX_LINK,            true);
        IOS_REF.SetBoolProp(IMP_FBX_SHAPE,           true);
        IOS_REF.SetBoolProp(IMP_FBX_GOBO,            true);
        IOS_REF.SetBoolProp(IMP_FBX_ANIMATION,       true);
        IOS_REF.SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, true);
    }


    // Import the scene
    bool status = importer->Import( fbx_scene );
    // deal with protected files
    if(status == false && importer->GetLastErrorID() == KFbxIO::ePASSWORD_ERROR)
    {
        char password[1024];
        std::cout << "Please enter password: ";
        password[0] = '\0';
        std::cin >> password;
        KString string( password );
        IOS_REF.SetStringProp( IMP_FBX_PASSWORD, string );
        IOS_REF.SetBoolProp( IMP_FBX_PASSWORD_ENABLE, true );
        status = importer->Import(fbx_scene);

        if(status == false &&
           importer->GetLastErrorID() == KFbxIO::ePASSWORD_ERROR)
        {
            std::cout << "\nPassword is wrong, import aborted." << std::endl;
        }
    }
    else
    {
        std::cerr << importer->GetLastErrorString();
    }

    // Destroy the importer.
    importer->Destroy();

    return status;
}

// -----------------------------------------------------------------------------

KFbxNode* find_root(KFbxNode* root, KFbxNodeAttribute::EAttributeType attrib)
{
    unsigned nb_children = root->GetChildCount();
    if(!nb_children) return 0;

    KFbxNode* child = 0;
    for(unsigned c = 0; c < nb_children; c++)
    {
        child = root->GetChild(c);
        if(!child->GetNodeAttribute())
            continue;
        if(child->GetNodeAttribute()->GetAttributeType() != attrib)
            continue;

        return child;
    }

    KFbxNode* joint = 0;
    for(unsigned c = 0; c < nb_children; c++)
    {
        child = root->GetChild(c);
        joint = find_root(child, attrib);
        if(joint) break;
    }
    return joint;
}

// -----------------------------------------------------------------------------

void print_hierarchy(KFbxNode* pNode, int pDepth);

void print_hierarchy(KFbxScene* pScene)
{
    int i;
    KFbxNode* lRootNode = pScene->GetRootNode();

    for( i = 0; i < lRootNode->GetChildCount(); i++)
    {
        print_hierarchy(lRootNode->GetChild(i), 0);
    }
}

// -----------------------------------------------------------------------------

void print_hierarchy(KFbxNode* pNode, int pDepth)
{
    std::string string;
    int i;

    for(i = 0; i < pDepth; i++)
        string += "     ";


    string += pNode->GetName();
    string += " attr type: ";
    string += to_string(pNode->GetNodeAttribute()->GetAttributeType());

    std::cout << string << std::endl;

    for(i = 0; i < pNode->GetChildCount(); i++)
        print_hierarchy(pNode->GetChild(i), pDepth + 1);

}

// -----------------------------------------------------------------------------

void print_anim_stacks(KFbxScene* scene)
{

    for(int i = 0; i < scene->GetSrcObjectCount(FBX_TYPE(KFbxAnimStack)); i++)
    {
        KFbxAnimStack* stack = KFbxCast<KFbxAnimStack>(scene->GetSrcObject(FBX_TYPE(KFbxAnimStack), i));

        KString str = "Animation Stack Name: ";
        str += stack->GetName();
        str += "\n";
        std::cout << str << std::endl;
    }
}

// -----------------------------------------------------------------------------

void set_global_frame(KFbxNode* pNode, KFbxXMatrix pGlobalPosition)
{
    KFbxXMatrix lLocalPosition;
    KFbxXMatrix lParentGlobalPosition;

    if (pNode->GetParent())
    {
        lParentGlobalPosition = get_global_frame(pNode->GetParent());
        lLocalPosition = lParentGlobalPosition.Inverse() * pGlobalPosition;
    }
    else
    {
        lLocalPosition = pGlobalPosition;
    }

    pNode->LclTranslation.Set(lLocalPosition.GetT());
    pNode->LclRotation.Set(lLocalPosition.GetR());
    pNode->LclScaling.Set(lLocalPosition.GetS());
}

// -----------------------------------------------------------------------------

KFbxXMatrix get_global_frame(const KFbxNode* pNode)
{
    KFbxXMatrix lLocalPosition;
    KFbxXMatrix lGlobalPosition;
    KFbxXMatrix lParentGlobalPosition;

    lLocalPosition.SetT(pNode->LclTranslation.Get());
    lLocalPosition.SetR(pNode->LclRotation.Get());
    lLocalPosition.SetS(pNode->LclScaling.Get());

    if(pNode->GetParent())
    {
        lParentGlobalPosition = get_global_frame(pNode->GetParent());
        lGlobalPosition = lParentGlobalPosition * lLocalPosition;
    }
    else
    {
        lGlobalPosition = lLocalPosition;
    }

    return lGlobalPosition;
}

// -----------------------------------------------------------------------------

KFbxXMatrix geometry_transfo(KFbxNode* pNode)
{
    KFbxVector4 lT, lR, lS;
    KFbxXMatrix tr;

    lT = pNode->GetGeometricTranslation(KFbxNode::eSOURCE_SET);
    lR = pNode->GetGeometricRotation   (KFbxNode::eSOURCE_SET);
    lS = pNode->GetGeometricScaling    (KFbxNode::eSOURCE_SET);

    tr.SetT( lT );
    tr.SetR( lR );
    tr.SetS( lS );

    return tr;
}

// -----------------------------------------------------------------------------

void copy(float* t, const fbxDouble3& d3)
{
    t[0] = (float)d3[0]; t[1] = (float)d3[1]; t[2] = (float)d3[2];
}

// -----------------------------------------------------------------------------

void copy( Loader::Material& m, const KFbxSurfacePhong* phong )
{
    m._name = std::string( phong->GetName() );
    Fbx_utils::copy(m._Ka, phong->Ambient.Get() );
    Fbx_utils::copy(m._Kd, phong->Diffuse.Get() );
    Fbx_utils::copy(m._Ks, phong->Specular.Get());
    m._Tf[0] = m._Tf[1] = m._Tf[2] = std::max(1.f - (float)phong->TransparencyFactor.Get(), 0.f);
    m._Ns = (float)phong->Shininess.Get();

    // textures
    KFbxFileTexture* fbx_tex = 0;
    fbx_tex = (KFbxFileTexture*) phong->Ambient.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Ka = std::string( fbx_tex->GetFileName() );

    fbx_tex = (KFbxFileTexture*) phong->Diffuse.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Kd = std::string( fbx_tex->GetFileName() );

    fbx_tex = (KFbxFileTexture*) phong->Specular.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Ks = std::string( fbx_tex->GetFileName() );

    fbx_tex = (KFbxFileTexture*) phong->Bump.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Bump = std::string( fbx_tex->GetFileName() );
}

// -----------------------------------------------------------------------------

void copy( Loader::Material& m, const KFbxSurfaceLambert* lbrt )
{
    m._name = std::string( lbrt->GetName() );
    Fbx_utils::copy(m._Ka, lbrt->Ambient.Get() );
    Fbx_utils::copy(m._Kd, lbrt->Diffuse.Get() );
    m._Tf[0] = m._Tf[1] = m._Tf[2] = std::max( 1.f - (float)lbrt->TransparencyFactor.Get(), 0.f);

    // textures
    KFbxFileTexture* fbx_tex = 0;
    fbx_tex = (KFbxFileTexture*) lbrt->Ambient.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Ka = std::string( fbx_tex->GetFileName() );

    fbx_tex = (KFbxFileTexture*) lbrt->Diffuse.GetSrcObject(KFbxTexture::ClassId);
    if( fbx_tex ) m._map_Kd = std::string( fbx_tex->GetFileName() );
}

// -----------------------------------------------------------------------------

Transfo to_transfo(const KFbxXMatrix& mat)
{
    Transfo tr;
    const KFbxXMatrix matT = mat.Transpose();
    for(int i = 0; i < 16; i++)
        tr[i] = (float)(((const double*)matT)[i]);

    return tr;
}

// -----------------------------------------------------------------------------

Transfo to_transfo(const KFbxMatrix& mat)
{
    Transfo tr;
    const KFbxMatrix matT = mat.Transpose();
    for(int i = 0; i < 16; i++)
        tr[i] = (float)(((const double*)matT)[i]);

    return tr;
}

// -----------------------------------------------------------------------------

Loader::Vertex to_lvertex( const KFbxVector4& vec)
{
    return Loader::Vertex( (float)vec[0], (float)vec[1], (float)vec[2]);
}

// -----------------------------------------------------------------------------

Loader::Normal to_lnormal( const KFbxVector4& vec)
{
    return Loader::Normal( (float)vec[0], (float)vec[1], (float)vec[2]);
}

// -----------------------------------------------------------------------------

Loader::TexCoord to_ltexcoord(const KFbxVector2& vec)
{
    return Loader::TexCoord((float)vec[0], (float)vec[1]);
}

// -----------------------------------------------------------------------------

std::string to_string(KFbxGeometryElement::EMappingMode type)
{
    std::string str;
    switch(type)
    {
    case (KFbxGeometryElement::eNONE):              str = "eNONE";              break;
    case (KFbxGeometryElement::eBY_CONTROL_POINT):  str = "eBY_CONTROL_POINT";  break;
    case (KFbxGeometryElement::eBY_POLYGON_VERTEX): str = "eBY_POLYGON_VERTEX"; break;
    case (KFbxGeometryElement::eBY_POLYGON):        str = "eBY_POLYGON";        break;
    case (KFbxGeometryElement::eBY_EDGE):           str = "eBY_EDGE";           break;
    case (KFbxGeometryElement::eALL_SAME):          str = "eALL_SAME";          break;
    }

    return str;
}

// -----------------------------------------------------------------------------

std::string to_string(KFbxNodeAttribute::EAttributeType type)
{
    std::string str;
    switch(type)
    {
    case (KFbxNodeAttribute::eUNIDENTIFIED):       str = "eUNIDENTIFIED";       break;
    case (KFbxNodeAttribute::eNULL):               str = "eNULL";               break;
    case (KFbxNodeAttribute::eMARKER):             str = "eMARKER";             break;
    case (KFbxNodeAttribute::eSKELETON):           str = "eSKELETON";           break;
    case (KFbxNodeAttribute::eMESH):               str = "eMESH";               break;
    case (KFbxNodeAttribute::eNURB):               str = "eNURB";               break;
    case (KFbxNodeAttribute::ePATCH):              str = "ePATCH";              break;
    case (KFbxNodeAttribute::eCAMERA):             str = "eCAMERA";             break;
    case (KFbxNodeAttribute::eCAMERA_STEREO):      str = "eCAMERA_STEREO";      break;
    case (KFbxNodeAttribute::eCAMERA_SWITCHER):    str = "eCAMERA_SWITCHER";    break;
    case (KFbxNodeAttribute::eLIGHT):              str = "eLIGHT";              break;
    case (KFbxNodeAttribute::eOPTICAL_REFERENCE):  str = "eOPTICAL_REFERENCE";  break;
    case (KFbxNodeAttribute::eOPTICAL_MARKER):     str = "eOPTICAL_MARKER";     break;
    case (KFbxNodeAttribute::eNURBS_CURVE):        str = "eNURBS_CURVE";        break;
    case (KFbxNodeAttribute::eTRIM_NURBS_SURFACE): str = "eTRIM_NURBS_SURFACE"; break;
    case (KFbxNodeAttribute::eBOUNDARY):           str = "eBOUNDARY";           break;
    case (KFbxNodeAttribute::eNURBS_SURFACE):      str = "eNURBS_SURFACE";      break;
    case (KFbxNodeAttribute::eSHAPE):              str = "eSHAPE";              break;
    case (KFbxNodeAttribute::eLODGROUP):           str = "eLODGROUP";           break;
    case (KFbxNodeAttribute::eSUBDIV):             str = "eSUBDIV";             break;
    case (KFbxNodeAttribute::eCACHED_EFFECT):      str = "eCACHED_EFFECT";      break;
    case (KFbxNodeAttribute::eLINE):               str = "eLINE";               break;
    }

    return str;
}

}// END NAMESPACE FBX_UTILS ====================================================
