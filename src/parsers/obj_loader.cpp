//------------------------------------------------------------------------------
///	\file	obj_loader.cpp
///	\author	Rob Bateman, mailto:robthebloke@hotmail.com
///	\date	11-4-05
///	\brief	A C++ objloader supporting materials, and any face data going, groups
///			(ie, different meshes), calculation of normals. All face data will be
///			triangulated (badly, though should work 99% of the time).
///
//------------------------------------------------------------------------------

#include "obj_loader.hpp"
#include <algorithm>
#include <string>
#include <math.h> // TODO: use cmath
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>



// =============================================================================
namespace Obj_loader {
// =============================================================================


// Normal Calculation Utilities ================================================

void DoFaceCalc(const Vertex& v1, const Vertex& v2, const Vertex& v3,
                Normal& n1, Normal& n2, Normal& n3)
{
    // calculate vector between v2 and v1
    Vertex e1;
    e1.x=	v1.x - v2.x;
    e1.y=	v1.y - v2.y;
    e1.z=	v1.z - v2.z;

    // calculate vector between v2 and v3
    Vertex e2;
    e2.x=	v3.x - v2.x;
    e2.y=	v3.y - v2.y;
    e2.z=	v3.z - v2.z;

    // cross product them
    Vertex e1_cross_e2;
    e1_cross_e2.x=	e2.y*e1.z - e2.z*e1.y;
    e1_cross_e2.y=	e2.z*e1.x - e2.x*e1.z;
    e1_cross_e2.z=	e2.x*e1.y - e2.y*e1.x;

    float itt = 1.0f/((float)sqrt( e1_cross_e2.x*e1_cross_e2.x +
                                  e1_cross_e2.y*e1_cross_e2.y +
                                  e1_cross_e2.z*e1_cross_e2.z ));

    // normalize
    e1_cross_e2.x *= itt;
    e1_cross_e2.y *= itt;
    e1_cross_e2.z *= itt;

    // sum the face normal into all the vertex normals this face uses
    n1.x += e1_cross_e2.x;
    n1.y += e1_cross_e2.y;
    n1.z += e1_cross_e2.z;

    n2.x += e1_cross_e2.x;
    n2.y += e1_cross_e2.y;
    n2.z += e1_cross_e2.z;

    n3.x += e1_cross_e2.x;
    n3.y += e1_cross_e2.y;
    n3.z += e1_cross_e2.z;
}

//------------------------------------------------------------------------------

void NormaliseNormals(std::vector<Normal>& norms) {
    std::vector<Normal>::iterator itn = norms.begin();
    for( ; itn != norms.end(); ++itn ) {
        float itt = 1.0f/((float)sqrt( itn->x*itn->x +
                                      itn->y*itn->y +
                                      itn->z*itn->z ));
        itn->x *= itt;
        itn->y *= itt;
        itn->z *= itt;
    }
}

//------------------------------------------------------------------------------

void ZeroNormals(std::vector<Normal>& norms) {
    // zero normals
    std::vector<Normal>::iterator itn = norms.begin();
    for( ; itn != norms.end(); ++itn )
        itn->x = itn->y = itn->z = 0;
}
// END Normal Calculation Utilities ============================================


// File Reading Utils ==========================================================
std::ostream& operator << (std::ostream& ofs,const Face& f) {
    ofs << "f ";
    for(int i=0;i!=3;++i) {
        ofs << (f.v[i]+1);
        if(f.n[i] != -1 || f.t[i] != -1) {
            ofs << "/";
            if(f.t[i] != -1)
                ofs << (f.t[i]+1);
            ofs << "/";
            if(f.n[i] != -1)
                ofs << (f.n[i]+1);
        }
        ofs << " ";
    }
    return ofs << std::endl;
}

// -----------------------------------------------------------------------------

bool HasOnlyVertex(const std::string& s) {
    std::string::const_iterator it = s.begin();
    for( ; it != s.end(); ++it ) {
        if(*it=='/')
            return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool MissingUv(const std::string& s) {
    std::string::const_iterator it = s.begin();
    while( *it != '/') {
        if(it == s.end())
            return true;
        ++it;
    }
    return *(it+1)=='/';
}

// -----------------------------------------------------------------------------

bool MissingNormal(const std::string& s) {
    return s[s.size()-1] == '/';
}

// -----------------------------------------------------------------------------

/// quick utility function to copy a range of data from the obj file arrays
/// into a surface array.
template< typename T >
void CopyArray(std::vector< T >& output, const std::vector< T >& input, unsigned start, unsigned end)
{
    output.resize( end-start+1 );
    typename std::vector< T >::iterator       ito   = output.begin();
    typename std::vector< T >::const_iterator it    = input.begin() + start;
    typename std::vector< T >::const_iterator itend = input.begin() + end + 1;
    for( ; it != itend; ++it,++ito )
    {
        *ito = *it;
    }
}

// -----------------------------------------------------------------------------

void DetermineIndexRange(unsigned int& s_vert,unsigned int& e_vert,
                         int& s_norm,int& e_norm,int& s_uv,int& e_uv,
                         std::vector<Face>::const_iterator it,std::vector<Face>::const_iterator end) {

    // need to determine start and end vertex/normal and uv indices
    s_vert=0xFFFFFFF;
    s_norm=0xFFFFFFF;
    s_uv  =0xFFFFFFF;
    e_vert= 0;
    e_norm=-1;
    e_uv  =-1;

    // loop through faces to find max/min indices
    for( ; it != end; ++it ) {
        for(int i=0;i!=3;++i) {
            if(it->v[i]<s_vert)	s_vert = it->v[i];
            if(it->v[i]>e_vert)	e_vert = it->v[i];
            if(it->n[i]!=-1) {
                if(it->n[i]<s_norm) s_norm = it->n[i];
                if(it->n[i]>e_norm) e_norm = it->n[i];
            }
            if(it->t[i]!=-1) {
                if(it->t[i]<s_uv) s_uv = it->t[i];
                if(it->t[i]>e_uv) e_uv = it->t[i];
            }
        }
    }
}

// -----------------------------------------------------------------------------

template< typename T >
void WriteArrayRange(std::ostream& ofs,const std::vector< T >& the_array,unsigned start,unsigned end) {
    typename std::vector< T >::const_iterator it    = the_array.begin() + start;
    typename std::vector< T >::const_iterator itend = the_array.begin() + end + 1;
    for( ; it != itend; ++it )
        ofs << *it;
}

// -----------------------------------------------------------------------------


// END File Reading Utils ======================================================


// Class VertexParam ===========================================================
VertexParam::VertexParam( std::istream& ifs ) {
    std::stringbuf current_line_sb(std::ios::in);
    ifs.get( current_line_sb );
    std::istream current_line(&current_line_sb);
    current_line >> u >> v;
    if(!current_line.eof()) current_line >> w;
}
// End Class VertexParam =======================================================


// Class BezierPatch ===========================================================
BezierPatch::BezierPatch() {
    memset(this,0,sizeof(BezierPatch));
    SetLod(10);
}
// End Class BezierPatch =======================================================


// CLASS Surface ===============================================================
Surface::Surface()
    : name(),
      m_Vertices(),
      m_Normals(),
      m_TexCoords(),
      m_Triangles(),
      m_AssignedMaterials()
{}

//------------------------------------------------------------------------------

Surface::Surface(const Surface& surface)
    : name(surface.name),
      m_Vertices(surface.m_Vertices),
      m_Normals(surface.m_Normals),
      m_TexCoords(surface.m_TexCoords),
      m_Triangles(surface.m_Triangles),
      m_AssignedMaterials(surface.m_AssignedMaterials)
{}

//------------------------------------------------------------------------------

void Surface::CalculateNormals() {
    // resize normal array if not present
    if(!m_Normals.size())
        m_Normals.resize(m_Vertices.size());

    ZeroNormals( m_Normals );

    // loop through each triangle in face
    std::vector<Face>::iterator it = m_Triangles.begin();
    for( ; it != m_Triangles.end(); ++it ) {
        // if no indices exist for normals, create them
        if(it->n[0]==-1)   it->n[0] = it->v[0];
        if(it->n[1]==-1)   it->n[1] = it->v[1];
        if(it->n[2]==-1)   it->n[2] = it->v[2];

        // calc face normal and sum into normal array
        DoFaceCalc(m_Vertices[ it->v[0] ],m_Vertices[ it->v[1] ],m_Vertices[ it->v[2] ],
                   m_Normals[ it->n[0] ],m_Normals[ it->n[1] ],m_Normals[ it->n[2] ]);
    }

    NormaliseNormals( m_Normals );
}
// END CLASS Surface ===========================================================


// CLASS VertexBuffer ==========================================================
VertexBuffer::VertexBuffer()
    : name(),
      m_Vertices(),
      m_Normals(),
      m_TexCoords(),
      m_Indices(),
      m_AssignedMaterials(),
      m_pFile(0)
{}

//------------------------------------------------------------------------------

VertexBuffer::VertexBuffer(const VertexBuffer& surface)
    : name(surface.name),
      m_Vertices(surface.m_Vertices),
      m_Normals(surface.m_Normals),
      m_TexCoords(surface.m_TexCoords),
      m_Indices(surface.m_Indices),
      m_AssignedMaterials(surface.m_AssignedMaterials),
      m_pFile(surface.m_pFile)
{}

//------------------------------------------------------------------------------

void VertexBuffer::CalculateNormals() {
    // resize normal array if not present
    if(m_Normals.size()!=m_Vertices.size())
        m_Normals.resize(m_Vertices.size());

    ZeroNormals( m_Normals );

    // loop through each triangle in face
    std::vector<unsigned int>::const_iterator it = m_Indices.begin();
    for( ; it < m_Indices.end(); it+=3 )
        DoFaceCalc(m_Vertices[ *it ],m_Vertices[ *(it+1) ],m_Vertices[ *(it+2) ],
                   m_Normals[ *it ],m_Normals[ *(it+1) ],m_Normals[ *(it+2) ]);

    NormaliseNormals( m_Normals );
}
// END CLASS VertexBuffer ======================================================


// CLASS Obj_File ==============================================================
std::string Obj_file::read_chunk(std::istream& ifs) {
    std::string s;
    do {
        char c = ifs.get();
        if (c=='\\') {
            while (ifs.get()!='\n') { /*empty*/
                if (ifs.eof()) {
                    break;
                }
            }
        } else if (c != '\n') {
            break;
        } else
            s += c;

        if (ifs.eof()) {
            break;
        }
    } while (1);
    return s;
}

// -----------------------------------------------------------------------------

/// releases all object data
void Obj_file::release()
{
    _lines.clear();

    _mesh._vertices.clear();
    _mesh._normals.clear();
    _mesh._texCoords.clear();
    _mesh._groups.clear();
    _mesh._materials.clear();
    _mesh._triangles.clear();

    _points.clear();
    _vertexParams.clear();
    _patches.clear();
    _lines.clear();
}

//------------------------------------------------------------------------------

void Obj_file::read_points(std::istream& ifs)
{
    char c;
    std::vector<std::string> VertInfo;

    c = ifs.get();
    // store all strings
    do {
        // strip white spaces
        if (ifs.eof()) {
            goto vinf;
        }
        while(c==' ' || c=='\t') {
            c=ifs.get();
            if (c=='\\') {
                while (ifs.get()!='\n') {
                    if (ifs.eof()) {
                        goto vinf;
                    }
                }
                c=ifs.get();
            }
            if (ifs.eof()) {
                goto vinf;
            }
        }
        std::string s;

        // read vertex info
        while(c!=' '&&c!='\t'&&c!='\n') {
            s+=c;
            c=ifs.get();
            if (ifs.eof()) {
                goto vinf;
            }
        }

        // store string
        VertInfo.push_back(s);
    } while(c!='\n'); // loop till end of line
    vinf: ;
    std::vector<std::string>::iterator it= VertInfo.begin();
    for( ; it != VertInfo.end(); ++it ) {
        int i;
        sscanf(it->c_str(),"%d",&i);
        if (i<0) {
            i=static_cast<int>(_mesh._vertices.size())+i;
        } else
            --i;
        _points.push_back(i);
    }
}

//------------------------------------------------------------------------------

void Obj_file::read_line(std::istream& ifs)
{
    char c;
    std::vector<std::string> VertInfo;

    c = ifs.get();
    // store all strings
    do {
        // strip white spaces
        if (ifs.eof()) {
            goto vinf;
        }
        while(c==' ' || c=='\t') {
            c=ifs.get();
            if (c=='\\') {
                while (ifs.get()!='\n') {
                    if (ifs.eof()) {
                        goto vinf;
                    }
                }
                c=ifs.get();
            }
            if (ifs.eof()) {
                goto vinf;
            }
        }
        std::string s;

        // read vertex info
        while(c!=' '&&c!='\t'&&c!='\n') {
            s+=c;
            c=ifs.get();
            if (ifs.eof()) {
                goto vinf;
            }
        }

        // store string
        VertInfo.push_back(s);
    } while(c!='\n'); // loop till end of line
    vinf: ;
    Line l;

    l.m_Vertices.resize(VertInfo.size());
    l.m_TexCoords.resize(_mesh._texCoords.size());

    std::vector<std::string>::iterator it= VertInfo.begin();
    for( ; it != VertInfo.end(); ++it ) {
        if(HasOnlyVertex(*it)) {
            int i;
            sscanf(it->c_str(),"%d",&i);
            if (i<0) {
                i=static_cast<int>(_mesh._vertices.size())+i;
            } else
                --i;
            l.m_Vertices.push_back(i);
        } else {
            int i,j;
            sscanf(it->c_str(),"%d/%d",&i,&j);
            if (i<0) {
                i=static_cast<int>(_mesh._vertices.size())+i;
            } else
                --i;
            if (j<0) {
                j=static_cast<int>(_mesh._texCoords.size())+j;
            } else
                --j;
            l.m_Vertices.push_back(i);
            l.m_TexCoords.push_back(j);
        }
    }
    _lines.push_back(l);
}

//------------------------------------------------------------------------------

void Obj_file::read_face(std::istream& ifs)
{
    char c;
    std::vector<std::string> VertInfo;

    // store all strings
    do {
        // strip white spaces
        c = ifs.get();
        if (ifs.eof()) {
            goto vinf;
        }
        while(c==' ' || c=='\t') {
            c=ifs.get();
            if (ifs.eof()) {
                goto vinf;
            }
        }
        std::string s;

        // read vertex info
        while(c!=' '&&c!='\t'&&c!='\n') {
            s+=c;
            c=ifs.get();
            if (ifs.eof()) {
                goto vinf;
            }
        }

        // store string
        VertInfo.push_back(s);
    } while(c!='\n'); // loop till end of line

    vinf: ;
    std::vector<int> verts;
    std::vector<int> norms;
    std::vector<int> uvs;
    // split strings into individual indices
    std::vector<std::string>::const_iterator it = VertInfo.begin();
    for( ; it != VertInfo.end(); ++it ) {
        int v, n=0, t=0;

        if(HasOnlyVertex(*it))
            sscanf(it->c_str(),"%d",&v);
        else
            if(MissingUv(*it))
                sscanf(it->c_str(),"%d//%d",&v,&n);
            else
                if(MissingNormal(*it))
                    sscanf(it->c_str(),"%d/%d/",&v,&t);
                else
                    sscanf(it->c_str(),"%d/%d/%d",&v,&t,&n);

        if (v<0) {
            v=static_cast<int>(_mesh._vertices.size())+v+1;
        }
        if (n<0) {
            n=static_cast<int>(_mesh._normals.size())+n+1;
        }
        if (t<0) {
            t=static_cast<int>(_mesh._texCoords.size())+t+1;
        }

        // obj indices are 1 based, change them to zero based indices
        --v; --n; --t;

        /* DEBUG
        if(v == -1)
        {
            int a=0;
            a++;
        }
        */

        verts.push_back(v);
        norms.push_back(n);
        uvs.push_back(t);
    }

    // construct triangles from indices
    for(unsigned i=2;i<verts.size();++i) {
        Face f;

        // construct triangle
        f.v[0] = verts[0];
        f.n[0] = norms[0];
        f.t[0] = uvs[0];
        f.v[1] = verts[i-1];
        f.n[1] = norms[i-1];
        f.t[1] = uvs[i-1];
        f.v[2] = verts[i];
        f.n[2] = norms[i];
        f.t[2] = uvs[i];

        // append to list
        _mesh._triangles.push_back(f);
    }
}

//------------------------------------------------------------------------------

void Obj_file::read_group(std::istream& ifs)
{
    std::string s;
    ifs >> s;
    // ignore the default group, it just contains the verts, normals & uv's for all
    // surfaces. Might as well ignore it!
    if(s!="default") {
        if(_mesh._groups.size()) {
            Group& gr = _mesh._groups[_mesh._groups.size()-1];
            gr.EndFace = static_cast<unsigned int>(_mesh._triangles.size());

            if(gr.m_AssignedMaterials.size())
                gr.m_AssignedMaterials[gr.m_AssignedMaterials.size()-1].m_EndFace
                        = static_cast<unsigned int>(_mesh._triangles.size());
        }
        Group g;
        g.name = s;
        g.StartFace = static_cast<unsigned int>(_mesh._triangles.size());
        _mesh._groups.push_back(g);
    }
}

// -----------------------------------------------------------------------------

void Obj_file::read_material_lib(std::istream& ifs)
{
    std::string file_name;
    ifs >> file_name;

    std::string mtl_file = _path+file_name;

    if( !load_mtl(mtl_file.c_str()) )
        std::cerr << "[WARNING] Unable to load material file: "+mtl_file+"\n";
}

//------------------------------------------------------------------------------

void Obj_file::read_use_material(std::istream& ifs)
{
    std::string mat_name;
    ifs >> mat_name;

    if(_mesh._materials.size())
    {
        // find material index
        unsigned mat=0;
        for( ; mat != _mesh._materials.size(); ++mat ) {
            if(_mesh._materials[mat].name == mat_name) {
                break;
            }
        }

        // if found
        if(mat != _mesh._materials.size())
        {

            // no groups ? add a default group
            if(_mesh._groups.size() == 0)
            {
                Group g;
                g.name = "";      // no name for the default group
                g.StartFace = 0u; // groups every lonely faces
                _mesh._groups.push_back(g);
            }

            Group& gr = _mesh._groups[_mesh._groups.size()-1];
            if(gr.m_AssignedMaterials.size())
                gr.m_AssignedMaterials[gr.m_AssignedMaterials.size()-1].m_EndFace
                        = static_cast<unsigned int>(_mesh._triangles.size());

            MaterialGroup mg;
            mg.m_MaterialIdx = mat;
            mg.m_StartFace = static_cast<unsigned int>(_mesh._triangles.size());
            gr.m_AssignedMaterials.push_back(mg);
        }
    }
}

//------------------------------------------------------------------------------

std::string Obj_file::eat_line(std::istream& ifs)
{
    char c = 'a';
    std::string line;
    while( !ifs.eof() )
    {
         c = ifs.get();
         if(c == '\n') break;

         line += c;
    }
    return line;
}

//------------------------------------------------------------------------------

bool Obj_file::load_file(const std::string& filename)
{
    Loader::Base_loader::load_file(filename);

    // just in case a model is already loaded
    release();

    std::ifstream ifs(filename.c_str());
    if (!ifs) return false;

    // loop through the file to the end
    unsigned line = 0;
    while(!ifs.eof())
    {
        std::string s;

        ifs >> s;

        line++;
        if(s.size() == 0)
            continue;
        else if(s[0]=='#') // comment, skip line
            eat_line(ifs);
        else if(s=="deg")
            std::cerr << "[ERROR] Unable to handle deg yet. Sorry! RB.\n";
        else if(s=="cstype")  // a new group of faces, ie a seperate mesh
            std::cerr << "[ERROR] Unable to handle cstype yet. Sorry! RB.\n";
        else if(s=="bzp") // a new group of faces, ie a seperate mesh
        {
            BezierPatch bzp;
            std::string text = read_chunk(ifs);

            sscanf(text.c_str(),"%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d",
                   &bzp.VertexIndices[0][0],
                   &bzp.VertexIndices[0][1],
                   &bzp.VertexIndices[0][2],
                   &bzp.VertexIndices[0][3],

                   &bzp.VertexIndices[1][0],
                   &bzp.VertexIndices[1][1],
                   &bzp.VertexIndices[1][2],
                   &bzp.VertexIndices[1][3],

                   &bzp.VertexIndices[2][0],
                   &bzp.VertexIndices[2][1],
                   &bzp.VertexIndices[2][2],
                   &bzp.VertexIndices[2][3],

                   &bzp.VertexIndices[3][0],
                   &bzp.VertexIndices[3][1],
                   &bzp.VertexIndices[3][2],
                   &bzp.VertexIndices[3][3]);

            // subtract 1 from all indices
            for(unsigned i=0;i!=4;++i)
                for(unsigned j=0;j!=4;++j)
                    --bzp.VertexIndices[i][j];
        }
        else if(s=="g") // a new group of faces, ie a seperate mesh
            read_group(ifs);
        else if(s=="f"||s=="fo") // face
            read_face(ifs);
        else if(s=="p") // points
            read_points(ifs);
        else if(s=="l") // lines
            read_line(ifs);
        else if(s=="vt") // texture coord
            _mesh._texCoords.push_back( TexCoord(ifs) );
        else if(s=="vn") // normal
            _mesh._normals.push_back( Normal(ifs) );
        else if(s=="v") // vertex
            _mesh._vertices.push_back( Vertex(ifs) );
        else if(s=="vp") // vertex parameter
            _vertexParams.push_back( VertexParam(ifs) );
        else if(s=="mtllib") // material library
            read_material_lib(ifs);
        else if(s=="usemtl") // material to apply
            read_use_material(ifs);
        else if(s=="end"  || s=="parm"|| s=="stech"|| s=="ctech"|| s=="curv"||
                s=="curv2"|| s=="surf"|| s=="bmat" || s=="res"  || s=="sp"  ||
                s=="trim" || s=="hole")
        {

            std::cerr << "[ERROR] Unable to handle " << s << " outside of cstype/end pair\n";
            std::cerr << "[ERROR] Unable to handle cstype yet. Sorry! RB.\n";
            read_chunk(ifs);
        }
        else
        {
            std::string field = s + eat_line(ifs);

            std::cerr << "WARNING line " << line << ": the field '"<< field;
            std::cerr << "' could not be read in file ";
            std::cerr << filename << std::endl;
        }
    }// END WHILE( END_OF_FILE )

    // if groups exist, terminate it.
    if(_mesh._groups.size())
    {
        Group& gr = _mesh._groups[_mesh._groups.size()-1];

        gr.EndFace = static_cast<unsigned int>(_mesh._triangles.size());

        // terminate any assigned materials
        if(gr.m_AssignedMaterials.size())
            gr.m_AssignedMaterials[gr.m_AssignedMaterials.size()-1].m_EndFace
                    = static_cast<unsigned int>(_mesh._triangles.size());
    }

    return true;
}

//------------------------------------------------------------------------------

bool Obj_file::save_file(const std::string& filename)
{
    Loader::Base_loader::save_file(filename);

    // if we have materials in the model, save the mtl file
    if(_mesh._materials.size()) {
        std::string file = filename;
        size_t len = file.size();
        // strip "obj" extension
        while(file[--len] != '.') /*empty*/ ;
        file.resize(len);
        file += ".mtl";
        save_mtl(file.c_str());
    }

    std::ofstream ofs(filename.c_str());
    if( !ofs.is_open() ) return false;

    if(_mesh._groups.size())
    {
        std::vector<Group>::const_iterator itg = _mesh._groups.begin();
        for( ; itg != _mesh._groups.end(); ++itg )
        {
            // need to determine start and end vertex/normal and uv indices
            unsigned int s_vert,e_vert;
            int s_norm,s_uv,e_norm,e_uv;

            DetermineIndexRange(s_vert,e_vert,s_norm,e_norm,s_uv,e_uv,
                                _mesh._triangles.begin() + itg->StartFace,
                                _mesh._triangles.begin() + itg->EndFace);

            // write default group
            ofs << "g default\n";

            // write groups vertices
            WriteArrayRange<Vertex>(ofs,_mesh._vertices,s_vert,e_vert);

            // write groups normals (if present)
            if(e_norm != -1)
                WriteArrayRange<Normal>(ofs,_mesh._normals,s_norm,e_norm);

            // write groups uv coords (if present)
            if(e_uv != -1)
                WriteArrayRange<TexCoord>(ofs,_mesh._texCoords,s_uv,e_uv);

            // write group name
            ofs << "g " << itg->name << std::endl;

            // write triangles in group
            if(itg->m_AssignedMaterials.size())	{
                // write out each material group
                std::vector<MaterialGroup>::const_iterator itmg = itg->m_AssignedMaterials.begin();
                for( ; itmg != itg->m_AssignedMaterials.end(); ++itmg ) {
                    unsigned int mat = itmg->m_MaterialIdx;
                    // write use material flag
                    ofs << "usemtl " << _mesh._materials[mat].name << "\n";

                    WriteArrayRange<Face>(ofs,_mesh._triangles,itmg->m_StartFace,itmg->m_EndFace-1);
                }
            }
            else
                WriteArrayRange<Face>(ofs,_mesh._triangles,itg->StartFace,itg->EndFace-1);
        }
    }
    else
    {
        // all part of default group
        ofs << "g default\n";
        WriteArrayRange<Vertex>(ofs,_mesh._vertices,0,static_cast<unsigned>(_mesh._vertices.size())-1);
        WriteArrayRange<TexCoord>(ofs,_mesh._texCoords,0,static_cast<unsigned>(_mesh._texCoords.size())-1);
        WriteArrayRange<Normal>(ofs,_mesh._normals,0,static_cast<unsigned>(_mesh._normals.size())-1);
        WriteArrayRange<Face>(ofs,_mesh._triangles,0,static_cast<unsigned>(_mesh._triangles.size())-1);
    }
    ofs.close();
    return true;


}

//------------------------------------------------------------------------------

// loads the specified material file
bool Obj_file::load_mtl(const char filename[])
{
    std::ifstream ifs(filename);

    if(!ifs.is_open()) return false;

    Material* pmat=0;
    while(!ifs.eof())
    {
        std::string s;
        ifs >> s;

        // To many discrepancies with obj implems so I lower everything to
        // catch as much as possible symbols
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if(s.size() == 0)
            continue;
        else if(s[0]=='#')
            eat_line(ifs);
        else if(s == "newmtl")
        {
            Material mat;
            ifs >> mat.name;
            _mesh._materials.push_back(mat);
            pmat = &(_mesh._materials[_mesh._materials.size()-1]);
        }
        else if(s == "illum") ifs >> pmat->illum;
        else if(s == "kd") ifs >> pmat->Kd[0] >> pmat->Kd[1] >> pmat->Kd[2];
        else if(s == "ka") ifs >> pmat->Ka[0] >> pmat->Ka[1] >> pmat->Ka[2];
        else if(s == "ks") ifs >> pmat->Ks[0] >> pmat->Ks[1] >> pmat->Ks[2];
        else if(s == "tf") ifs >> pmat->Tf[0] >> pmat->Tf[1] >> pmat->Tf[2];
        else if(s == "d"){ ifs >> pmat->Tf[0]; pmat->Tf[1] = pmat->Tf[2] = pmat->Tf[0]; }
        else if(s == "ni") ifs >> pmat->Ni;
        else if(s == "ns") ifs >> pmat->Ns;
        else if(s == "map_ka") ifs >> pmat->map_Ka;
        else if(s == "map_kd") ifs >> pmat->map_Kd;
        else if(s == "map_ks") ifs >> pmat->map_Ks;
        else if(s == "bump" || s == "map_bump")
            ifs >> pmat->map_Bump;
        else // unknown entry, we skip it
        {
            std::string field = s + eat_line(ifs);

            std::cerr << "WARNING: the material field '"<< field << "' could not be read in file ";
            std::cerr << filename << std::endl;
        }
    }// END WHILE( END_OF_FILE )
    ifs.close();
    return true;
}

// -----------------------------------------------------------------------------

std::ostream& operator << (std::ostream& ofs,const Material& f)
{
    ofs << "newmtl " << f.name << "\n";
    ofs << "illum " << f.illum << "\n";
    ofs << "Kd " << f.Kd[0] << " " << f.Kd[1] << " " << f.Kd[2] << "\n";
    ofs << "Ka " << f.Ka[0] << " " << f.Ka[1] << " " << f.Ka[2] << "\n";
    ofs << "Ks " << f.Ks[0] << " " << f.Ks[1] << " " << f.Ks[2] << "\n";
    ofs << "Tf " << f.Tf[0] << " " << f.Tf[1] << " " << f.Tf[2] << "\n";
    ofs << "Ni " << f.Ni << "\n";
    ofs << "Ns " << f.Ns << "\n";
    if(f.map_Kd.size())
        ofs << "map_Kd " << f.map_Kd << "\n";
    if(f.map_Ka.size())
        ofs << "map_Ka " << f.map_Ka << "\n";
    if(f.map_Ks.size())
        ofs << "map_Ks " << f.map_Ks << "\n";
    if(f.map_Bump.size())
        ofs <<"bump " << f.map_Bump << " -bm " << f.Bm << "\n";
    return ofs;
}

//------------------------------------------------------------------------------

// loads the specified material file
bool Obj_file::save_mtl(const char filename[]) const
{
    std::ofstream ofs(filename);
    if(ofs) {
        std::vector<Material>::const_iterator it = _mesh._materials.begin();
        for( ; it != _mesh._materials.end();++it)
            ofs << *it;
        ofs.close();
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------

void Obj_file::compute_normals()
{
    if(!_mesh._triangles.size()) return;

    if(_mesh._triangles[0].n[0] == -1)
    {
        _mesh._normals.resize(_mesh._vertices.size());


        std::vector<Face>::iterator it = _mesh._triangles.begin();
        for( ; it != _mesh._triangles.end(); ++it ) {

            // use vertex indices for normal indices
            it->n[0] = it->v[0];
            it->n[1] = it->v[1];
            it->n[2] = it->v[2];
        }
    }

    // resize normal array if not present
    ZeroNormals( _mesh._normals );

    // loop through each triangle in face
    std::vector<Face>::const_iterator it = _mesh._triangles.begin();
    for( ; it != _mesh._triangles.end(); ++it ) {
        DoFaceCalc(_mesh._vertices[ it->v[0] ],_mesh._vertices[ it->v[1] ],_mesh._vertices[ it->v[2] ],
                   _mesh._normals[ it->n[0] ],_mesh._normals[ it->n[1] ],_mesh._normals[ it->n[2] ]);
    }
    NormaliseNormals( _mesh._normals );
}

//------------------------------------------------------------------------------

void Obj_file::groups_to_surfaces(std::vector<Surface>& surface_list)
{
    if(_mesh._groups.size()) {
        surface_list.resize(_mesh._groups.size());

        std::vector<Surface>::iterator its = surface_list.begin();

        std::vector<Group>::const_iterator itg = _mesh._groups.begin();
        for( ; itg != _mesh._groups.end(); ++itg, ++its ) {

            // need to determine start and end vertex/normal and uv indices
            unsigned int s_vert,e_vert;
            int s_norm,s_uv,e_norm,e_uv;

            DetermineIndexRange(s_vert,e_vert,s_norm,e_norm,s_uv,e_uv,
                                _mesh._triangles.begin() + itg->StartFace,
                                _mesh._triangles.begin() + itg->EndFace);

            // set file pointer
            its->m_pFile = this;

            // set name
            its->name = itg->name;

            // copy material groups for surface
            its->m_AssignedMaterials = itg->m_AssignedMaterials;

            // make material groups relative to material start
            std::vector<MaterialGroup>::iterator itmg = its->m_AssignedMaterials.begin();
            for( ;itmg != its->m_AssignedMaterials.end(); ++itmg ) {
                itmg->m_StartFace -= itg->StartFace;
                itmg->m_EndFace   -= itg->StartFace;
            }

            // resize triangles
            its->m_Triangles.resize( itg->EndFace - itg->StartFace );

            std::vector<Face>::iterator ito = its->m_Triangles.begin();
            std::vector<Face>::const_iterator it = _mesh._triangles.begin() + itg->StartFace;
            std::vector<Face>::const_iterator end = _mesh._triangles.begin() + itg->EndFace;

            for( ; it != end; ++it, ++ito ) {
                for(int i=0;i!=3;++i) {
                    ito->v[i] = it->v[i]-s_vert;
                    ito->n[i] = (e_norm==-1) ? -1 : (it->n[i]-s_norm);
                    ito->t[i] = (e_uv==-1) ? -1 : (it->t[i]-s_uv);
                }
            }

            // copy over vertices
            CopyArray< Vertex >( its->m_Vertices, _mesh._vertices, s_vert, e_vert );

            // copy over normals
            if(e_norm!=-1)
                CopyArray< Normal >( its->m_Normals, _mesh._normals, s_norm, e_norm );

            // copy over tex coords
            if(e_uv!=-1)
                CopyArray< TexCoord >( its->m_TexCoords, _mesh._texCoords, s_uv, e_uv );
        }
    }
    else {
        surface_list.resize(1);
        surface_list[0].m_Vertices = _mesh._vertices;
        surface_list[0].m_Normals = _mesh._normals;
        surface_list[0].m_TexCoords = _mesh._texCoords;
        surface_list[0].m_Triangles = _mesh._triangles;
        surface_list[0].m_pFile = this;
        surface_list[0].name = "default";
    }
}

//------------------------------------------------------------------------------

void Obj_file::groups_to_vertex_arrays(std::vector<VertexBuffer>& surface_list)
{
    // first split into surfaces
    std::vector<Surface> surfaces;
    groups_to_surfaces(surfaces);

    // now convert each surface into a vertex array
    surface_list.resize(surfaces.size());

    std::vector<VertexBuffer>::iterator itb = surface_list.begin();
    std::vector<Surface>::iterator its = surfaces.begin();

    for( ; itb != surface_list.end(); ++itb, ++its ) {

        // set name
        itb->name = its->name;

        // set file
        itb->m_pFile = this;

        // copy material assignments
        itb->m_AssignedMaterials = its->m_AssignedMaterials;

        // determine new vertex and index arrays.
        std::vector<Face>::iterator itf = its->m_Triangles.begin();
        for( ; itf != its->m_Triangles.end(); ++itf ) {
            for(int i=0;i!=3;++i) {

                const Vertex* v   = &its->m_Vertices[ itf->v[i] ];
                const Normal* n   = 0;
                if(itf->n[i]!=-1)
                    n = &its->m_Normals[ itf->n[i] ];
                const TexCoord* t = 0;
                if(itf->t[i]!=-1)
                    t = &its->m_TexCoords[ itf->t[i] ];

                unsigned int idx = 0;
                if(n&&t) {
                    std::vector<Vertex>::const_iterator itv = itb->m_Vertices.begin();
                    std::vector<Normal>::const_iterator itn = itb->m_Normals.begin();
                    std::vector<TexCoord>::const_iterator itt = itb->m_TexCoords.begin();
                    for( ; itv != itb->m_Vertices.end(); ++itv,++itn,++itt,++idx )
                        if( *v == *itv && *n == *itn && *t == *itt )
                            break;
                    if( itv == itb->m_Vertices.end() ) {
                        itb->m_Vertices.push_back(*v);
                        itb->m_Normals.push_back(*n);
                        itb->m_TexCoords.push_back(*t);
                    }
                    itb->m_Indices.push_back(idx);
                }
                else if(n) {
                    std::vector<Vertex>::const_iterator itv = itb->m_Vertices.begin();
                    std::vector<Normal>::const_iterator itn = itb->m_Normals.begin();
                    for( ; itv != itb->m_Vertices.end(); ++itv,++itn,++idx )
                        if( *v == *itv && *n == *itn )
                            break;
                    if( itv == itb->m_Vertices.end() ) {
                        itb->m_Vertices.push_back(*v);
                        itb->m_Normals.push_back(*n);
                    }
                    itb->m_Indices.push_back(idx);
                }
                else if(t) {
                    std::vector<Vertex>::const_iterator itv   = itb->m_Vertices.begin();
                    std::vector<TexCoord>::const_iterator itt = itb->m_TexCoords.begin();
                    for( ; itv != itb->m_Vertices.end(); ++itv,++itt,++idx )
                        if( *v == *itv && *t == *itt )
                            break;
                    if( itv == itb->m_Vertices.end() ) {
                        itb->m_Vertices.push_back(*v);
                        itb->m_TexCoords.push_back(*t);
                    }
                    itb->m_Indices.push_back(idx);
                }
                else {
                    std::vector<Vertex>::const_iterator itv = itb->m_Vertices.begin();
                    for( ; itv != itb->m_Vertices.end(); ++itv,++idx )
                        if( *v == *itv )
                            break;
                    itb->m_Indices.push_back(idx);
                }
            }
        }
    }
}

//------------------------------------------------------------------------------

void Obj_file::get_mesh(Loader::Abs_mesh& mesh)
{
    mesh._vertices.clear();
    mesh._vertices.resize( _mesh._vertices.size() );
    for (unsigned i=0; i<_mesh._vertices.size(); ++i){
        Vertex v = _mesh._vertices[i];
        mesh._vertices[i] = Loader::Vertex( v.x, v.y, v.z );
    }
    mesh._normals.clear();
    mesh._normals.resize( _mesh._normals.size() );
    for (unsigned i=0; i<_mesh._normals.size(); ++i){
        Normal n = _mesh._normals[i];
        mesh._normals[i] = Loader::Normal( n.x, n.y, n.z );
    }
    mesh._texCoords.clear();
    mesh._texCoords.resize( _mesh._texCoords.size() );
    for (unsigned i=0; i<_mesh._texCoords.size(); ++i){
        TexCoord t = _mesh._texCoords[i];
        mesh._texCoords[i] = Loader::TexCoord( t.u, t.v );
    }
    mesh._triangles.clear();
    mesh._triangles.resize( _mesh._triangles.size() );
    for (unsigned i=0; i<_mesh._triangles.size(); ++i){
        Face f = _mesh._triangles[i];
        Loader::Tri_face F;
        F.v[0] = f.v[0]; F.v[1] = f.v[1]; F.v[2] = f.v[2];
        F.n[0] = f.n[0]; F.n[1] = f.n[1]; F.n[2] = f.n[2];
        F.t[0] = f.t[0]; F.t[1] = f.t[1]; F.t[2] = f.t[2];
        mesh._triangles[i] = F;
    }
    mesh._materials.clear();
    mesh._materials.resize( _mesh._materials.size() );
    for (unsigned i=0; i<_mesh._materials.size(); ++i){
        Material m = _mesh._materials[i];
        Loader::Material M;
        M._name = m.name;
        M._illum = m.illum;
        M._Ka[0] = m.Ka[0]; M._Ka[1] = m.Ka[1]; M._Ka[2] = m.Ka[2]; M._Ka[3] = m.Ka[3];
        M._Kd[0] = m.Kd[0]; M._Kd[1] = m.Kd[1]; M._Kd[2] = m.Kd[2]; M._Kd[3] = m.Kd[3];
        M._Ks[0] = m.Ks[0]; M._Ks[1] = m.Ks[1]; M._Ks[2] = m.Ks[2]; M._Ks[3] = m.Ks[3];
        M._Tf[0] = m.Tf[0]; M._Tf[1] = m.Tf[1]; M._Tf[2] = m.Tf[2];
        M._Ni = m.Ni;
        M._Ns = m.Ns;
        M._map_Ka = m.map_Ka;
        M._map_Kd = m.map_Kd;
        M._map_Ks = m.map_Ks;
        M._map_Bump = m.map_Bump;
        M._Bm = m.Bm;
        mesh._materials[i] = M;
    }
    mesh._groups.clear();
    mesh._groups.resize( _mesh._groups.size() );
    for (unsigned i=0; i<_mesh._groups.size(); ++i){
        Group g = _mesh._groups[i];
        Loader::Group G;
        G._start_face = g.StartFace;
        G._end_face = g.EndFace;
        //G.start_point = g.StartPoint;
        //G._end_point = g.EndPoint;
        G._name = g.name;
        G._assigned_mats.resize( g.m_AssignedMaterials.size() );
        for (unsigned j=0; j<g.m_AssignedMaterials.size(); ++j){
            MaterialGroup mg = g.m_AssignedMaterials[j];
            Loader::MaterialGroup MG;
            MG._start_face = mg.m_StartFace;
            MG._end_face = mg .m_EndFace;
            //MG._start_point = mg.StartPoint;
            //MG._end_point = mg.EndPoint;
            MG._material_idx = mg.m_MaterialIdx;
            G._assigned_mats[j] = MG;
        }
        mesh._groups[i] = G;
    }
}

//------------------------------------------------------------------------------

void Obj_file::set_mesh(const Loader::Abs_mesh& mesh)
{
    _mesh._vertices.clear();
    _mesh._vertices.resize( mesh._vertices.size() );
    for (unsigned i=0; i<mesh._vertices.size(); ++i){
        Loader::Vertex v = mesh._vertices[i];
        _mesh._vertices[i] = Vertex( v.x, v.y, v.z );
    }
    _mesh._normals.clear();
    _mesh._normals.resize( mesh._normals.size() );
    for (unsigned i=0; i<mesh._normals.size(); ++i){
        Loader::Normal n = mesh._normals[i];
        _mesh._normals[i] = Normal( n.x, n.y, n.z );
    }
    _mesh._texCoords.clear();
    _mesh._texCoords.resize( mesh._texCoords.size() );
    for (unsigned i=0; i<mesh._texCoords.size(); ++i){
        Loader::TexCoord t = mesh._texCoords[i];
        _mesh._texCoords[i] = TexCoord( t.u, t.v );
    }
    _mesh._triangles.clear();
    _mesh._triangles.resize( mesh._triangles.size() );
    for (unsigned i=0; i<mesh._triangles.size(); ++i){
        Loader::Tri_face f = mesh._triangles[i];
        Face F;
        F.v[0] = f.v[0]; F.v[1] = f.v[1]; F.v[2] = f.v[2];
        F.n[0] = f.n[0]; F.n[1] = f.n[1]; F.n[2] = f.n[2];
        F.t[0] = f.t[0]; F.t[1] = f.t[1]; F.t[2] = f.t[2];
        _mesh._triangles[i] = F;
    }
    _mesh._materials.clear();
    _mesh._materials.resize( mesh._materials.size() );
    for (unsigned i=0; i<mesh._materials.size(); ++i){
        Loader::Material m = mesh._materials[i];
        Material M;
        M.name = m._name;
        M.illum = m._illum;
        M.Ka[0] = m._Ka[0]; M.Ka[1] = m._Ka[1]; M.Ka[2] = m._Ka[2]; M.Ka[3] = m._Ka[3];
        M.Kd[0] = m._Kd[0]; M.Kd[1] = m._Kd[1]; M.Kd[2] = m._Kd[2]; M.Kd[3] = m._Kd[3];
        M.Ks[0] = m._Ks[0]; M.Ks[1] = m._Ks[1]; M.Ks[2] = m._Ks[2]; M.Ks[3] = m._Ks[3];
        M.Tf[0] = m._Tf[0]; M.Tf[1] = m._Tf[1]; M.Tf[2] = m._Tf[2];
        M.Ni = m._Ni;
        M.Ns = m._Ns;
        M.map_Ka = m._map_Ka;
        M.map_Kd = m._map_Kd;
        M.map_Ks = m._map_Ks;
        M.map_Bump = m._map_Bump;
        M.Bm = m._Bm;
        _mesh._materials[i] = M;
    }
    _mesh._groups.clear();
    _mesh._groups.resize( mesh._groups.size() );
    for (unsigned i=0; i<mesh._groups.size(); ++i){
        Loader::Group g = mesh._groups[i];
        Group G;
        G.StartFace = g._start_face;
        G.EndFace = g._end_face;
        //G.StartPoint = g.start_point;
        //G.EndPoint = g._end_point;
        G.name = g._name;
        G.m_AssignedMaterials.resize( g._assigned_mats.size() );
        for (unsigned j = 0; j < g._assigned_mats.size(); ++j){
            Loader::MaterialGroup mg = g._assigned_mats[j];
            MaterialGroup MG;
            MG.m_StartFace = mg._start_face;
            MG.m_EndFace = mg ._end_face;
            //MG.StartPoint = mg._start_point;
            //MG.EndPoint = mg._end_point;
            MG.m_MaterialIdx = mg._material_idx;
            G.m_AssignedMaterials[j] = MG;
        }
        _mesh._groups[i] = G;
    }
}

// END CLASS Obj_File ==========================================================


// =============================================================================
} // namespace Obj_Loader
// =============================================================================

