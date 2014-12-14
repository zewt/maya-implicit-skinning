//--------------------------------------------------------------------------------
///	\file	obj_loader.hpp
///	\author	Rob Bateman [mailto:robthebloke@hotmail.com] [http://robthebloke.org]
///	\date	11-4-05
///	\brief	A C++ loader for the alias wavefront obj file format. This loader is
///			intended to be as complete as possible (ie, it handles everything that
///			Maya can spit at it, as well as all the parametric curve and surface
///			stuff not supported by Maya's objexport plugin).
///
///			If you want to add obj import/export capabilities to your applications,
///			the best method is probably to derive a class from Obj::File and then
///			use the data in the arrays provided (or fill the data arrays with the
///			correct info, then export).
///
///			In addition to simply accessing the data arrays, you can also overload
///			the functions, PreSave,PostSave,PreLoad,PostLoad and HandleUnknown to
///			extend the capabilities of the file format (if you really think that
///			is wise!).
///
///			Generally speaking, the obj file format is actually very very complex.
///			As a result, not all of the Curve and Surface types are fully supported
///			in the openGL display parts of this code. Bezier, Bspline & NURBS curves
///			and surface should be fine. Cardinal, taylor and basis matrix types
///			will be displayed as CV's / HULLS only. Trim curves and holes don't get
///			displayed either. I'll leave that as an excersice for the reader.
///
///			The material lib files (*.mtl) are also supported and parsed, this
///			includes a user extensible interface to load the image files as textures.
///			See the texture manager example for more info.
///
///			One nice aspect of this code is the ability to calculate normals for
///			the polygonal surfaces, and the ability to convert the objfile into
///			surfaces and /or vertex arrays. Hopefully it may be useful to someone.
///
///			If you wish to use this code within a game, I **STRONGLY** RECOMMEND
///			converting the data to Vertex Arrays and saving as your own custom
///			file format. The code to handle your own objects will be substantially
///			more efficient than this code which is designed for completeness, not
///			speed! See the obj->game converter for a quick example of this.
///
// TODO: Handle the smoothing groups ('s off' 's 1' 's 2' 's n'...)
// delete sscanf usage in the methods
///
///
//--------------------------------------------------------------------------------

#ifndef OBJ_LOADER_HPP__
#define OBJ_LOADER_HPP__

#include <fstream>
#include <string>
#include <set>
#include <map>
#include <vector>
#include <iostream>

#define FCOMPARE(x,y) (((x)-0.0001f)<(y) && ((x)+0.00001f)>(y))

#include "loader.hpp"

/**
  @namespace Obj_loader
  @brief Data structures and class for storing/parsing Wavefront obj files


*/
// =============================================================================
namespace Obj_loader {
// =============================================================================

// forward declarations --------------------------------------------------------

class Obj_file;

// structs internal to obj loader ----------------------------------------------

struct Vertex {
    float x; ///< x component of point
    float y; ///< y component of point
    float z; ///< z component of point


    /// ctors
    Vertex()
        : x(0),y(0),z(0) {
    }
    Vertex(float _x, float _y, float _z)
        : x(_x),y(_y),z(_z) {
    }
    Vertex( const Vertex& v )
        : x(v.x),y(v.y),z(v.z) {
    }
    /// ctor from stream
    Vertex( std::istream& ifs ) {
        ifs >> x >> y >> z;
    }


    friend std::ostream& operator<< ( std::ostream& ofs, const Vertex& v ) {
        return ofs << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }
    bool operator== ( const Vertex& v ) const {
        return FCOMPARE(x,v.x) && FCOMPARE(y,v.y) && FCOMPARE(z,v.z);
    }
};

// -----------------------------------------------------------------------------

struct VertexParam {
    float u; ///< x component of point
    float v; ///< y component of point
    float w; ///< z component of point


    /// ctors
    VertexParam()
        : u(0),v(0),w(0) {
    }
    VertexParam(float _u, float _v, float _w)
        : u(_u),v(_v),w(_w) {
    }
    VertexParam(const VertexParam& v_)
        : u(v_.u),v(v_.v),w(v_.w) {
    }
    /// ctor from input stream
    VertexParam( std::istream& ifs );


    friend std::ostream& operator<< ( std::ostream& ofs, const VertexParam& v_ ) {
        return ofs << "vp " << v_.u << " " << v_.v << " " << v_.w << std::endl;
    }
    bool operator== ( const VertexParam& v_ ) const {
        return FCOMPARE(u,v_.u) && FCOMPARE(v,v_.v) && FCOMPARE(w,v_.w) ;
    }
};

// -----------------------------------------------------------------------------

struct Normal {
    float x; ///< x component of vector
    float y; ///< y component of vector
    float z; ///< z component of vector


    /// ctors
    Normal()
        : x(0),y(0),z(0) {
    }
    Normal(float _x, float _y, float _z)
        : x(_x),y(_y),z(_z) {
    }
    Normal(const Normal& n)
        : x(n.x),y(n.y),z(n.z) {
    }
    /// ctor from input stream
    Normal( std::istream& ifs ) {
        ifs >> x >> y >> z;
    }


    friend std::ostream& operator<< ( std::ostream& ofs, const Normal& n ) {
        return ofs << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
    }
    bool operator== ( const Normal& n ) const {
        return FCOMPARE(x,n.x) && FCOMPARE(y,n.y) && FCOMPARE(z,n.z);
    }

};

// -----------------------------------------------------------------------------

struct TexCoord {
    float u; ///< u tex coord
    float v; ///< v tex coord


    /// ctors
    TexCoord()
        : u(0),v(0) {
    }
    TexCoord(float _u, float _v)
        : u(_u),v(_v) {
    }
    TexCoord(const TexCoord& uv)
        : u(uv.u),v(uv.v) {
    }
    /// ctor from stream
    TexCoord( std::istream& ifs ) {
        ifs >> u >> v;
    }


    friend std::ostream& operator<< ( std::ostream& ofs, const TexCoord& uv ) {
        return ofs << "vt " << uv.u << " " << uv.v << std::endl;
    }
    bool operator== ( const TexCoord& uv ) const {
        return FCOMPARE(u,uv.u) && FCOMPARE(v,uv.v);
    }
};

// -----------------------------------------------------------------------------

struct Line {
    /// the material applied to this line
    unsigned short m_Material;
    /// the group to which this line belongs
    unsigned short m_Group;
    /// the vertex indices for the line
    std::vector<unsigned> m_Vertices;
    /// the texture coord indices for the line
    std::vector<unsigned> m_TexCoords;


    /// ctors
    Line()
        : m_Vertices(), m_TexCoords() {
    }
    Line(const Line& l)
        : m_Vertices(l.m_Vertices), m_TexCoords(l.m_TexCoords) {
    }


    friend std::ostream& operator<< ( std::ostream& ofs, const Line& l ) {
        ofs << "l";
        if ( l.m_TexCoords.size() ) {
            std::vector<unsigned>::const_iterator itv= l.m_Vertices.begin();
            std::vector<unsigned>::const_iterator itt= l.m_TexCoords.begin();
            for( ; itv != l.m_Vertices.end();++itv) {
                ofs << " " << *itv << "/" << *itt;
            }
            ofs << "\n";
        } else {
            std::vector<unsigned>::const_iterator itv= l.m_Vertices.begin();
            for( ; itv != l.m_Vertices.end();++itv) {
                ofs << " " << *itv ;
            }
            ofs << "\n";
        }
        return ofs;
    }
};

// -----------------------------------------------------------------------------

struct Face {
    unsigned v[3]; ///< vertex indices for the triangle
    int n[3];      ///< normal indices for the triangle
    int t[3];      ///< texture coordinate indices for the triangle


    /// ctor
    Face() {
        v[0] = v[1] = v[2] =  0;
        // -1 indicates not used
        n[0] = n[1] = n[2] = -1;
        t[0] = t[1] = t[2] = -1;
    }


    friend std::ostream& operator<< ( std::ostream& ofs,const Face& f );
};

// -----------------------------------------------------------------------------

struct Material {
    /// material name
    std::string name;
    /// don't know :| Seems to always be 4
    int illum;
    float Ka[4]; ///< ambient
    float Kd[4]; ///< diffuse
    float Ks[4]; ///< specular
    float Tf[3]; ///< transparency
    float Ni;    ///< intensity
    float Ns;    ///< specular power
    std::string map_Ka;   ///< ambient texture map
    std::string map_Kd;   ///< diffuse texture map
    std::string map_Ks;   ///< specular texture map
    std::string map_Bump; ///< bump texture map
    /// bump map depth. Only used if bump is relevent.
    float Bm;


    /// ctors
    Material() : name(), illum(4), Ni(1), Ns(10), map_Ka(), map_Kd(), map_Ks(), map_Bump(), Bm(1) {
        Ka[0] = Ka[1] = Ka[2] = Kd[0] = Kd[1] = Kd[2] = Ks[0] = Ks[1] = Ks[2] = 0;
        Ka[3] = Kd[3] = Ks[3] = 1;
        Tf[0] = Tf[1] = Tf[2] = 1;
        illum = 4;
        Ni = Ns = 0.5f;
        map_Ka = map_Kd = map_Ks = map_Bump = "";
        Bm = 0;
    }
    Material(const Material& mat) {
        Ka[0] = mat.Ka[0]; Ka[1] = mat.Ka[1]; Ka[2] = mat.Ka[2]; Ka[3] = mat.Ka[3];
        Kd[0] = mat.Kd[0]; Kd[1] = mat.Kd[1]; Kd[2] = mat.Kd[2]; Kd[3] = mat.Kd[3];
        Ks[0] = mat.Ks[0]; Ks[1] = mat.Ks[1]; Ks[2] = mat.Ks[2]; Ks[3] = mat.Ks[3];
        Tf[0] = mat.Tf[0]; Tf[1] = mat.Tf[1]; Tf[2] = mat.Tf[2];
        Ni = mat.Ni;
        Ns = mat.Ns;
        name = mat.name;
        map_Ka = mat.map_Ka;
        map_Kd = mat.map_Kd;
        map_Ks = mat.map_Ks;
        map_Bump = mat.map_Bump;
        illum = mat.illum;
        Bm = mat.Bm;
    }
    /// dtor
    ~Material(){
    }


    friend std::ostream& operator<< ( std::ostream& ofs, const Material& f );
};

// -----------------------------------------------------------------------------

struct MaterialGroup {
    /// the material applied to a set of faces
    unsigned m_MaterialIdx;
    /// the starting index of the face to which the material is applied
    unsigned m_StartFace;
    /// the ending index of the face to which the material is applied
    unsigned m_EndFace;
    /// start index for points to which the material is applied
    unsigned StartPoint;
    /// end index for points to which the material is applied
    unsigned EndPoint;

    /// ctors
    MaterialGroup() : m_MaterialIdx(0), m_StartFace(0), m_EndFace(0) {}
    MaterialGroup(const MaterialGroup& mg) :
        m_MaterialIdx(mg.m_MaterialIdx),
        m_StartFace(mg.m_StartFace),
        m_EndFace(mg.m_EndFace)
    {    }
};

// -----------------------------------------------------------------------------

struct Group {
    /// start index for faces in the group (surface)
    unsigned StartFace;
    /// end index for faces in the group (surface)
    unsigned EndFace;
    /// start index for points in the group (surface)
    unsigned StartPoint;
    /// end index for points in the group (surface)
    unsigned EndPoint;
    /// name of the group
    std::string name;
    /// a set of material groupings within this surface. ie, which
    /// materials are assigned to which faces within this group.
    std::vector<MaterialGroup> m_AssignedMaterials;


    /// ctors
    Group()
        : StartFace(0),EndFace(0),name(""),m_AssignedMaterials() {
    }
    Group(const Group& g)
        : StartFace(g.StartFace), EndFace(g.EndFace), name(g.name), m_AssignedMaterials(g.m_AssignedMaterials) {
    }
};

// -----------------------------------------------------------------------------

struct BezierPatch {
    /// the material applied to this patch
    unsigned short m_Material;
    /// the group to which this patch belongs
    unsigned short m_Group;
    /// a set of 16 vertex indices
    int VertexIndices[4][4];
    /// an array of vertices/normals/texcoords. Each vertex has 8 floats
    float* VertexData;
    /// an array of vertices/normals/texcoords. Each vertex has 8 floats
    float* BlendFuncs;
    /// an array of vertex indices for triangle strips
    unsigned* IndexData;
    /// the level of detail.
    unsigned LOD;


    /// ctors
    BezierPatch();
    BezierPatch(const BezierPatch& bzp){
        // copy over all indices
        for(int i=0;i!=4;++i)
            for(int j=0;j!=4;++j)
                VertexIndices[i][j] = bzp.VertexIndices[i][j];
        /// prevents SetLOD() from attempting to delete invalid data
        IndexData=0;
        VertexData=0;
        BlendFuncs=0;
        /// set level of detail of surface
        SetLod(bzp.LOD);
    }
    /// dtor
    ~BezierPatch(){
        delete [] IndexData;
        delete [] VertexData;
        delete [] BlendFuncs;
        IndexData=0;
        VertexData=0;
        BlendFuncs=0;
    }


    /// sets the level of detail and does a bit of internal caching to
    /// speed things up a little.
    void SetLod(unsigned new_lod){
        delete [] VertexData;
        delete [] BlendFuncs;
        delete [] IndexData;
        LOD = new_lod;
        // allocate new blend funcs array. This just caches the values for tesselation
        BlendFuncs = new float[4*(LOD+1)];
        float* ptr = BlendFuncs;
        for(unsigned i=0;i<=LOD;++i) {
            float t = static_cast<float>(i/LOD);
            float t2 = t*t;
            float t3 = t2*t;
            float it =1.0f-t;
            float it2 = it*it;
            float it3 = it2*it;

            *ptr = t3; ++ptr;
            *ptr = 3*it*t2; ++ptr;
            *ptr = 3*it2*t; ++ptr;
            *ptr = it3; ++ptr;

            // calculate texture coordinates since they never change
            {

            }
            // calculate texture coordinates since they never change
            {

            }
        }

        // allocate vertex data array
        VertexData = new float[ 8*(LOD+1)*(LOD+1) ];

        // allocate indices for triangle strips to render the patch
        IndexData = new unsigned [ (LOD+1)*LOD*2 ];

        {
            // calculate the vertex indices for the triangle strips.
            unsigned *iptr = IndexData;
            unsigned *end = IndexData + (LOD+1)*LOD*2;
            unsigned ii=0;
            for( ; iptr != end; ++ii) {
                *iptr = ii;       ++iptr;
                *iptr = ii+LOD+1; ++iptr;
            }
        }
    }
    friend std::ostream& operator<< ( std::ostream& ofs, const BezierPatch& bzp ) {
        ofs << "bzp ";
        for(int i=0;i!=4;++i)
            for(int j=0;j!=4;++j)
                ofs << " " << bzp.VertexIndices[i][j];
        return ofs << "\n";
    }
};

// -----------------------------------------------------------------------------

/**
  @class Wavefront_mesh
  @brief Internal representation of a mesh for the Obj_file class
  The class Obj_file in charge of loading '.obj' files use this structure
  to store the mesh once the file is parsed
  @see Obj_file
*/
struct Wavefront_mesh {
    std::vector<Vertex>   _vertices;  ///< the vertices in the .obj
    std::vector<Normal>   _normals;   ///< the normals from the .obj
    std::vector<TexCoord> _texCoords; ///< the tex coords from the .obj
    std::vector<Face>     _triangles; ///< the triangles in the .obj
    std::vector<Group>    _groups;    ///< the groups in the .obj
    std::vector<Material> _materials; ///< the materials from the .mtl
};

// -----------------------------------------------------------------------------

/// the obj file can be split into seperate surfaces
/// where all indices are relative to the data in the surface,
/// rather than all data in the obj file.
struct Surface {
    friend class Obj_file ;
public:

    /// ctor
    Surface();

    /// copy ctor
    Surface(const Surface& surface);

    /// this function will generate vertex normals for the current
    /// surface and store those within the m_Normals array
    void CalculateNormals();

public:

    /// the name of the surface
    std::string name;

    /// the vertices in the obj file
    std::vector<Vertex>   m_Vertices;
    /// the normals from the obj file
    std::vector<Normal>   m_Normals;
    /// the tex coords from the obj file
    std::vector<TexCoord> m_TexCoords;
    /// the triangles in the obj file
    std::vector<Face>     m_Triangles;
    /// the lines in the obj file
    std::vector<Line>     m_Lines;
    /// the points in the obj file
    std::vector<unsigned> m_Points;
    /// a set of material groupings within this surface. ie, which
    /// materials are assigned to which faces within this group.
    std::vector<MaterialGroup> m_AssignedMaterials;


private:
    /// pointer to file to access material data
    Obj_file* m_pFile;
};

//------------------------------------------------------------------------------

struct GL_Line {
    struct {
        unsigned int numVerts:16;
        unsigned int hasUvs:1;
        unsigned int material:15;
    };
    /// the line indices in the obj file
    std::vector<unsigned int> m_Indices;
};

//------------------------------------------------------------------------------

/// The obj file can be split into seperate vertex arrays,
/// ie each group is turned into a surface which uses a
/// single index per vertex rather than seperate vertex,
/// normal and uv indices.
struct VertexBuffer {
    friend class Obj_file ;
public:

    /// ctor
    VertexBuffer();

    /// copy ctor
    VertexBuffer(const VertexBuffer& surface) ;

    /// this function will generate vertex normals for the current
    /// surface and store those within the m_Normals array
    void CalculateNormals();

public:

    /// the name of the surface
    std::string name;

    /// the vertices in the obj file
    std::vector<Vertex>   m_Vertices;
    /// the normals from the obj file
    std::vector<Normal>   m_Normals;
    /// the tex coords from the obj file
    std::vector<TexCoord> m_TexCoords;
    /// the triangles in the obj file
    std::vector<unsigned int> m_Indices;
    /// a set of material groupings within this surface. ie, which
    /// materials are assigned to which faces within this group.
    std::vector<MaterialGroup> m_AssignedMaterials;
    /// the lines in the obj file.
    std::vector<GL_Line> m_Lines;

private:
    /// pointer to file to access material data
    Obj_file* m_pFile;
};

//------------------------------------------------------------------------------

/** @class Obj_File
    @brief main interface for an alias wavefront obj file.

    Note on the data structure :
    The provided list of data  (i.e : _vertices, _VertexParams, _normals,
    _texCoords, _points, _lines, _triangles, _groups, _mat_groups etc.)
    respect the .obj file layout. There is no garante that a list
    reference all the elements of another list.

    In other words : _triangles does not necessarily points to every vertices
    indices if there are lonely vertices. Another example : some groups don't
    have materials. this often means the material of the previous group should
    be used. We don't make that asumption and leave it at the user discretion,
    thus some group's material are empty.

    for faces without groups we make an exception and create  default group
    with name "" in order to reference every faces.

    _triangles layout is the same as the obj layout. Material groups are not
    necessarily contigus in _triangles. Therefore when rendering, doing a loop
    over the material groups doesn't garantee to be efficient as one material
    might be activated and desactivated several time.

*/
class Obj_file : public Loader::Base_loader {
    // vertex buffer needs to access custom materials
    friend struct VertexBuffer;
    // surfaces may also need to access custom materials if split.
    friend struct Surface;
public:

    Obj_file(){}

    Obj_file(const std::string& filename) :
        Loader::Base_loader(filename)
    {
        load_file(filename);
    }

    ~Obj_file(){ release(); }

    /// The loader type
    Loader::Loader_t type() const { return Loader::OFF; }

    bool import_file(const std::string& file_path){ return load_file(file_path); }
    bool export_file(const std::string& file_path){ return save_file(file_path); }

    /// Obj files have no animation frames
    void get_anims(std::vector<Loader::Base_anim_eval*>& anims) const { anims.clear(); }


    // ----------------------------------------------------------------------------------------------------------
    // TODO: delete these
    bool load_file(const std::string& file_name);
    bool save_file(const std::string& file_name);
    // ----------------------------------------------------------------------------------------------------------

    /// transform internal representation into generic representation
    void get_mesh(Loader::Abs_mesh& mesh);
    /// transform generic representation into internal representation
    void set_mesh(const Loader::Abs_mesh& mesh);

    /// releases all object data
    void release();

private:

    // -------------------------------------------------------------------------
    /// @name class tools
    // -------------------------------------------------------------------------

    /// loads the specified material file
    bool load_mtl(const char filename[]);

    /// saves the mtl file
    bool save_mtl(const char filename[]) const;

    /// this function will generate vertex normals for the current
    /// surface and store those within the m_Normals array
    void compute_normals();

    /// splits the obj file into seperate surfaces based upon object grouping.
    /// The returned list of surfaces will use indices relative to the start of
    /// this surface.
    void groups_to_surfaces(std::vector<Surface>& surface_list);

    /// splits the obj file into sets of vertex arrays for quick rendering.
    void groups_to_vertex_arrays(std::vector<VertexBuffer>& surface_list);

    /// Eat the current line from the stream untill the character new line
    /// is reached
    /// @return The string of all eaten characters
    std::string eat_line(std::istream& ifs);

    std::string read_chunk(std::istream& ifs);

    /// utility function
    void read_points(std::istream&);

    /// utility function to parse
    void read_line(std::istream&);

    /// utility function to parse a face
    void read_face(std::istream&);

    /// a utility function to parse a group
    void read_group(std::istream&);

    /// a utility function to parse material file
    void read_material_lib(std::istream& ifs);

    /// a utility function to parse a use material statement
    void read_use_material(std::istream& ifs);

     // ------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    /// Obj representation of the mesh (or multiples meshes)
    Wavefront_mesh _mesh;
    std::vector<VertexParam> _vertexParams;
    std::vector<BezierPatch> _patches;
    std::vector<unsigned>    _points;
    std::vector<Line>        _lines;
};

//------------------------------------------------------------------------------


}
// END Obj NAMESPACE ===========================================================

#endif // OBJ_LOADER_HPP__
