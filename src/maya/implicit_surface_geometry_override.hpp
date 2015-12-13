#ifndef IMPLICIT_SURFACE_GEOMETRY_OVERRIDE_HPP
#define IMPLICIT_SURFACE_GEOMETRY_OVERRIDE_HPP

#include <maya/MPxGeometryOverride.h>

#include "marching_cubes.hpp"

struct ImplicitSurfaceGeometryOverrideSource
{
    virtual const MeshGeom &get_mesh_geometry() = 0;
};

class ImplicitSurfaceGeometryOverride: public MHWRender::MPxGeometryOverride
{
public:
    static MString drawRegistrantId;
    static MString drawDbClassification;

    static MPxGeometryOverride *Creator(const MObject &obj)
    {
        return new ImplicitSurfaceGeometryOverride(obj);
    }

    ~ImplicitSurfaceGeometryOverride();

    MHWRender::DrawAPI supportedDrawAPIs() const;
    void updateDG();
    void updateRenderItems(const MDagPath &path, MHWRender::MRenderItemList &list);
    void populateGeometry(const MHWRender::MGeometryRequirements &requirements, const MHWRender::MRenderItemList &renderItems, MHWRender::MGeometry &data);
    void cleanUp();

    static MStatus initialize();
    static MStatus uninitialize();

protected:
    ImplicitSurfaceGeometryOverride(const MObject &obj);

    MObject implicitSurfaceNode;
    const MeshGeom *meshGeometry;
};

#endif
