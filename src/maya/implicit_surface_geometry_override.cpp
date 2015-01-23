#define NO_CUDA

#include "implicit_surface_geometry_override.hpp"

#include <maya/MFnDependencyNode.h>
#include <maya/MPxNode.h>
#include <maya/MHWGeometry.h>
#include <maya/MShaderManager.h>
#include <maya/MDrawRegistry.h>

MString ImplicitSurfaceGeometryOverride::drawRegistrantId("implicitSurfaceGeometryOverride");
MString ImplicitSurfaceGeometryOverride::drawDbClassification("drawdb/geometry/implicitSurface");

ImplicitSurfaceGeometryOverride::ImplicitSurfaceGeometryOverride(const MObject& obj):
    MPxGeometryOverride(obj),
    implicitSurfaceNode(obj)
{
}

ImplicitSurfaceGeometryOverride::~ImplicitSurfaceGeometryOverride() { }

MHWRender::DrawAPI ImplicitSurfaceGeometryOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}


void ImplicitSurfaceGeometryOverride::updateDG()
{
    // Get the MPxNode we've been pointed at.
    MStatus status = MStatus::kSuccess;

    MFnDependencyNode plugDep(implicitSurfaceNode, &status);
    if(status != MS::kSuccess) return;

    MPxNode *node = plugDep.userNode(&status);
    if(status != MS::kSuccess) return;

    // Get the ImplicitSurfaceGeometryOverrideSource interface.  If it's missing, we can't render.
    ImplicitSurfaceGeometryOverrideSource *surface = dynamic_cast<ImplicitSurfaceGeometryOverrideSource *>(node);
    if(surface == NULL)
        return;

    // Get the geometry, updating it if needed.  This pointer is valid for the lifetime of the
    // object we're rendering, but it will change in-place when it's updated.
    meshGeometry = &surface->get_mesh_geometry();
}

void ImplicitSurfaceGeometryOverride::updateRenderItems(const MDagPath &path, MHWRender::MRenderItemList &list)
{
    MHWRender::MRenderer *renderer = MHWRender::MRenderer::theRenderer();
    if(!renderer)
        return;
    const MHWRender::MShaderManager *shaderManager = renderer->getShaderManager();
    if(!shaderManager)
        return;

    // add render item for drawing wireframe on the mesh
    MHWRender::MRenderItem *wireframeItem = NULL;
    int index = list.indexOf("wireframe");
    if(index < 0)
    {
        wireframeItem = MHWRender::MRenderItem::Create("wireframe", MHWRender::MRenderItem::DecorationItem, MHWRender::MGeometry::kLines);
        wireframeItem->setDrawMode(MHWRender::MGeometry::kWireframe);
        wireframeItem->depthPriority(MHWRender::MRenderItem::sActiveWireDepthPriority);
        list.append(wireframeItem);

        MHWRender::MShaderInstance *shader = shaderManager->getStockShader(MHWRender::MShaderManager::k3dSolidShader);
        if(shader) {
            static const float theColor[] = {1.0f, 0.0f, 0.0f, 1.0f};
            shader->setParameter("solidColor", theColor);

            wireframeItem->setShader(shader);
            shaderManager->releaseShader(shader);
        }
    }
    else
        wireframeItem = list.itemAt(index);

    if(wireframeItem)
        wireframeItem->enable(true);
}

void ImplicitSurfaceGeometryOverride::populateGeometry(const MHWRender::MGeometryRequirements &requirements,
    const MHWRender::MRenderItemList &renderItems, MHWRender::MGeometry &data)
{

    // Copy the results of MarchingCubes into the output vertex and index buffers.
    {
        MHWRender::MVertexBufferDescriptor desc("", MHWRender::MGeometry::kPosition, MHWRender::MGeometry::DataType::kFloat, 3);

        MHWRender::MVertexBuffer *vertexBuffer = data.createVertexBuffer(desc);
        int numVertices = (int)meshGeometry->vertices.size();
        float *buf = (float *)vertexBuffer->acquire(numVertices*3, true);
        for(int i = 0; i < numVertices; i++) {
            buf[i*3+0] = meshGeometry->vertices[i].pos.x;
            buf[i*3+1] = meshGeometry->vertices[i].pos.y;
            buf[i*3+2] = meshGeometry->vertices[i].pos.z;
        }
        vertexBuffer->commit(buf);
    }

    int index = renderItems.indexOf("wireframe");
    if(index != -1)
    {
        const MHWRender::MRenderItem *item = renderItems.itemAt(index);
        MHWRender::MIndexBuffer *indexBuffer = data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);

        MHWRender::MIndexBufferDescriptor desc(MHWRender::MIndexBufferDescriptor::kVertexPoint, "", MHWRender::MGeometry::kPoints, 3);

        unsigned int *buf = (unsigned int*)indexBuffer->acquire((int) meshGeometry->vertices.size(), true);
        for(int i = 0; i < (int) meshGeometry->vertices.size(); i++)
            buf[i] = i;
        indexBuffer->commit(buf);
        item->associateWithIndexBuffer(indexBuffer);
    }
}

void ImplicitSurfaceGeometryOverride::cleanUp()
{
}

MStatus ImplicitSurfaceGeometryOverride::initialize()
{
    MStatus status = MHWRender::MDrawRegistry::registerGeometryOverrideCreator(ImplicitSurfaceGeometryOverride::drawDbClassification, ImplicitSurfaceGeometryOverride::drawRegistrantId, ImplicitSurfaceGeometryOverride::Creator);
    if (!status) {
       status.perror("registerGeometryOverrideCreator");
       return status;
    }

    return MStatus::kSuccess;
}

MStatus ImplicitSurfaceGeometryOverride::uninitialize()
{
    MStatus status = MHWRender::MDrawRegistry::deregisterGeometryOverrideCreator(ImplicitSurfaceGeometryOverride::drawDbClassification, ImplicitSurfaceGeometryOverride::drawRegistrantId);
    if (!status) {
       status.perror("deregisterGeometryOverrideCreator");
       return status;
    }
    return MStatus::kSuccess;
}



