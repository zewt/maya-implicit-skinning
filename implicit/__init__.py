import maya.cmds as cmds
import maya.mel as mel

import maya.OpenMaya as om
import maya.OpenMayaAnim as omanim

def create_deformer(node):
    # Find the skinCluster for the selected node.
    history_nodes = mel.eval('listHistory -pdo true "%s"' % node)
    skin_clusters = mel.eval('ls -type "skinCluster" %s' % ' '.join(history_nodes))

    if not skin_clusters:
        print 'Couldn\'t find the skinCluster for "%s".  Is the geometry skinned?'
        return
    if len(skin_clusters) > 1:
        # This isn't normally possible.
        print '"%s" has more than one skinCluster.'
        return
    skin_cluster = skin_clusters[0]

    # Create implicit surfaces for each bone in the skin cluster, and an implicit
    # blend tying them together.  The implicit blend node will be returned.
    implicit_blend_node = mel.eval('implicitSkin -createShapes "%s"' % skin_cluster)[0]

    # Create the deformer.  Request a name related to the geometry name.  Note that
    # we may not get the exact name we request, if it's already in use.
    deformer_name = mel.eval('deformer -type implicitDeformer -name "%sImplicit" "%s"' % (node, node))[0]

    # Hack: We want this deformer to be just after the skinCluster, especially
    # if there is non-deformer history like deleteComponent nodes after it.  However,
    # the reorderDeformers command is a bit broken: it doesn't understand anything
    # on the chain except deformers, so if we just tell it to put our deformer
    # after the skinCluster it'll just say that it's already in order, even
    # if there's other stuff in between.
    #
    # Work around this by reversing the order, then putting it back.  This
    # results in the order we want, with our deformer before construction
    # history.
    #
    # If this causes problems it can be removed, but if there's incompatible
    # construction history between the implicitDeformer and the skinCluster,
    # the implicitDeformer will turn itself off.
    mel.eval('reorderDeformers "%s" "%s"' % (skin_cluster, deformer_name))
    mel.eval('reorderDeformers "%s" "%s"' % (deformer_name, skin_cluster))

    # Connect the blend node to the deformer.
    mel.eval('connectAttr -f "%s.worldImplicit" "%s.implicit"' % (implicit_blend_node, deformer_name))

    # XXX: base potential
    mel.eval('implicitSkin -updateBase "%s"' % deformer_name)

def getOnePlugByName(nodeName):
    slist = om.MSelectionList()
    slist.add(nodeName)

    matches = slist.length()

    if matches > 1:
        raise RuntimeError('Multiple nodes found: %s' % nodeName)

    plug = om.MPlug()
    slist.getPlug(0, plug)
    return plug

def getDeformerByName(nodeName):
    implicitPlug = getOnePlugByName(nodeName)

    # Verify that this is one of our nodes.
    plugDep = om.MFnDependencyNode(implicitPlug.node())
    typeId = plugDep.typeId().id()
    ImplicitSkinDeformerId = 0xea115
    if typeId != ImplicitSkinDeformerId:
        raise RuntimeError('Node not an implicitDeformer: ' + nodeName)

    return implicitPlug

def getTransformNode(node):
    """
    Given a deformer node, get its transform node.
    """
    geometryFilter = omanim.MFnGeometryFilter(node)
    dagPath = om.MDagPath()
    geometryFilter.getPathAtIndex(0, dagPath)
    dagPath.pop()
    return dagPath

def main():
#    node = 'pCylinder2Implicit'
#    implicitPlug = getDeformerByName(node)
#
#
#    implicitGeometryOutputDagPath = getTransformNode(implicitPlug.node())
#
#    # Get the inverse transformation, which is the transformation to go from world space to
#    # object space.
#    transformNode = om.MFnTransform(implicitGeometryOutputDagPath.node())
#    objectToWorldSpaceMat = transformNode.transformationMatrix()
#    worldToObjectSpaceMat = objectToWorldSpaceMat.inverse()
##    print worldToObjectSpaceMat.transformation()
#    print dir(worldToObjectSpaceMat)
#
#    # Store objectToWorldSpaceMat on geomMatrixAttr.
##    DagHelpers::setMatrixPlug(implicitPlug.node(), ImplicitSkinDeformer::geomMatrixAttr, objectToWorldSpaceMat);
#    x = 'setAttr "%s.geomMatrix" 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1' % node
#    cmds.xform('%s.input[0].inputGeometry' % node, t=True, q=True, ws=True)
##    cmds.setAttr('%s.geomMatrix' % node, [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], type='matrix')
#
#    return
    selection = cmds.ls(sl=True)
    if len(selection) != 1:
        # XXX: Allow more than one selection after this is better tested.
        print 'Select one node to deform'
        return

    for node in selection:
        create_deformer(node)

def create_locators():
    node = cmds.ls(sl=True)[0]
    parent_node = cmds.listRelatives(node, p=True)
    pts = cmds.getAttr('%s.point[*]' % node)
    all_names = []
    for x in pts:
        n=cmds.spaceLocator(p=x)[0]
        all_names.append(n)
        cmds.setAttr(n+'.localScale', 0.1, 0.1, 0.1)
    cmds.select(all_names)
    group_name = cmds.group()
    cmds.parent(group_name, parent_node)
    cmds.xform(group_name, ro=(0,0,0), s=(1,1,1), t=(0,0,0), os=True)
    return group_name

