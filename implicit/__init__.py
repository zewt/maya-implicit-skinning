import maya.cmds as cmds
import maya.mel as mel

import maya.OpenMaya as om
import maya.OpenMayaAnim as omanim

def find_skin_cluster(node):
    # Find the skinCluster for the selected node.
    history_nodes = mel.eval('listHistory -pdo true "%s"' % node)
    if not history_nodes:
        return None

    skin_clusters = mel.eval('ls -type "skinCluster" %s' % ' '.join(history_nodes))
    if not skin_clusters:
        return None

    if len(skin_clusters) > 1:
        # This isn't normally possible.
        raise Exception('"%s" has more than one skinCluster.' % node)

    return skin_clusters[0]

def create_deformer(node):
    # Find the skinCluster for the selected node.
    skin_cluster = find_skin_cluster(node)
    if not skin_cluster:
        print 'Couldn\'t find the skinCluster for "%s".  Is the geometry skinned?' % node
        return

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

    # Constrain the group containing the locators to the implicit surface.
    cmds.parentConstraint(node, group_name, weight=1)

    return group_name

def create_end_cap_locators():
    """
    Create locators that can be used to edit the selected surfaces' end
    caps.  The locators can be deleted after edits are finished.

    Important: After making edits to the surface, the attached implicitDeformers
    must be updated.  In bind pose, run:

    implicitSkin -updateBase deformer_name

    We should do this automatically in the future, or at least not require that
    the mesh be in bind pose.
    """
    nodes = cmds.ls(sl=True, l=True)
    for node in nodes:
        # If the implicitShape itself is selected, move to the transform.
        if cmds.nodeType(node) == 'implicitSurface':
            node = cmds.listRelatives(node, p=True)[0]

        # Check the node type.
        if not cmds.listRelatives(node, typ='implicitSurface'):
            print 'Node %s isn\'t an implicitSurface.' % node
            continue


        print node
        points = cmds.getAttr('%s.point[*]' % node)
        end_cap_1 = len(points) - 1
        end_cap_2 = len(points) - 2

        for idx in (end_cap_1, end_cap_2):
            if idx < 0:
                continue

            # End caps aren't specially tagged, but they're always at the end of the
            # point list, and since they lie on the bone, they always have two out of three
            # translate coordinates at 0.  This is a bit of a hack, but it's cleaner than
            # trying to keep track of which samples are caps.
            zero_coordinates = 0
            non_zero_coord = -1
            for coord in range(3):
                if abs(points[idx][coord]) < 0.0001:
                    zero_coordinates += 1
                else:
                    cap_axis = ('X', 'Y', 'Z')[coord]

            if zero_coordinates != 2:
                continue
                
            # This is an end cap, and the axis that the coordinate should be moved along is cap_axis.
            # Check if the point is already connected.
            pointPath = '%s.point[%i]' % (node, idx)
            if cmds.listConnections(pointPath):
                print 'Skipping surface point %s that\'s already connected.' % pointPath
                continue

            # Create the locator.  Don't specify a name here; the return value isn't a fully qualified node
            # path, so if we give one that's ambiguous we may fail to address it below.  Let Maya pick a name,
            # and then rename it after we've parented it.
            cap_locator_name = cmds.spaceLocator()[0]

            cap_locator_name = cmds.parent(cap_locator_name, node)[0]
            print points[idx]
            cmds.xform(cap_locator_name, t=points[idx], ro=(0,0,0), os=True)
            cap_locator_name = cmds.rename(cap_locator_name, 'CapPosition1')
            cmds.connectAttr('%s.translate' % cap_locator_name, pointPath)

            # Lock the translate axes that aren't the cap axis.  If they're changed we won't identify this as a cap
            # in the future, and it's probably not what the user is meaning to change.  Lock rotation, since
            # it doesn't affect the output and changing it just makes the locator not aligned with the cap.
            for attr in ('X', 'Y', 'Z'):
                if attr == cap_axis:
                    continue
                cmds.setAttr('%s.translate%s' % (cap_locator_name, attr), lock=True);
            for attr in ('rotate', ):
                cmds.setAttr('%s.%s' % (cap_locator_name, attr), lock=True);

            print 'Created locator', cap_locator_name, 'for', node

