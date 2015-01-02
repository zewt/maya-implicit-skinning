/*
    Copyright (c) 2008-2009 NetAllied Systems GmbH

	This file is part of COLLADAMaya.

    Portions of the code are:
    Copyright (c) 2005-2007 Feeling Software Inc.
    Copyright (c) 2005-2007 Sony Computer Entertainment America
    Copyright (c) 2004-2005 Alias Systems Corp.

    Licensed under the MIT Open Source License,
    for details please see LICENSE file or the website
    http://www.opensource.org/licenses/mit-license.php
*/

#define NO_CUDA
#include "maya_helpers.hpp"

#include <maya/MPxDeformerNode.h> 
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshEdge.h>
#include <maya/MPxLocatorNode.h> 
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnDependencyNode.h>
#include <maya/MFnSkinCluster.h>

#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MFnMesh.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>

#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MMatrix.h>

#include <maya/MDagModifier.h>
#include "transfo.hpp"

#include <algorithm>
using namespace std;

namespace DagHelpers
{
    Transfo MMatrixToTransfo(const MMatrix &mmat)
    {
        Transfo mat;
        mat[0] = (float) mmat[0][0];
        mat[1] = (float) mmat[1][0];
        mat[2] = (float) mmat[2][0];
        mat[3] = (float) mmat[3][0];
        mat[4] = (float) mmat[0][1];
        mat[5] = (float) mmat[1][1];
        mat[6] = (float) mmat[2][1];
        mat[7] = (float) mmat[3][1];
        mat[8] = (float) mmat[0][2];
        mat[9] = (float) mmat[1][2];
        mat[10] = (float) mmat[2][2];
        mat[11] = (float) mmat[3][2];
        mat[12] = (float) mmat[0][3];
        mat[13] = (float) mmat[1][3];
        mat[14] = (float) mmat[2][3];
        mat[15] = (float) mmat[3][3];
        return mat;
    }


    MStatus getConnectedPlugWithName(MObject inputNode, std::string name, MObject &result)
    {
        MStatus status = MStatus::kSuccess;

        MFnDependencyNode inputPlugDep(inputNode);
        MObject geomObj = inputPlugDep.attribute(name.c_str(), &status);
        if(status != MS::kSuccess)
        {
            fprintf(stderr, "error finding inputGeom\n");
            return status;
        }

        MPlug geomObjPlug(inputNode, geomObj);
    
        MPlugArray connPlugs;
        geomObjPlug.connectedTo(connPlugs, true, false);
        int connLength = connPlugs.length();
        if(connLength == 0) {
            fprintf(stderr, "no connection\n");
            return MStatus::kFailure;
        }

        result = connPlugs[0].node();
        return MStatus::kSuccess;
    }

    MMatrix getMatrixFromPlug(MPlug plug, MStatus *status)
    {
        MObject obj;
        plug.getValue(obj);

        MFnMatrixData fnMat(obj);
        return fnMat.matrix();
    }

    
    MStatus findAncestorDeformer(MObject node, MFn::Type type, MObject &resultNode)
    {
        while(true)
        {
            MStatus status = DagHelpers::getConnectedPlugWithName(node, "inputGeometry", node);
            if(status != MS::kSuccess)
                return status;

            if(node.apiType() == type)
            {
                resultNode = node;
                return MStatus::kSuccess;
            }
        }
    }

    static int compare_length(const MDagPath &lhs, const MDagPath &rhs) { return lhs.fullPathName().length() < rhs.fullPathName().length(); }

    /* Return the names of the skinCluster's influence objects, sorted parents before children. */
    MStatus getMDagPathsFromSkinCluster(MObject skinClusterNode, std::vector<MDagPath> &out)
    {
        MStatus status = MStatus::kSuccess;
        MFnSkinCluster skinCluster(skinClusterNode, &status);
        if(status != MS::kSuccess) return status;

        // Get the influence objects (joints) for the skin cluster.
        MDagPathArray paths;
        skinCluster.influenceObjects(paths, &status);
        if(status != MS::kSuccess)
            return status;

        // Convert to a vector.
        out.clear();
        for(unsigned i = 0; i < paths.length(); ++i) {
            int logicalIndex = skinCluster.indexForInfluenceObject(paths[i], &status);
            if(status != MS::kSuccess) return status;

            // Make space for the item, if needed.
            if(out.size() <= logicalIndex)
                out.resize(logicalIndex+1);
            out[logicalIndex] = paths[i];
        }

        // Sort the influence objects by the length of their full path.  Since the name of
        // an object is prefixed by its parents, eg. "parent1|parent2|object", this guarantees
        // that a parent is before all of its children.
        sort(out.begin(), out.end(), compare_length);

        return MStatus::kSuccess;
    }

    /* Find the nearest ancestor to path.  "a|b|c" is an ancestor of "a|b|c|d|e|f".
     * If no nodes are an ancestor of path, return -1. */
    int findClosestAncestor(const vector<MDagPath> &dagPaths, MDagPath dagPath)
    {
        string path = dagPath.fullPathName().asChar();

        int best_match = -1;
        int best_match_length = -1;
        for(size_t i = 0; i < dagPaths.size(); ++i) {
            string parentPath = dagPaths[i].fullPathName().asChar();
            if(parentPath == path)
                continue;

            // If path doesn't begin with this path plus |, it's not an ancestor.
            string compareTo = parentPath + "|";
            if(path.compare(0, compareTo.size(), compareTo, 0, compareTo.size()) != 0)
                continue;

            if((int) parentPath.size() > best_match_length)
            {
                best_match = (int) i;
                best_match_length = (int) parentPath.size();
            }
        }
        return best_match;
    }



    MStatus getInputGeometryForSkinClusterPlug(MObject skinClusterNode, MPlug &outPlug)
    {
        // This gets the original geometry, not the geometry input into the skinCluster:
        /*
        MObjectArray inputGeometries;
        status = skinCluster.getInputGeometry(inputGeometries);
        if(status != MS::kSuccess)
            return status;

        if(inputGeometries.length() == 0)
        {
            printf("skinCluster has no input geometry\n");
            return MS::kFailure;
        }

        // skinClusters don't support more than one input geometry.
        MObject inputValue = inputGeometries[0];
        */

        MStatus status = MStatus::kSuccess;
        MObject inputAttr = MFnDependencyNode(skinClusterNode).attribute("input", &status);
        if(status != MS::kSuccess) return status;

        MPlug inputPlug(skinClusterNode, inputAttr);
        if(status != MS::kSuccess) return status;

        // Select input[0].  skinClusters only support a single input.
        inputPlug = inputPlug.elementByPhysicalIndex(0, &status);
        if(status != MS::kSuccess) return status;

        MFnDependencyNode inputPlugDep(inputPlug.node());
        MObject inputObject = inputPlugDep.attribute("inputGeometry", &status);
        if(status != MS::kSuccess) return status;
            
        outPlug = inputPlug.child(inputObject, &status);
        return status;
    }

    MStatus setPlug(MPlug &plug, MMatrix value)
    {
        MStatus status = MStatus::kSuccess;

        // Create a MFnMatrixData holding the matrix.
        MFnMatrixData fnMat;
        MObject matObj = fnMat.create(&status);
        if(status != MS::kSuccess) return status;

        // Set the value.
        fnMat.set(value);

        status = plug.setValue(matObj);
        if(status != MS::kSuccess) return status;

        return MStatus::kSuccess;
    }


    MStatus setMatrixPlug(MObject node, MObject attr, MMatrix matrix)
    {
        MStatus status;

        MPlug attrPlug(node, attr);

        // Create a MFnMatrixData holding the matrix.
        MFnMatrixData fnMat;
        MObject matObj = fnMat.create(&status);
        if(status != MS::kSuccess) return status;

        // Set the worldMatrixInverse attribute.
        fnMat.set(matrix);
        status = attrPlug.setValue(matObj);
        if(status != MS::kSuccess) return status;

        return MStatus::kSuccess;
    }

    MStatus getMatrixPlug(MObject node, MObject attr, MMatrix &matrix)
    {
        MStatus status;
        MPlug attrPlug(node, attr);

        MObject matObj;
        status = attrPlug.getValue(matObj);
        if(status != MS::kSuccess) return status;

        // Create a MFnMatrixData holding the matrix.
        MFnMatrixData fnMat(matObj, &status);
        if(status != MS::kSuccess) return status;
        matrix = fnMat.matrix(&status);

        return status;
    }

    //---------------------------------------------------
    //
    // Find a node connected to a node's attribute
    //
    MObject getNodeConnectedTo( const MObject& node, const MString &attribute )
    {
        MPlug plug;

        if ( getPlugConnectedTo ( node, attribute, plug ) )
        {
            return plug.node();
        }

        else
        {
            return MObject::kNullObj;
        }
    }

    //---------------------------------------------------
    MObject getSourceNodeConnectedTo ( const MObject& node, const MString& attribute )
    {
        MStatus status;
        MFnDependencyNode dgFn ( node );
        MPlug plug = dgFn.findPlug ( attribute, &status );
        if ( status == MS::kSuccess && plug.isConnected() )
        {
            // Get the connection - there can be at most one input to a plug
            MPlugArray connections;
            plug.connectedTo ( connections, true, false );
            size_t length = connections.length();
            if ( connections.length() > 0 )
            {
                return connections[0].node();
            }
        }

        return MObject::kNullObj;
    }

    //---------------------------------------------------
    MObject getSourceNodeConnectedTo ( const MPlug& inPlug )
    {
        MStatus status;
        MPlugArray connections;
        inPlug.connectedTo ( connections, true, false );

        if ( connections.length() > 0 )
        {
            return connections[0].node();
        }

        return MObject::kNullObj;
    }

    //---------------------------------------------------
    bool getPlugConnectedTo ( const MObject& node, const MString &attribute, MPlug& connectedPlug )
    {
        MStatus status;
        MFnDependencyNode dgFn ( node );
        MPlug plug = dgFn.findPlug ( attribute, &status );

        if ( status == MS::kSuccess && plug.isConnected() )
        {
            // Get the connection - there can be at most one input to a plug
            MPlugArray connections;
            plug.connectedTo ( connections, true, true );

            if ( connections.length() > 0 )
            {
                connectedPlug = connections[0];
                return true;
            }
        }

        return false;
    }

    //---------------------------------------------------
    bool getPlugConnectedTo ( const MPlug& inPlug, MPlug& plug )
    {
        MStatus status;

        MPlugArray connections;
        inPlug.connectedTo ( connections, true, true );

        if ( connections.length() > 0 )
        {
            plug = connections[0];
            return true;
        }

        return false;
    }

    //---------------------------------------------------
    //
    // Get the node connected to a plug
    //
    MObject getNodeConnectedTo ( const MPlug& plug )
    {
        MStatus status;
        MPlugArray connections;
        plug.connectedTo ( connections, true, true, &status );
        if ( status != MStatus::kSuccess ) return MObject::kNullObj;
        if ( connections.length() <= 0 ) return MObject::kNullObj;

        return connections[0].node();
    }
#if 0

    //---------------------------------------------------
    //
    // Find a node connected to a node's array attribute
    //
    bool getPlugArrayConnectedTo ( const MObject& node, const MString& attribute, MPlug& connectedPlug )
    {
        MStatus status;
        MFnDependencyNode dgFn ( node );
        MPlug plug = dgFn.findPlug ( attribute, &status );

        if ( status != MS::kSuccess )
            return false;

        if ( plug.numElements() < 1 )
            return false;

        MPlug firstElementPlug = plug.connectionByPhysicalIndex ( 0 );

        // Get the connection - there can be at most one input to a plug
        //
        MPlugArray connections;
        firstElementPlug.connectedTo ( connections, true, true );

        if ( connections.length() == 0 )
            return false;

        connectedPlug = connections[0];

        return true;
    }

    //---------------------------------------------------
    bool connect ( const MObject& source, const MString& sourceAttribute, const MObject& destination, const MString& destinationAttribute )
    {
        MStatus status;
        MFnDependencyNode srcFn ( source );
        MFnDependencyNode destFn ( destination );

        MPlug src = srcFn.findPlug ( sourceAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        MPlug dest = destFn.findPlug ( destinationAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        MDGModifier modifier;

        modifier.connect ( src, dest );

        status = modifier.doIt();

        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool connect ( const MObject& source, const MString& sourceAttribute, const MPlug& destination )
    {
        MStatus status;
        MFnDependencyNode srcFn ( source );

        MPlug src = srcFn.findPlug ( sourceAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        MDGModifier modifier;

        modifier.connect ( src, destination );

        status = modifier.doIt();

        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool connect ( const MPlug& source, const MObject& destination, const MString& destinationAttribute )
    {
        MStatus status;
        MFnDependencyNode destFn ( destination );

        MPlug dst = destFn.findPlug ( destinationAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        MDGModifier modifier;

        status = modifier.connect ( source, dst );

        if ( status != MStatus::kSuccess ) return false;

        status = modifier.doIt();

        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool connect ( const MPlug& source, const MPlug& destination )
    {
        MDGModifier modifier;
        modifier.connect ( source, destination );
        MStatus status = modifier.doIt();

        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool connectToList ( const MObject& source, const MString& sourceAttribute, const MObject& destination, const MString& destinationAttribute, int* index )
    {
        MStatus status;
        MFnDependencyNode srcFn ( source );

        MPlug src = srcFn.findPlug ( sourceAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        return connectToList ( src, destination, destinationAttribute, index );
    }

    //---------------------------------------------------
    bool connectToList ( const MPlug& source, const MObject& destination, const MString& destinationAttribute, int* _index )
    {
        MStatus status;
        MFnDependencyNode destFn ( destination );
        MPlug dest = destFn.findPlug ( destinationAttribute, &status );

        if ( status != MStatus::kSuccess ) return false;

        if ( !dest.isArray() ) return false;

        int index = ( _index != NULL ) ? *_index : -1;

        if ( index < 0 )
        {
            index = getNextAvailableIndex ( dest, ( int ) dest.evaluateNumElements() );

            if ( _index != NULL ) *_index = index;
        }

        MPlug d = dest.elementByLogicalIndex ( index );

        MDGModifier modifier;
        modifier.connect ( source, d );
        status = modifier.doIt();
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    int getNextAvailableIndex ( const MObject& object, const MString& attribute, int startIndex )
    {
        MPlug p = MFnDependencyNode ( object ).findPlug ( attribute );

        if ( p.node().isNull() ) return -1;

        return getNextAvailableIndex ( p, startIndex );
    }

    //---------------------------------------------------
    int getNextAvailableIndex ( const MPlug& p, int startIndex )
    {
        if ( startIndex < 0 ) startIndex = 0;

        // Look for the next available plug
        for ( uint i = ( uint ) startIndex; i < ( uint ) startIndex + 100; ++i )
        {
            MPlug possibleDestination = p.elementByLogicalIndex ( i );

            if ( !hasConnection ( possibleDestination ) )
            {
                return i;
            }
        }

        return -1;
    }

    //---------------------------------------------------
    bool hasConnection ( const MPlug& plug, bool asSource, bool asDestination )
    {
        MPlugArray plugs;
        plug.connectedTo ( plugs, asDestination, asSource );
        if ( plugs.length() > 0 ) return true;

        return plug.numConnectedChildren() > 0;
    }

    //---------------------------------------------------
    // Retrieve the bind pose for a controller/joint std::pair
    //
    MMatrix getBindPoseInverse ( const MObject& controller, const MObject& influence )
    {
        MStatus status;
        if ( controller.apiType() == MFn::kSkinClusterFilter )
        {
            MFnSkinCluster controllerFn ( controller );

            // Find the correct index for the pre-bind matrix
            uint index = controllerFn.indexForInfluenceObject ( MDagPath::getAPathTo ( influence ), &status );
            if ( status != MStatus::kSuccess ) return MMatrix::identity;

            MPlug preBindMatrixPlug = controllerFn.findPlug ( "bindPreMatrix", &status );
            preBindMatrixPlug = preBindMatrixPlug.elementByLogicalIndex ( index, &status );
            if ( status != MStatus::kSuccess ) return MMatrix::identity;

            // Get the plug's matrix
            MMatrix ret;
            if ( !getPlugValue ( preBindMatrixPlug, ret ) ) return MMatrix::identity;

            return ret;
        }

        else if ( controller.apiType() == MFn::kJointCluster )
        {
            MMatrix ret;
            getPlugValue ( influence, "bindPose", ret );
            return ret.inverse();
        }

        else return MMatrix::identity;
    }

    //---------------------------------------------------
    // set the bind pose for a transform
    //
    MStatus setBindPoseInverse ( const MObject& node, const MMatrix& bindPoseInverse )
    {
        MStatus status;
        MFnDependencyNode dgFn ( node );
        MPlug bindPosePlug = dgFn.findPlug ( "bindPose", &status );
        if ( status != MS::kSuccess )
        {
            MGlobal::displayWarning ( MString ( "No bindPose found on node " ) + dgFn.name() );
            return status;
        }

        MFnMatrixData matrixFn;
        MObject val = matrixFn.create ( bindPoseInverse.inverse(), &status );
        MObject invval = matrixFn.create ( bindPoseInverse, &status );
        if ( status != MS::kSuccess )
        {
            MGlobal::displayWarning ( MString ( "Error setting bindPose on node " ) + dgFn.name() );
            return status;
        }

        // set the bind pose on the joint itself
        bindPosePlug.setValue ( val );

        // Now, perhaps more significantly, see if there's a
        // skinCluster using this bone and update its bind
        // pose (as the joint bind pose is not connected to
        // the skin - it's set at bind time from the joint's
        // current position, and our importer may not want to
        // disturb the current scene state just to put bones
        // in a bind position before creating skin clusters)
        MObject _node ( node );
        MItDependencyGraph it ( _node, MFn::kSkinClusterFilter );
        while ( !it.isDone() )
        {
            MPlug plug = it.thisPlug();
            unsigned int idx = plug.logicalIndex();
            MFnDependencyNode skinFn ( plug.node() );
            MPlug skinBindPosePlug = skinFn.findPlug ( "bindPreMatrix", &status );

            if ( status == MS::kSuccess )
            {
                // The skinCluster stores inverse inclusive matrix
                // so notice we use invval (the MObject created off
                // the inverse matrix here)
                skinBindPosePlug = skinBindPosePlug.elementByLogicalIndex ( idx );
                skinBindPosePlug.setValue ( invval );
            }

            it.next();
        }

        return status;
    }

    //---------------------------------------------------
    MPlug getChildPlug ( const MPlug& parent, const MString& name, MStatus* rc )
    {
        MStatus st;
        uint childCount = parent.numChildren ( &st );
        if ( st != MStatus::kSuccess )
        {
            if ( rc != NULL ) *rc = st;
            return parent;
        }

        // Check shortNames first
        for ( uint i = 0; i < childCount; ++i )
        {
            MPlug child = parent.child ( i, &st );
            if ( st != MStatus::kSuccess )
            {
                if ( rc != NULL ) *rc = st;
                return parent;
            }

            MFnAttribute attributeFn ( child.attribute() );
            MString n = attributeFn.shortName();
            if ( n == name )
            {
                if ( rc != NULL ) *rc = MStatus::kSuccess;
                return child;
            }
        }

        // Check longNames second, use shortNames!
        for ( uint i = 0; i < childCount; ++i )
        {
            MPlug child = parent.child ( i, &st );
            if ( st != MStatus::kSuccess )
            {
                if ( rc != NULL ) *rc = st;
                return parent;
            }

            MFnAttribute attributeFn ( child.attribute() );
            MString n = attributeFn.name();
            if ( n == name )
            {
                if ( rc != NULL ) *rc = MStatus::kSuccess;
                return child;
            }
        }

        if ( rc != NULL ) *rc = MStatus::kNotFound;
        return parent;
    }

    //---------------------------------------------------
    int getChildPlugIndex ( const MPlug& parent, const MString& name, MStatus* rc )
    {
        MStatus st;
        uint childCount = parent.numChildren ( &st );
        CHECK_STATUS_AND_RETURN ( st, -1 );

        // Check shortNames first
        for ( uint i = 0; i < childCount; ++i )
        {
            MPlug child = parent.child ( i, &st );
            CHECK_STATUS_AND_RETURN ( st, -1 );

            MFnAttribute attributeFn ( child.attribute() );
            MString n = attributeFn.shortName();

            if ( n == name )
            {
                if ( rc != NULL ) *rc = MStatus::kSuccess;

                return i;
            }
        }

        // Check longNames second, use shortNames!
        for ( uint i = 0; i < childCount; ++i )
        {
            MPlug child = parent.child ( i, &st );
            CHECK_STATUS_AND_RETURN ( st, -1 );

            MFnAttribute attributeFn ( child.attribute() );
            MString n = attributeFn.name();

            if ( n == name )
            {
                if ( rc != NULL ) *rc = MStatus::kSuccess;

                return i;
            }
        }

        if ( rc != NULL ) *rc = MStatus::kNotFound;

        return -1;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, double &value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        status = plug.getValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, float &value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        status = plug.getValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, float& x )
    {
        MStatus status;
        status = plug.getValue ( x );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, int& value )
    {
        MDGContext context = MDGContext::fsNormal;

        MStatus status;
        status = plug.getValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, uint32& value )
    {
        MStatus status;
        int temp;
        status = plug.getValue ( temp );
        value = static_cast<uint32> ( temp );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, short& value )
    {
        MStatus status;
        status = plug.getValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, uint16& value )
    {
        MStatus status;
        short temp;
        status = plug.getValue ( temp );
        value = static_cast<uint16> ( temp );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, char& value )
    {
        MStatus status;
        status = plug.getValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, uint8_t &value )
    {
        MStatus status;
        char temp;
        status = plug.getValue ( temp );
        value = static_cast<uint8> ( temp );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const MString attributeName, MEulerRotation& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        if ( plug.isCompound() && plug.numChildren() >= 3 )
        {
            status = plug.child ( 0 ).getValue ( value.x );
            if ( status != MStatus::kSuccess ) return false;

            status = plug.child ( 1 ).getValue ( value.y );
            if ( status != MStatus::kSuccess ) return false;

            status = plug.child ( 2 ).getValue ( value.z );
            if ( status != MStatus::kSuccess ) return false;

            return true;
        }

        else return false;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, bool& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        status = plug.getValue ( value );
        if ( status != MStatus::kSuccess ) return false;

        return true;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, bool& value )
    {
        MStatus status;
        status = plug.getValue ( value );
        if ( status != MStatus::kSuccess ) return false;

        return true;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, int& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        status = plug.getValue ( value );
        if ( status != MStatus::kSuccess ) return false;

        return true;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, MMatrix& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        return getPlugValue ( plug, value );
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, MMatrix& value )
    {
        MStatus status;

        MFnMatrixData mxData;
        MObject object = mxData.create();
        status = plug.getValue ( object );
        if ( status != MStatus::kSuccess ) return false;

        mxData.setObject ( object );
        // MFnMatrixData mxData(o, &status);
        if ( status != MStatus::kSuccess ) return false;

        value = mxData.matrix();

        return true;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, MColor& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        return getPlugValue ( plug, value );
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, MColor& value )
    {
        MStatus status;
        if ( plug.isCompound() && plug.numChildren() >= 3 )
        {
            status = plug.child ( 0 ).getValue ( value.r );
            if ( status != MStatus::kSuccess ) return false;

            status = plug.child ( 1 ).getValue ( value.g );
            if ( status != MStatus::kSuccess ) return false;

            status = plug.child ( 2 ).getValue ( value.b );
            if ( status != MStatus::kSuccess ) return false;

            if ( plug.numChildren() >= 4 )
            {
                status = plug.child ( 3 ).getValue ( value.a );
                if ( status != MStatus::kSuccess ) return false;
            }
            else value.a = 1.0f;

            return true;
        }

        else return false;
    }

    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, MString& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        return plug.getValue ( value );
    }

    //---------------------------------------------------
    void getPlugValue (
        const MObject& node,
        const String attributeName,
        MStringArray& output,
        MStatus* status )
    {
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), status );
        getPlugValue ( plug, output, status );
    }

    //---------------------------------------------------
    void getPlugValue ( const MPlug& plug, MStringArray& output, MStatus* status )
    {
        MObject str_obj;
        plug.getValue ( str_obj );
        MFnStringArrayData f_astr ( str_obj, status );
        unsigned int len = f_astr.length();
        for ( unsigned int i = 0; i < len; ++i )
        {
            const MString& val = f_astr[i];
            output.append ( val );
        }
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, float& x, float& y )
    {
        MObject obj;
        plug.getValue ( obj );
        MStatus status;
        MFnNumericData fcolor ( obj, &status );
        if ( !status ) return 0;

        fcolor.getData ( x , y );
        return 1;
    }
#endif

    MStatus getPlugValue(const MPlug& plug, float& x, float& y, float& z)
    {
        MObject obj;
        plug.getValue(obj);

        MStatus status;
        MFnNumericData value(obj, &status);
        if(!status) return status;

        value.getData(x, y, z);
        return MStatus::kSuccess;
    }

#if 0
    //---------------------------------------------------
    bool getPlugValue ( const MObject& node, const String attributeName, MVector& value )
    {
        MStatus status;
        MPlug plug = MFnDependencyNode ( node ).findPlug ( attributeName.c_str(), &status );
        if ( status != MStatus::kSuccess ) return false;

        return getPlugValue ( plug, value );
    }

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, MVector& value )
    {
        MObject obj;
        plug.getValue ( obj );

        MStatus status = plug.getValue ( obj );
		COLLADABU_ASSERT ( status );
        MFnNumericData data ( obj );

        double x, y, z;
        status = data.getData ( x, y, z );
		COLLADABU_ASSERT ( status );
        value = MVector ( x,y,z );

        return true;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, const MVector& value )
    {
        MStatus status;
        MFnNumericData dataCreator;

        MObject float3Data = dataCreator.create ( MFnNumericData::k3Float, &status );
        if ( status != MStatus::kSuccess ) return false;

        dataCreator.setData ( ( float ) value.x, ( float ) value.y, ( float ) value.z );
        status = plug.setValue ( float3Data );
        if ( status != MStatus::kSuccess ) return false;

        return true;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, const MColor& value )
    {
        MStatus status;
        if ( plug.isCompound() && plug.numChildren() >= 3 )
        {
            MPlug rPlug = plug.child ( 0, &status );
            if ( status != MStatus::kSuccess ) return false;

            status = rPlug.setValue ( value.r );
            if ( status != MStatus::kSuccess ) return false;

            MPlug gPlug = plug.child ( 1, &status );
            if ( status != MStatus::kSuccess ) return false;

            status = gPlug.setValue ( value.g );
            if ( status != MStatus::kSuccess ) return false;

            MPlug bPlug = plug.child ( 2, &status );
            if ( status != MStatus::kSuccess ) return false;

            status = bPlug.setValue ( value.b );
            if ( status != MStatus::kSuccess ) return false;

            if ( plug.numChildren() >= 4 )
            {
                MPlug aPlug = plug.child ( 3, &status );
                if ( status != MStatus::kSuccess ) return false;

                status = aPlug.setValue ( value.a );
                if ( status != MStatus::kSuccess ) return false;
            }
        }

        return true;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, const MMatrix& value )
    {
        MStatus status;
        MFnMatrixData dataCreator;

        MObject matrixData = dataCreator.create ( value, &status );
        if ( status != MStatus::kSuccess ) return false;

        status = plug.setValue ( matrixData );
        if ( status != MStatus::kSuccess ) return false;

        return true;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, const String& value )
    {
        MStatus status;
        status = plug.setValue ( value.c_str() );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
#ifdef UNICODE
    bool setPlugValue ( MPlug& plug, const String& value )
    {
        MStatus status;
        status = plug.setValue ( MString ( value.c_str() ) );
        return status == MStatus::kSuccess;
    }

#endif // UNICODE

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, float value )
    {
        MStatus status;
        status = plug.setValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, double value )
    {
        MStatus status;
        status = plug.setValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
#endif

    MStatus setPlugValue(MPlug &plug, float x, float y)
    {
        MFnNumericData data;
        MObject obj = data.create(MFnNumericData::k2Float);
        data.setData(x, y);
        return plug.setValue(obj);
    }

    MStatus setPlugValue(MPlug &plug, float x, float y, float z)
    {
        MFnNumericData data;
        MObject obj = data.create(MFnNumericData::k3Float);
        data.setData(x, y, z);
        return plug.setValue(obj);
    }

#if 0
    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, int value )
    {
        MStatus status;
        status = plug.setValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, bool value )
    {
        MStatus status;
        status = plug.setValue ( value );
        return status == MStatus::kSuccess;
    }

    //---------------------------------------------------
    bool setPlugValue ( MPlug& plug, MStringArray& stringArray )
    {
        MObject obj;
        MFnStringArrayData fstr;
        obj = fstr.create ( stringArray );
        return plug.setValue ( obj );
    }

    //---------------------------------------------------
    void setArrayPlugSize ( MPlug& plug, uint size )
    {
        if ( plug.node().isNull() ) return;

#if MAYA_API_VERSION >= 800
        MStatus status = plug.setNumElements ( size );
        CHECK_STAT ( status );

#else
        MObject node = plug.node();
        MString plugPath = plug.info();
        if ( node.hasFn ( MFn::kDagNode ) )
        {
            MFnDagNode dagFn ( node );
            int dot = plugPath.index ( '.' );
            plugPath = dagFn.fullPathName() + plugPath.substring ( dot, plugPath.length() );
        }

        MString command = MString ( "setAttr -s " ) + size + " \"" + plugPath + "\";";

        MGlobal::executeCommand ( command );
#endif // MAYA 8.00+
    }

    //---------------------------------------------------
    // Creates a typed attribute. Used for maya "notes" attributes.
    MObject createAttribute (
    	const MObject& node,
    	const char* attributeName,
    	const char* attributeShortName,
    	MFnNumericData::Type type,
    	const char *value )
    {
        // Before creating a new attribute: verify that an old one doesn't already exist
        MStatus status;
        MObject attribute;
        MFnDependencyNode nodeFn ( node );
        MPlug plug = nodeFn.findPlug ( attributeShortName, status );

        if ( status != MStatus::kSuccess )
        {
            MFnNumericAttribute attr;
            MStatus status;
            attribute = attr.create ( attributeName,attributeShortName,type,0,&status );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;

            attr.setStorable ( true );
            attr.setKeyable ( false );
            attr.setCached ( true );
            attr.setReadable ( true );
            attr.setWritable ( true );
            status = nodeFn.addAttribute ( attribute, MFnDependencyNode::kLocalDynamicAttr );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;

            plug = nodeFn.findPlug ( attribute, &status );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;
        }

        status = plug.setValue ( value );
        if ( status != MStatus::kSuccess ) return MObject::kNullObj;

        return attribute;
    }

    //---------------------------------------------------
    MObject createAttribute (
        const MObject& node,
        const char* attributeName,
        const char* attributeShortName,
        MFnData::Type type,
        const char *value )
    {
        // Before creating a new attribute: verify that an old one doesn't already exist
        MStatus status;
        MObject attribute;
        MFnDependencyNode nodeFn ( node );
        MPlug plug = nodeFn.findPlug ( attributeShortName, status );

        if ( status != MStatus::kSuccess )
        {
            MFnTypedAttribute attr;
            MStatus status;
            attribute = attr.create ( attributeName,attributeShortName,type,&status );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;

            attr.setStorable ( true );
            attr.setKeyable ( false );
            attr.setCached ( true );
            attr.setReadable ( true );
            attr.setWritable ( true );
            status = nodeFn.addAttribute ( attribute, MFnDependencyNode::kLocalDynamicAttr );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;

            plug = nodeFn.findPlug ( attribute, &status );
            if ( status != MStatus::kSuccess ) return MObject::kNullObj;
        }

        status = plug.setValue ( value );
        if ( status != MStatus::kSuccess ) return MObject::kNullObj;

        return attribute;
    }

    //---------------------------------------------------
    MPlug addAttribute ( const MObject& node, const MObject& attribute )
    {
        MPlug plug;
        MFnAttribute attributeFn ( attribute );
        MFnDependencyNode depFn ( node );
        MStatus status = depFn.addAttribute ( attribute, MFnDependencyNode::kLocalDynamicAttr );
        if ( status == MStatus::kSuccess )
        {
            plug = depFn.findPlug ( attribute );
        }

        return plug;
    }

    //---------------------------------------------------
    // Get a dag path or node from a String
    MDagPath getShortestDagPath ( const MObject& node )
    {
        MDagPathArray paths;
        MDagPath::getAllPathsTo ( node, paths );
        MDagPath shortestPath;
        if ( paths.length() > 0 )
        {
            shortestPath = paths[0];
            for ( uint i = 1; i < paths.length(); ++i )
            {
                if ( shortestPath.length() > paths[i].length() )
                {
                    shortestPath = paths[i];
                }
            }
        }

        return shortestPath;
    }

    //---------------------------------------------------
    MObject getNode ( const MString& name )
    {
        MSelectionList selection;
        selection.add ( name );

        MObject nodeObject;
        selection.getDependNode ( 0, nodeObject );

        return nodeObject;
    }

    //---------------------------------------------------
    MObject createAnimationCurve ( const MObject& node, const char* attributeName, const char* curveType )
    {
        MFnDependencyNode fn ( node );
        return createAnimationCurve ( fn.findPlug ( attributeName ), curveType );
    }

    //---------------------------------------------------
    MObject createAnimationCurve ( const MPlug& plug, const char* curveType )
    {
        MStatus rc;
        MFnDependencyNode curveFn;
        curveFn.create ( curveType, &rc );

        if ( rc == MStatus::kSuccess )
        {
            connect ( curveFn.object(), "output", plug );
        }

        return curveFn.object();
    }

    //---------------------------------------------------
    MObject createExpression ( const MPlug& plug, const MString& expression )
    {
        MFnDependencyNode expressionFn;
        MObject expressionObj = expressionFn.create ( "expression" );
        setPlugValue ( expressionObj, "expression", expression );

        MPlug output = expressionFn.findPlug ( "output" );
        MPlug firstOutput = output.elementByLogicalIndex ( 0 );
        connect ( firstOutput, plug );
        return expressionObj;
    }

    //---------------------------------------------------
    bool plugHasAnimation ( const MPlug& plug )
    {
        MPlugArray connections;
        plug.connectedTo ( connections, true, false );
        unsigned int connectionsCount = connections.length();

        for ( unsigned int i = 0; i < connectionsCount; i++ )
        {
            MPlug connectedToPlug = connections[i];
            MObject nodeObj = connectedToPlug.node();
            MFnDependencyNode nodeFn ( nodeObj );
            MString typeName = nodeFn.typeName();

            if ( ( typeName == "animCurveTU" ) || ( typeName == "animCurveTL" )
                    || ( typeName == "animCurveTA" ) )
            {
                return true;
            }
        }

        return false;
    }

    //---------------------------------------------------
    // get the first empty item in the named array plug
    unsigned getFirstEmptyElementId ( const MPlug& parent )
    {
        unsigned num_element = 1;

        if ( parent.numElements() > num_element ) num_element = parent.numElements();

        for ( unsigned i = 0; i< num_element; i++ )
        {
            if ( !parent.elementByLogicalIndex ( i ).isConnected() ) return i;
        }

        return parent.numElements() +1;
    }
#endif

// Changes to from affect to, and everything that to effects.
void MayaDependencies::add(const MObject &from, const MObject &to)
{
    dependencies[&from].insert(&to);
}

// Apply all dependencies.
MStatus MayaDependencies::apply()
{
    set<const MObject *> stack;
    MStatus status = MStatus::kSuccess;

    // Apply each dependency from it source to its destination.
    for(map<const MObject*, set<const MObject*> >::iterator it = dependencies.begin(); it != dependencies.end(); ++it)
    {
        const MObject *from = it->first;
        const set<const MObject*> &deps = it->second;
        for(set<const MObject*>::iterator it2 = deps.begin(); it2 != deps.end(); ++it2)
        {
            MStatus status = apply_one(from, *it2, stack);
            if(status != MS::kSuccess) return status;
        }
    }

    return MStatus::kSuccess;
}

MStatus MayaDependencies::apply_one(const MObject *from, const MObject *to, set<const MObject*> &stack)
{
    MStatus status = MS::kSuccess;

    // If this assertion triggers, there's a circular dependency.
    assert(stack.find(to) == stack.end());

    status = MPxNode::attributeAffects(*from, *to);
    if(status != MS::kSuccess) return status;

    stack.insert(to);

    const set<const MObject *> &deps = dependencies[to];
    for(set<const MObject*>::iterator it = deps.begin(); it != deps.end(); ++it)
    {
        status = apply_one(from, *it, stack);
        if(status != MS::kSuccess) break;
    }

    stack.erase(to);

    return status;
}

}



