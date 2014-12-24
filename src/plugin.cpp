//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

//
//  File: offset.cc
//
//  Description:
//         Example implementation of a deformer. This node
//        offsets vertices according to the CV's weights.
//        The weights are set using the set editor or the
//        percent command.
//

#include <string.h>
#include <maya/MIOStream.h>
#include <math.h>

#include <maya/MPxDeformerNode.h> 
#include <maya/MItGeometry.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MPxLocatorNode.h> 
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnPlugin.h>
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

#include "loader_mesh.hpp"
#include "loader_skel.hpp"

#include "Interface.h"

#include <algorithm>
#include <map>
using namespace std;







namespace DagHelpers
{
    bool getPlugConnectedTo(const MObject& node, const MString &attribute, MPlug& connectedPlug);

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

    //---------------------------------------------------
    bool getPlugValue ( const MPlug& plug, float& x, float& y, float& z )
    {
        MObject obj;
        plug.getValue ( obj );
        MStatus status;
        MFnNumericData fcolor ( obj, &status );
        if ( !status ) return 0;

        fcolor.getData ( x , y , z );

        return 1;
    }

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
    bool setPlugValue ( MPlug& plug, float x, float y )
    {
        MFnNumericData data;
        MObject obj = data.create ( MFnNumericData::k2Float );
        data.setData ( x, y );
        return plug.setValue ( obj );
    }

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
}








class offset : public MPxDeformerNode
{
public:
    static  MTypeId        id;

    virtual ~offset() { }

    static void *creator() { return new offset(); }
    static MStatus initialize();

    MObject &accessoryAttribute() const;
    MStatus accessoryNodeSetup(MDagModifier& cmd);

//    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

private:
    // local node attributes
    static  MObject     offsetMatrix;     // offset center and axis
};

MTypeId     offset::id( 0x11229090 );

// local attributes
MObject        offset::offsetMatrix;

MStatus offset::initialize()
{
    // local attribute initialization

    MFnMatrixAttribute  mAttr;
    offsetMatrix=mAttr.create( "locateMatrix", "lm");
        mAttr.setStorable(false);
        mAttr.setConnectable(true);

    //  deformation attributes
    addAttribute( offsetMatrix);

    attributeAffects( offset::offsetMatrix, offset::outputGeom );

    return MStatus::kSuccess;
}

MStatus getConnectedPlugWithName(MPlug inputPlug, string name, MPlug &result)
{
    MStatus status = MStatus::kSuccess;

    MFnDependencyNode inputPlugDep(inputPlug.node());
    MObject geomObj = inputPlugDep.attribute(name.c_str(), &status);
    if(status != MS::kSuccess)
    {
        fprintf(stderr, "error finding inputGeom\n");
        return status;
    }

    MPlug geomObjPlug(inputPlug.node(), geomObj);
    fprintf(stderr, "inputGeom: %s\n", geomObjPlug.name().asChar());

    
    MPlugArray connPlugs;
    geomObjPlug.connectedTo(connPlugs, true, false);
    int connLength = connPlugs.length();
    if(connLength == 0) {
        fprintf(stderr, "no connection\n");
        return status;
    }

    result = connPlugs[0];
    return MStatus::kSuccess;
}

static int compare_length(const MDagPath &lhs, const MDagPath &rhs) { return lhs.fullPathName().length() < rhs.fullPathName().length(); }

// Find the nearest ancestor to path.  "a|b|c" is an ancestor of "a|b|c|d|e|f".
// If no nodes are an ancestor of path, return -1.
int find_closest_ancestor(const vector<MDagPath> &dagPaths, MDagPath dagPath)
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

Loader::CpuTransfo MMatrixToCpuTransfo(const MMatrix &dagMat)
{
    Loader::CpuTransfo mat;
    mat[0] = (float) dagMat[0][0];
    mat[1] = (float) dagMat[1][0];
    mat[2] = (float) dagMat[2][0];
    mat[3] = (float) dagMat[3][0];
    mat[4] = (float) dagMat[0][1];
    mat[5] = (float) dagMat[1][1];
    mat[6] = (float) dagMat[2][1];
    mat[7] = (float) dagMat[3][1];
    mat[8] = (float) dagMat[0][2];
    mat[9] = (float) dagMat[1][2];
    mat[10] = (float) dagMat[2][2];
    mat[11] = (float) dagMat[3][2];
    mat[12] = (float) dagMat[0][3];
    mat[13] = (float) dagMat[1][3];
    mat[14] = (float) dagMat[2][3];
    mat[15] = (float) dagMat[3][3];
    return mat;
}

MStatus load_mesh(MObject inputObject, Loader::Abs_mesh &mesh)
{
    MStatus status = MS::kSuccess;
//        loader.get_mesh( mesh );
//        if(mesh._vertices.size() == 0)
//            return;

//        ptr_mesh->load( mesh, loader._path);
//        Cuda_ctrl::load_mesh( ptr_mesh );


// vertices ################################################################
    MItMeshVertex meshIt(inputObject);

    int num_verts = meshIt.count(&status);
    if(status != MS::kSuccess) return status;

    mesh._vertices.resize(num_verts);
    mesh._normals.resize(num_verts);

    int idx = 0;
    for ( ; !meshIt.isDone(); meshIt.next())
    {
        MPoint point = meshIt.position(MSpace::kWorld, &status);
        if(status != MS::kSuccess) return status;

        mesh._vertices[idx] = Loader::Vertex((float)point.x, (float)point.y, (float)point.z);

        // XXX: What are we supposed to do with unshared normals?
        MVectorArray normalArray;
        status = meshIt.getNormals(normalArray);
//        status = meshIt.getNormal(normal, 0, MSpace::kWorld);
//        status = meshIt.getNormal(normal, MSpace::kWorld);
        if(status != MS::kSuccess) return status;

        MVector normal = normalArray[0];
        mesh._normals[idx] = Loader::Normal((float)normal[0], (float)normal[1], (float)normal[2]);

        ++idx;
    }

    MItMeshPolygon polyIt(inputObject);
    int num_faces = polyIt.count(&status);
    if(status != MS::kSuccess) return status;

    mesh._triangles.resize(num_faces);

    idx = 0;
    for ( ; !polyIt.isDone(); polyIt.next())
    {
        if(polyIt.polygonVertexCount() < 3)
            continue;

        // Load only the first three points of the face, since the data structure only understands
        // triangles.  We could triangulate the faces, but we only use it for calculating normals,
        // and the result is the same taking only the first tri so long as the face is planar.
        Loader::Tri_face f;
        for(unsigned faceIdx = 0; faceIdx < polyIt.polygonVertexCount() && faceIdx < 3; ++faceIdx)
        {
            int vertexIndex = polyIt.vertexIndex(faceIdx, &status);
            if(status != MS::kSuccess) return status;

            f.v[faceIdx] = vertexIndex;
            f.n[faceIdx] = vertexIndex; // XXX?
        }

        mesh._triangles[idx] = f;
        //MPointArray pointArray;
        //polyIt.getPoints(pointArray, MSpace::kWorld, &status);
        //if(status != MS::kSuccess) return status;


        ++idx;
    }

    return MS::kSuccess;
}

MStatus loadSkeletonFromSkinCluster(const MFnSkinCluster &skinCluster, Loader::Abs_skeleton &skeleton)
{
    // Get the influence objects (joints) for the skin cluster.
    MStatus status = MStatus::kSuccess;
    MDagPathArray paths;
    skinCluster.influenceObjects(paths, &status);
    if(status != MS::kSuccess)
        return status;

    // Convert to a vector.
    vector<MDagPath> joints;
    for(unsigned i = 0; i < paths.length(); ++i) 
        joints.push_back(paths[i]);

    // Sort the influence objects by the length of their full path.  Since the name of
    // an object is prefixed by its parents, eg. "parent1|parent2|object", this guarantees
    // that a parent is before all of its children.
    sort(joints.begin(), joints.end(), compare_length);

    // Create a root bone.
    // XXX put this back
    skeleton._root = 0;
/*    {
        Loader::Abs_bone bone;
        bone._name = "root";
        bone._frame = Loader::CpuTransfo::identity();
        skeleton._bones.push_back(bone);
        skeleton._sons.push_back(std::vector<int>());
        skeleton._parents.push_back(-1);
        skeleton._root = 0;
    }
    */
    // Create the bones.
    for(int i = 0; i < joints.size(); ++i)
    {
        const MDagPath &dagPath = joints[i];
        int boneIdx = (int) skeleton._bones.size(); 
        string jointName = dagPath.fullPathName().asChar();

        // Fill in the bone.  We don't need to calculate _length, since compute_bone_lengths will
        // do it below.
        Loader::Abs_bone bone;
        bone._name = jointName;
        bone._frame = MMatrixToCpuTransfo(dagPath.inclusiveMatrix());
        skeleton._bones.push_back(bone);

        // Find this bone's closest ancestor to be its parent.  If it has no ancestors, use the root.
        int parentIdx = find_closest_ancestor(joints, dagPath);
        if(i == 0)
            parentIdx = -1; // XXX
/*        if(parentIdx == -1)
            parentIdx = 0;
        else
            parentIdx++; // skip the root bone added above
            */
        skeleton._parents.push_back(parentIdx);

        // Add this bone as a child of its parent.
        skeleton._sons.push_back(std::vector<int>());
        if(parentIdx != -1)
            skeleton._sons[parentIdx].push_back(boneIdx);
    }

    Loader::compute_bone_lengths(skeleton);
    return MStatus::kSuccess;
}

MStatus offset::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;
        
    if (plug.attribute() != outputGeom)
        return MStatus::kUnknownParameter;

    // The logical index of the output that we've been told to compute:
    unsigned int logicalOutputIndex = plug.logicalIndex();

    // Get the corresponding input plug for this output:
    // MObject inputAttr = MFnDependencyNode(plug.node()).attribute("input", &status); // == this.input
    MPlug inputPlug(plug.node(), input);
    fprintf(stderr, "inPlug %s %i\n", inputPlug.name().asChar(), inputPlug.numConnectedElements());

    // Select input[index].
    inputPlug.selectAncestorLogicalIndex(logicalOutputIndex, input);

    // Select the connection to input[index], which should be a groupParts node.
    MPlug groupParts;
    status = getConnectedPlugWithName(inputPlug, "inputGeometry", groupParts);
    if(groupParts.node().apiType() != MFn::kGroupParts)
    {
        fprintf(stderr, "Expected to find a groupParts above the deformer, found a %s instead\n", groupParts.node().apiTypeStr());
        return MStatus::kUnknownParameter;
    }

    // The input geometry plugged into the groupParts node.  This is normally the next
    // deformer in the chain, or the initial geometry if we're the first deformer.  We
    // expect to be after skinning, since we need to work on geometry after skinning.
    // XXX: There might be other deformers between us and the skinCluster; skip past them
    // the deformers until we find it.  However, this all needs to be moved out of compute()
    // and into a separate method, and that should probably be done in Python and not native
    // anyway, so let's solve that when that happens.
    MPlug skinClusterPlug;
    status = getConnectedPlugWithName(groupParts, "inputGeometry", skinClusterPlug);
    if(skinClusterPlug.node().apiType() != MFn::kSkinClusterFilter)
    {
        fprintf(stderr, "Expected to find a skinCluster above the deformer, found a %s instead\n", skinClusterPlug.node().apiTypeStr());
        return MStatus::kUnknownParameter;
    }


    MFnSkinCluster skinCluster(skinClusterPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    Loader::Abs_skeleton skeleton;
    status = loadSkeletonFromSkinCluster(skinCluster, skeleton);
    if(status != MS::kSuccess) return status;








    MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status);
    if(status != MS::kSuccess) return status;
    inputArray.jumpToElement(logicalOutputIndex);

    MDataHandle inputGeomData = inputArray.inputValue(&status);
    if(status != MS::kSuccess) return status;

    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);














    MObject inputObject = inputGeomDataHandle.data();
    if (inputObject.apiType() != MFn::kMeshData) {
        // XXX: only meshes are supported
        return MS::kFailure;
    }

    MFnMesh myMesh(inputObject, &status);
    if(status != MS::kSuccess) {
        printf("Couldn't get input geometry as mesh\n");
        return status;
    }

    // Mesh* ptr_mesh = new Mesh();
    Loader::Abs_mesh mesh;
    status = load_mesh(inputObject, mesh);
    if(status != MS::kSuccess) return status;

    vector<Loader::Vec3> result_verts;
    PluginInterface::go(mesh, skeleton, result_verts);






    MDataHandle hInput = dataBlock.inputValue(inputPlug);
//    MDataHandle inputGeomDataHandle = hInput.child(inputGeom);

    // The groupId of this groupParts node:
    MDataHandle hGroup = hInput.child(groupId);
    unsigned int groupId = hGroup.asLong();
    MDataHandle hOutput = dataBlock.outputValue(plug);
    hOutput.copy(inputGeomDataHandle);

    // do the deformation
    MItGeometry iter(hOutput, groupId, false);
    int idx = 0;
    for ( ; !iter.isDone(); iter.next()) {
        if(idx >= result_verts.size())
            break;
        MPoint pt = iter.position();

        Loader::Vec3 v = result_verts[idx];
        pt = MPoint(v.x, v.y, v.z);
        iter.setPosition(pt, MSpace::kWorld);
        idx++;
    }

    return MStatus::kSuccess;
}

// Description:   Deform the point with a squash algorithm
//
// Arguments:
//   block        : the datablock of the node
//     iter        : an iterator for the geometry to be deformed
//   m            : matrix to transform the point into world space
//     multiIndex : the index of the geometry that we are deforming
#if 0
MStatus offset::deform(MDataBlock& block, MItGeometry &iter, const MMatrix& m, unsigned int geomIndex)
{
    MStatus status;

    MArrayDataHandle inputArray = block.inputArrayValue(input, &status);
    if(status != MS::kSuccess) return status;
    inputArray.jumpToElement(geomIndex);

    MDataHandle inputGeomData = inputArray.inputValue(&status);
    if(status != MS::kSuccess) return status;

    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);
    MObject inputObject = inputGeomDataHandle.data();

    if (inputObject.apiType() != MFn::kMeshData) {
        // XXX: only meshes are supported
        return status;
    }

    MFnMesh myMesh(inputObject, &status);
    if(status != MS::kSuccess) {
        cout << "Couldn't get input geometry as mesh" << endl;
        return status;
    }

    // Mesh* ptr_mesh = new Mesh();
    Loader::Abs_mesh mesh;
    load_mesh(inputObject, mesh);


    PluginInterface::go(mesh);



    
    // Envelope data from the base class.
    // The envelope is simply a scale factor.
    MDataHandle envData = block.inputValue(envelope, &status);
    if(status != MS::kSuccess) return status;
    float env = envData.asFloat();    

    // Get the matrix which is used to define the direction and scale
    // of the offset.
    MDataHandle matData = block.inputValue(offsetMatrix, &status);
    if (status != MS::kSuccess) return status;
    MMatrix omat = matData.asMatrix();
    MMatrix omatinv = omat.inverse();

    for ( ; !iter.isDone(); iter.next()) {
        MPoint pt = iter.position();
        pt *= omatinv;
        
        float weight = weightValue(block, geomIndex, iter.index());
        pt.y = pt.y + env*weight;

        pt *= omat;
        iter.setPosition(pt);
    }

    return MS::kSuccess;
}
#endif

//      This method returns a the attribute to which an accessory    
//    shape is connected. If the accessory shape is deleted, the deformer
//      node will automatically be deleted.
//
//    This method is optional.
MObject &offset::accessoryAttribute() const
{
    return offset::offsetMatrix;
}

//        This method is called when the deformer is created by the
//        "deformer" command. You can add to the cmds in the MDagModifier
//        cmd in order to hook up any additional nodes that your node needs
//        to operate.
//
//        In this example, we create a locator and attach its matrix attribute
//        to the matrix input on the offset node. The locator is used to
//        set the direction and scale of the random field.
//
//    Description:
//        This method is optional.
//
MStatus offset::accessoryNodeSetup(MDagModifier& cmd)
{
    MStatus result;

    // hook up the accessory node
    MObject objLoc = cmd.createNode(MString("locator"), MObject::kNullObj, &result);
    if (result != MS::kSuccess == result)
        return result;

    MFnDependencyNode fnLoc(objLoc);
    MString attrName;
    attrName.set("matrix");
    MObject attrMat = fnLoc.attribute(attrName);

    return cmd.connect(objLoc,attrMat,this->thisMObject(),offset::offsetMatrix);
}

MStatus initializePlugin(MObject obj)
{
    PluginInterface::init();

    MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");
    return plugin.registerNode( "offset", offset::id, offset::creator, 
                                  offset::initialize, MPxNode::kDeformerNode );
}

MStatus uninitializePlugin(MObject obj)
{
    PluginInterface::shutdown();

    MFnPlugin plugin(obj);
    return plugin.deregisterNode(offset::id);
}
