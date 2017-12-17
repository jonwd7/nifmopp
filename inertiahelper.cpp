// ***** BEGIN LICENSE BLOCK *****
//
// Copyright (c) 2006-2008, NIF File Format Library and Tools.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//    * Neither the name of the NIF File Format Library and Tools
//      project nor the names of its contributors may be used to endorse
//      or promote products derived from this software without specific
//      prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ***** END LICENSE BLOCK *****

#include "NifMopp.h"

//
// Math and base include


#include <Physics/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>
#include <Common/Base/Types/Geometry/hkGeometry.h>
#include <Common/Base/Types/Geometry/hkStridedVertices.h>
#include <Common/Internal/ConvexHull/hkGeometryUtility.h>

#pragma comment(lib, "hkBase.lib")
#pragma comment(lib, "hkSerialize.lib")
#pragma comment(lib, "hkpInternal.lib")
#pragma comment(lib, "hkpUtilities.lib")
#pragma comment(lib, "hkpCollide.lib")
#pragma comment(lib, "hkpConstraintSolver.lib")
#pragma comment(lib, "hkpDynamics.lib")

#include <float.h>

#ifdef _MANAGED
#pragma managed(push, off)
#endif

/*! Return mass and inertia matrix for a sphere of given radius and
*	density.
*/
extern "C" 
void __stdcall CalcMassPropertiesSphere(float radius, 
									 float density, bool solid, 
									 float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkReal hkRadius = radius;
	hkReal hkMass = scaleDensity ? 1.0f : mass;
	__try
	{
		if (solid)
			hkpInertiaTensorComputer::computeSphereVolumeMassProperties(hkRadius, hkMass, massProperties);
		else
			hkpInertiaTensorComputer::computeSphereSurfaceMassProperties(hkRadius, hkMass, FLT_EPSILON, massProperties);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
	}
	if (scaleDensity)
		massProperties.scaleToDensity(density);
	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}

/*! Return mass and inertia matrix for a box of given size and
*   density.
*/
extern "C" 
void __stdcall CalcMassPropertiesBox(Point3 size, 
								  float density, bool solid, 
								  float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkVector4 boxSize ( size.x, size.y, size.z );
	hkReal hkMass = scaleDensity ? 1.0f : mass;
	__try
	{
		if (solid)
			hkpInertiaTensorComputer::computeBoxVolumeMassProperties(boxSize, hkMass, massProperties);
		else
			hkpInertiaTensorComputer::computeBoxSurfaceMassProperties(boxSize, hkMass, FLT_EPSILON, massProperties);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
	}
	if (scaleDensity)
		massProperties.scaleToDensity(density);
	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}
/*! Return mass and inertia matrix for a cylinder of given radius, 
*   height and density.
*/
extern "C"
void __stdcall CalcMassPropertiesCylinder(Point3 startAxis, Point3 endAxis, float radius,
									   float density, bool solid,
									   float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkReal hkRadius = radius;
	hkReal hkMass = scaleDensity ? 1.0f : mass;
	hkVector4 hkStart(startAxis.x, startAxis.y, startAxis.z), hkEnd(endAxis.x, endAxis.y, endAxis.z);
	__try
	{
		if (solid)
			hkpInertiaTensorComputer::computeCylinderVolumeMassProperties(hkStart, hkEnd, hkRadius, hkMass, massProperties);
		else
			hkpInertiaTensorComputer::computeCylinderVolumeMassProperties(hkStart, hkEnd, hkMass, FLT_EPSILON, massProperties);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
	}
	if (scaleDensity)
		massProperties.scaleToDensity(density);
	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}

/*! Return mass and inertia matrix for a capsule of given radius, 
*	height and density.
*/
extern "C"
void __stdcall CalcMassPropertiesCapsule(Point3 startAxis, Point3 endAxis, float radius,
									  float density, bool solid,
									  float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkReal hkRadius = radius;
	hkReal hkMass = scaleDensity ? 1.0f : mass;
	hkVector4 hkStart(startAxis.x, startAxis.y, startAxis.z), hkEnd(endAxis.x, endAxis.y, endAxis.z);
	__try
	{
		if (solid)
			hkpInertiaTensorComputer::computeCapsuleVolumeMassProperties(hkStart, hkEnd, hkRadius, hkMass, massProperties);
		else
			hkpInertiaTensorComputer::computeCapsuleVolumeMassProperties(hkStart, hkEnd, hkRadius, hkMass, massProperties);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
	}
	if (scaleDensity)
		massProperties.scaleToDensity(density);
	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}

/*! Return mass and inertia matrix for a capsule of given radius, 
*	height and density.
*/
extern "C"
void __stdcall CalcMassPropertiesPolyhedron(
	int nVerts, Point3 const* verts, 
	int nTris, Triangle const *tris,
	float density, bool solid,
	float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	//hkGeometry geom;
	//for (int i=0;i<nVerts; ++i)
	//	geom.m_vertices.pushBack( hkVector4(verts[i].x, verts[i].y, verts[i].z) );
	//for (int i=0;i<nTris; ++i) {
	//	hkGeometry::Triangle t; t.set(tris[i].a, tris[i].b, tris[i].c);
	//	geom.m_triangles.pushBack( t );
	//}
	hkStridedVertices vertsIn;
	vertsIn.m_vertices = reinterpret_cast<const float *>(&verts[0]);
	vertsIn.m_numVertices = nVerts;
	vertsIn.m_striding = sizeof(Point3);
	hkGeometry* geom = new hkGeometry();
	hkInplaceArrayAligned16<hkVector4,32> transformedPlanes;
	hkGeometryUtility::createConvexGeometry(vertsIn, *geom, transformedPlanes);


	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkReal hkMass = scaleDensity ? 1.0f : mass;
	try
	{
		if (solid)
			hkpInertiaTensorComputer::computeGeometryVolumeMassPropertiesChecked(geom, hkMass, massProperties);
		else
			hkpInertiaTensorComputer::computeGeometrySurfaceMassProperties(geom, FLT_EPSILON, true, hkMass, massProperties);
	} catch (...) {}
	delete geom;

	if (scaleDensity)
		massProperties.scaleToDensity(density);
	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}


extern "C"
void __stdcall CombineMassProperties(
	int nItems,
	float* masses, float* volumes, Point3* centers, Matrix43* inertias, Matrix44* transforms,
	float& mass, float& volume, Point3& center, Matrix43 &inertia)
{
	InitializeHavok();
	bool scaleDensity = (mass == 0.0f);
	hkpMassProperties massProperties;
	hkReal hkMass = scaleDensity ? 1.0f : mass;

	hkArray<hkpMassElement> elements;

	for (int i=0; i<nItems; ++i){
		hkpMassElement elem;
		elem.m_transform.set4x4ColumnMajor(reinterpret_cast<hkReal*>(&transforms[i]));
		elem.m_properties.m_mass = masses[i];
		elem.m_properties.m_volume = volumes[i];
		elem.m_properties.m_centerOfMass.set(centers[i].x, centers[i].y, centers[i].z);
		memcpy(&elem.m_properties.m_inertiaTensor, &inertias[i], sizeof(elem.m_properties.m_inertiaTensor));
		elements.pushBack(elem);
	}

	try
	{
		hkpInertiaTensorComputer::combineMassProperties(elements, massProperties);
	}
	catch(...)
	{
	}

	mass = massProperties.m_mass;
	volume = massProperties.m_volume;
	center.Set(massProperties.m_centerOfMass(0), massProperties.m_centerOfMass(1), massProperties.m_centerOfMass(2));
	memcpy(&inertia, &massProperties.m_inertiaTensor, sizeof(inertia));
}


#ifdef _MANAGED
#pragma managed(pop)
#endif

