// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef DY_PATCH_FRICTION_H
#define DY_PATCH_FRICTION_H

#include "DyCorrelationBuffer.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"

namespace physx
{
namespace Dy
{

// Uniform pressure over a planar face overlap, or the contact hull fallback.
// These are friction integration samples, not collision/normal contacts.
struct PatchFrictionSamples
{
	static const PxU32 SAMPLE_COUNT = 32;
	// Offsets from the first contact point, avoiding a rounded world-space round trip.
	PxVec3 primary[SAMPLE_COUNT], secondary[SAMPLE_COUNT];
	PxReal primaryWeight[SAMPLE_COUNT], secondaryWeight[SAMPLE_COUNT];
};

namespace PatchFrictionIntegration
{
struct Point
{
	double x, y;
};
// Two convex faces have at most 255 vertices each; their intersection
// has at most the sum. Clipping cells adds at most four vertices.
static const PxU32 CAPACITY = 2 * PxContactBuffer::MAX_CONTACTS + 4;

PX_FORCE_INLINE double cross(const Point& a, const Point& b, const Point& c)
{
	return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}

PX_FORCE_INLINE PxU32 hull(Point* p, PxU32 count, Point* out)
{
	for(PxU32 i=1; i<count; ++i)
	{
		const Point value=p[i];
		PxU32 j=i;
		while(j && (p[j-1].x>value.x || (p[j-1].x==value.x && p[j-1].y>value.y)))
		{
			p[j]=p[j-1];
			--j;
		}
		p[j]=value;
	}
	PxU32 unique=0;
	for(PxU32 i=0; i<count; ++i)
		if(!unique || p[i].x!=p[unique-1].x || p[i].y!=p[unique-1].y)
			p[unique++]=p[i];
	if(unique<2)
	{
		if(unique) out[0]=p[0];
		return unique;
	}
	PxU32 n=0;
	for(PxU32 i=0; i<unique; ++i)
	{
		while(n>=2 && cross(out[n-2],out[n-1],p[i])<=0.) --n;
		out[n++]=p[i];
	}
	const PxU32 lower=n+1;
	for(PxI32 i=PxI32(unique)-2; i>=0; --i)
	{
		while(n>=lower && cross(out[n-2],out[n-1],p[i])<=0.) --n;
		out[n++]=p[i];
	}
	return n-1;
}

PX_FORCE_INLINE PxU32 clip(const Point* in, PxU32 count, Point* out,
	PxU32 axis, double bound)
{
	if(!count) return 0;
	Point previous=in[count-1];
	double pv=axis ? previous.y : previous.x;
	bool wasInside=pv<=bound;
	PxU32 n=0;
	for(PxU32 i=0; i<count; ++i)
	{
		const Point current=in[i];
		const double cv=axis ? current.y : current.x;
		const bool inside=cv<=bound;
		if(inside!=wasInside)
		{
			const double t=(bound-pv)/(cv-pv);
			out[n++]=Point{previous.x+t*(current.x-previous.x), previous.y+t*(current.y-previous.y)};
		}
		if(inside) out[n++]=current;
		previous=current;
		pv=cv;
		wasInside=inside;
	}
	PX_ASSERT(n<CAPACITY);
	return n;
}

// Return twice the area and six times the first moments, without normalizing.
PX_FORCE_INLINE double moments(const Point* p, PxU32 count, Point& firstMoment)
{
	firstMoment=Point{0.,0.};
	if(count<3) return 0.;
	double twiceArea=0.;
	Point previous=p[count-1];
	for(PxU32 i=0; i<count; ++i)
	{
		const double a=previous.x*p[i].y-p[i].x*previous.y;
		twiceArea+=a;
		firstMoment.x+=(previous.x+p[i].x)*a;
		firstMoment.y+=(previous.y+p[i].y)*a;
		previous=p[i];
	}
	return twiceArea;
}

// Project a face relative to the contact origin. Subtract the translations first
// so distant world coordinates do not round away the local footprint detail.
PX_FORCE_INLINE Point projectVertex(const PxVec3& vertex, const PxTransform& pose, const PxVec3& origin,
									const PxVec3& u, const PxVec3& v)
{
	const PxVec3 rotated = pose.rotate(vertex);
	const double x = double(pose.p.x) - origin.x + rotated.x;
	const double y = double(pose.p.y) - origin.y + rotated.y;
	const double z = double(pose.p.z) - origin.z + rotated.z;
	return Point{x * u.x + y * u.y + z * u.z, x * v.x + y * v.y + z * v.z};
}

PX_FORCE_INLINE PxU32 projectFace(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& outward,
								  const PxVec3& origin, const PxVec3& u, const PxVec3& v, Point* points)
{
	const PxVec3 localNormal = pose.rotateInv(outward);
	PxU32 count = 0;
	if(geometry.getType() == PxGeometryType::eBOX)
	{
		const PxVec3& half = static_cast<const PxBoxGeometry&>(geometry).halfExtents;
		const PxVec3 absolute = localNormal.abs();
		const PxU32 axis =
			absolute.x > absolute.y ? (absolute.x > absolute.z ? 0u : 2u) : (absolute.y > absolute.z ? 1u : 2u);
		if(absolute[axis] < 0.999f)
			return 0;
		const PxU32 a = (axis + 1) % 3, b = (axis + 2) % 3;
		for(PxU32 i = 0; i < 4; ++i)
		{
			PxVec3 vertex(0.f);
			vertex[axis] = localNormal[axis] > 0.f ? half[axis] : -half[axis];
			vertex[a] = i == 1 || i == 2 ? half[a] : -half[a];
			vertex[b] = i >= 2 ? half[b] : -half[b];
			points[i] = projectVertex(vertex, pose, origin, u, v);
		}
		count = 4;
	}
	else if(geometry.getType() == PxGeometryType::eCONVEXMESH)
	{
		const PxConvexMeshGeometry& convex = static_cast<const PxConvexMeshGeometry&>(geometry);
		const PxConvexMesh& mesh = *convex.convexMesh;
		const PxMat33 inverseScale = convex.scale.getInverse().toMat33();
		PxHullPolygon face;
		PxReal best = 0.999f;
		for(PxU32 i = 0, n = mesh.getNbPolygons(); i < n; ++i)
		{
			PxHullPolygon candidate;
			mesh.getPolygonData(i, candidate);
			const PxVec3 normal = inverseScale * PxVec3(candidate.mPlane[0], candidate.mPlane[1], candidate.mPlane[2]);
			const PxReal dot = normal.dot(localNormal);
			// Reject before normalizing; scaling can change the face normal.
			const PxReal lengthSquared = normal.magnitudeSquared();
			if(dot <= 0.f || dot * dot <= best * best * lengthSquared)
				continue;
			best = dot * PxRecipSqrt(lengthSquared);
			face = candidate;
			count = face.mNbVerts;
		}
		if(!count)
			return 0;
		const PxMat33 scale = convex.scale.toMat33();
		const PxVec3* vertices = mesh.getVertices();
		const PxU8* indices = mesh.getIndexBuffer() + face.mIndexBase;
		for(PxU32 i = 0; i < count; ++i)
			points[i] = projectVertex(scale * vertices[indices[i]], pose, origin, u, v);
	}
	else
		return 0;

	Point moment;
	const double area = moments(points, count, moment);
	if(area == 0.)
		return 0;
	if(area < 0.)
		for(PxU32 i = 0; i < count / 2; ++i)
		{
			const Point value = points[i];
			points[i] = points[count - 1 - i];
			points[count - 1 - i] = value;
		}
	return count;
}

PX_FORCE_INLINE PxU32 intersectFaces(Point* polygon, PxU32 count, const Point* other, PxU32 otherCount)
{
	Point scratch[CAPACITY];
	for(PxU32 edge = 0; edge < otherCount && count; ++edge)
	{
		const Point a = other[edge], b = other[(edge + 1) % otherCount];
		Point previous = polygon[count - 1];
		double previousDistance = cross(a, b, previous);
		PxU32 n = 0;
		for(PxU32 i = 0; i < count; ++i)
		{
			const Point current = polygon[i];
			const double distance = cross(a, b, current);
			if((distance >= 0.) != (previousDistance >= 0.))
			{
				if(n == CAPACITY)
					return 0;
				const double t = previousDistance / (previousDistance - distance);
				scratch[n++] =
					Point{previous.x + t * (current.x - previous.x), previous.y + t * (current.y - previous.y)};
			}
			if(distance >= 0.)
			{
				if(n == CAPACITY)
					return 0;
				scratch[n++] = current;
			}
			previous = current;
			previousDistance = distance;
		}
		count = n;
		for(PxU32 i = 0; i < count; ++i)
			polygon[i] = scratch[i];
	}
	// Leave room for the integration-cell cuts, including any duplicate vertices
	// produced by numerically coincident clipping edges. Overflow uses the contact hull.
	return count <= CAPACITY - 4 ? count : 0;
}

PX_FORCE_INLINE PxU32 geometryFootprint(const PxContactPoint& first, const PxContactPoint* contacts,
										const PxU32* indices, PxU32 count, const PxVec3& u, const PxVec3& v,
										Point* polygon)
{
	const PxContactFrictionPatch* data = first.getAnisotropy()->getPatchData();
	// A split or partially ignored pair must not apply the entire footprint to
	// each surviving patch. Respect callback point edits and use its contact hull.
	if(!data || count != data->contactCount || count < 3 || first.normal.dot(data->normal) < 0.999f)
		return 0;
	for(PxU32 i = 0; i < count; ++i)
	{
		const PxContactPoint& contact = contacts[indices[i]];
		const PxContactAnisotropy& extra = *contact.getAnisotropy();
		if(extra.getPatchData() != data || contact.point != extra.originalPoint)
			return 0;
	}
	const PxGeometryType::Enum type0 = data->geometry[0]->getType(), type1 = data->geometry[1]->getType();
	const bool plane0 = type0 == PxGeometryType::ePLANE, plane1 = type1 == PxGeometryType::ePLANE;
	if(plane0 || plane1)
	{
		const PxU32 plane = plane0 ? 0u : 1u, solid = 1 - plane;
		if(PxAbs(data->pose[plane].rotate(PxVec3(1.f, 0.f, 0.f)).dot(first.normal)) < 0.999f)
			return 0;
		return projectFace(*data->geometry[solid], data->pose[solid], solid ? first.normal : -first.normal, first.point,
						   u, v, polygon);
	}
	const PxU32 n = projectFace(*data->geometry[0], data->pose[0], -first.normal, first.point, u, v, polygon);
	if(!n)
		return 0;
	Point other[CAPACITY];
	const PxU32 m = projectFace(*data->geometry[1], data->pose[1], first.normal, first.point, u, v, other);
	return m ? intersectFaces(polygon, n, other, m) : 0;
}

PX_FORCE_INLINE void samples(const Point* polygon, PxU32 count, PxU32 axis, PxU32 resolution,
	const PxVec3& u, const PxVec3& v, PxVec3* points, PxReal* weights,
	double slipAtOrigin=0., double slipGradient=0.)
{
	Point wholeMoment;
	const double twiceArea=moments(polygon,count,wholeMoment);
	const Point center=twiceArea>0. ? Point{wholeMoment.x/(3.*twiceArea),wholeMoment.y/(3.*twiceArea)} : Point{0.,0.};
	double lo=axis ? polygon[0].y : polygon[0].x, hi=lo;
	for(PxU32 i=1; i<count; ++i)
	{
		const double coordinate=axis ? polygon[i].y : polygon[i].x;
		lo=PxMin(lo,coordinate);
		hi=PxMax(hi,coordinate);
	}
	// A strip straddling zero slip contains opposing tractions. Evaluating both
	// at its centroid leaves a dead band in the net force. Put a boundary at
	// the current slip reversal, retaining the same total number of samples.
	// The ordinary PGS rows still determine the coupled impulses each iteration.
	const double split=slipGradient!=0. ? -slipAtOrigin/slipGradient : lo;
	const PxU32 lowerCount=resolution>1 && split>lo && split<hi
		? PxMax(1u,PxMin(resolution-1,PxU32((split-lo)/(hi-lo)*resolution+.5))) : 0;
	Point clipped[CAPACITY];
	Point previousMoment={0.,0.};
	double previousArea=0.;
	double sum=0.;
	for(PxU32 i=0; i<resolution; ++i)
	{
		Point centroid;
		double fraction;
		if(twiceArea>0. && hi>lo)
		{
			// Adjacent cumulative cuts differ by exactly one strip. Reuse the
			// preceding cut instead of clipping both sides of every strip.
			Point upperMoment=wholeMoment;
			double upperArea=twiceArea;
			if(i+1<resolution)
			{
				const double cut=lowerCount
					? (i+1<=lowerCount ? lo+(split-lo)*(i+1)/lowerCount
						: split+(hi-split)*(i+1-lowerCount)/(resolution-lowerCount))
					: lo+(hi-lo)*(i+1)/resolution;
				const PxU32 n=clip(polygon,count,clipped,axis,cut);
				upperArea=moments(clipped,n,upperMoment);
			}
			const double cellArea=upperArea-previousArea;
			fraction=PxMax(0.,cellArea/twiceArea);
			centroid=cellArea>0. ? Point{(upperMoment.x-previousMoment.x)/(3.*cellArea),
				(upperMoment.y-previousMoment.y)/(3.*cellArea)} : center;
			previousArea=upperArea;
			previousMoment=upperMoment;
		}
		else
		{
			// The degenerate limits are uniform line contact and point contact.
			const double t=(i+.5)/resolution;
			centroid=Point{polygon[0].x+t*(polygon[count-1].x-polygon[0].x),
				polygon[0].y+t*(polygon[count-1].y-polygon[0].y)};
			fraction=1./resolution;
		}
		points[i]=u*PxReal(centroid.x)+v*PxReal(centroid.y);
		weights[i]=PxReal(fraction);
		sum+=weights[i];
	}
	PX_ASSERT(sum>0.);
	for(PxU32 i=0; i<resolution; ++i) weights[i]=PxReal(weights[i]/sum);
}
PX_FORCE_INLINE void gridSamples(const Point* polygon, PxU32 count, PxU32 resolution, const PxVec3& u, const PxVec3& v,
								 PatchFrictionSamples& result, double slipU, double slipV, double spin)
{
	Point wholeMoment;
	const double total = moments(polygon, count, wholeMoment);
	if(!(total > 0.) || resolution == 1)
	{
		samples(polygon, count, 1, resolution, u, v, result.primary, result.primaryWeight);
		for(PxU32 i = 0; i < resolution; ++i)
		{
			result.secondary[i] = result.primary[i];
			result.secondaryWeight[i] = result.primaryWeight[i];
		}
		return;
	}
	Point lo = polygon[0], hi = lo;
	for(PxU32 i = 1; i < count; ++i)
	{
		lo.x = PxMin(lo.x, polygon[i].x);
		lo.y = PxMin(lo.y, polygon[i].y);
		hi.x = PxMax(hi.x, polygon[i].x);
		hi.y = PxMax(hi.y, polygon[i].y);
	}
	const PxU32 nx = 4, ny = resolution / nx;
	const double splitX = spin != 0. ? slipV / spin : lo.x;
	const double splitY = spin != 0. ? -slipU / spin : lo.y;
	const PxU32 lowerX =
		splitX > lo.x && splitX < hi.x ? PxMax(1u, PxMin(nx - 1, PxU32((splitX - lo.x) / (hi.x - lo.x) * nx + .5))) : 0;
	const PxU32 lowerY =
		splitY > lo.y && splitY < hi.y ? PxMax(1u, PxMin(ny - 1, PxU32((splitY - lo.y) / (hi.y - lo.y) * ny + .5))) : 0;
	double cutsX[nx + 1], cutsY[PatchFrictionSamples::SAMPLE_COUNT / nx + 1];
	for(PxU32 k = 0; k <= nx; ++k)
		cutsX[k] = lowerX ? (k <= lowerX ? lo.x + (splitX - lo.x) * k / lowerX
										 : splitX + (hi.x - splitX) * (k - lowerX) / (nx - lowerX))
						  : lo.x + (hi.x - lo.x) * k / nx;
	for(PxU32 k = 0; k <= ny; ++k)
		cutsY[k] = lowerY ? (k <= lowerY ? lo.y + (splitY - lo.y) * k / lowerY
										 : splitY + (hi.y - splitY) * (k - lowerY) / (ny - lowerY))
						  : lo.y + (hi.y - lo.y) * k / ny;

	Point strip[CAPACITY], clipped[CAPACITY];
	double sum = 0.;
	for(PxU32 ix = 0; ix < nx; ++ix)
	{
		const double left = cutsX[ix];
		const double right = cutsX[ix + 1];
		PxU32 n = clip(polygon, count, clipped, 0, right);
		for(PxU32 j = 0; j < n; ++j)
			clipped[j].x = -clipped[j].x;
		n = clip(clipped, n, strip, 0, -left);
		for(PxU32 j = 0; j < n; ++j)
			strip[j].x = -strip[j].x;
		Point previousMoment = {0., 0.};
		double previousArea = 0.;
		for(PxU32 iy = 0; iy < ny; ++iy)
		{
			const PxU32 sample = ix * ny + iy;
			const double top = cutsY[iy + 1];
			const PxU32 m = clip(strip, n, clipped, 1, top);
			Point nextMoment;
			const double nextArea = moments(clipped, m, nextMoment);
			const double area = nextArea - previousArea;
			const double x = area > 0. ? (nextMoment.x - previousMoment.x) / (3. * area) : wholeMoment.x / (3. * total);
			const double y = area > 0. ? (nextMoment.y - previousMoment.y) / (3. * area) : wholeMoment.y / (3. * total);
			result.primary[sample] = result.secondary[sample] = u * PxReal(x) + v * PxReal(y);
			result.primaryWeight[sample] = result.secondaryWeight[sample] = PxReal(PxMax(0., area / total));
			sum += result.primaryWeight[sample];
			previousArea = nextArea;
			previousMoment = nextMoment;
		}
	}
	for(PxU32 i = 0; i < resolution; ++i)
		result.primaryWeight[i] = result.secondaryWeight[i] = PxReal(result.primaryWeight[i] / sum);
}


}


PX_FORCE_INLINE void buildPatchFrictionSamples(const CorrelationBuffer& c, PxU32 patch,
	const PxContactPoint* contacts, PatchFrictionSamples& samples,
	const PxVec3& slipAtOrigin=PxVec3(0), const PxVec3& relativeAngularVelocity=PxVec3(0))
{
	using namespace PatchFrictionIntegration;
	PxU32 indices[PxContactBuffer::MAX_CONTACTS];
	const PxU32 count=getFrictionContactIndices(c,patch,indices);
	PX_ASSERT(count);
	const PxContactPoint& first=contacts[indices[0]];
	const PxVec3 origin=first.point;
	const PxVec3 u=(first.getAnisotropy()->frictionDirection-first.normal*first.normal.dot(first.getAnisotropy()->frictionDirection)).getNormalized();
	const PxVec3 v=first.normal.cross(u);
	// The contact hull needs at most twice the contact count as workspace;
	// CAPACITY also accommodates the intersection of two full convex faces.
	Point points[PxContactBuffer::MAX_CONTACTS], polygon[CAPACITY];
	PxU32 vertices=geometryFootprint(first,contacts,indices,count,u,v,polygon);
	if(!vertices)
	{
		for(PxU32 i=0; i<count; ++i)
		{
			const PxVec3 offset=contacts[indices[i]].point-origin;
			points[i]=Point{offset.dot(u),offset.dot(v)};
		}
		vertices=hull(points,count,polygon);
	}
	const PxU32 resolution=c.getAreaSampleCount(patch);
	PX_ASSERT(vertices && vertices<CAPACITY && resolution<=PatchFrictionSamples::SAMPLE_COUNT);
	// Slip in the u direction varies only along v on a planar rigid patch;
	// both tangent rows share each cell when the friction ellipse has two axes.
	const double spin=relativeAngularVelocity.dot(first.normal);
	const PxContactAnisotropy& material=*first.getAnisotropy();
	if((first.staticFriction>0 || first.dynamicFriction>0) && (material.staticFrictionSecondary>0 || material.dynamicFrictionSecondary>0))
	{
		gridSamples(polygon,vertices,resolution,u,v,samples,slipAtOrigin.dot(u),slipAtOrigin.dot(v),spin);
		return;
	}
	PatchFrictionIntegration::samples(polygon,vertices,1,resolution,u,v,samples.primary,samples.primaryWeight,slipAtOrigin.dot(u),spin);
	PatchFrictionIntegration::samples(polygon,vertices,0,resolution,u,v,samples.secondary,samples.secondaryWeight,slipAtOrigin.dot(v),-spin);
}

}
}
#endif
