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

namespace physx
{
namespace Dy
{

// Uniform pressure over the convex hull of a coplanar contact manifold.
// These are friction integration samples, not collision/normal contacts.
struct PatchFrictionSamples
{
	static const PxU32 SAMPLE_COUNT = 16;
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
static const PxU32 CAPACITY = PxContactBuffer::MAX_CONTACTS + 4;

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

PX_FORCE_INLINE void samples(const Point* polygon, PxU32 count, PxU32 axis, PxU32 resolution,
	const PxVec3& u, const PxVec3& v, PxVec3* points, PxReal* weights)
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
				const PxU32 n=clip(polygon,count,clipped,axis,lo+(hi-lo)*(i+1)/resolution);
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
}

PX_FORCE_INLINE void buildPatchFrictionSamples(const CorrelationBuffer& c, PxU32 patch,
	const PxContactPoint* contacts, PatchFrictionSamples& samples)
{
	using namespace PatchFrictionIntegration;
	PxU32 indices[PxContactBuffer::MAX_CONTACTS];
	const PxU32 count=getFrictionContactIndices(c,patch,indices);
	PX_ASSERT(count);
	const PxContactPoint& first=contacts[indices[0]];
	const PxVec3 origin=first.point;
	const PxVec3 u=(first.getAnisotropy()->frictionDirection-first.normal*first.normal.dot(first.getAnisotropy()->frictionDirection)).getNormalized();
	const PxVec3 v=first.normal.cross(u);
	Point points[CAPACITY], polygon[2*CAPACITY];
	for(PxU32 i=0; i<count; ++i)
	{
		const PxVec3 offset=contacts[indices[i]].point-origin;
		points[i]=Point{offset.dot(u),offset.dot(v)};
	}
	const PxU32 vertices=hull(points,count,polygon);
	const PxU32 resolution=c.getAreaSampleCount(patch);
	PX_ASSERT(vertices && vertices<CAPACITY && resolution<=PatchFrictionSamples::SAMPLE_COUNT);
	// Slip in the u direction varies only along v on a planar rigid patch;
	// the v direction is integrated independently along u (box friction law).
	PatchFrictionIntegration::samples(polygon,vertices,1,resolution,u,v,samples.primary,samples.primaryWeight);
	PatchFrictionIntegration::samples(polygon,vertices,0,resolution,u,v,samples.secondary,samples.secondaryWeight);
}

}
}
#endif
