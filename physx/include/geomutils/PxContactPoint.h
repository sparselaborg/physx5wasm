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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_CONTACT_POINT_H
#define PX_CONTACT_POINT_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	class PxGeometry;

	// Internal, optional geometry shared by an anisotropic pair. Shape geometry is
	// immutable during simulation; the poses belong to this contact-generation pass.
	// This is used only during CPU rigid-body PGS preparation, never by reports.
	PX_ALIGN_PREFIX(16)
	struct PxContactFrictionPatch
	{
		const PxGeometry* geometry[2];
		PxTransform pose[2];
		PxU32 contactCount;
		PxVec3 normal;
	} PX_ALIGN_SUFFIX(16);

	// Optional per-contact data, allocated only for anisotropic contact pairs.
	struct PxContactAnisotropy
	{
		// World-space material axis; project onto the final normal during solver preparation.
		PxVec3 frictionDirection;
		PxReal staticFrictionSecondary;
		PxReal dynamicFrictionSecondary;
		// Offset from this entry to the shared geometry, preserved by stream copies.
		// Zero for contacts supplied directly to the immediate-mode solver.
		PxU32 patchDataOffset;
		PxVec3 originalPoint;
		PxContactAnisotropy() : frictionDirection(0.f), staticFrictionSecondary(0.f),
			dynamicFrictionSecondary(0.f), patchDataOffset(0), originalPoint(0.f) {}
		PX_FORCE_INLINE const PxContactFrictionPatch* getPatchData() const
		{
			return patchDataOffset ? reinterpret_cast<const PxContactFrictionPatch*>(
				reinterpret_cast<const PxU8*>(this) + patchDataOffset) : NULL;
		}
		PX_FORCE_INLINE static PxU32 getDataSize(PxU32 count)
		{
			return ((count * sizeof(PxContactAnisotropy) + 15) & ~15u) + sizeof(PxContactFrictionPatch);
		}
	};

	struct PxContactPoint
	{
		// Internal contact-preparation flag, outside the PxMaterialFlag bits.
		enum { eHAS_AREA_FRICTION = 1 << 7, eAREA_FRICTION_PAIR = 1 << 6 };
		/**
		\brief The normal of the contacting surfaces at the contact point.

		For two shapes s0 and s1, the normal points in the direction that s0 needs to move in to resolve the contact with s1.
		*/
		PX_ALIGN(16, PxVec3	normal);

		/**
		\brief The separation of the shapes at the contact point. A negative separation denotes a penetration.
		*/
		PxReal	separation;

		/**
		\brief The point of contact between the shapes, in world space. 
		*/
		PX_ALIGN(16, PxVec3	point);	

		/**
		\brief The max impulse permitted at this point
		*/
		PxReal maxImpulse;

		PX_ALIGN(16, PxVec3 targetVel);

		/**
		\brief The static friction coefficient
		*/
		PxReal staticFriction;

		/**
		\brief Material flags for this contact (eDISABLE_FRICTION, eDISABLE_STRONG_FRICTION). \see PxMaterialFlag
		*/
		PxU8 materialFlags;

		/**
		\brief The surface index of shape 1 at the contact point. This is used to identify the surface material.

		\note This field is only supported by triangle meshes and heightfields, else it will be set to PXC_CONTACT_NO_FACE_INDEX.
		*/
		PxU32   internalFaceIndex1;

		/**
		\brief The dynamic friction coefficient
		*/
		PxReal dynamicFriction;

		/**
		\brief The restitution coefficient
		*/
		PxReal restitution;

		/**
		\brief Damping coefficient (for compliant contacts)
		*/
		PxReal damping;

		// Optional solver-preparation data, stored in existing tail padding.
		// Never read or initialized for ordinary contacts.
		const PxContactAnisotropy* anisotropyData;
		PX_FORCE_INLINE const PxContactAnisotropy* getAnisotropy() const
		{
			return anisotropyData;
		}
		PX_FORCE_INLINE void setAnisotropy(const PxContactAnisotropy* data)
		{
			anisotropyData = data;
		}
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
