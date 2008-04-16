/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)
For the latest info, see http://www.ogre3d.org/phpBB2addons/viewforum.php?f=10

Copyright (c) 2007 tuan.kuranes@gmail.com (Use it Freely, even Statically, but have to contribute any changes)



This program is free software; you can redistribute it and/or modify it under
the terms of the GPL General Public License with runtime exception as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GPL General Public License with runtime exception for more details.

You should have received a copy of the GPL General Public License with runtime exception along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
-----------------------------------------------------------------------------
*/

#include "OgreBulletDynamics.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsRigidBody.h"

#include "OgreBulletDynamicsConstraint.h"
#include "Constraints/OgreBulletDynamics6DofConstraint.h"

using namespace Ogre;

namespace OgreBulletDynamics
{
    // -------------------------------------------------------------------------
    SixDofConstraint::SixDofConstraint(RigidBody * rbA, RigidBody * rbB, 
        const Vector3& FrameInAVector3, const Quaternion& FrameInAOrientation,
        const Vector3& FrameInBVector3, const Quaternion& FrameInBOrientation):
    TypedConstraint(rbA, rbB)
    {
        btTransform frameInA (OgreBulletCollisions::OgreBtConverter::to (FrameInAOrientation), 
                              OgreBulletCollisions::OgreBtConverter::to (FrameInAVector3));
        btTransform frameInB (OgreBulletCollisions::OgreBtConverter::to (FrameInBOrientation),
                              OgreBulletCollisions::OgreBtConverter::to (FrameInBVector3));

        mConstraint = new btGeneric6DofConstraint(
            *rbA->getBulletRigidBody (),
            *rbB->getBulletRigidBody (), 
            frameInA,
            frameInB,
			true); // Eric added this because Bullet 2.61 has a new argument (useLinearReferenceFrameA)
    }
    // -------------------------------------------------------------------------
    SixDofConstraint::~SixDofConstraint()
    {
	}
	// -------------------------------------------------------------------------
	void	 SixDofConstraint::setLinearLowerLimit(const Ogre::Vector3& linearLower)
	{
		static_cast<btGeneric6DofConstraint* > (mConstraint)->setLinearLowerLimit(OgreBulletCollisions::OgreBtConverter::to (linearLower));
	}
	// -------------------------------------------------------------------------
	void	 SixDofConstraint::setLinearUpperLimit(const Ogre::Vector3& linearUpper)
	{
		static_cast<btGeneric6DofConstraint* > (mConstraint)->setLinearLowerLimit(OgreBulletCollisions::OgreBtConverter::to (linearUpper));
	}
	// -------------------------------------------------------------------------
	void	 SixDofConstraint::setAngularLowerLimit(const Ogre::Vector3& angularLower)
	{
		static_cast<btGeneric6DofConstraint* > (mConstraint)->setLinearLowerLimit(OgreBulletCollisions::OgreBtConverter::to (angularLower));
	}
	// -------------------------------------------------------------------------
	void	 SixDofConstraint::setAngularUpperLimit(const Ogre::Vector3& angularUpper)
	{
		static_cast<btGeneric6DofConstraint* > (mConstraint)->setLinearLowerLimit(OgreBulletCollisions::OgreBtConverter::to (angularUpper));
	}
	// -------------------------------------------------------------------------
	void	 SixDofConstraint::setLimit(const int axis, const Ogre::Real lo, const Ogre::Real hi)
	{
		static_cast<btGeneric6DofConstraint* > (mConstraint)->setLimit(axis, lo, hi);
	}
	// -------------------------------------------------------------------------
	bool	 SixDofConstraint::isLimited(int limitIndex)
	{
		return static_cast<btGeneric6DofConstraint* > (mConstraint)->isLimited(limitIndex);
	}
}