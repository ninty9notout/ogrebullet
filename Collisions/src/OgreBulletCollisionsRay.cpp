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

#include "OgreBulletCollisions.h"

#include "OgreBulletCollisionsWorld.h"
#include "OgreBulletCollisionsRay.h"
#include "Utils/OgreBulletConverter.h"

#include "OgreBulletCollisionsObject.h"
#include "Debug/OgreBulletCollisionsDebugShape.h"

using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletCollisions
{
    // -------------------------------------------------------------------------
	CollisionRayResultCallback::CollisionRayResultCallback(const Ogre::Ray &ray, 
							CollisionsWorld *world, Ogre::Real max_distance,
							bool init):
        mRayResultCallback(0),
        mWorld(world),
        mRay (ray),
		mMaxDistance(max_distance)
    {
        if (init)
        {
            //mRay = new btCollisionWorld::RayResultCallback (
            //    OgreBtConverter::to(ray.getOrigin ()), 
            //   OgreBtConverter::to(ray.getDirection ()));
        }
	}
    // -------------------------------------------------------------------------
    CollisionRayResultCallback::~CollisionRayResultCallback()
    {
        if (mRayResultCallback)
        {
            delete mRayResultCallback;
            mRayResultCallback = 0;
        }
    }
    Ogre::Vector3 CollisionRayResultCallback::getRayStartPoint() const
    {
      return mRay.getOrigin();
    }
    // -------------------------------------------------------------------------
    Ogre::Vector3 CollisionRayResultCallback::getRayEndPoint() const
    {
      return mRay.getPoint(mMaxDistance);
    }
    // -------------------------------------------------------------------------
    bool  CollisionRayResultCallback::doesCollide() const
    {
        return mRayResultCallback->hasHit();
    }
    // -------------------------------------------------------------------------
    Object  *CollisionClosestRayResultCallback::getCollidedObject () const
    {        
        return mWorld->findObject(static_cast<btCollisionWorld::ClosestRayResultCallback *> (mRayResultCallback)->m_collisionObject);
	}
    // -------------------------------------------------------------------------
	CollisionClosestRayResultCallback::CollisionClosestRayResultCallback(const Ogre::Ray &ray, CollisionsWorld *world, Ogre::Real max_distance) :
        CollisionRayResultCallback(ray, world, max_distance, false)
    {
        mRayResultCallback = new btCollisionWorld::ClosestRayResultCallback (
            OgreBtConverter::to(getRayStartPoint ()), 
            OgreBtConverter::to(getRayEndPoint ()));
    } 
    // -------------------------------------------------------------------------
    Vector3 CollisionClosestRayResultCallback::getCollisionPoint() const
    {
        return BtOgreConverter::to(getBulletClosestRayResultCallback()->m_hitPointWorld);
	}
	// -------------------------------------------------------------------------
	Vector3 CollisionClosestRayResultCallback::getCollisionNormal() const
	{
		return BtOgreConverter::to(getBulletClosestRayResultCallback()->m_hitNormalWorld);
	}
}

