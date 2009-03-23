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

#include "OgreBulletCollisionsShape.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsObjectState.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletDynamicsConstraint.h"

#include "Constraints/OgreBulletDynamicsRaycastVehicle.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletDynamics
{

    DynamicsWorld::DynamicsWorld(Ogre::SceneManager *mgr, 
                const Ogre::AxisAlignedBox &bounds,  
                const Ogre::Vector3 &gravity,
                bool init) :
			 CollisionsWorld(mgr, bounds, false)
    {
        //btSequentialImpulseConstraintSolver
        //btSequentialImpulseConstraintSolver3
        mConstraintsolver = new btSequentialImpulseConstraintSolver();

        //only if init is true, otherwise you have to create mWorld manually later on
        if (init) {
            mWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mConstraintsolver, &mDefaultCollisionConfiguration);
			static_cast <btDiscreteDynamicsWorld *> (mWorld)->setGravity(btVector3(gravity.x,gravity.y,gravity.z));

			//btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(mWorld->getDispatcher());
			//btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
		}

    }
    // -------------------------------------------------------------------------
    DynamicsWorld::~DynamicsWorld()
    {
        delete mConstraintsolver;
    }

    // -------------------------------------------------------------------------
    void DynamicsWorld::addRigidBody (RigidBody *rb, short collisionGroup, short collisionMask)
    {
        mObjects.push_back (static_cast <Object *> (rb));

		if (collisionGroup == 0 && collisionMask == 0)
		{
			// use default collision group/mask values (dynamic/kinematic/static)
			static_cast <btDiscreteDynamicsWorld *> (mWorld)->addRigidBody(rb->getBulletRigidBody());      
		}
		else
		{
			static_cast <btDiscreteDynamicsWorld *> (mWorld)->addRigidBody(rb->getBulletRigidBody(), collisionGroup, collisionMask);      
		}
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::stepSimulation(const Ogre::Real elapsedTime, int maxSubSteps, const Ogre::Real fixedTimestep)
    {
        // Reset Debug Lines
        if (mDebugDrawer) 
			mDebugDrawer->clear ();
		if (mDebugContactPoints)  
			mDebugContactPoints->clear ();

        static_cast <btDiscreteDynamicsWorld *> (mWorld)->stepSimulation(elapsedTime, maxSubSteps, fixedTimestep);

		if (mDebugContactPoints) 
		{
			///one way to draw all the contact points is iterating over contact manifolds / points:
			const unsigned int  numManifolds = mWorld->getDispatcher()->getNumManifolds();
			for (unsigned int i=0;i < numManifolds; i++)
			{
				btPersistentManifold* contactManifold = mWorld->getDispatcher()->getManifoldByIndexInternal(i);

				btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

				contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());

				const unsigned int numContacts = contactManifold->getNumContacts();
				for (unsigned int j = 0;j < numContacts; j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);

					if (mShowDebugContactPoints)
					{
						btVector3 ptA = pt.getPositionWorldOnA();
						btVector3 ptB = pt.getPositionWorldOnB();
						btVector3 ptDistB = ptB  + pt.m_normalWorldOnB *100;

						mDebugContactPoints->addLine(ptA.x(),ptA.y(),ptA.z(),
							ptB.x(),ptB.y(),ptB.z());
						mDebugContactPoints->addLine(ptB.x(),ptB.y(),ptB.z(),
							ptDistB.x(),ptDistB.y(),ptDistB.z());
					}
				}
				//you can un-comment out this line, and then all points are removed
				//contactManifold->clearManifold();	
			}
			// draw lines that step Simulation sent.
			mDebugContactPoints->draw();
		}

		if (mDebugDrawer) 
		{
			// draw lines that step Simulation sent.
			mDebugDrawer->draw();



			const bool drawFeaturesText = (mDebugDrawer->getDebugMode () & btIDebugDraw::DBG_DrawFeaturesText) != 0;
			if (drawFeaturesText)
			{
				// on all bodies we have
				// we get all shapes and draw more information
				//depending on mDebugDrawer mode.
				std::deque<Object*>::iterator it = mObjects.begin();
				while (it != mObjects.end())
				{
					//(*it)->drawFeaturesText();
					++it;
				}
			}
		}
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::removeConstraint(TypedConstraint *constraint)
    {
        getBulletDynamicsWorld()->removeConstraint(constraint->getBulletTypedConstraint());
        std::deque <TypedConstraint*>::iterator it = mConstraints.begin();
        while (it != mConstraints.end())
        {
            if ((*it) == constraint)
            {
                mConstraints.erase (it);
                break;
            }
            ++it;
        }
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::addConstraint(TypedConstraint *constraint)
    {
        getBulletDynamicsWorld()->addConstraint(constraint->getBulletTypedConstraint());
        mConstraints.push_back(constraint);
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::addVehicle(RaycastVehicle *v)
    {
        getBulletDynamicsWorld()->addVehicle(v->getBulletVehicle ());
        mActionInterface.push_back(static_cast <ActionInterface *> (v));

        //mVehicles.push_back(v);
    }
    // -------------------------------------------------------------------------
    bool DynamicsWorld::isConstraintRegistered(TypedConstraint *constraint) const
    {
        std::deque <TypedConstraint*>::const_iterator it = mConstraints.begin();
        while (it != mConstraints.end())
        {
            if ((*it) == constraint)
                return true;
            ++it;
        }
        return false;
    }
}

