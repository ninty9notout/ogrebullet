/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)
For the latest info, see http://www.ogre3d.org/phpBB2addons/viewforum.php?f=10

Copyright (c) 2007 tuan.kuranes@gmail.com



This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
-----------------------------------------------------------------------------
*/

#include "OgreBulletDynamics.h"

#include "OgreBulletCollisionsShape.h"
#include "OgreBulletCollisionsObject.h"
#include "OgreBulletCollisionsWorld.h"
#include "OgreBulletCollisionsObjectState.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "Prefab/OgreBulletDynamicsRagDoll.h"

using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletDynamics
{

	// -------------------------------------------------------------------------
	RagDoll::RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: 
		m_ownerWorld (ownerWorld)
	{
		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);


		btTransform transform;

		// for
		{
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
			m_bodies.push_back(
				localCreateRigidBody(btScalar(1.), 
										offset*transform, 
										new btCapsuleShape(btScalar(0.15), btScalar(0.20))
										)
									);
		}

		// Setup some damping on the m_bodies
		for(std::vector<btRigidBody* >::iterator i=m_bodies.begin();
			i!=m_bodies.end();
			++i) 
		{
			(*i)->setDamping(0.05, 0.85);
			(*i)->setDeactivationTime(0.8);
			(*i)->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		//btConeTwistConstraint* coneC;
		btTransform localA, localB;

		// for
		{
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0, Ogre::Math::TWO_PI,0); 
			localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
			localB.getBasis().setEulerZYX(0,Ogre::Math::TWO_PI,0); 
			localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
			hingeC =  new btHingeConstraint(*m_bodies[0], *m_bodies[1], 
				localA, localB);
			hingeC->setLimit(btScalar(-Ogre::Math::TWO_PI*2), btScalar(Ogre::Math::TWO_PI));
			m_joints.push_back(hingeC);
			m_ownerWorld->addConstraint(hingeC, true);
		}

	}
    // -------------------------------------------------------------------------
	RagDoll::~RagDoll ()
	{
		std::vector<btCollisionShape* >  m_shapes;
		std::vector<btRigidBody* >		 m_bodies;
		std::vector<btTypedConstraint* > m_joints;
		// Remove all constraints
		for(std::vector<btTypedConstraint* >::iterator i=m_joints.begin();
			i!=m_joints.end();
			++i) 
		{
			m_ownerWorld->removeConstraint(*i);
			delete *i;
		}

		// Remove all bodies and shapes
		for(std::vector<btRigidBody* >::iterator i=m_bodies.begin();
			i!=m_bodies.end();
			++i) 
		{
			m_ownerWorld->removeRigidBody(*i);

			delete (*i)->getMotionState();
			delete *i;
		}
		for(std::vector<btCollisionShape* >::iterator i=m_shapes.begin();
			i!=m_shapes.end();
			++i) 
		{
			delete *i;
		}
	}
	// -------------------------------------------------------------------------
	btRigidBody* RagDoll::localCreateRigidBody (btScalar mass, 
		const btTransform& startTransform, 
		btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}
}

