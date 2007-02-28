
#include "OgreBulletCollisions.h"

#include "OgreBulletCollisionsEmptyShape.h"

using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletCollisions
{
    // -------------------------------------------------------- //
    EmptyCollisionShape::EmptyCollisionShape():	
        CollisionShape()
    {
            mShape = new btEmptyShape();
    }
    // -------------------------------------------------------- //
    EmptyCollisionShape::~EmptyCollisionShape()
    {
    }

}

