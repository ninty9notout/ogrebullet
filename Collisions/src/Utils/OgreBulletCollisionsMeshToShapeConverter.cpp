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

#include "Utils/OgreBulletCollisionsMeshToShapeConverter.h"

#include "Shapes/OgreBulletCollisionsTrimeshShape.h"
#include "Shapes/OgreBulletCollisionsCylinderShape.h"
#include "Shapes/OgreBulletCollisionsSphereShape.h"
#include "Shapes/OgreBulletCollisionsBoxShape.h"
#include "Shapes/OgreBulletCollisionsConvexHullShape.h"
#include "Shapes/OgreBulletCollisionsCapsuleShape.h"

using namespace OgreBulletCollisions;
using namespace Ogre;

//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addStaticVertexData(const VertexData *vertex_data)
{
	if (!vertex_data) 
        return;

	const VertexData *data = vertex_data;

	const unsigned int prev_size = mVertexCount;
    mVertexCount += (unsigned int)data->vertexCount;

    Ogre::Vector3* tmp_vert = new Ogre::Vector3[mVertexCount];
	if (mVertexBuffer)
	{
		memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
		delete[] mVertexBuffer;
	}
	mVertexBuffer = tmp_vert;

	// Get the positional buffer element
    {	
        const Ogre::VertexElement* posElem = data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);			
        Ogre::HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
        const unsigned int vSize = (unsigned int)vbuf->getVertexSize();

	    unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
	    float* pReal;
        Ogre::Vector3 * curVertices = &mVertexBuffer[prev_size];
        const unsigned int vertexCount = (unsigned int)data->vertexCount;
	    for(unsigned int j = 0; j < vertexCount; ++j)
	    {
		    posElem->baseVertexPointerToElement(vertex, &pReal);
            vertex += vSize;

		    curVertices->x = (*pReal++);
		    curVertices->y = (*pReal++);
		    curVertices->z = (*pReal++);

		    *curVertices = mTransform * (*curVertices);
            
            curVertices++;
        }
	    vbuf->unlock();
    }
}

//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addAnimatedVertexData(const Ogre::VertexData *vertex_data,
											   const Ogre::VertexData *blend_data,
											   const Ogre::Mesh::IndexMap *indexMap)
{	
	// Get the bone index element
	assert(vertex_data);

	const VertexData *data = blend_data;
	const unsigned int prev_size = mVertexCount;
	mVertexCount += (unsigned int)data->vertexCount;
	Ogre::Vector3* tmp_vert = new Ogre::Vector3[mVertexCount];
	if (mVertexBuffer)
	{
		memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
		delete[] mVertexBuffer;
	}
	mVertexBuffer = tmp_vert;

	// Get the positional buffer element
	{	
		const Ogre::VertexElement* posElem = data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);	
		assert (posElem);
		Ogre::HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
		const unsigned int vSize = (unsigned int)vbuf->getVertexSize();

		unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
		float* pReal;
		Ogre::Vector3 * curVertices = &mVertexBuffer[prev_size];
		const unsigned int vertexCount = (unsigned int)data->vertexCount;
		for(unsigned int j = 0; j < vertexCount; ++j)
		{
			posElem->baseVertexPointerToElement(vertex, &pReal);
			vertex += vSize;

			curVertices->x = (*pReal++);
			curVertices->y = (*pReal++);
			curVertices->z = (*pReal++);

			*curVertices = mTransform * (*curVertices);

			curVertices++;
		}
		vbuf->unlock();
	}
	{
		const Ogre::VertexElement* bneElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_BLEND_INDICES);
		assert (bneElem);
		
		Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(bneElem->getSource());
		const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
		unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

		unsigned char* pBone;

		if (!mBoneIndex)
			mBoneIndex = new BoneIndex();	
		BoneIndex::iterator i;

		Ogre::Vector3 * curVertices = &mVertexBuffer[prev_size];

		const unsigned int vertexCount = (unsigned int)vertex_data->vertexCount;
		for(unsigned int j = 0; j < vertexCount; ++j)
		{
			bneElem->baseVertexPointerToElement(vertex, &pBone);
			vertex += vSize;

			const unsigned char currBone = (indexMap) ? (*indexMap)[*pBone] : *pBone;
			i = mBoneIndex->find (currBone);
			Vector3Array* l = 0;
			if (i == mBoneIndex->end())
			{
				l = new Vector3Array;
				mBoneIndex->insert(BoneKeyIndex(currBone, l));
			}						
			else 
			{
				l = i->second;
			}

			l->push_back(*curVertices);

			curVertices++;
		}
		vbuf->unlock();	
	}
}

//------------------------------------------------------------------------------------------------
void VertexIndexToShape::addIndexData(IndexData *data, const unsigned int offset)
{
    const unsigned int prev_size = mIndexCount;
    mIndexCount += (unsigned int)data->indexCount;

	unsigned int* tmp_ind = new unsigned int[mIndexCount];
	if (mIndexBuffer)
	{
		memcpy (tmp_ind, mIndexBuffer, sizeof(unsigned int) * prev_size);
		delete[] mIndexBuffer;
	}
	mIndexBuffer = tmp_ind;

	const unsigned int numTris = (unsigned int) data->indexCount / 3;
	HardwareIndexBufferSharedPtr ibuf = data->indexBuffer;	
	const bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
    unsigned int index_offset = prev_size;

	if (use32bitindexes) 
    {
        const unsigned int* pInt = static_cast<unsigned int*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
        for(unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset ++] = offset + *pInt++;
            mIndexBuffer[index_offset ++] = offset + *pInt++;
            mIndexBuffer[index_offset ++] = offset + *pInt++;
        }
        ibuf->unlock();
    }
	else 
    {
        const unsigned short* pShort = static_cast<unsigned short*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
		for(unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
        }
        ibuf->unlock();
    }

}
//------------------------------------------------------------------------------------------------
Real VertexIndexToShape::getRadius()
{
	if (mBoundRadius == (-1))
	{
		getSize();
		mBoundRadius = (std::max(mBounds.x,std::max(mBounds.y,mBounds.z)) * 0.5);
	}
	return mBoundRadius;
}
//------------------------------------------------------------------------------------------------
Vector3 VertexIndexToShape::getSize()
{
    const unsigned int vCount = getVertexCount();
	if (mBounds == Ogre::Vector3(-1,-1,-1) && vCount > 0)
	{

		const Ogre::Vector3 * const v = getVertices();

        Ogre::Vector3 vmin(v[0]);
        Ogre::Vector3 vmax(v[0]);

		for(unsigned int j = 1; j < vCount; j++)
		{
			vmin.x = std::min(vmin.x, v[j].x);
			vmin.y = std::min(vmin.y, v[j].y);
			vmin.z = std::min(vmin.z, v[j].z);

			vmax.x = std::max(vmax.x, v[j].x);
			vmax.y = std::max(vmax.y, v[j].y);
			vmax.z = std::max(vmax.z, v[j].z);
		}

		mBounds.x = vmax.x - vmin.x;
		mBounds.y = vmax.y - vmin.y;
		mBounds.z = vmax.z - vmin.z;
	}

	return mBounds;
}
//------------------------------------------------------------------------------------------------
const Ogre::Vector3* VertexIndexToShape::getVertices()
{
	return mVertexBuffer;
}
//------------------------------------------------------------------------------------------------
unsigned int VertexIndexToShape::getVertexCount()
{
	return mVertexCount;
}
//------------------------------------------------------------------------------------------------
const unsigned int* VertexIndexToShape::getIndices()
{
	return mIndexBuffer;
}
//------------------------------------------------------------------------------------------------
unsigned int VertexIndexToShape::getIndexCount()
{
	return mIndexCount;
}

//------------------------------------------------------------------------------------------------
SphereCollisionShape* VertexIndexToShape::createSphere()
{
	const Ogre::Real rad = getRadius();
	assert((rad > 0.0) && 
        ("Sphere radius must be greater than zero"));
    SphereCollisionShape* shape = new SphereCollisionShape(rad);

    return shape;
}
//------------------------------------------------------------------------------------------------
BoxCollisionShape* VertexIndexToShape::createBox()
{
	const Ogre::Vector3 sz = getSize();

	assert((sz.x > 0.0) && (sz.y > 0.0) && (sz.y > 0.0) && 
        ("Size of box must be greater than zero on all axes"));

	BoxCollisionShape* shape = new BoxCollisionShape(sz);
	return shape;
}
//------------------------------------------------------------------------------------------------
CylinderCollisionShape* VertexIndexToShape::createCylinder()
{
    const Ogre::Vector3 sz = getSize();

    assert((sz.x > 0.0) && (sz.y > 0.0) && (sz.y > 0.0) && 
        ("Size of Cylinder must be greater than zero on all axes"));

    CylinderCollisionShape* shape = new CylinderCollisionShape(sz, Vector3::UNIT_X);
    return shape;
}
//------------------------------------------------------------------------------------------------
ConvexHullCollisionShape* VertexIndexToShape::createConvex()
{
    assert(mVertexCount && (mIndexCount >= 6) && 
        ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

    return new ConvexHullCollisionShape(&mVertexBuffer[0].x, mVertexCount, sizeof(Vector3));
}
//------------------------------------------------------------------------------------------------
TriangleMeshCollisionShape* VertexIndexToShape::createTrimesh()
{
	assert(mVertexCount && (mIndexCount >= 6) && 
        ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

	return new TriangleMeshCollisionShape(mVertexBuffer, mVertexCount,mIndexBuffer, mIndexCount);
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::~VertexIndexToShape()
{
	delete[] mVertexBuffer;
	delete[] mIndexBuffer;

	if (mBoneIndex)
	{
		for(BoneIndex::iterator i = mBoneIndex->begin();
            i != mBoneIndex->end();
            ++i)
		{
			delete i->second;
		}
		delete mBoneIndex;
	}
}
//------------------------------------------------------------------------------------------------
VertexIndexToShape::VertexIndexToShape(const Matrix4 &transform) :
	mVertexBuffer (0),
	mIndexBuffer (0),
	mVertexCount (0),
	mIndexCount (0),
	mBounds (Vector3(-1,-1,-1)),
	mBoundRadius (-1),
	mBoneIndex (0),
	mTransform (transform)
{	
}
//------------------------------------------------------------------------------------------------
StaticMeshToShapeConverter::StaticMeshToShapeConverter() :
VertexIndexToShape(),
	mEntity (0),
	mNode (0)
{
}
//------------------------------------------------------------------------------------------------
StaticMeshToShapeConverter::~StaticMeshToShapeConverter()
{
}
//------------------------------------------------------------------------------------------------
StaticMeshToShapeConverter::StaticMeshToShapeConverter(Entity *entity,  const Matrix4 &transform) :
	VertexIndexToShape(transform),
	mEntity (0),
	mNode (0)
{
	addEntity(entity, transform);	
}
//------------------------------------------------------------------------------------------------
StaticMeshToShapeConverter::StaticMeshToShapeConverter(Renderable *rend, const Matrix4 &transform) :
	VertexIndexToShape(transform),
	mEntity (0),
	mNode (0)
{
	RenderOperation op;
	rend->getRenderOperation(op);
	VertexIndexToShape::addStaticVertexData(op.vertexData);
	if(op.useIndexes)
		VertexIndexToShape::addIndexData(op.indexData);

}
//------------------------------------------------------------------------------------------------
void StaticMeshToShapeConverter::addEntity(Entity *entity,const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

	mEntity = entity;
	mNode = (SceneNode*)(mEntity->getParentNode());
	mTransform = transform;

	if (mEntity->getMesh()->sharedVertexData)
	{
		VertexIndexToShape::addStaticVertexData (mEntity->getMesh()->sharedVertexData);
	}

	for (unsigned int i = 0;i < mEntity->getNumSubEntities();++i)
	{
		SubMesh *sub_mesh = mEntity->getSubEntity(i)->getSubMesh();

		if (!sub_mesh->useSharedVertices)
		{
			VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
			VertexIndexToShape::addStaticVertexData (sub_mesh->vertexData);
		}
		else 
		{
			VertexIndexToShape::addIndexData (sub_mesh->indexData);
		}

	}
}
//------------------------------------------------------------------------------------------------
void StaticMeshToShapeConverter::addMesh(const MeshPtr &mesh, const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

	//_entity = entity;
	//_node = (SceneNode*)(_entity->getParentNode());
	mTransform = transform;

	if (mesh->hasSkeleton ())
		Ogre::LogManager::getSingleton().logMessage("MeshToShapeConverter::addMesh : Mesh " + mesh->getName () + " as skeleton but added to trimesh non animated");

	if (mesh->sharedVertexData)
	{
		VertexIndexToShape::addStaticVertexData (mesh->sharedVertexData);
	}

	for(unsigned int i = 0;i < mesh->getNumSubMeshes();++i)
	{
		SubMesh *sub_mesh = mesh->getSubMesh(i);

		if (!sub_mesh->useSharedVertices)
		{
			VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
			VertexIndexToShape::addStaticVertexData (sub_mesh->vertexData);
		}
		else 
		{
			VertexIndexToShape::addIndexData (sub_mesh->indexData);
		}

	}
}
//------------------------------------------------------------------------------------------------
AnimatedMeshToShapeConverter::AnimatedMeshToShapeConverter(Entity *entity,const Matrix4 &transform) :
VertexIndexToShape(transform),
mEntity (0),
mNode (0),
mTransformedVerticesTemp(0),
mTransformedVerticesTempSize(0)
{
	addEntity(entity, transform);	
}
//------------------------------------------------------------------------------------------------
AnimatedMeshToShapeConverter::AnimatedMeshToShapeConverter() :
VertexIndexToShape(),
mEntity (0),
mNode (0),
mTransformedVerticesTemp(0),
mTransformedVerticesTempSize(0)
{
}
//------------------------------------------------------------------------------------------------
AnimatedMeshToShapeConverter::~AnimatedMeshToShapeConverter()
{
	delete[] mTransformedVerticesTemp;
}
//------------------------------------------------------------------------------------------------
void AnimatedMeshToShapeConverter::addEntity(Entity *entity,const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

	mEntity = entity;
	mNode = (SceneNode*)(mEntity->getParentNode());
	mTransform = transform;

	assert (entity->getMesh()->hasSkeleton ());

	mEntity->addSoftwareAnimationRequest(false);
	mEntity->_updateAnimation();

	if (mEntity->getMesh()->sharedVertexData)
	{
		VertexIndexToShape::addAnimatedVertexData (mEntity->getMesh()->sharedVertexData, 
			mEntity->_getSkelAnimVertexData(),
			&mEntity->getMesh()->sharedBlendIndexToBoneIndexMap); 
	}

	for (unsigned int i = 0;i < mEntity->getNumSubEntities();++i)
	{
		SubMesh *sub_mesh = mEntity->getSubEntity(i)->getSubMesh();

		if (!sub_mesh->useSharedVertices)
		{
			VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);

			VertexIndexToShape::addAnimatedVertexData (sub_mesh->vertexData, 
				mEntity->getSubEntity(i)->_getSkelAnimVertexData(),
				&sub_mesh->blendIndexToBoneIndexMap); 
		}
		else 
		{
			VertexIndexToShape::addIndexData (sub_mesh->indexData);
		}

	}

	mEntity->removeSoftwareAnimationRequest(false);
}
//------------------------------------------------------------------------------------------------
void AnimatedMeshToShapeConverter::addMesh(const MeshPtr &mesh, const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

	//_entity = entity;
	//_node = (SceneNode*)(_entity->getParentNode());
	mTransform = transform;

	assert (mesh->hasSkeleton ());

	if (mesh->sharedVertexData)
	{
		VertexIndexToShape::addAnimatedVertexData (mesh->sharedVertexData, 
			0,
			&mesh->sharedBlendIndexToBoneIndexMap); 
	}

	for(unsigned int i = 0;i < mesh->getNumSubMeshes();++i)
	{
		SubMesh *sub_mesh = mesh->getSubMesh(i);

		if (!sub_mesh->useSharedVertices)
		{
			VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);

			VertexIndexToShape::addAnimatedVertexData (sub_mesh->vertexData, 
				0,
				&sub_mesh->blendIndexToBoneIndexMap); 
		}
		else 
		{
			VertexIndexToShape::addIndexData (sub_mesh->indexData);
		}

	}
}
//------------------------------------------------------------------------------------------------
bool AnimatedMeshToShapeConverter::getBoneVertices(unsigned char bone, 
													 unsigned int &vertex_count, 
													 Ogre::Vector3* &vertices,
													 const Vector3 &bonePosition)
{
	BoneIndex::iterator i = mBoneIndex->find(bone);

	if (i == mBoneIndex->end()) 
		return false;

	if (i->second->empty()) 
		return false;

	vertex_count = (unsigned int) i->second->size() + 1;
	if (vertex_count > mTransformedVerticesTempSize)
	{	
		if (mTransformedVerticesTemp)
			delete[] mTransformedVerticesTemp;

		mTransformedVerticesTemp = new Ogre::Vector3[vertex_count];

	}

	vertices = mTransformedVerticesTemp;
	vertices[0] = bonePosition;
	//mEntity->_getParentNodeFullTransform() * 
	//	mEntity->getSkeleton()->getBone(bone)->_getDerivedPosition();

	//mEntity->getSkeleton()->getBone(bone)->_getDerivedOrientation()
	unsigned int currBoneVertex = 1;
	Vector3Array::iterator j = i->second->begin();
	while(j != i->second->end())
	{
		vertices[currBoneVertex] = (*j);
		++j;
		++currBoneVertex; 
	}       
	return true;
}
//------------------------------------------------------------------------------------------------
BoxCollisionShape* AnimatedMeshToShapeConverter::createAlignedBox(unsigned char bone, 
																  const Vector3 &bonePosition,
																  const Quaternion &boneOrientation)
{
	unsigned int vertex_count;
	Vector3* vertices;

	if (!getBoneVertices(bone, vertex_count, vertices, bonePosition)) 
		return 0;

	Vector3 min_vec(vertices[0]);
	Vector3 max_vec(vertices[0]);

	for(unsigned int j = 1; j < vertex_count ;j++)
	{
		min_vec.x = std::min(min_vec.x,vertices[j].x);
		min_vec.y = std::min(min_vec.y,vertices[j].y);
		min_vec.z = std::min(min_vec.z,vertices[j].z);

		max_vec.x = std::max(max_vec.x,vertices[j].x);
		max_vec.y = std::max(max_vec.y,vertices[j].y);
		max_vec.z = std::max(max_vec.z,vertices[j].z);
	}
	const Ogre::Vector3 maxMinusMin(max_vec - min_vec);
	BoxCollisionShape* box = new BoxCollisionShape(maxMinusMin);

	const Ogre::Vector3 pos
		(min_vec.x + (maxMinusMin.x * 0.5),
		min_vec.y + (maxMinusMin.y * 0.5),
		min_vec.z + (maxMinusMin.z * 0.5));

	//box->setPosition(pos);

	return box;
}
//------------------------------------------------------------------------------------------------
bool AnimatedMeshToShapeConverter::getOrientedBox(unsigned char bone, 
					 const Vector3 &bonePosition,
					 const Quaternion &boneOrientation,
					 Vector3 &box_afExtent,
					 Vector3 *box_akAxis,
					 Vector3 &box_kCenter)
{
	unsigned int vertex_count;
	Vector3* vertices;

	if (!getBoneVertices(bone, vertex_count, vertices, bonePosition))
		return false;

	 box_kCenter = Vector3::ZERO;

	 {
		 for(unsigned int c = 0 ;c < vertex_count;c++)
		 {
			 box_kCenter += vertices[c];
		 }
		 const Ogre::Real invVertexCount = 1.0 / vertex_count;
		 box_kCenter *= invVertexCount;
	 }
	Quaternion orient = boneOrientation;
	orient.ToAxes(box_akAxis);


	// Let C be the box center and let U0, U1, and U2 be the box axes.  Each
	// input point is of the form X = C + y0*U0 + y1*U1 + y2*U2.  The
	// following code computes min(y0), max(y0), min(y1), max(y1), min(y2),
	// and max(y2).  The box center is then adjusted to be
	//   C' = C + 0.5*(min(y0)+max(y0))*U0 + 0.5*(min(y1)+max(y1))*U1 +
	//        0.5*(min(y2)+max(y2))*U2

	Ogre::Vector3 kDiff (vertices[1] - box_kCenter);
	Ogre::Real fY0Min = kDiff.dotProduct(box_akAxis[0]), fY0Max = fY0Min;
	Ogre::Real fY1Min = kDiff.dotProduct(box_akAxis[1]), fY1Max = fY1Min;
	Ogre::Real fY2Min = kDiff.dotProduct(box_akAxis[2]), fY2Max = fY2Min;

	for (unsigned int i = 2; i < vertex_count; i++)
	{
		kDiff = vertices[i] - box_kCenter;

		const Ogre::Real fY0 = kDiff.dotProduct(box_akAxis[0]);
		if ( fY0 < fY0Min )
			fY0Min = fY0;
		else if ( fY0 > fY0Max )
			fY0Max = fY0;

		const Ogre::Real fY1 = kDiff.dotProduct(box_akAxis[1]);
		if ( fY1 < fY1Min )
			fY1Min = fY1;
		else if ( fY1 > fY1Max )
			fY1Max = fY1;

		const Ogre::Real fY2 = kDiff.dotProduct(box_akAxis[2]);
		if ( fY2 < fY2Min )
			fY2Min = fY2;
		else if ( fY2 > fY2Max )
			fY2Max = fY2;
	}

	box_afExtent.x = ((Real)0.5)*(fY0Max - fY0Min);
	box_afExtent.y = ((Real)0.5)*(fY1Max - fY1Min);
	box_afExtent.z = ((Real)0.5)*(fY2Max - fY2Min);

	box_kCenter += (0.5*(fY0Max+fY0Min))*box_akAxis[0] +
		(0.5*(fY1Max+fY1Min))*box_akAxis[1] +
		(0.5*(fY2Max+fY2Min))*box_akAxis[2];

	box_afExtent *= 2.0;

	return true;
}
//------------------------------------------------------------------------------------------------
BoxCollisionShape* AnimatedMeshToShapeConverter::createOrientedBox(unsigned char bone, 
																   const Vector3 &bonePosition,
																   const Quaternion &boneOrientation)
{
	Ogre::Vector3 box_akAxis[3];
	Ogre::Vector3 box_afExtent;
	Ogre::Vector3 box_afCenter;

	if (!getOrientedBox(bone, bonePosition, boneOrientation,
						box_afExtent,
						box_akAxis,
						box_afCenter))
		return 0;

	BoxCollisionShape *geom = new BoxCollisionShape(box_afExtent);
	//geom->setOrientation(Quaternion(box_akAxis[0],box_akAxis[1],box_akAxis[2]));
	//geom->setPosition(box_afCenter);
	return geom; 
}
//------------------------------------------------------------------------------------------------
CapsuleCollisionShape* AnimatedMeshToShapeConverter::createOrientedCapsuleCollisionShape(unsigned char bone, 
																							 const Vector3 &bonePosition,
																							 const Quaternion &boneOrientation)
{
	Ogre::Vector3 box_akAxis[3];
	Ogre::Vector3 box_afExtent;
	Ogre::Vector3 box_afCenter;

	if (!getOrientedBox(bone,  bonePosition, boneOrientation,
						box_afExtent,
						box_akAxis,
						box_afCenter))
		return 0;

	// find axes with longest length

	Vector3 cap_dir;    //std::max(box_afExtent.x,std::max(box_afExtent.y,box_afExtent.z));
	Real cap_dirLength;  //std::max(box_afExtent.x,std::max(box_afExtent.y,box_afExtent.z));
	Real cap_radius;	 // max axe length
	if (box_afExtent.x - box_afExtent.y > 0 && box_afExtent.x - box_afExtent.z > 0)
	{
		cap_dir = Vector3::UNIT_X;
		cap_dirLength = box_afExtent.x;
		cap_radius = std::max (box_afExtent.y, box_afExtent.z);
	}
	else if (box_afExtent.y - box_afExtent.x > 0 && box_afExtent.y - box_afExtent.z > 0)
	{
		cap_dir = Vector3::UNIT_Y;
		cap_dirLength = box_afExtent.y;
		cap_radius = std::max (box_afExtent.x, box_afExtent.z);
	}
	else if (box_afExtent.z - box_afExtent.y > 0 && box_afExtent.z - box_afExtent.y > 0)
	{
		cap_dir = Vector3::UNIT_Z;
		cap_dirLength = box_afExtent.z;
		cap_radius = std::max (box_afExtent.x, box_afExtent.z);
	}

	CapsuleCollisionShape* geom = new CapsuleCollisionShape(cap_radius, cap_dirLength, cap_dir);

	//geom->setOrientation(orient);
	//geom->setPosition(cap_orig + (boneOrientation * (cap_dirLength * 0.5)));

	return geom;
}