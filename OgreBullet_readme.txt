Readme for OgreBullet
==================

Chaster's Changelog: (latest changes at top)
============================================
11/8/2007 - added "getLinearVelocity()" function to OgreBullet RigidBody class.
11/1/2007 - Fix ray in demos and added CollisionClosestRayResultCallback::getCollisionNormal()
11/1/2007 - added "applyForce()" function to OgreBullet RigidBody class.  Similar to applyImpulse().
10/29/2007 - added fixes from Dermont for Linux Compiling.
10/25/2007 - small syntax fix for CollisionClosestRayResultCallback().  Thanks andy
10/24/2007 - multiple enhancements and fixes:
		case sensitivity for OSX/Linux compiles, 
	     	changed default world from btSimpleDynamicsWorld to btDiscreteDynamicsWorld,
	     	added optional maxSubsteps argument for StepSimulation,
		added optional CollisionGroup & CollisionMask bitmask arguments to RigidBody constructor,
		removed OgreBulletCollisionsConvexCast.cpp since it doesn't seem to be used and doesn't compile,
		added some missing paths for the project so can compile using OgreCVS instead of just OgreSDK,
		added lib folder hierarchy with "keepme" dummy files so compile doesn't fail,
		added various small Linux build fixes. 

DOWNLOADED VERSION
======================
Brief build instructions follow for OgreBullet 0.2 . 

1) You should have extracted this archive next to $OGRE_TOP, where $OGRE_TOP
   is the root directory of your Ogre installation, usually named 'ogresdk'.
   All file paths in this build environment should be relative and all the DLLs
   and executables should get sent to the correct place upon building.

2) Bullet is available from http://www.Bullet.org/ It requires that you get Latest Bullet version
 which is actually available only in Bullet subversion repository.


5) Use the solution OgreBullet_SDK.sln to build the  OgreBullet library with 
    Microsoft Visual C++ 2005. The DLLs and executables will be built to the $OGRE_TOP/bin/Debug
   and Release directories.

   Simply open up that solution and perform a batch build.

6) There are some scripts which should build OgreBullet under Linux in the scripts/Linux
   directory. They are provided courtesy of Pablo, please see the Readme in that directory
   for further details.

Post On Ogre Forums for details.

CVS VERSION
======================
Brief build instructions follow for OgreBullet CVS Version.
Chaster (Eric Cha) has done updates to this version.  I (Chaster) will try to keep this readme file
updated with changes as I go.

1) This version of OgreBullet can be compiled against version 
   1.4.X branch of the Ogre library aka Eihort, and version 1.5/1.6 (Shoggoth - unreleased!).

2) you need to get Bullet library (version 2.64RC2 as of this writing) using subversion or SDK release. 
   Bullet download instruction and community is available from http://www.Bullet.org/
   
3) you have to set the BULLET_HOME envirronment variable in Windows to the corresponding directory. 
    If you don't use the one shipped, you'll have to make sure Bullet is compiled against 
    the DLL version of msvcrt. 
    Compile with "RunTime Library : Multi-threaded DLL (/MD)" and  "RunTime Library : Multi-threaded Debug DLL (/MDd)"

5) Use the OgreBullet_SDK.sln solution  to build the core OgreBullet library, 
   with Microsoft Visual C++ 2005. The DLLs and executables will be copied to
   the ..\OgreSDK\bin\debug and  ..\OgreSDK\bin\release

   Simply open up that solution and perform a batch build.

Post On Ogre Forums for details.
