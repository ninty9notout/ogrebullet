Readme for OgreBullet
==================

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
Brief build instructions follow for OgreBullet 1.2.0 Final Edition. 
Consider this version of OgreBullet deprecated. A new and exciting development
is on the horizon. Watch this space.

1) This version of OgreBullet is only supported when compiled against version 
   1.2.X branch of the Ogre library aka Dagon,.

2) you need to get Bullet library using subversion 
   Bullet download instruction and communauty is available from http://www.Bullet.org/
   
3) You can use bullet.zip from http://tuan.kuranes.free.fr/bullet.zip but you 
   anyway have to set the BULLET_HOME envirronment variable in Windows to the corresponding directory. 
    If you don't use the one shipped, you'll have to make sure Bullet is compiled against 
    the DLL version of msvcrt. 
    Compile with "RunTime Library : Multi-threaded DLL (/MD)" and  "RunTime Library : Multi-threaded Debug DLL (/MDd)"

5) Use the OgreBullet_SDK.sln solution  to build the core OgreBullet library, 
   with Microsoft Visual C++ 2005 3. The DLLs and executables will be copied to
   the ..\OgreSDK\bin\debug and  ..\OgreSDK\bin\release

   Simply open up that solution and perform a batch build.

Post On Ogre Forums for details.
