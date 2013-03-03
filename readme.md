Eye-Tracking Robotic Arm
========================

This project is looking at the building of a robotic arm capable of lifting objects off a table, controlled by a user using eye-tracking. The movement control of the arm is assisted using 3D vision and object detection. The eye-tracking and object detection is made possible by utilising a PS3 Eye camera and the Xbox Kinect sensor.


Install Dependencies
---------------------

###Ubuntu###
In Terminal run the following commands to download and install the dependencies.

1. Clone source (if not already cloned). 
1. Move into the repo.
1. Install deps.

```bash
git clone git://github.com/jimjibone/robotarm.git
cd robotarm
./install.sh
```

###Other OS's###
Soz guys but there are no installs for other OS's because of the PS Eye camera being stoopid.


What does `install.sh` do?
--------------------------

###Install all the dependencies of the dependencies###

Does that.

###OpenKinect###

In order to get OK to bridge over properly with PCL we need to edit some files before OK installation. This has already been done in the jimjibone fork of libfreenect (jimjibone/libfreenect). This is then downloaded, built, installed and permissions + drivers fixed.

###OpenCV###

Just a standard install. But it is the cutting edge version from GitHub! BEWARE!

###Point Cloud Library###

Good ol' Ubuntu has a PCL install using `apt-get`. Easy.


Build `object_detect` files
---------------------------

This can be done very simply by using Cmake:

1. In the Terminal `cd` into object_detect
1. `mkdir build`
1. `cd build`
1. Various `cmake` commands can now be run to produce makefiles or IDE projects:
	1. Normal binary makefile - `cmake ..`
	1. Xcode project - `cmake .. -G 'Xcode'`
	1. For other IDE's run `cmake` and a list of Generators will appear at the bottom
1. To make the binaries if you used `cmake ..` then run `make`
1. Otherwise, open the project file in your IDE and build that way.
