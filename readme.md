Eye-Tracking Robotic Arm
========================

This project is looking at the building of a robotic arm capable of lifting objects off a table, controlled by a user using eye-tracking. The movement control of the arm is assisted using 3D vision and object detection. The eye-tracking and object detection is made possible by utilising a PS3 Eye camera and the Xbox Kinect sensor.


Whats Here?
-----------

1. Dependencies
1. `install.sh`?
1. Object Detection
1. Eye Tracking


Install Dependencies
---------------------

###The Dependencies###
So far:

- libfreenect (OpenKinect)
- Point Cloud Library
- OpenGL/GLUT (but we don't need to worry about this...)

###OS X and Ubuntu##
In the Terminal run these commands, the script will do the rest, hopefully.

1. Clone source (if not already cloned). 
1. `cd` into the repo.
1. Install deps.

```bash
git clone git://github.com/jimjibone/robotarm.git
cd robotarm
./deps/install.sh
```

###Other OS's (Windoze)###

Soz guys but there are no installs for other OS's because of the PS Eye camera being stoopid.
Well, the PS Eye camera does not work on the latest version of OS X, *sadface* but does work on Linux and Windows reportedly. Apart from that everything works on all platforms! Except this dependency install. Maybe later.


What does `install.sh` do?
--------------------------

###Install all the dependencies of the dependencies###

Does that.

###OpenKinect###

In order to get OK to bridge over properly with PCL we need to edit some files before OK installation. This has already been done in the jimjibone fork of libfreenect (jimjibone/libfreenect). This is then downloaded, built, installed and permissions + drivers fixed (on Linux).

###OpenCV###

OS X: A simple Homebrew install :) `brew install opencv`
Linux: Just a standard install. But it is the cutting edge version from GitHub! BEWARE!

###Point Cloud Library###

OS X: Not very easy. For more info see the `install.sh` script and see how I have had to modify the [Homebrew](http://mxcl.github.com/homebrew/) Formula.
Linux: Good ol' Ubuntu has a PCL install using `apt-get`. Easy.


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


Build `EyeTracker` files
--------------------------

Kinda need to do this...

