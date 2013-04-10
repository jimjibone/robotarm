Kinect Object Detector
======================

Cocoa based interface utilising the libfreenect library to process and display the Kinect data, giving the ability to detect table planes and objects on its surface.

![Latest build of the Kinect Object Detector application](../documentation/Kinect\ Object Detector\ Images/Screen\ Shot\ 2013-04-10\ at\ 22.07.23.png)


Main Features
-------------

- Utilises the Kinect sensor and the [libfreenect](https://github.com/jimjibone/libfreenect) library to grab Kinect 3D data.
- Contains an Objective-C wrapper for proper use of the libfreenect library with Objective-C and Cocoa applications.
- Displays a 3D point cloud of the Kinect data on the screen for the user.
- Calculates the plane coefficients of a table using an implementation of the RANSAC algorithm (C++).
- Calculates the border of the found table using an implementation of the Convex Hull algorithm (C++).
- Displays the found plane equation and table border along with the 3D point cloud.


Extra Coolness
--------------

There have been many hours pumped into this project, trying to get an efficient implementation of almost every aspect. Kinect data memory management, RANSAC and Convex Hull algorithms. The RANSAC and Convex Hull algorithms were then rewritten in C++ to make them more portable in other programs and more efficient (objective-c can be very hefty when trying to do some hardcore algorithms, so it's much better to wrap the C++ classes in objective-c and utilise all the super powers of C++ for algorithms).


Credits
-------

This was all coded by the master, James Reuss (jimjibone). Where relevant, credits to the sources of algorithms or other cool things have been added into the comments next to code.

Usage
-----

Please feel free to use! But please also keep my name and website address with all uses of the code. Thanks :)
