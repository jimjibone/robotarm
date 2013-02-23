/*---------------------------------------------*-
 * Object Detection Main File
 *
 * Eye Tracking Robotic Arm
 * By James Reuss
 * Copyright 2013
-*---------------------------------------------*-
 * Main.cpp
 * This files does some stuff.
-*---------------------------------------------*/

/*#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/msac.h>

#include <boost/thread/thread.hpp>*/

#include <iostream>
#include <libfreenect.hpp>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualisation/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <boost/lexical_cast.hpp>

// Mutex Class
// What does this do? CPP wrapper for pthread?
class Mutex {
private:
	pthread_mutex_t m_mutex;
public:
	Mutex() { pthread_mutex_init(&m_mutex, NULL); }
	void lock() { pthread_mutex_lock(&m_mutex); }
	void unlock() { pthread_mutex_unlock(&m_mutex); }
}

// Scoped Lock Class
// What does this do?
class ScopedLock {
private:
	pthread_mutex_t m_mutex;
public:
	Mutex & _mutex;
	ScopedLock(Mutex & mutex) : _mutex(mutex) {
		_mutex.lock();
	}
	~ScopedLock() {
		_mutex.unlock();
	}
}

// Kinect Hardware Connection Class
// Thanks to Yoda---- from IRC
class MyFreenectDevice : public Freenect::FreenectDevice {
private:
	
public:
	MyFreenectDevice(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes), m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_new_rgb_frame(false), m_new_depth_frame(false) {}
	//~MyFreenectDevice() {}
	
	// Do not call directly even in child!
	//loc 1147 of 4458
}