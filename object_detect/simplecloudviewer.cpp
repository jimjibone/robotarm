/*---------------------------------------------*-
 * Simple Kinect Access with PCL
 *
 * Eye Tracking Robotic Arm
 * By James Reuss
 * Copyright 2013
-*---------------------------------------------*-
 * simple.cpp
-*---------------------------------------------*/

#include <iostream>
#include <libfreenect.hpp>
#include <libfreenect/libfreenect-registration.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include "../cloud_viewer/CloudViewer.h"
#include "../helpers/Mutex.hpp"



// Kinect Hardware Connection Class
// Thanks to Yoda---- from IRC (libfreenect)
class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index):
			Freenect::FreenectDevice(_ctx, _index),
			depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_DEPTH_REGISTERED).bytes),
			m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_VIDEO_RGB).bytes),
			m_new_rgb_frame(false), m_new_depth_frame(false)
			{
			}
	//~MyFreenectDevice(){}
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			Mutex::ScopedLock lock(m_rgb_mutex);
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
			m_new_rgb_frame = true;
		};
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			Mutex::ScopedLock lock(m_depth_mutex);
			depth.clear();
			uint16_t* call_depth = static_cast<uint16_t*>(_depth);
			for (size_t i = 0; i < 640*480 ; i++) depth.push_back(call_depth[i]);
			m_new_depth_frame = true;
		}
		bool getRGB(std::vector<uint8_t>&buffer) {
			//printf("Getting RGB!\n");
			Mutex::ScopedLock lock(m_rgb_mutex);
			if (!m_new_rgb_frame) {
				//printf("No new RGB Frame.\n");
				return false;
			}
			buffer.swap(m_buffer_video);
			m_new_rgb_frame = false;
			return true;
		}
		bool getDepth(std::vector<uint16_t>&buffer) {
			Mutex::ScopedLock lock(m_depth_mutex);
			if (!m_new_depth_frame) return false;
			buffer.swap(depth);
			m_new_depth_frame = false;
			return true;
		}
	private:
		std::vector<uint16_t> depth;
		std::vector<uint8_t> m_buffer_video;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};




/*---------------------------------------------*-
 * START THE ENGINES! IT'S BRIDGING TIME!
 * PCL->OK OK?
-*---------------------------------------------*/
// libfreenect
Freenect::Freenect freenect;
MyFreenectDevice* device;
freenect_video_format requested_format(FREENECT_VIDEO_RGB);
double freenect_angle(0);
int got_frames(0),window(0);
int g_argc;
char **g_argv;
int user_data = 0;
// point clouds 'n' that.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
unsigned int cloud_id = 0;




CloudViewer *viewer;

/*---------------------------------------------*-
 * PROCESSING LOOP
-*---------------------------------------------*/
bool killProcessing = 0;
static void *processingThreadFunc(void *v)
{
	// Some more Kinect setup
	static std::vector<uint16_t> mdepth(640*480);
	static std::vector<uint8_t> mrgb(640*480*4);
	
	// Actually starting up the Kinect
	printf("Start the Kinect\n");
	device = &freenect.createDevice<MyFreenectDevice>(0);
	printf("\tcreated device\n");
	device->startVideo();
	printf("\tstarted video\n");
	device->startDepth();
	printf("\tstarted depth\n");
	boost::this_thread::sleep (boost::posix_time::seconds (1));
	printf("\tslept for 1 sec\n");
	// Grab until clean returns...
	int DepthCount = 0;
	printf("\tbegin grabs till clean returns\n");
	while (DepthCount == 0) {
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		for (size_t i = 0;i < 480*640;i++) DepthCount+=mdepth[i];
	}
	printf("\tfinished grabs\n");
	device->setVideoFormat(requested_format);
	printf("\tset video format\n");
	
	// Main Processing Loop Time. Mmmmm continuous steak.
	printf("Main Loop Time\n");
	double x = NULL;
	double y = NULL;
	int iRealDepth = 0;
	int count = 0;
	while (!killProcessing)
	{
		//printf("\tprocessed %d frames\n", ++count);
		device->updateState();	// Use this to make sure the Kinect is still alive.
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		
		size_t i = 0;
		for (size_t v=0 ; v<480 ; v++)
		{
			for ( size_t u=0 ; u<640 ; u++, i++)
			{
				iRealDepth = mdepth[i];
				freenect_camera_to_world(device->getDevice(), u, v, iRealDepth, &x, &y);
				cloud->points[i].x = x;
				cloud->points[i].y = y;
				cloud->points[i].z = iRealDepth;
				cloud->points[i].r = mrgb[i*3];
				cloud->points[i].g = mrgb[(i*3)+1];
				cloud->points[i].b = mrgb[(i*3)+2];
			}
		}
		
		viewer->updateCloud(cloud);
	}
	device->stopVideo();
	device->stopDepth();
	printf("processing loop finished\n");
	
	return;
}



/*---------------------------------------------*-
 * MAIN
 * Steak?
-*---------------------------------------------*/
int main (int argc, char** argv)
{
	printf("Hello\n");

	// Fill in the cloud data
	printf("Make a cloud\n");
	cloud->width = 640;
	cloud->height = 480;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	cloud->makeShared();
	
	// Start up the Kinect processing
	pthread_t processing_thread;
	pthread_create(&processing_thread, NULL, processingThreadFunc, 0);
	
	// Create and start the viewer and NEVER EVER RETURN!
	printf("Make a viewer\n");
	viewer = new CloudViewer("Kinect Cloud v2");
	
	viewer->startCloudViewer(&argc, argv, &killProcessing);
	
	printf("Goodbye\n");
	return 0;
}
