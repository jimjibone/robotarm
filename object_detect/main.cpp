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
#include <pcl/visualization/pcl_visualizer.h>
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
	
	// Scoped Lock Class
	// What does this do?
	class ScopedLock {
	public:
		Mutex & _mutex;
		ScopedLock(Mutex & mutex) : _mutex(mutex) {
			_mutex.lock();
		}
		~ScopedLock() {
			_mutex.unlock();
		}
	};
};

// Kinect Hardware Connection Class
// Thanks to Yoda---- from IRC
class MyFreenectDevice : public Freenect::FreenectDevice {
private:
	std::vector<uint16_t> depth;
	std::vector<uint8_t> m_buffer_video;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
public:
	MyFreenectDevice(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes), m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_new_rgb_frame(false), m_new_depth_frame(false) {}
	//~MyFreenectDevice() {}
	
	// Do not call directly even in child!
	void VideoCallback(void* _rgb, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
		m_new_rgb_frame = true;
	}
	// Do not call directly even in child!
	void DepthCallback(void* _depth, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_depth_mutex);
		depth.clear();
		uint16_t* call_depth = static_cast<uint16_t*>(_depth);
		for (size_t i = 0; i < 640*480; i++) {
			depth.push_back(call_depth[i]);
		}
		m_new_depth_frame = true;
	}
	bool getRGB(std::vector<uint8_t>&buffer) {
		//printf("Getting RGB!\n");
		Mutex::ScopedLock lock(m_rgb_mutex);
		if (!m_new_rgb_frame) {
			//printf("No new RGB frame.\n");
			return false;
		}
		buffer.swap(m_buffer_video);
		m_new_rgb_frame = false;
		return true;
	}
	bool getDepth(std::vector<uint16_t>&buffer) {
		Mutex::ScopedLock lock(m_depth_mutex);
		if (!m_new_depth_frame)
			return false;
		buffer.swap(depth);
		m_new_depth_frame = false;
		return true;
	}
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
int got_frames(0), window(0);
int g_argc;
char **g_argv;
int user_data = 0;
// point clouds 'n' that.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
unsigned int cloud_id = 0;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);




/*---------------------------------------------*-
 * MAIN
 * Steak?
-*---------------------------------------------*/
int main(int argc, char** argv) {
	// Some more Kinect setup
	static std::vector<uint16_t> mdepth(640*480);
	static std::vector<uint8_t> mrgb(640*480*4);
	
	// Fill in the cloud data
	cloud->width	= 640;
	cloud->height	= 480;
	cloud->is_dense	= false;
	cloud->points.resize(cloud->width * cloud->height);
	
	// Create and setup viewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Kinect Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Kinect Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	
	// Actually starting up the Kinect
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->setartDepth();
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	// Grab until clean returns...
	int depthCount = 0;
	while (depthCount == 0) {
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		for (size_t i = 0; i < 640*480; i++)
			depthCount += mdepth[i];
	}
	device->setVideoFormat(requested_format);
	
	// Main Loop. Mmmmm continuous steak.
	double x = NULL;
	double y = NULL;
	int iRealDepth = 0;
	while (!viewer->wasStopped()) {
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		
		size_t i = 0;
		size_t cinput = 0;
		for (size_t v = 0; v < 480; v++) {
			for (size_t u = 0; u < 640; u++) {
				iRealDepth = mdepth[i];
				freenect_camera_to_world(device->getDevice(), u, v, iRealDepth, &x, &y);
				cloud->points[i].x = x;
				cloud->points[i].y = y;
				cloud->points[i].z = iRealDepth;
				cloud->points[i].r = mrgb[i*3];
				cloud->points[i].g = mrgb[i*3+1];
				cloud->points[i].b = mrgb[i*3+2];
			}
		}
		
		viewer->updatePointCloud(cloud, "Kinect Cloud");
		viewer->spinOnce();
	}
	
	device->stopVideo();
	device->stopDepth();
	return 0;
}




/*---------------------------------------------*-
 * KEYBOARD EVENT TRACKING
 * Keys we like: (* = all PCL owned)
 * 'q' = quit*
 * 'r' = recentre the view*
 * 'c' = capture
-*---------------------------------------------*/
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "c" && event.keyDown()) {
		std::cout << "c was pressed => capturing a point cloud" << std::endl;
		std::string filename = "KinectCap";
		filename.append(boost::lexical_cast<std::string>(cloud_id));
		filename.append(".pcd");
		pcl::io::savePCDFileASCII(filename, *cloud);
		cloud_id++;
	}
}
