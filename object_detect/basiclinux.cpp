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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>

#include "../helpers/Mutex.hpp"
#include "../helpers/Freenect.hpp"




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
std::vector<uint16_t> mdepth(640*480);
std::vector<uint8_t> mrgb(640*480*4);
bool killKinect = false;
// point clouds 'n' that.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
unsigned int cloud_id = 0;

/*---------------------------------------------*-
 * Table Detection Stuff
-*---------------------------------------------*/
bool print_randpoint = false; size_t randPoint = 0; bool print_lastpoint = false; bool print_midpoint = false;
bool do_detection = false;
bool done_ransac = false;

bool done_detection = false;
bool show_clusters = true;
bool show_table_hull = true;
bool show_table_cloud = false;
bool fit_objects = false;




/*---------------------------------------------*-
 * KEYBOARD EVENT TRACKING
 * Keys we like: (* = all PCL owned)
 * 'q' = quit*
 * 'r' = recentre the view*
 * 'c' = capture
-*---------------------------------------------*/
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "c"&&event.keyDown ())
	{
		std::cout <<"c was pressed => capturing a pointcloud"<< std::endl;
		std::string filename = "KinectCap";
		filename.append(boost::lexical_cast<std::string>(cloud_id));
		filename.append(".pcd");
		pcl::io::savePCDFileASCII (filename, *cloud);
		cloud_id++;
	}
	if (event.getKeySym() == "d" && event.keyDown())
	{
		std::cout << "d was pressed => going to detect now" << std::endl;
		do_detection = true;
	}
}




/*---------------------------------------------*-
 * MAIN
 * Steak?
-*---------------------------------------------*/
int main (int argc, char** argv)
{
	printf("Hello\n");
	// Some more Kinect setup
	static std::vector<uint16_t> mdepth(640*480);
	static std::vector<uint8_t> mrgb(640*480*4);
	
	// Fill in the cloud data
	printf("Make a cloud\n");
	cloud->width = 640;
	cloud->height = 480;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);

	// Create and setup the viewer
	printf("Make a viewer\n");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Kinect Cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "Kinect Cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	
	// Actually starting up the Kinect
	printf("Start the Kinect\n");
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->startDepth();
	boost::this_thread::sleep (boost::posix_time::seconds (1));
	// Grab until clean returns...
	printf("Start Kinect checks\n");
	int DepthCount = 0;
	int check = 0;
	while (DepthCount == 0) {
		printf("\tchecking... %d\n", check++);
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		for (size_t i = 0;i < 480*640;i++) DepthCount+=mdepth[i];
	}
	device->setVideoFormat(requested_format);

	// Main Loop. Mmmmm continuous steak.
	printf("Main Loop Time\n");
	double x = NULL;
	double y = NULL;
	int iRealDepth = 0;
	while (!viewer->wasStopped ())
	{
		device->updateState();	// Use this to make sure the Kinect is still alive.
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		
		size_t i = 0;
		size_t cinput = 0;
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
		
		// Table & Object Detection
		if (do_detection) {
			printf("Doing some detection...\n");
			done_ransac = false;
			done_detection = false;
			
			// Create the segmentation object and do RANSAC
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.01);
			seg.setInputCloud(cloud);
			seg.segment(*inliers, *coefficients);
			std::cerr << "PointCloud after segmentation has: " << inliers->indices.size () << " inliers." << std::endl;
			std::cerr << "Plane coefficients:  a:" << coefficients->values[0] <<
											"  b:" << coefficients->values[1] <<
											"  c:" << coefficients->values[2] <<
											"  d:" << coefficients->values[3] << std::endl;
			done_ransac = true;
			
			double boxSize = 300.0;
			double bl_x = -boxSize, bl_y = -boxSize;
			double tl_x = -boxSize, tl_y =  boxSize;
			double tr_x =  boxSize, tr_y =  boxSize;
			double br_x =  boxSize, br_y = -boxSize;
			// ax + by + cz + d = 0
			// ax + by + d = -cz
			// -(ax + by + d)/c = z
			double bl_z = -((coefficients->values[0]*bl_x) + (coefficients->values[1]*bl_y) + coefficients->values[3])/(coefficients->values[2]*1.0);
			double tl_z = -((coefficients->values[0]*tl_x) + (coefficients->values[1]*tl_y) + coefficients->values[3])/(coefficients->values[2]*1.0);
			double tr_z = -((coefficients->values[0]*tr_x) + (coefficients->values[1]*tr_y) + coefficients->values[3])/(coefficients->values[2]*1.0);
			double br_z = -((coefficients->values[0]*br_x) + (coefficients->values[1]*br_y) + coefficients->values[3])/(coefficients->values[2]*1.0);
			
			std::cerr << "Plane in 3D is  BL: x:" << bl_x << "  y:" << bl_y << "  z:" << bl_z << std::endl;
			std::cerr << "                TL: x:" << tl_x << "  y:" << tl_y << "  z:" << tl_z << std::endl;
			std::cerr << "                TR: x:" << tr_x << "  y:" << tr_y << "  z:" << tr_z << std::endl;
			std::cerr << "                BR: x:" << br_x << "  y:" << br_y << "  z:" << br_z << std::endl;
			std::cerr << "Off-Mid Point (" << (size_t)(cloud->size()/2+(640*100+46)) <<
							") is  x:" << cloud->points[(size_t)(cloud->size()/2+(640*100+46))].x <<
								"  y:" << cloud->points[(size_t)(cloud->size()/2+(640*100+46))].y <<
								"  z:" << cloud->points[(size_t)(cloud->size()/2+(640*100+46))].z << std::endl;
			
			done_detection = true;
			do_detection = false;
			
			viewer->addPlane(*coefficients, "plane", 0);
			
			printf("Detection complete.\n");
		}

		viewer->updatePointCloud (cloud, "Kinect Cloud");
		viewer->spinOnce ();
	}
	printf("render loop finished\n");
	device->stopVideo();
	device->stopDepth();
	printf("Goodbye\n");
	return 0;
}
