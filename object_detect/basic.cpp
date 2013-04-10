/*---------------------------------------------*-
 * Table Plane Detection with PCL
 *
 * Eye Tracking Robotic Arm
 * By James Reuss
 * Copyright 2013
-*---------------------------------------------*-
 * basic.cpp
-*---------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/segmentation/sac_segmentation.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "../helpers/Mutex.hpp"
#include "../helpers/Freenect.hpp"




/*---------------------------------------------*-
 * START THE ENGINES! IT'S BRIDGING TIME!
 * PCL->OK OK?
-*---------------------------------------------*/
// libfreenect
Freenect::Freenect freenect;
MyFreenectDevice* device;
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_plane_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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
 * GLUT CALLBACKS
 * There's no food here... Go home.
-*---------------------------------------------*/
int window_id;
float zoom=1.2;
float offset[3] = {0, 0, 0};
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles

void drawCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Modelview matrix
	glLoadIdentity();
	glScalef(zoom, zoom, 1);	// Matrix tranformations to alter where the object is rendered.
	glTranslatef(0, 0, -1000);
	glTranslatef(offset[0], offset[1], -offset[2]);
	glRotatef(rotangles[0], 1, 0, 0);
	glRotatef(rotangles[1], 0, 1, 0);
	glTranslatef(0, 0, 750);
	
	//show point cloud here
	glPointSize(1.0f);
	glBegin(GL_POINTS);
	
	for (unsigned int i = 0; i < cloud->size(); i++) {
		glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
		glVertex3f(cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
	}
	
	glEnd();
	
	if (done_ransac) {
		/*glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		//show ransac indices
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
		
		glBegin(GL_QUADS);
		glColor4f(.392156863, 1, .392156863, 0.3);
		// bl, tl, tr, br
		glVertex3d(bl_x, -bl_y, -bl_z);
		glVertex3d(tl_x, -tl_y, -tl_z);
		glVertex3d(tr_x, -tr_y, -tr_z);
		glVertex3d(br_x, -br_y, -br_z);
		
		
		glEnd();*/
		
		glPointSize(1.0f);
		glBegin(GL_POINTS);
		
		for (unsigned int i = 0; i < table_plane_cloud->size(); i++) {
			//glColor3ub(table_plane_cloud->points[i].r, table_plane_cloud->points[i].g, table_plane_cloud->points[i].b);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(table_plane_cloud->points[i].x, -table_plane_cloud->points[i].y, -table_plane_cloud->points[i].z);
		}
		
		glEnd();
	}
	
	
	
	// end drawing cloud
	
	/*printf("\tDrawing Cloud - Mid point:: x:%.3f y:%.3f z:%.3f r:%d g:%d b:%d\n",
		   cloud->points[cloud->size()/2].x,
		   cloud->points[cloud->size()/2].y,
		   cloud->points[cloud->size()/2].z,
		   cloud->points[cloud->size()/2].r,
		   cloud->points[cloud->size()/2].g,
		   cloud->points[cloud->size()/2].b);
	printf("\tDrawing Cloud - Mid point:: x:%.3f y:%.3f z:%.3f r:%d g:%d b:%d    +10\n",
		   cloud->points[cloud->size()/2+10].x,
		   cloud->points[cloud->size()/2+10].y,
		   cloud->points[cloud->size()/2+10].z,
		   cloud->points[cloud->size()/2+10].r,
		   cloud->points[cloud->size()/2+10].g,
		   cloud->points[cloud->size()/2+10].b);
	printf("\n");*/
	
	glutSwapBuffers();
}
void glutIdleCallback()
{
	//printf("Start Idle\n");
	static double x = NULL;
	static double y = NULL;
	static int iRealDepth = 0;
	
	// This is where we grab the new frames and fill the cloud
	if (!killKinect) {
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
		
		if (print_randpoint) {
			print_randpoint = false;
			randPoint = rand()%307200 - 1;
			std::cerr << "Rand Point (" << randPoint <<
			") is  x:" << cloud->points[randPoint].x <<
			"  y:" << cloud->points[randPoint].y <<
			"  z:" << cloud->points[randPoint].z << std::endl;
		}
		if (print_lastpoint) {
			print_lastpoint = false;
			std::cerr << "Last Point (" << randPoint <<
			") is  x:" << cloud->points[randPoint].x <<
			"  y:" << cloud->points[randPoint].y <<
			"  z:" << cloud->points[randPoint].z << std::endl;
		}
		if (print_midpoint) {
			print_midpoint = false;
			std::cerr << "Mid Point (" << 153600 <<
			") is  x:" << cloud->points[153600].x <<
			"  y:" << cloud->points[153600].y <<
			"  z:" << cloud->points[153600].z << std::endl;
		}
	}
	
	// Table & Object Detection
	if (do_detection && !killKinect) {
		printf("Doing some detection...\n");
		done_ransac = false;
		done_detection = false;
		
		// Create the segmentation object and do RANSAC
		/*pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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
		 
		 pcl::sample
		 
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
		 "  z:" << cloud->points[(size_t)(cloud->size()/2+(640*100+46))].z << std::endl;*/
		
		// http://pointclouds.org/documentation/tutorials/random_sample_consensus.php
		std::vector<int> inliers;
		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr planeModel (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (planeModel);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
		// copy all the inliers of the model to another point cloud.
		pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *table_plane_cloud);
		done_ransac = true;
		
		if (show_table_cloud) {
			
		}
		
		if (show_table_hull) {
			
		}
		
		if (show_clusters) {
			
		}
		
		if (fit_objects) {
			//fit objects now. not included in table_top_detector. must do manually.
		}
		
		done_detection = true;
		do_detection = false;
		printf("Detection complete.\n");
	}
	
	glutPostRedisplay();
	//printf("End Idle\n");
}
void resizeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, 4/3., 0.3, 4000);
	glMatrixMode(GL_MODELVIEW);
}
void keyboardCallback(unsigned char key, int x, int y)
{
	if (key == 27 || key == 'q') {
		glutDestroyWindow(window_id);
		killKinect = true;
		device->stopVideo();
		device->stopDepth();
		printf("All stopped. Bye bye.");
		exit(0);	// gonna kill the program right now! careful!
	}
	if (key == 'w')
		offset[2] += 10.0f;
	if (key == 's')
		offset[2] -= 10.0f;
	if (key == 'r')
		print_randpoint = true;
	if (key == 'l')
		print_lastpoint = true;
	if (key == 'm')
		print_midpoint = true;
	if (key == 'd')
		do_detection = true;
}
void movementCallback(int x, int y)
{
	if (mx >= 0 && my >= 0) {
		rotangles[0] += y - my;
		rotangles[1] += x - mx;
	}
	mx = x;
	my = y;
}
void mouseCallback(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mx = x;
		my = y;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
		mx = -1;
		my = -1;
	}
}




/*---------------------------------------------*-
 * MAIN
 * Steak?
-*---------------------------------------------*/
int main (int argc, char** argv)
{
	printf("RobotArm - SimpleGL Viewer: starting...\n");
	
	// Fill in the cloud data
	cloud->width = 640;
	cloud->height = 480;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	
	// Initialise the detector
	
	
	// Actually starting up the Kinect
	printf("Starting Kinect...\n");
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->startDepth();
	boost::this_thread::sleep (boost::posix_time::seconds (1));
	// Grab until clean returns...
	printf("Kinect Started. Now waiting for clean depth returns.\n");
	int DepthCount = 0;
	int checkCount = 0;
	while (DepthCount == 0) {
		checkCount++;
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		for (size_t i = 0;i < 480*640;i++) DepthCount+=mdepth[i];
		if (checkCount > 10) {
			printf("Kinect not responding correctly. Exiting.\n");
			exit(1);
		}
	}
	device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
	device->setVideoFormat(FREENECT_VIDEO_RGB);
	printf("All done. Lets go.\n");
	
	
	// Initialise GLUT library
	glutInit(&argc, argv);
	
	// Create GLUT window
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	window_id = glutCreateWindow("Simple OpenGL PCL Kinect");
	
	// Setup GLUT callbacks
	glutDisplayFunc(drawCallback);
	glutReshapeFunc(resizeCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMotionFunc(movementCallback);
	glutMouseFunc(mouseCallback);
	glutIdleFunc(glutIdleCallback);
	
	// Default settings
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glEnable(GL_DEPTH_TEST);
	
	resizeCallback(640, 480);
	
	// Main Loop. Mmmmm continuous steak. I will never return!
	glutMainLoop();
	
	return 0;
}