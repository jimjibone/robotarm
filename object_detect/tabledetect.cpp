/*---------------------------------------------*-
 * Table Plane Detection with PCL
 *
 * Eye Tracking Robotic Arm
 * By James Reuss
 * Copyright 2013
-*---------------------------------------------*-
 * tabledetect.cpp
-*---------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/msac.h>

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
#include "../helpers/CloudUtils.hpp"
#include "../helpers/TableTopDetector.h"




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
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colour (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
unsigned int cloud_id = 0;

/*---------------------------------------------*-
 * Table Detection Stuff
-*---------------------------------------------*/
TableTopDetector<pcl::PointXYZ> detector;
bool do_detection = 0;
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
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles

void drawCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Modelview matrix
	glLoadIdentity();
	glScalef(zoom, zoom, 1);	// Matrix tranformations to alter where the object is rendered.
	glTranslatef(0, 0, -1000);
	glRotatef(rotangles[0], 1, 0, 0);
	glRotatef(rotangles[1], 0, 1, 0);
	glTranslatef(0, 0, 750);
	
	//show point cloud here
	glPointSize(1.0f);
	glBegin(GL_POINTS);
	
	for (unsigned int i = 0; i < cloud->size(); i++) {
		//glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
		glColor3f(1.0, 1.0, 1.0);
		glVertex3f(cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
	}
	
	glEnd();
	
	
	if (done_detection) {
		//show downsampled cloud here
		glPointSize(2.0f);
		glBegin(GL_POINTS);
		
		for (unsigned int i = 0; i < detector.cloud_downsampled_->size(); i++) {
			//glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(detector.cloud_downsampled_->points[i].x, -detector.cloud_downsampled_->points[i].y, -detector.cloud_downsampled_->points[i].z);
		}
		
		glEnd();
	}
	
	// end drawing cloud
	
	/*printf("\tDrawing Cloud - Mid point:: x:%.3f y:%.3f z:%.3f r:%d g:%d b:%d\n",
		   cloud_colour->points[cloud->size()/2].x,
		   cloud_colour->points[cloud->size()/2].y,
		   cloud_colour->points[cloud->size()/2].z,
		   cloud_colour->points[cloud->size()/2].r,
		   cloud_colour->points[cloud->size()/2].g,
		   cloud_colour->points[cloud->size()/2].b);
	printf("\tDrawing Cloud - Mid point:: x:%.3f y:%.3f z:%.3f r:%d g:%d b:%d    +10\n",
		   cloud_colour->points[cloud->size()/2+10].x,
		   cloud_colour->points[cloud->size()/2+10].y,
		   cloud_colour->points[cloud->size()/2+10].z,
		   cloud_colour->points[cloud->size()/2+10].r,
		   cloud_colour->points[cloud->size()/2+10].g,
		   cloud_colour->points[cloud->size()/2+10].b);
	printf("\n");*/
	
	glutSwapBuffers();
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
		zoom *= 1.1f;
	if (key == 's')
		zoom /= 1.1f;
	if (key == 'c');
		//colour = !colour;
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
				//cloud->points[i].r = mrgb[i*3];
				//cloud->points[i].g = mrgb[(i*3)+1];
				//cloud->points[i].b = mrgb[(i*3)+2];
			}
		}
	}
	
	// Table & Object Detection
	if (do_detection) {
		printf("Doing some detection...\n");
		done_detection = false;
#warning need to add table detect funcs here
		// Create a smaller cloud and detect it!
		//pcl::PointCloud<pcl::PointNormal>::Ptr small_cloud = preprocessImageCloud(cloud);
		//detector.detect(*small_cloud);
		//detector.detect(*cloud);
		
		detector.filter(*cloud);
		detector.findTable();
		
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
	detector.setDepthLimits(10, 3000);
	detector.setBackgroundVoxelSize(0.05);
	detector.initialize();
	
	
	// Actually starting up the Kinect
	printf("Starting Kinect...\n");
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->startDepth();
	boost::this_thread::sleep (boost::posix_time::seconds (1));
	// Grab until clean returns...
	printf("Kinect Started. Now waiting for clean depth returns.\n");
	int DepthCount = 0;
	while (DepthCount == 0) {
		device->updateState();
		device->getDepth(mdepth);
		device->getRGB(mrgb);
		for (size_t i = 0;i < 480*640;i++) DepthCount+=mdepth[i];
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