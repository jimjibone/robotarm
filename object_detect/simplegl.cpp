/*---------------------------------------------*-
 * Simple Kinect Access with PCL and OpenGL
 *
 * Eye Tracking Robotic Arm
 * By James Reuss
 * Copyright 2013
-*---------------------------------------------*-
 * simplegl.cpp
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
#include <pcl/point_types.h>

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
unsigned int cloud_id = 0;




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
		glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
		glVertex3f(cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
	}
	
	glEnd();
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
	if (key == 27) {
		glutDestroyWindow(window_id);
		killKinect = true;
		device->stopVideo();
		device->stopDepth();
		exit(0);	// gonna kill the program right now! careful!
	}
	if (key == 'w')
		zoom *= 1.1f;
	if (key == 's')
		zoom /= 1.1f;
	if (key == 'c');
		//colour = !colour;
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
				cloud->points[i].r = mrgb[i*3];
				cloud->points[i].g = mrgb[(i*3)+1];
				cloud->points[i].b = mrgb[(i*3)+2];
			}
		}
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