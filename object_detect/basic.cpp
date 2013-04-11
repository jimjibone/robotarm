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
#include <boost/shared_ptr.hpp>

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



using namespace std;
using namespace pcl;



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
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
Eigen::VectorXf plane_coefficients (4);
enum rotationType {
	ABCD = 0,//normal
	BACD,//ab
	CBAD,//ac
	DBCA,//ad
	ACBD,//bc
	ADCB,//bd
	ABDC //cd
};
rotationType rotation = ABCD;

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
		//glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
		glColor3f(0.8, 0.8, 1.0);
		glVertex3f(cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
	}
	
	glEnd();
	
	if (done_ransac) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		//show ransac indices
		double boxSize = 600.0;
		double bl_x = -boxSize, bl_y = -boxSize;
		double tl_x = -boxSize, tl_y =  boxSize;
		double tr_x =  boxSize, tr_y =  boxSize;
		double br_x =  boxSize, br_y = -boxSize;
		// ax + by + cz + d = 0
		// ax + by + d = -cz
		// -(ax + by + d)/c = z
		double bl_z = -((plane_coefficients[0]*bl_x) + (plane_coefficients[1]*bl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
		double tl_z = -((plane_coefficients[0]*tl_x) + (plane_coefficients[1]*tl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
		double tr_z = -((plane_coefficients[0]*tr_x) + (plane_coefficients[1]*tr_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
		double br_z = -((plane_coefficients[0]*br_x) + (plane_coefficients[1]*br_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
		
		switch (rotation) {
//				ABCD = 0,//normal
//				BACD,//ab
//				CBAD,//ac
//				DBCA,//ad
//				ACBD,//bc
//				ADCB,//bd
//				ABDC //cd
			case BACD:
				bl_z = -((plane_coefficients[1]*bl_x) + (plane_coefficients[0]*bl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				tl_z = -((plane_coefficients[1]*tl_x) + (plane_coefficients[0]*tl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				tr_z = -((plane_coefficients[1]*tr_x) + (plane_coefficients[0]*tr_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				br_z = -((plane_coefficients[1]*br_x) + (plane_coefficients[0]*br_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				break;
			case CBAD:
				bl_z = -((plane_coefficients[2]*bl_x) + (plane_coefficients[1]*bl_y) + plane_coefficients[0])/(plane_coefficients[2]*1.0);
				tl_z = -((plane_coefficients[2]*tl_x) + (plane_coefficients[1]*tl_y) + plane_coefficients[0])/(plane_coefficients[2]*1.0);
				tr_z = -((plane_coefficients[2]*tr_x) + (plane_coefficients[1]*tr_y) + plane_coefficients[0])/(plane_coefficients[2]*1.0);
				br_z = -((plane_coefficients[2]*br_x) + (plane_coefficients[1]*br_y) + plane_coefficients[0])/(plane_coefficients[2]*1.0);
				break;
			case DBCA:
				bl_z = -((plane_coefficients[3]*bl_x) + (plane_coefficients[1]*bl_y) + plane_coefficients[2])/(plane_coefficients[0]*1.0);
				tl_z = -((plane_coefficients[3]*tl_x) + (plane_coefficients[1]*tl_y) + plane_coefficients[2])/(plane_coefficients[0]*1.0);
				tr_z = -((plane_coefficients[3]*tr_x) + (plane_coefficients[1]*tr_y) + plane_coefficients[2])/(plane_coefficients[0]*1.0);
				br_z = -((plane_coefficients[3]*br_x) + (plane_coefficients[1]*br_y) + plane_coefficients[2])/(plane_coefficients[0]*1.0);
				break;
			case ACBD:
				bl_z = -((plane_coefficients[0]*bl_x) + (plane_coefficients[2]*bl_y) + plane_coefficients[1])/(plane_coefficients[3]*1.0);
				tl_z = -((plane_coefficients[0]*tl_x) + (plane_coefficients[2]*tl_y) + plane_coefficients[1])/(plane_coefficients[3]*1.0);
				tr_z = -((plane_coefficients[0]*tr_x) + (plane_coefficients[2]*tr_y) + plane_coefficients[1])/(plane_coefficients[3]*1.0);
				br_z = -((plane_coefficients[0]*br_x) + (plane_coefficients[2]*br_y) + plane_coefficients[1])/(plane_coefficients[3]*1.0);
				break;
			case ADCB:
				bl_z = -((plane_coefficients[0]*bl_x) + (plane_coefficients[3]*bl_y) + plane_coefficients[2])/(plane_coefficients[1]*1.0);
				tl_z = -((plane_coefficients[0]*tl_x) + (plane_coefficients[3]*tl_y) + plane_coefficients[2])/(plane_coefficients[1]*1.0);
				tr_z = -((plane_coefficients[0]*tr_x) + (plane_coefficients[3]*tr_y) + plane_coefficients[2])/(plane_coefficients[1]*1.0);
				br_z = -((plane_coefficients[0]*br_x) + (plane_coefficients[3]*br_y) + plane_coefficients[2])/(plane_coefficients[1]*1.0);
				break;
			case ABDC:
				bl_z = -((plane_coefficients[0]*bl_x) + (plane_coefficients[1]*bl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				tl_z = -((plane_coefficients[0]*tl_x) + (plane_coefficients[1]*tl_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				tr_z = -((plane_coefficients[0]*tr_x) + (plane_coefficients[1]*tr_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				br_z = -((plane_coefficients[0]*br_x) + (plane_coefficients[1]*br_y) + plane_coefficients[3])/(plane_coefficients[2]*1.0);
				break;
				
			default:
				//ABCD or Original
				break;
		}
		
		
		glBegin(GL_QUADS);
		glColor4f(.392156863, 1, .392156863, 0.3);
		// bl, tl, tr, br
		glVertex3d(bl_x, -bl_y, -bl_z);
		glVertex3d(tl_x, -tl_y, -tl_z);
		glVertex3d(tr_x, -tr_y, -tr_z);
		glVertex3d(br_x, -br_y, -br_z);
		
		
		glEnd();
		
		/*glPointSize(1.0f);
		glBegin(GL_POINTS);
		
		for (unsigned int i = 0; i < table_plane_cloud->size(); i++) {
			//glColor3ub(table_plane_cloud->points[i].r, table_plane_cloud->points[i].g, table_plane_cloud->points[i].b);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(table_plane_cloud->points[i].x, -table_plane_cloud->points[i].y, -table_plane_cloud->points[i].z);
		}
		
		glEnd();*/
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
				//cloud->points[i].r = mrgb[i*3];
				//cloud->points[i].g = mrgb[(i*3)+1];
				//cloud->points[i].b = mrgb[(i*3)+2];
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
		
		// Perform the RANSAC algorithm!
		// http://pointclouds.org/documentation/tutorials/random_sample_consensus.php
		// PCL-segmentation.pdf
		// Create a shared plane model pointer directly
		SampleConsensusModelPlane<PointXYZ>::Ptr model (new SampleConsensusModelPlane<PointXYZ>(cloud));
		// Create the RANSAC object
		RandomSampleConsensus<PointXYZ> sac(model, 0.03); // 3cm
		// Perform the segmentation step
		bool result = sac.computeModel();
		if (result) {
			cout << "RANSAC computeModel() was successful" << endl;
		} else {
			cout << "RANSAC computeModel() was UNsuccessful" << endl;
		}
		// Get inlier indices
		boost::shared_ptr<vector<int> > inliers (new vector<int>);
		sac.getInliers(*inliers);	// retrieve the best set of inliers
		cout << "Found model with " << inliers->size() << " inliers";
		// Get model coefficients
		Eigen::VectorXf coeff;
		sac.getModelCoefficients(coeff);	// ..and the corresponding plane model coefficients
		cout << ". plane normal is: " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << ", " << coeff[3] << endl;
		// Then some optional stuff
		// Perform a refitting step
		Eigen::VectorXf coeff_refined;
		model->optimizeModelCoefficients(*inliers, coeff, coeff_refined);
		model->selectWithinDistance(coeff_refined, 0.03, *inliers);
		cout << "After refitting, model contains " << inliers->size() << " inliers";
		cout << ", plane normal is: " << coeff_refined[0] << ", "
									  << coeff_refined[1] << ", "
									  << coeff_refined[2] << ", "
									  << coeff_refined[3] << "." << endl;
		// Projection
		PointCloud<PointXYZ> proj_points;
		model->projectPoints(*inliers, coeff_refined, proj_points);
		// Set the global coefficients
		plane_coefficients[0] = coeff_refined[0];
		plane_coefficients[1] = coeff_refined[1];
		plane_coefficients[2] = coeff_refined[2];
		plane_coefficients[3] = coeff_refined[3];
		
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
	if (key == 'c') {
//				ABCD = 0,//normal
//				BACD,//ab
//				CBAD,//ac
//				DBCA,//ad
//				ACBD,//bc
//				ADCB,//bd
//				ABDC //cd
		if (rotation == ABDC) {
			rotation = ABCD;
		} else {
			rotation++;
		}
		cout << "rotating the plane coefficients to type " << rotation << endl;
	}
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