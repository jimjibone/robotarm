//
//  CloudViewer.cpp
//  Project
//
//  Created by James Reuss on 03/03/2013.
//
//

#include "CloudViewer.h"

#ifdef __use_C_method__
CloudViewer *g_CurrentInstance;
extern "C"

void drawCallback_C() {
	g_CurrentInstance->draw();
}
void resizeCallback_C(int width, int height) {
	g_CurrentInstance->resize(width, height);
}
void keyboardCallback_C(unsigned char key, int x, int y) {
	g_CurrentInstance->keyboard(key, x, y);
}
void movementCallback_C(int x, int y) {
	g_CurrentInstance->movement(x, y);
}
void mouseCallback_C(int button, int state, int x, int y) {
	g_CurrentInstance->mouse(button, state, x, y);
}
#endif


/*---------------------------------------------*-
 * SETUP FUNCTIONS
 -*---------------------------------------------*/
void CloudViewer::startCloudViewer(int *argc, char **argv, bool *aKillBool)
{
	glutInit(argc, argv);	// Initialise GLUT library
	
	killBoolPtr = aKillBool;
	
	// Create GLUT window
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow(windowName.data());
	
	setupGLUTCallbacks();
	
	// Default settings
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glEnable(GL_DEPTH_TEST);
	
	resize(640, 480);
	glutMainLoop();	// This main loop will never return!
	printf("glut main loop finished! somethings wrong there.\n");
}

void CloudViewer::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &aCloud)
{
	Mutex::ScopedLock lock(cloud_mutex);
	
	// Copy over the new point cloud
	//pcl::copyPointCloud(*aCloud, *cloud);
	//printf("update cloud\n");
	cloud = aCloud;
	cloudIsSet = true;
}

void glutIdle() {
	printf("glut is being idle\n");
}

void CloudViewer::setupGLUTCallbacks()
{
#ifdef __use_C_method__
	::g_CurrentInstance = this;
	::glutDisplayFunc(::drawCallback_C);
	::glutReshapeFunc(::resizeCallback_C);
	::glutKeyboardFunc(::keyboardCallback_C);
	::glutMotionFunc(::movementCallback_C);
	::glutMouseFunc(::mouseCallback_C);
	::glutIdleFunc(glutIdle);
#else
	currentInstance = this;
	::glutDisplayFunc(CloudViewer::drawCallback);
	::glutReshapeFunc(CloudViewer::resizeCallback);
	::glutKeyboardFunc(CloudViewer::keyboardCallback);
	::glutMotionFunc(CloudViewer::movementCallback);
	::glutMouseFunc(CloudViewer::mouseCallback);
#endif
}





/*---------------------------------------------*-
 * CALLBACK FUNCTIONS
 -*---------------------------------------------*/
#ifdef __use_C_method__
void CloudViewer::drawCallback()
{
	printf("\t drawCallback() start\n");
	g_CurrentInstance->draw();
	printf("\t drawCallback() end\n");
}

void CloudViewer::resizeCallback(int width, int height)
{
	g_CurrentInstance->resize(width, height);
}

void CloudViewer::keyboardCallback(unsigned char key, int x, int y)
{
	g_CurrentInstance->keyboard(key, x, y);
}

void CloudViewer::movementCallback(int x, int y)
{
	g_CurrentInstance->movement(x, y);
}

void CloudViewer::mouseCallback(int button, int state, int x, int y)
{
	g_CurrentInstance->mouse(button, state, x, y);
}
#else
void CloudViewer::drawCallback()
{
	currentInstance->draw();
}

void CloudViewer::resizeCallback(int width, int height)
{
	currentInstance->resize(width, height);
}

void CloudViewer::keyboardCallback(unsigned char key, int x, int y)
{
	currentInstance->keyboard(key, x, y);
}

void CloudViewer::movementCallback(int x, int y)
{
	currentInstance->movement(x, y);
}

void CloudViewer::mouseCallback(int button, int state, int x, int y)
{
	currentInstance->mouse(button, state, x, y);
}
#endif



/*---------------------------------------------*-
 * DRAWING FUNCTIONS
 -*---------------------------------------------*/
void CloudViewer::draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Modelview matrix
	glLoadIdentity();
	glScalef(zoom, zoom, 1);	// Matrix tranformations to alter where the object is rendered.
	glTranslatef(0, 0, -3.5);
	glRotatef(rotangles[0], 1, 0, 0);
	glRotatef(rotangles[1], 0, 1, 0);
	glTranslatef(0, 0, 1.5);
	
	// Show point cloud n shizz here.
	printf("\tstart draw()\n");
	drawCloud();
	
	glutSwapBuffers();
	
	//if (killBoolPtr) {
	//	exit(0);//careful!
	//}
	printf("\tend draw()\n");
	return;
}

void CloudViewer::drawCloud()
{
	printf("\t\tstart draw cloud\n");
	//bool goforit = cloudIsSet;
	
	printf("\t\t\tdraw isSet = ");
	if (cloudIsSet == true) {
		printf("yes\n");
	} else {
		printf("no\n");
	}
	Mutex::ScopedLock lock(cloud_mutex);
	if (cloudIsSet) {
		// checkout book loc.2362 of 4458
		// or link http://www.pcl-users.org/Passing-point-color-data-to-opengl-td3047230.html
		
		//SIMPLE
		glPointSize(1.0f);
		glBegin(GL_POINTS);
		
		for (unsigned int i = 0; i < cloud->size(); i++) {
			glColor3ub(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);
			glVertex3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		}
		
		glEnd();
		
		printf("\t\t\tpoint %d = x:%.3f y:%.3f z:%.3f r:%d g:%d b:%d\n",
			   (int)cloud->size()/2,
			   cloud->points[(int)cloud->size()/2].x,
			   cloud->points[(int)cloud->size()/2].y,
			   cloud->points[(int)cloud->size()/2].z,
			   cloud->points[(int)cloud->size()/2].r,
			   cloud->points[(int)cloud->size()/2].g,
			   cloud->points[(int)cloud->size()/2].b);
		
		//COMPLICATED
		/*
		// Set up arrays
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		
		glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZRGB), &cloud->points[0].x);
		glColorPointer(4, GL_UNSIGNED_SHORT, sizeof(pcl::PointXYZRGB), &cloud->points[0].b);
		
		glDrawElements(GL_POINTS, cloud->size(), GL_UNSIGNED_INT, indices);
		
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
		
		//for using RGB from camera
		glEnable(GL_TEXTURE_2D);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, cloud->width, cloud->height, 0, GL_BGR, GL_UNSIGNED_BYTE, &cloud->points[0].b);
		//no implementation of non rgb yet
		
		glPointSize(1.0f);
		
		glDrawElements(GL_POINTS, cloud->size(), GL_UNSIGNED_INT, indices);
		 */
	}
	printf("\t\tend draw cloud\n");
}

void CloudViewer::resize(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, 4/3., 0.3, 200);
	glMatrixMode(GL_MODELVIEW);
}

void CloudViewer::keyboard(unsigned char key, int x, int y)
{
	if (key == 27) {
        glutDestroyWindow(window);
		*killBoolPtr = true;
        //exit(0);	// gonna kill the program right now! careful!
    }
    if (key == 'w')
        zoom *= 1.1f;
    if (key == 's')
        zoom /= 1.1f;
    if (key == 'c')
        colour = !colour;
}

void CloudViewer::movement(int x, int y)
{
	if (mx >= 0 && my >= 0) {
        rotangles[0] += y - my;
        rotangles[1] += x - mx;
    }
    mx = x;
    my = y;
}

void CloudViewer::mouse(int button, int state, int x, int y)
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
