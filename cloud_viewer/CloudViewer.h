//
//  CloudViewer.h
//  Project
//
//  Created by James Reuss on 03/03/2013.
//
//

#ifndef __robotarm__CloudViewer__
#define __robotarm__CloudViewer__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <pthread.h>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include "../helpers/Mutex.hpp"

#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define __use_C_method__

// The good stuff from http://stackoverflow.com/questions/3589422/using-opengl-glutdisplayfunc-within-class

class CloudViewer
{
public:
	CloudViewer(const std::string _name) : windowName(_name)
	{
		rotangles[0] = 0;
		rotangles[1] = 0;
		zoom = 1;
		colour = 1;
		
		cloudIsSet = false;
		indexCount = 0;
		for (int i = 0; i < 480; i++) {
			for (int j = 0; j < 640; j++) {
				indices[i][j] = i*640+j;
				indexCount++;
			}
		}
	};
	~CloudViewer();
	void startCloudViewer(int *argc, char **argv, bool *aKillBool);	// NEVER RETURNS (well, until close..)
	void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &aCloud);
	
#ifdef __use_C_method__
	// Funcs
	void draw();
	void drawCloud();
	//otherdraws
	void resize(int width, int height);
	void keyboard(unsigned char key, int x, int y);
	void movement(int x, int y);
	void mouse(int button, int state, int x, int y);
#endif
	
private:
	static CloudViewer* currentInstance;
	int window;				// GLUT reference to the window we create.
	std::string windowName;	// User chosen window name for GLUT.
	bool *killBoolPtr;		// A pointer to a boolean that will tell whoever is outside to stop.
	
	// Orientation
	int mx, my;			// Previous mouse coords.
	int rotangles[2];	// Panning angles.
	float zoom;			// Zoom factor.
	int colour;			// Use RGB camera data or not.
	
	// Cloud stuff
	Mutex cloud_mutex;
	bool cloudIsSet;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	unsigned int indices[480][640];
	unsigned int indexCount;
	
	// Setup
	void setupGLUTCallbacks();
	
	// Callbacks
	static void drawCallback();
	static void resizeCallback(int width, int height);
	static void keyboardCallback(unsigned char key, int x, int y);
	static void movementCallback(int x, int y);
	static void mouseCallback(int button, int state, int x, int y);
	
#ifndef __use_C_method__
	// Funcs
	void draw();
	void drawCloud();
	//otherdraws
	void resize(int width, int height);
	void keyboard(unsigned char key, int x, int y);
	void movement(int x, int y);
	void mouse(int button, int state, int x, int y);
#endif
};

#endif /* defined(__robotarm__CloudViewer__) */
