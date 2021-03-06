//
//  GLView.h
//  KinectiCopter Mark 2
//
//  Created by James Reuss on 03/06/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <CoreVideo/CoreVideo.h>
#import <OpenGL/OpenGL.h>
#import <OpenGL/gl.h>
#import <OpenGL/glu.h>
#import <GLUT/GLUT.h>
#import "GLProgram.h"
#import "JRKinectHelpers.h"
#import "JRObjectDetector.h"
#import "JRObjectDetectionWrapper.h"
#import "JRObjectHelpers.h"
#import "JRPointTypes.h"
//#import "JRPointRotationXYZ.h"

//#define REMOVE_VIEW_DEPTH
#define REMOVE_VIEW_DEPTH_VALUE 1000

@class GLProgram;
@class JRObjectDetector;
@interface GLView : NSOpenGLView {
	IBOutlet NSWindow *window;
    
    CVDisplayLinkRef _displayLink;
	
	BOOL _newDepth, _newRGB;
	
	uint16_t *_intDepth;
	uint8_t *_intRGB;
    GLuint _depthTex, _videoTex, _colormapTex;
    GLuint _pointBuffer;
    
    GLuint *_indicies;
    int _nTriIndicies;
    
    GLProgram *_pointProgram;
    GLProgram *_depthProgram;
	
	// Object Detection
	PlaneCoefficients *_segmented_planes;
	size_t _segmented_planes_count;
	PointXYZ *_segmented_planes_points;
	size_t *_segmented_planes_points_counts;
	
	PlaneCoefficients _dominant_plane_hull_coefficients;
	PointXYZ *_dominant_plane_hull_points;
	size_t _dominant_plane_hull_point_count;
	
	size_t _objects_count;
	PointXYZ *_objects_points;
	size_t *_objects_points_counts;
	PointXYZ *_objects_centroids;
	double *_objects_radii;
    
    // 3D navigation
    NSPoint _lastPos, _lastPosRight;
    float _offset[3];
    float _angle, _tilt, _roll;
	//JRPointRotationXYZ *_rotation;
	
	// Draw Modes
	BOOL _drawFrustrum;
	BOOL _drawSegmentedPlanes;
	BOOL _drawDominantPlanePoints;
	BOOL _drawDominantPlane;
	BOOL _drawObjectsPoints;
	BOOL _normals;
    BOOL _mirror;
    BOOL _natural;
#ifndef DRAW_MODE
#define DRAW_MODE
    enum drawMode {
        MODE_2D = 0,
        MODE_POINTS = 1,
        MODE_MESH = 2,
    };
#endif
	enum drawMode _drawMode;
}

- (void)setDrawMode:(enum drawMode)newDrawMode;
- (void)swapInNewDepthFrame:(uint16_t**)newDepth RGBFrame:(uint8_t**)newRGB;

- (void)setObjectDetectionData:(JRObjectDetectionWrapper*)objectDetector;

- (void)stopDrawing;

- (IBAction)resetView:(id)sender;
- (IBAction)rightView:(id)sender;
- (IBAction)clearDetection:(id)sender;

@end
