//
//  GLView.h
//  KinectiCopter Mark 2
//
//  Created by James Reuss on 03/06/2012.
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
#import "JRObjectHelpers.h"

typedef struct {
	double x, y, z;	// 3D points from the original->projectd-to-plane data.
} PointXYZ;

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
	_ransacConfidentPlane _planeData;
	PointXYZ *_convexHullPoints;	unsigned int _convexHullPointCount;
    
    // 3D navigation
    NSPoint _lastPos, _lastPosRight;
    float _offset[3];
    float _angle, _tilt, _roll;
	
	// Draw Modes
	BOOL _drawFrustrum;
	BOOL _drawPlane;
	BOOL _drawConvexHull;
	BOOL _normals;
    BOOL _mirror;
    BOOL _natural;
    enum drawMode {
        MODE_2D = 0,
        MODE_POINTS = 1,
        MODE_MESH = 2,
    } _drawMode;
}

- (void)swapInNewDepthFrame:(uint16_t**)newDepth RGBFrame:(uint8_t**)newRGB;
- (void)setPlaneData:(_ransacConfidentPlane)newPlane;
- (void)showPlane:(BOOL)mode;
- (void)resetConvexHullPoints;
- (void)setConvexHullPoints:(PointXYZ*)newPoints Count:(unsigned int)count;
- (void)showConvexHull:(BOOL)mode;

- (void)stopDrawing;

- (IBAction)resetView:(id)sender;
- (IBAction)rightView:(id)sender;

@end
