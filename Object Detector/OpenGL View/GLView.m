//
//  GLView.m
//  KinectiCopter Mark 2
//
//  Created by James Reuss on 03/06/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "GLView.h"

@interface GLView ()
- (void)initScene;
- (void)closeScene;
- (void)drawScene;
- (void)frameForTime:(const CVTimeStamp*)outputTime;
@end

@implementation GLView
// This is the renderer output callback function
static CVReturn displayLinkCallback(CVDisplayLinkRef displayLink, const CVTimeStamp* now, const CVTimeStamp* outputTime, CVOptionFlags flagsIn, CVOptionFlags* flagsOut, void* displayLinkContext) {
    NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
    [(GLView*)displayLinkContext frameForTime:outputTime];
    [pool release];
    return kCVReturnSuccess;
}
- (id)initWithFrame:(NSRect)frame {
	GLuint attribs[] =
    {
        NSOpenGLPFAAccelerated,
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFANoRecovery,
        NSOpenGLPFAColorSize, 32,
        NSOpenGLPFADepthSize, 24,
		NSOpenGLPFAAlphaSize, 32,
        0
    };
	NSOpenGLPixelFormat* fmt = [[NSOpenGLPixelFormat alloc] initWithAttributes: (NSOpenGLPixelFormatAttribute*) attribs];
	if((self = [super initWithFrame:frame pixelFormat: [fmt autorelease]])) {
    }
    return self;
}
- (void)stopDrawing {
	CVDisplayLinkRelease(_displayLink);
    [self closeScene];
}
- (void)dealloc {
    [self stopDrawing];
    [super dealloc];
}
- (void)prepareOpenGL {
    // Synchronize buffer swaps with vertical refresh rate
    GLint swapInt = 1;
    [[self openGLContext] setValues:&swapInt forParameter:NSOpenGLCPSwapInterval];
    
    [self initScene];
    
    CVDisplayLinkCreateWithActiveCGDisplays(&_displayLink);
    CVDisplayLinkSetOutputCallback(_displayLink, &displayLinkCallback, self);
    
    // Set the display link for the current renderer
    CGLContextObj cglContext = [[self openGLContext] CGLContextObj];
    CGLPixelFormatObj cglPixelFormat = [[self pixelFormat] CGLPixelFormatObj];
    CVDisplayLinkSetCurrentCGDisplayFromOpenGLContext(_displayLink, cglContext, cglPixelFormat);
    
    CVDisplayLinkStart(_displayLink);
}
- (void)update {
    NSOpenGLContext *context = [self openGLContext];
    CGLLockContext([context CGLContextObj]);
    [super update];
    CGLUnlockContext([context CGLContextObj]);
}
- (void)reshape {
    NSOpenGLContext *context = [self openGLContext];
    CGLLockContext([context CGLContextObj]);
    NSView *view = [context view];
    if(view) {
        NSSize size = [self bounds].size;
        [context makeCurrentContext];
        glViewport(0, 0, size.width, size.height);
    }
    CGLUnlockContext([context CGLContextObj]);
}
- (void)frameForTime:(const CVTimeStamp*)outputTime {
    [self drawRect:NSZeroRect];
}
- (void)drawRect:(NSRect)dirtyRect {
    NSOpenGLContext *context = [self openGLContext];
    CGLLockContext([context CGLContextObj]);
    NSView *view = [context view];
    if(view) {
        [context makeCurrentContext];
        
        [self drawScene];
        
        GLenum err = glGetError();
        if(err != GL_NO_ERROR) NSLog(@"GLError %4x", err);
        
        [context flushBuffer];
    }
    CGLUnlockContext([context CGLContextObj]);
}


#pragma mark custom drawing
- (void)initScene {
	// Set up draw modes
	_drawFrustrum = YES;
	_drawSegmentedPlanes = NO;
	_drawDominantPlane = NO;
	_drawDominantPlanePoints = NO;
	_drawObjectsPoints = NO;
	_normals = NO;
    _mirror = NO;
    _natural = YES;
    _drawMode = MODE_MESH;
	
	//_planeData = (_ransacConfidentPlane){0, 0, 0, 0, 0};
    
	_intDepth = (uint16_t*)malloc(FREENECT_FRAME_PIX * sizeof(uint16_t));
	_intRGB = (uint8_t*)malloc(FREENECT_FRAME_PIX * 3 * sizeof(uint8_t));
	
    //uint8_t *empty = (uint8_t*)malloc(FREENECT_FRAME_PIX * 3);
    //bzero(empty, FREENECT_FRAME_PIX * 3);
	
	bzero(_intDepth, FREENECT_FRAME_PIX);
	bzero(_intRGB, FREENECT_FRAME_PIX * 3);
    
	glGenTextures(1, &_depthTex);
	glBindTexture(GL_TEXTURE_2D, _depthTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE16, FREENECT_FRAME_W, FREENECT_FRAME_H, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, empty);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE16, FREENECT_FRAME_W, FREENECT_FRAME_H, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, _intDepth);
    
	glGenTextures(1, &_videoTex);
	glBindTexture(GL_TEXTURE_2D, _videoTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, FREENECT_FRAME_W, FREENECT_FRAME_H, 0, GL_RGB, GL_UNSIGNED_BYTE, empty);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, FREENECT_FRAME_W, FREENECT_FRAME_H, 0, GL_RGB, GL_UNSIGNED_BYTE, _intRGB);
    
    //free(empty);
	
	uint8_t map[2048*3];
    for(int i = 0; i < 2048; i++) {
        float v = i/2048.0;
		v = powf(v, 3)* 6;
        uint16_t gamma = v*6*256;
        
        int lb = gamma & 0xff;
		switch (gamma>>8) {
			case 0: // white -> red
                map[i*3+0] = 255;
				map[i*3+1] = 255-lb;
				map[i*3+2] = 255-lb;
				break;
			case 1: // red -> orange
				map[i*3+0] = 255;
				map[i*3+1] = lb;
				map[i*3+2] = 0;
				break;
			case 2: // orange -> green
				map[i*3+0] = 255-lb;
				map[i*3+1] = 255;
				map[i*3+2] = 0;
				break;
			case 3: // green -> cyan
				map[i*3+0] = 0;
				map[i*3+1] = 255;
				map[i*3+2] = lb;
				break;
			case 4: // cyan -> blue
				map[i*3+0] = 0;
				map[i*3+1] = 255-lb;
				map[i*3+2] = 255;
				break;
			case 5: // blue -> black
				map[i*3+0] = 0;
				map[i*3+1] = 0;
				map[i*3+2] = 255-lb;
				break;
			default: // black
				map[i*3+0] = 0;
				map[i*3+1] = 0;
				map[i*3+2] = 0;
				break;
		}
	}
    glGenTextures(1, &_colormapTex);
    glBindTexture(GL_TEXTURE_1D, _colormapTex);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, 2048, 0, GL_RGB, GL_UNSIGNED_BYTE, map);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	
    
    _depthProgram = [[GLProgram alloc] initWithName:
                     @"depth"
                                                 VS:
                     "void main() {\n"
                     "	gl_TexCoord[0] = gl_MultiTexCoord0;\n"
                     "	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
                     "}\n"
                                                 FS:
                     "uniform sampler1D colormap;\n"
                     "uniform sampler2D depth;\n"
                     "uniform sampler2D video;\n"
                     "uniform int normals;\n"
                     "uniform int natural;\n"
                     ""
                     "const float kMinDistance = 0.0;\n"
                     "const float kDepthScale  = 0.00174;\n"
                     ""
                     "void main() {\n"
                     "	float z  = texture2D(depth, gl_TexCoord[0].st).r*10.0;\n"
                     "   vec4 rgba;\n"
                     "   if(natural > 0) {\n"
                     ""
                     "      vec2 pos = gl_TexCoord[0].st*2.0-vec2(1.0);\n" // -1..+1
                     "      float d = z*2048.0;\n" // 0..2048.0
                     ""
                     "      float zd = (d > 1.0 && d < 10000.0) ? d : 100000.0;\n"
                     ""
                     "      float zs = (zd+kMinDistance)*kDepthScale;\n"
                     "      vec4 world = vec4(pos.x*320.0*zs, pos.y*240.0* zs, 200.0-zd, 1.0);\n"
                     ""
                     "      float cs = 1.0/((zd+kMinDistance)*kDepthScale);\n"
                     "	   vec2 st = vec2( (world.x*cs)/640.0 + 0.5,   (world.y*cs)/480.0 + 0.5);\n"
                     ""
                     "      rgba = texture2D(video, st);\n"
                     "   } else {\n"
                     "      rgba = texture1D(colormap, z);\n" // scale to 0..1 range
                     "   }\n"
                     ""
                     "   if(normals > 0) {\n"
                     "      float zx =  texture2D(depth, gl_TexCoord[0].st+vec2(2.0/640.0, 0.0)).r*10.0;\n"
                     "      float zy =  texture2D(depth, gl_TexCoord[0].st+vec2(0.0, 2.0/480.0)).r*10.0;\n"
                     "      vec3 n = vec3(zx-z, zy-z, -0.0005);\n"
                     "      n = normalize(n);\n"
                     "      rgba *= max(0.1, dot(vec3(0.0, -0.3, -0.95), n));\n"
                     "   }\n"
                     ""
                     "   gl_FragColor = rgba;\n"
                     "}\n"];
    [_depthProgram bind];
    [_depthProgram setUniformInt:0 forName:@"video"];
    [_depthProgram setUniformInt:1 forName:@"depth"];
    [_depthProgram setUniformInt:2 forName:@"colormap"];
    [_depthProgram unbind];
    
    // create grid of points
    struct glf2 {
        GLfloat x,y;
    } *verts = (struct glf2*)malloc(FREENECT_FRAME_W*FREENECT_FRAME_H*sizeof(struct glf2));
    for(int x = 0; x < FREENECT_FRAME_W; x++) {
        for(int y = 0; y < FREENECT_FRAME_H; y++) {
            struct glf2 *v = verts+x+y*FREENECT_FRAME_W;
            v->x = (x+0.5)/(FREENECT_FRAME_W*0.5) - 1.0;
            v->y = (y+0.5)/(FREENECT_FRAME_H*0.5) - 1.0;
        }
    }
    glGenBuffers(1, &_pointBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, _pointBuffer);
    glBufferData(GL_ARRAY_BUFFER, FREENECT_FRAME_W*FREENECT_FRAME_H*sizeof(struct glf2), verts, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    free(verts);
    
    _pointProgram = [[GLProgram alloc] initWithName:
                     @"point"
                                                 VS:
                     "uniform sampler2D depth;\n"
                     ""
                     "const float kMinDistance = 0.0;\n"
                     "const float kDepthScale  = 0.00174;\n"
                     ""
                     "void main() {\n"
                     "   vec3 pos = gl_Vertex.xyz;\n"
                     "   vec2 xy = (vec2(1.0)+pos.xy)*0.5;\n" // 0..1
                     "   float d = texture2D(depth, xy).r*32.0*2048.0;\n" // 0..2048.0
                     ""
                     "   float z = (d > 1.0 && d < 10000.0) ? d : 100000.0;\n"
                     ""
                     "   float zs = (z+kMinDistance)*kDepthScale;\n"
                     //"   vec4 world = vec4(pos.x*320.0*zs, pos.y*240.0* zs, 200.0-z, 1.0);\n"
#ifndef REMOVE_VIEW_DEPTH
					 "   vec4 world = vec4(pos.x*320.0*zs, pos.y*240.0* zs, -z, 1.0);\n"
#else
					 "   vec4 world = vec4(pos.x*320.0, pos.y*240.0, -1000, 1.0);\n"
#endif
                     ""
                     "   float cs = 1.0/((z+kMinDistance)*kDepthScale);\n"
                     "	gl_TexCoord[1].st = vec2( (world.x*cs)/640.0 + 0.5,   (world.y*cs)/480.0 + 0.5);\n"
                     "	gl_TexCoord[0].st = xy;\n"
                     ""
                     "	gl_Position = gl_ModelViewProjectionMatrix * world;\n"
                     "}\n"
                                                 FS:
                     "uniform sampler1D colormap;\n"
                     "uniform sampler2D depth;\n"
                     "uniform sampler2D video;\n"
                     "uniform int normals;\n"
                     "uniform int natural;\n"
                     ""
                     "void main() {\n"
                     "	float z  = texture2D(depth, gl_TexCoord[0].st).r*10.0;\n" // 0..1
                     "   vec4 rgba = (natural > 0) ? texture2D(video, gl_TexCoord[1].st) : texture1D(colormap, z);\n"
                     ""
                     "   if(normals > 0) {\n"
                     "      float zx =  texture2D(depth, gl_TexCoord[0].st+vec2(2.0/640.0, 0.0)).r*10.0;\n"
                     "      float zy =  texture2D(depth, gl_TexCoord[0].st+vec2(0.0, 2.0/480.0)).r*10.0;\n"
                     "      vec3 n = vec3(zx-z, zy-z, -0.0005);\n"
                     "      n = normalize(n);\n"
                     "      rgba *= max(0.1, dot(vec3(0.0, -0.3, -0.95), n));\n"
                     "   }\n"
                     ""
                     "   gl_FragColor = rgba;\n"
                     "}\n"
                     ];
    [_pointProgram bind];
    [_pointProgram setUniformInt:0 forName:@"video"];
    [_pointProgram setUniformInt:1 forName:@"depth"];
    [_pointProgram setUniformInt:2 forName:@"colormap"];
    [_pointProgram unbind];
    
    // create indicies for mesh
    _indicies = (GLuint*)malloc(FREENECT_FRAME_W*FREENECT_FRAME_H*6*sizeof(GLuint));
    _nTriIndicies = 0;
	
	[self resetView:nil];
    
    // set up texture units 0,1,2 permantely and only bind the textures once
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_1D, _colormapTex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _depthTex);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _videoTex);
	
	//glEnable(GL_BLEND); //Enable alpha blending
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set the blend function
	
	//_rotation = [[JRPointRotationXYZ alloc] initWithRotationX:0 Y:0 Z:0];
}
- (void)closeScene {
    glDeleteTextures(1, &_depthTex);
    glDeleteTextures(1, &_videoTex);
    glDeleteTextures(1, &_colormapTex);
    glDeleteBuffers(1, &_pointBuffer);
    
    free(_indicies);
	free(_intDepth);
	free(_intRGB);
    
    [_depthProgram release];
    [_pointProgram release];
	
	//[_rotation release];
}
- (void)drawFrustrum {
    struct glf3 {
        GLfloat x,y,z;
    } verts[] = {
        {0,    0,  30},
        {640,  0,  30},
        {640,480,  30},
        {0,  480,  30},
        {0,    0,2048},
        {640,  0,2048},
        {640,480,2048},
        {0,  480,2048}
    };
    for(int i = 0; i < sizeof(verts)/sizeof(verts[0]); i++) {
        struct glf3 *v = verts+i;
		const float KinectMinDistance = REGISTERED_MIN_DEPTH;
        const float KinectDepthScaleFactor = REGISTERED_SCALE_FACTOR;
        v->x = (v->x - FREENECT_FRAME_W/2) * (v->z + KinectMinDistance) * KinectDepthScaleFactor ;
        v->y = (v->y - FREENECT_FRAME_H/2) * (v->z + KinectMinDistance) * KinectDepthScaleFactor ;
        v->z = 200 - v->z;
    }
    GLubyte inds[] = {0,1, 1,2 , 2,3, 3,0,   4,5, 5,6, 6,7, 7,4, 0,4,   1,5, 2,6, 3, 7}; // front, back, side
    
    glColor4f(1, 1, 1, 0.5);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(verts[0]), &verts->x);
    glDrawElements(GL_LINES, sizeof(inds)/sizeof(inds[0]), GL_UNSIGNED_BYTE, inds);
    glDisableClientState(GL_VERTEX_ARRAY);
}
- (void)drawPlane {
	/*XYZPoint bl = (XYZPoint){-600, 600,
	 zWorldFromPlaneAndWorldXY(_planeData, -600, -600)};
	 XYZPoint tl = (XYZPoint){600, 600,
	 zWorldFromPlaneAndWorldXY(_planeData, 600, -600)};
	 XYZPoint tr = (XYZPoint){600, -600,
	 zWorldFromPlaneAndWorldXY(_planeData, 600, 600)};
	 XYZPoint br = (XYZPoint){-600, -600,
	 zWorldFromPlaneAndWorldXY(_planeData, -600, 600)};
	 
	 glBegin(GL_QUADS);
	 glColor4f(.392156863, 1, .392156863, 0.3);
	 // bl, tl, tr, br
	 glVertex3d(bl.x, bl.y, -bl.z);
	 glVertex3d(tl.x, tl.y, -tl.z);
	 glVertex3d(tr.x, tr.y, -tr.z);
	 glVertex3d(br.x, br.y, -br.z);
	 glEnd();*/
}
- (void)drawConvexHull {
	/*GLfloat prevPointSize = 0;
	 glGetFloatv(GL_POINT_SIZE, &prevPointSize);
	 glPointSize(8);
	 
	 glBegin(GL_POINTS);
	 glColor4f(1, 0, 0, 0.8);
	 for (int i = 0; i < _convexHullPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_convexHullPoints[i].x, -_convexHullPoints[i].y, -_convexHullPoints[i].z+2);
	 }
	 glEnd();
	 
	 glPointSize(prevPointSize);
	 
	 GLfloat prevLineWidth = 0;
	 glGetFloatv(GL_LINE_WIDTH, &prevLineWidth);
	 glLineWidth(2);
	 
	 glBegin(GL_LINE_LOOP);
	 glColor4f(1, 1, 0, 0.6);
	 for (int i = 0; i < _convexHullPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_convexHullPoints[i].x, -_convexHullPoints[i].y, -_convexHullPoints[i].z+2);
	 }
	 glEnd();
	 
	 glLineWidth(prevLineWidth);*/
}
- (void)drawBoundedPlaneHull {
	// Draw points of the hull
	/*GLfloat prevPointSize = 0;
	 glGetFloatv(GL_POINT_SIZE, &prevPointSize);
	 glPointSize(8);
	 
	 glBegin(GL_POINTS);
	 glColor4f(1, 0, 0, 0.8);
	 for (int i = 0; i < _convexHullPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_convexHullPoints[i].x, -_convexHullPoints[i].y, -_convexHullPoints[i].z+2);
	 }
	 glEnd();
	 
	 glPointSize(prevPointSize);
	 
	 // Draw the hull lines and fill
	 GLfloat prevLineWidth = 0;
	 glGetFloatv(GL_LINE_WIDTH, &prevLineWidth);
	 glLineWidth(2);
	 
	 glColor4f(.392156863, 1, .392156863, 0.3);
	 glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	 glBegin(GL_TRIANGLE_FAN);
	 for (int i = 0; i < _convexHullPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_convexHullPoints[i].x, -_convexHullPoints[i].y, -_convexHullPoints[i].z+2);
	 }
	 glEnd();
	 
	 glColor4f(1, 1, 0, 0.6);
	 glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	 glBegin(GL_LINE_LOOP);
	 for (int i = 0; i < _convexHullPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_convexHullPoints[i].x, -_convexHullPoints[i].y, -_convexHullPoints[i].z+2);
	 }
	 glEnd();
	 
	 glLineWidth(prevLineWidth);*/
}
- (void)drawObjectsCloud {
	/*GLfloat prevPointSize = 0;
	 glGetFloatv(GL_POINT_SIZE, &prevPointSize);
	 glPointSize(8);
	 
	 if (_objectsCloudPointCount > 0) {
	 glBegin(GL_POINTS);
	 glColor4f(0, 0, 1, 0.8);
	 for (int i = 0; i < _objectsCloudPointCount-1; i++) {
	 // Don't draw the last point because it is the same as the last.
	 glVertex3d(_objectsCloudPoints[i].x, -_objectsCloudPoints[i].y, -_objectsCloudPoints[i].z-2);
	 }
	 glEnd();
	 }
	 
	 glPointSize(prevPointSize);*/
}
- (void)drawSegmentedPlanes
{
	if (_segmented_planes_count > 0) {
		for (size_t i = 0; i < _segmented_planes_count; i++) {
			
			XYZPoint bl = (XYZPoint){-600, 600,
				zWorldFromPlaneAndWorldXY(_segmented_planes[i], -600, -600)};
			XYZPoint tl = (XYZPoint){600, 600,
				zWorldFromPlaneAndWorldXY(_segmented_planes[i], 600, -600)};
			XYZPoint tr = (XYZPoint){600, -600,
				zWorldFromPlaneAndWorldXY(_segmented_planes[i], 600, 600)};
			XYZPoint br = (XYZPoint){-600, -600,
				zWorldFromPlaneAndWorldXY(_segmented_planes[i], -600, 600)};
			
			float randColour = arc4random_uniform(255) / 255;
			
			glBegin(GL_QUADS);
			glColor4f(.392156863, randColour, .392156863, 0.3);
			// bl, tl, tr, br
			glVertex3d(bl.x, bl.y, -bl.z);
			glVertex3d(tl.x, tl.y, -tl.z);
			glVertex3d(tr.x, tr.y, -tr.z);
			glVertex3d(br.x, br.y, -br.z);
			glEnd();
			
		}
	}
}
- (void)drawSegmentedPlanesPoints
{
	if (_segmented_planes_count > 0) {
		
		uint coloursSize = 7;
		float colours[7][3] = {
			{1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 0.0, 1.0},
			{1.0, 1.0, 0.0},
			{0.0, 1.0, 1.0},
			{1.0, 0.0, 1.0},
			{1.0, 1.0, 1.0}
		};
		
		GLfloat prevPointSize = 0;
		glGetFloatv(GL_POINT_SIZE, &prevPointSize);
		glPointSize(2);
		
		glBegin(GL_POINTS);
		
		size_t currentPos = 0;
		for (size_t i = 0; i < _segmented_planes_count; i++) {
			
			glColor4f(colours[(i % coloursSize)][0], colours[(i % coloursSize)][1], colours[(i % coloursSize)][2], 0.6);
			
			for (size_t j = 0; j < _segmented_planes_points_counts[i]; j++) {
				
				// Don't draw the last point because it is the same as the last.
#ifndef REMOVE_VIEW_DEPTH
				//glVertex3d(_segmented_planes_points[currentPos].x, -_segmented_planes_points[currentPos].y, -_segmented_planes_points[currentPos].z+1);
				@try {
					glVertex3d(_segmented_planes_points[currentPos].x, -_segmented_planes_points[currentPos].y, -_segmented_planes_points[currentPos].z+1);
				}
				@catch (NSException *exception) {
					NSLog(@"GLView %@ Caught Exception : %@", NSStringFromSelector(_cmd), [exception description]);
				}
#else
				uint x, y = 0;
				frameFromWorld(&x, &y, _segmented_planes_points[currentPos].x, _segmented_planes_points[currentPos].y, _segmented_planes_points[currentPos].z);
				glVertex3d((double)320-x, (double)-y-480, (double)-REMOVE_VIEW_DEPTH_VALUE);
				//printf("drawSegmentedPlanesPoints x = %d  y = %d  z = %d.", x, y, REMOVE_VIEW_DEPTH_VALUE);
#endif
				currentPos++;
			}
			
		}
		
		glEnd();
		glPointSize(prevPointSize);
		
	}
}
- (void)drawDominantPlanePoints
{
	// PLANE
	
	/*XYZPoint bl = (XYZPoint){-600, 600,
	 zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, -600, -600)};
	 XYZPoint tl = (XYZPoint){600, 600,
	 zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, 600, -600)};
	 XYZPoint tr = (XYZPoint){600, -600,
	 zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, 600, 600)};
	 XYZPoint br = (XYZPoint){-600, -600,
	 zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, -600, 600)};
	 
	 glBegin(GL_QUADS);
	 glColor4f(.392156863, 1, .392156863, 0.3);
	 // bl, tl, tr, br
	 glVertex3d(bl.x, bl.y, -bl.z);
	 glVertex3d(tl.x, tl.y, -tl.z);
	 glVertex3d(tr.x, tr.y, -tr.z);
	 glVertex3d(br.x, br.y, -br.z);
	 glEnd();*/
	
	// LOOP
	
	GLfloat prevPointSize = 0;
	glGetFloatv(GL_POINT_SIZE, &prevPointSize);
	glPointSize(8);
	
	glBegin(GL_POINTS);
	glColor4f(1, 0, 0, 0.8);
	for (int i = 0; i < _dominant_plane_hull_point_count-1; i++) {
		// Don't draw the last point because it is the same as the last.
		glVertex3d(_dominant_plane_hull_points[i].x, -_dominant_plane_hull_points[i].y, -_dominant_plane_hull_points[i].z+2);
	}
	glEnd();
	
	glPointSize(prevPointSize);
}
- (void)drawDominantPlane
{
	// PLANE
	
	/*XYZPoint bl = (XYZPoint){-600, 600,
		zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, -600, -600)};
	XYZPoint tl = (XYZPoint){600, 600,
		zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, 600, -600)};
	XYZPoint tr = (XYZPoint){600, -600,
		zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, 600, 600)};
	XYZPoint br = (XYZPoint){-600, -600,
		zWorldFromPlaneAndWorldXY(_dominant_plane_hull_coefficients, -600, 600)};
	
	glBegin(GL_QUADS);
	glColor4f(.392156863, 1, .392156863, 0.3);
	// bl, tl, tr, br
	glVertex3d(bl.x, bl.y, -bl.z);
	glVertex3d(tl.x, tl.y, -tl.z);
	glVertex3d(tr.x, tr.y, -tr.z);
	glVertex3d(br.x, br.y, -br.z);
	glEnd();*/
	
	// LOOP
	
	GLfloat prevLineWidth = 0;
	glGetFloatv(GL_LINE_WIDTH, &prevLineWidth);
	glLineWidth(2);
	
	glBegin(GL_LINE_LOOP);
	glColor4f(1, 1, 0, 0.6);
	for (int i = 0; i < _dominant_plane_hull_point_count-1; i++) {
		// Don't draw the last point because it is the same as the last.
		glVertex3d(_dominant_plane_hull_points[i].x, -_dominant_plane_hull_points[i].y, -_dominant_plane_hull_points[i].z+2);
	}
	glEnd();
	
	glLineWidth(prevLineWidth);
}
- (void)drawObjectsPoints
{
	if (_objects_count > 0) {
		
		uint coloursSize = 7;
		float colours[7][3] = {
			{0.6, 0.0, 0.0},
			{0.0, 0.6, 0.0},
			{0.0, 0.0, 0.6},
			{0.6, 0.6, 0.0},
			{0.0, 0.6, 0.6},
			{0.6, 0.0, 0.6},
			{0.6, 0.6, 0.6}
		};
		
		GLfloat prevPointSize = 0;
		glGetFloatv(GL_POINT_SIZE, &prevPointSize);
		glPointSize(2);
		
		glBegin(GL_POINTS);
		
		size_t currentPos = 0;
		for (size_t o = 0; o < _objects_count; o++) {
			
			glColor4f(colours[(o % coloursSize)][0], colours[(o % coloursSize)][1], colours[(o % coloursSize)][2], 0.6);
			
			for (size_t p = 0; p < _objects_points_counts[o]; p++) {
				
				glVertex3d(_objects_points[currentPos].x, -_objects_points[currentPos].y, -_objects_points[currentPos].z+1);
				currentPos++;
			}
			
		}
		
		glEnd();
		glPointSize(prevPointSize);
		
	}
}
- (void)drawObjectsRadii
{
	if (_objects_count > 0) {
		
		// Draw circle showing object radius.
		GLfloat prevLineWidth = 0;
		glGetFloatv(GL_LINE_WIDTH, &prevLineWidth);
		glLineWidth(2);
		
		
		glColor4f(1, 1, 0, 0.6);
		for (int i = 0; i < _objects_count; i++) {
			
			glBegin(GL_LINE_LOOP);
			for (int angle = 0; angle < 360; angle += 5) {
				glVertex3d(_objects_centroids[i].x + sin(angle*3.14159/180) * _objects_radii[i],
						   -_objects_centroids[i].y - cos(angle*3.14159/180) * _objects_radii[i],
						   -_objects_centroids[i].z);
			}
			glEnd();
			
		}
		
		
		glLineWidth(prevLineWidth);
		
	}
}
- (void)drawGLStringX:(GLfloat)x Y:(GLfloat)y Z:(GLfloat)z String:(NSString*)string {
	//glColor3f(255, 255, 255);
	glRasterPos3f(x, y, -z);
	for (NSUInteger i = 0; i < [string length]; i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, [string characterAtIndex:i]);
	}
}
- (void)drawScene {
    NSSize size = [self bounds].size;
	
    if(_newDepth) {
        
        if(_drawMode == MODE_MESH) {
            // naive - most common GPUs will start too choke when pushing 640x480 different
			//  triangles each frame.. (2009)
            const float md = 5; // tolerance in z, increase to join more mesh triangles
            _nTriIndicies = 0;
            for(int x = 0; x < FREENECT_FRAME_W-1; x++) {
                for(int y = 0; y < FREENECT_FRAME_H-1; y++) {
                    int idx = x+y*FREENECT_FRAME_W;
                    
                    int d = _intDepth[idx];
                    if(d > 1 && d < 10000) {
                        int d01 = _intDepth[idx+FREENECT_FRAME_W];
                        int d10 = _intDepth[idx+1];
                        int d11 = _intDepth[idx+FREENECT_FRAME_W+1];
                        float z = d*0.1;
                        float z01 = (d01 > 1.0 && d01 < 10000.0) ? d01*0.1 : 100000.0;
                        float z10 = (d10 > 1.0 && d10 < 10000.0) ? d10*0.1 : 100000.0;
                        float z11 = (d11 > 1.0 && d11 < 10000.0) ? d11*0.1 : 100000.0;
						
                        if(fabsf(z01-z) < md && fabsf(z10-z) < md && fabsf(z11-z) < md) {
                            _indicies[_nTriIndicies++] = idx;
                            _indicies[_nTriIndicies++] = idx+1;
                            _indicies[_nTriIndicies++] = idx+1+FREENECT_FRAME_W;
                            _indicies[_nTriIndicies++] = idx;
                            _indicies[_nTriIndicies++] = idx+1+FREENECT_FRAME_W;
                            _indicies[_nTriIndicies++] = idx+FREENECT_FRAME_W;
                        }
                    }
                }
            }
        }
        
        glActiveTexture(GL_TEXTURE1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, FREENECT_FRAME_W, FREENECT_FRAME_H, GL_LUMINANCE, GL_UNSIGNED_SHORT, _intDepth);
        glActiveTexture(GL_TEXTURE0);
		_newDepth = NO;
    }
	
    if(_newRGB) {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, FREENECT_FRAME_W, FREENECT_FRAME_H, GL_RGB, GL_UNSIGNED_BYTE, _intRGB);
		_newRGB = NO;
    }
    
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    if(_drawMode == MODE_POINTS || _drawMode == MODE_MESH) {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
		gluPerspective(40, size.width/size.height, 0.05, 1000);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
		float s = 0.02;
		
		glTranslatef(0.0, 0.0, -1000.0*s);
        glTranslatef(_offset[0], _offset[1], -_offset[2]);
		glRotatef(_angle, 0, 1, 0);
        glRotatef(_tilt, -1, 0, 0);
		glRotatef(_roll, 0, 0, -1);
		
		//PointXYZ rot = [_rotation rotation];
		//glRotatef(rot.y, 0, 1, 0);
        //glRotatef(rot.x, -1, 0, 0);
		//glRotatef(rot.z, 0, 0, -1);
		
		glTranslatef(0.0, 0.0, 1000.0*s);
        
        glScalef(_mirror?-s:s, -s, s); // flip y,  flipping the scene x is an incredibly stupid way to mirror
        
        glEnable(GL_DEPTH_TEST);
        
        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE2);
        glEnable(GL_TEXTURE_1D);
        glActiveTexture(GL_TEXTURE1);
        glEnable(GL_TEXTURE_2D);
        
        [_pointProgram bind];
        [_pointProgram setUniformInt:(_normals?1:0) forName:@"normals"];
        [_pointProgram setUniformInt:(_natural?1:0) forName:@"natural"];
		
		glPointSize(1);
        
        glEnableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, _pointBuffer);
        glVertexPointer(2, GL_FLOAT, 0, NULL);
        if(_drawMode == MODE_POINTS) {
            glDrawArrays(GL_POINTS, 0, FREENECT_FRAME_W*FREENECT_FRAME_H);
        } else {
            glDrawElements(GL_TRIANGLES, _nTriIndicies, GL_UNSIGNED_INT, _indicies);
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDisableClientState(GL_VERTEX_ARRAY);
        
        [_pointProgram unbind];
        
        glActiveTexture(GL_TEXTURE2);
        glDisable(GL_TEXTURE_1D);
        glActiveTexture(GL_TEXTURE1);
        glDisable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        glDisable(GL_TEXTURE_2D);
		
		GLint prevDepthMask = 0, prevDepthFunc = 0;
		
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		//if (_drawPlane) [self drawPlane];
		
		glGetIntegerv(GL_DEPTH_WRITEMASK, &prevDepthMask);
		glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);
		
		glDepthMask(GL_FALSE);
		glDepthFunc(GL_ALWAYS);
		
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_POLYGON_SMOOTH);
		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
        
        if (_drawFrustrum)			[self drawFrustrum];
		//if (_drawSegmentedPlanes)	[self drawSegmentedPlanes];
		//if (_drawSegmentedPlanes)	[self drawSegmentedPlanesPoints];
		//if (_drawDominantPlanePoints)		[self drawDominantPlanePoints];
		if (_drawDominantPlane)		[self drawDominantPlane];
		//if (_drawObjectsPoints)		[self drawObjectsPoints];
		if (_drawObjectsPoints)		[self drawObjectsRadii];
		
		glDisable(GL_POINT_SMOOTH);
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POLYGON_SMOOTH);
		
		glDepthFunc(prevDepthFunc);
		glDepthMask(prevDepthMask);
		
		glDisable(GL_BLEND);
        
    } else {
        // ortho
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0f, size.width, size.height, 0.0f, -1.0f, 1.0f); // y-flip
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        glDisable(GL_DEPTH_TEST);
        
        glColor4f(1.0, 1.0, 1.0, 1.0);
        
        // draw rgb
        
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        /*glTexCoord2f(_mirror?1:0, 0); glVertex2f(0,               0);
        glTexCoord2f(_mirror?0:1, 0); glVertex2f(FREENECT_FRAME_W,0);
        glTexCoord2f(_mirror?0:1, 1); glVertex2f(FREENECT_FRAME_W,FREENECT_FRAME_H);
        glTexCoord2f(_mirror?1:0, 1); glVertex2f(0,FREENECT_FRAME_H);*/
		glTexCoord2f(_mirror?1:0, 0); glVertex2f(0,               0);
        glTexCoord2f(_mirror?0:1, 0); glVertex2f(FREENECT_FRAME_W/2,0);
        glTexCoord2f(_mirror?0:1, 1); glVertex2f(FREENECT_FRAME_W/2,FREENECT_FRAME_H/2);
        glTexCoord2f(_mirror?1:0, 1); glVertex2f(0,FREENECT_FRAME_H/2);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        
        
        
        /*glTranslatef(FREENECT_FRAME_W, 0, 0);
        
        // draw depth
        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE2);
        glEnable(GL_TEXTURE_1D);
        glActiveTexture(GL_TEXTURE1);
        glEnable(GL_TEXTURE_2D);
        
        [_depthProgram bind];
        [_depthProgram setUniformInt:(_normals?1:0) forName:@"normals"];
        [_depthProgram setUniformInt:(_natural?1:0) forName:@"natural"];
		[_depthProgram setUniformFloat:0 forName:@"kColorScale"];
        [_depthProgram setUniformFloat:0 forName:@"kColorX"];
        [_depthProgram setUniformFloat:0 forName:@"kColorY"];
        
        glBegin(GL_QUADS);
        glTexCoord2f(_mirror?1:0, 0); glVertex2f(0,               0);
        glTexCoord2f(_mirror?0:1, 0); glVertex2f(FREENECT_FRAME_W,0);
        glTexCoord2f(_mirror?0:1, 1); glVertex2f(FREENECT_FRAME_W,FREENECT_FRAME_H);
        glTexCoord2f(_mirror?1:0, 1); glVertex2f(0,FREENECT_FRAME_H);
        glEnd();
        
        [_depthProgram unbind];*/
        
        glActiveTexture(GL_TEXTURE2);
        glDisable(GL_TEXTURE_1D);
        glActiveTexture(GL_TEXTURE1);
        glDisable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        glDisable(GL_TEXTURE_2D);
    }
    
    GLint e = glGetError();
    if(e != 0) NSLog(@"GLERROR: %04x", e);
}


#pragma mark Instance Methods
- (void)setDrawMode:(enum drawMode)newDrawMode
{
	@synchronized (self) {
		_drawMode = newDrawMode;
	}
}
- (void)swapInNewDepthFrame:(uint16_t**)newDepth RGBFrame:(uint8_t**)newRGB {
	if (newDepth) {
		@synchronized (self) {
			swapPtr16(newDepth, &_intDepth);
			_newDepth = YES;
		}
	}
	if (newRGB) {
		@synchronized (self) {
			swapPtr8(newRGB, &_intRGB);
			_newRGB = YES;
		}
	}
}
- (void)setPlaneData:(_ransacConfidentPlane)newPlane {
	//@synchronized (self) {
	//	_planeData = newPlane;
	//}
	//#define GLViewSetPlaneDEBUG
#ifdef GLViewSetPlaneDEBUG
	printf("GLView:\n");
	printf("memloc = %x\n", &_planeData);
	printf("plane: %.3fx + %.3fy + %.3fz + %.3f = 0 \n", _planeData.a,
		   _planeData.b, _planeData.c, _planeData.d);
	printf("isValid? %d \n", _planeData.isValid);
	
	printf("plane - point 1: wx=%f wy=%f wz=%f\n", _planeData.point1.x, _planeData.point1.y, _planeData.point1.z);
	printf("plane - point 2: wx=%f wy=%f wz=%f\n", _planeData.point2.x, _planeData.point2.y, _planeData.point2.z);
	printf("plane - point 3: wx=%f wy=%f wz=%f\n\n", _planeData.point3.x, _planeData.point3.y, _planeData.point3.z);
#endif
}
- (void)setObjectDetectionData:(JRObjectDetectionWrapper*)objectDetector
{
	@synchronized (self) {
		_drawSegmentedPlanes = NO;
		_drawDominantPlane = NO;
		_drawObjectsPoints = NO;
	}
	
	_segmented_planes_count = [objectDetector getNumberOfPlaneClusters];
	free(_segmented_planes);
	_segmented_planes = (PlaneCoefficients*)malloc(_segmented_planes_count * sizeof(PlaneCoefficients));
	
	//printf("GLView setting segmented_planes of count = %zd:\n", _segmented_planes_count);
	for (size_t i = 0; i < _segmented_planes_count; i++) {
		[objectDetector getA:&_segmented_planes[i].a B:&_segmented_planes[i].b C:&_segmented_planes[i].c D:&_segmented_planes[i].d forPlaneCluster:i];
		//printf("\tPlane %zd has coeffs   :   A = %.3f  B = %.3f  C = %.3f  D = %.3f\n", i, _segmented_planes[i].a, _segmented_planes[i].b, _segmented_planes[i].c, _segmented_planes[i].d);
	}
	
	free(_segmented_planes_points_counts);
	_segmented_planes_points_counts = (size_t*)malloc(_segmented_planes_count * sizeof(size_t));
	size_t total_planes_points_count = 0;
	for (size_t i = 0; i < _segmented_planes_count; i++) {
		_segmented_planes_points_counts[i] = [objectDetector getNumberOfIndicesInPlaneCluster:i];
		total_planes_points_count += [objectDetector getNumberOfIndicesInPlaneCluster:i];
	}
	
	free(_segmented_planes_points);
	_segmented_planes_points = (PointXYZ*)malloc(total_planes_points_count * sizeof(PointXYZ));
	size_t currentPlanePointPos = 0;
	for (size_t i = 0; i < _segmented_planes_count; i++) {
		
		for (size_t j = 0; j < _segmented_planes_points_counts[i]; j++) {
			
			[objectDetector getX:&_segmented_planes_points[currentPlanePointPos].x Y:&_segmented_planes_points[currentPlanePointPos].y Z:&_segmented_planes_points[currentPlanePointPos].z forPoint:j forPlaneCluster:i];
			currentPlanePointPos++;
		}
		
	}
	
	
	[objectDetector getDominantPlaneA:&_dominant_plane_hull_coefficients.a B:&_dominant_plane_hull_coefficients.b C:&_dominant_plane_hull_coefficients.c D:&_dominant_plane_hull_coefficients.d];
	_dominant_plane_hull_point_count = [objectDetector getDominantPlaneHullPointCount];
	free(_dominant_plane_hull_points);
	_dominant_plane_hull_points = (PointXYZ*)malloc(_dominant_plane_hull_point_count * sizeof(PointXYZ));
	for (size_t i = 0; i < _dominant_plane_hull_point_count; i++) {
		[objectDetector getDominantPlaneHullPointX:&_dominant_plane_hull_points[i].x Y:&_dominant_plane_hull_points[i].y Z:&_dominant_plane_hull_points[i].z forPoint:i];
	}

	
	_objects_count = [objectDetector getObjectClustersCount];
	free(_objects_points);
	free(_objects_points_counts);
	free(_objects_centroids);
	free(_objects_radii);
	_objects_points_counts = (size_t*)malloc(_objects_count * sizeof(size_t));
	_objects_centroids = (PointXYZ*)malloc(_objects_count * sizeof(PointXYZ));
	_objects_radii = (double*)malloc(_objects_count * sizeof(double));
	size_t total_objects_points_count = 0;
	for (size_t i = 0; i < _objects_count; i++) {
		_objects_points_counts[i] = [objectDetector getObjectPointCountForCluster:i];
		total_objects_points_count += [objectDetector getObjectPointCountForCluster:i];
		[objectDetector getObjectCentroidX:&_objects_centroids[i].x Y:&_objects_centroids[i].y Z:&_objects_centroids[i].z forCluster:i];
		_objects_radii[i] = [objectDetector getObjectRadiusForCluster:i];
	}
	
	_objects_points = (PointXYZ*)malloc(total_objects_points_count * sizeof(PointXYZ));
	size_t current_objects_points_pos = 0;
	for (size_t o = 0; o < _objects_count; o++) {
		for (size_t p = 0; p < _objects_points_counts[o]; p++) {
			[objectDetector getObjectX:&_objects_points[current_objects_points_pos].x Y:&_objects_points[current_objects_points_pos].y Z:&_objects_points[current_objects_points_pos].z forPoint:p inCluster:o];
			current_objects_points_pos++;
		}
	}
	
	
	_drawSegmentedPlanes = YES;
	_drawDominantPlane = YES;
	_drawObjectsPoints = YES;
}


#pragma mark event handling
- (BOOL)acceptsFirstResponder { return YES; }
- (void)mouseDown:(NSEvent*)event {
    _lastPos = [self convertPoint:[event locationInWindow] fromView:nil];
}
- (void)mouseDragged:(NSEvent*)event {
    if(_drawMode == MODE_2D) return;
    NSPoint pos = [self convertPoint:[event locationInWindow] fromView:nil];
    NSPoint delta = NSMakePoint((pos.x-_lastPos.x)/[self bounds].size.width, (pos.y-_lastPos.y)/[self bounds].size.height);
    _lastPos = pos;
    
    if([event modifierFlags] & NSShiftKeyMask) {
        //_offset[0] += 2*delta.x;
        //_offset[1] += 2*delta.y;
    } else {
        _angle += 50*delta.x;
        _tilt  += 50*delta.y;
		
		//[_rotation rotateByX:(50*delta.x) Y:(50*delta.y) Z:0];
		//NSLog(@"%@ newX:%f  newY:%f  newZ:%f", NSStringFromSelector(_cmd), [_rotation rotation].x, [_rotation rotation].y, [_rotation rotation].z);
    }
}
- (void)rightMouseDragged:(NSEvent *)theEvent {
	if(_drawMode == MODE_2D) return;
    NSPoint pos = [self convertPoint:[theEvent locationInWindow] fromView:nil];
    NSPoint delta = NSMakePoint((pos.x-_lastPos.x)/[self bounds].size.width, (pos.y-_lastPos  .y)/[self bounds].size.height);
    _lastPos = pos;
	
	_offset[0] += 2*delta.x;
	_offset[1] += 2*delta.y;
}
- (void)scrollWheel:(NSEvent *)event {
    if(_drawMode == MODE_2D) return;
	float d = ([event modifierFlags] & NSShiftKeyMask) ? [event deltaY]*10.0 : [event deltaY];
    _offset[2] -= d*0.1;
    if(_offset[2] < 0.5) _offset[2] = 0.5;
	
	//float r = ([event modifierFlags] & NSShiftKeyMask) ? [event deltaX] : 0;
	float r = [event deltaX]*0.5;
    _roll += r;
    if(_roll <= -180) _roll = 180;
	else if (_roll > 180) _roll = -180;
}
- (void)keyDown:(NSEvent *)event {
    if(_drawMode == MODE_2D) return;
    unichar key = [[event charactersIgnoringModifiers] characterAtIndex:0];
    switch(key) {
		case 'p':
            NSLog(@"offset x:%f y:%f z:%f", _offset[0], _offset[1], _offset[2]);
            NSLog(@"angle:%f",_angle);
            NSLog(@"tilt:%f", _tilt);
            break;
        case NSLeftArrowFunctionKey:  _offset[0]-=10; break;
        case NSRightArrowFunctionKey: _offset[0]+=10; break;
        case NSDownArrowFunctionKey:  _offset[1]-=10; break;
        case NSUpArrowFunctionKey:    _offset[1]+=10; break;
    }
}

- (IBAction)resetView:(id)sender {
	_offset[0] = 0;
	_offset[1] = 0;
	_offset[2] = 10;
	_angle = 0;
	_tilt = 0;
	_roll = 0;
}
- (IBAction)rightView:(id)sender {
	_offset[0] = 0;
	_offset[1] = 1;
	_offset[2] = 39;
	_angle = -90;
	_tilt = 0;
}
- (IBAction)clearDetection:(id)sender {
	@synchronized (self) {
		_drawSegmentedPlanes = NO;
		_drawDominantPlane = NO;
		_drawObjectsPoints = NO;
	}
}

@end
