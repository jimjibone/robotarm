//
//  JRObjectDetectionWrapper.m
//  Kinect Object Detector
//
//  Created by James Reuss on 14/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRObjectDetectionWrapper.h"
#import "JRObjectDetection.h"

@interface JRObjectDetectionWrapper ()
@property (nonatomic, readwrite, assign) JRObjectDetectionWrapperOpaque *cpp;
@end

@implementation JRObjectDetectionWrapper
@synthesize cpp = _cpp;

struct JRObjectDetectionWrapperOpaque {
public:
	JRObjectDetectionWrapperOpaque() {};
	ObjectDetection::ObjectDetection wrapper;
};




#pragma mark -
#pragma mark Objective-C Methods

- (id)init
{
	self = [super init];
	if (self) {
		self.cpp = new JRObjectDetectionWrapperOpaque();
	}
	return self;
}

- (void)dealloc
{
	delete _cpp;
	_cpp = NULL;
	
    [super dealloc];
}




#pragma mark -
#pragma mark Each 'New Frame' Functions
// Execute in order shown.

- (void)setZPoints:(uint16_t*)zPoints size:(size_t)size
{
	self.cpp->wrapper.setZPoints(zPoints, size);
}
- (void)calculateSurfaceNormals
{
	self.cpp->wrapper.calculateSurfaceNormals();
}
- (void)segmentPlanes
{
	self.cpp->wrapper.segmentPlanes();
}




#pragma mark -
#pragma mark Get Data Methods

- (size_t)getNumberOfPlaneClusters
{
	return self.cpp->wrapper.plane_clusters.size();
}
- (size_t)getNumberOfIndicesInPlaneCluster:(size_t)cluster
{
	return self.cpp->wrapper.plane_clusters[cluster].indices.size();
}
- (void)getX:(double*)x Y:(double*)y Z:(double*)z forPoint:(size_t)point forPlaneCluster:(size_t)cluster
{
	size_t index = self.cpp->wrapper.plane_clusters[cluster].indices[point];
	*x = self.cpp->wrapper.input_cloud[index].x;
	*y = self.cpp->wrapper.input_cloud[index].y;
	*z = self.cpp->wrapper.input_cloud[index].z;
}
- (void)getA:(double*)a B:(double*)b C:(double*)c D:(double*)d forPlaneCluster:(size_t)cluster
{
	size_t midPoint = (size_t)(self.cpp->wrapper.plane_clusters[cluster].indices.size() / 2);
	size_t index = self.cpp->wrapper.plane_clusters[cluster].indices[midPoint];
	*a = self.cpp->wrapper.input_cloud_normals[index].a;
	*b = self.cpp->wrapper.input_cloud_normals[index].b;
	*c = self.cpp->wrapper.input_cloud_normals[index].c;
	*d = self.cpp->wrapper.input_cloud_normals[index].d;
}




@end
