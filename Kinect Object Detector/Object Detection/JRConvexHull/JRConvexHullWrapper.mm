//
//  JRConvexHullWrapper.m
//  Convex Hull ObjC CPP
//
//  Created by James Reuss on 01/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRConvexHullWrapper.h"
#import "JRConvexHull.h"

@interface JRConvexHullWrapper ()
@property (nonatomic, readwrite, assign) JRConvexHullWrapperOpaque *cpp;
@end

@implementation JRConvexHullWrapper
@synthesize cpp = _cpp;

struct JRConvexHullWrapperOpaque {
public:
	JRConvexHullWrapperOpaque() : wrapper() {};
	ConvexHull::ConvexHull wrapper;
};

#pragma mark - Objective-C Methods
- (id)init {
    self = [super init];
    if (self) {
        self.cpp = new JRConvexHullWrapperOpaque();
    }
    return self;
}

- (void)dealloc {
    delete _cpp;
	_cpp = NULL;
	
    [super dealloc];
}

- (unsigned int)convexHullPointsCount {
	return (unsigned int)self.cpp->wrapper.convexHullPoints.size();
}
- (void)getConvexHullPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ {
	*outX = self.cpp->wrapper.convexHullPoints[i].x;
	*outY = self.cpp->wrapper.convexHullPoints[i].y;
	*outZ = self.cpp->wrapper.convexHullPoints[i].z;
}

#pragma mark - Main C++ Wrapped Functions
- (void)setPlaneA:(double)newA B:(double)newB C:(double)newC D:(double)newD Tolerance:(double)newTol {
	self.cpp->wrapper.setPlane(newA, newB, newC, newD, newTol);
}
- (bool)addPointX:(double)newX Y:(double)newY Z:(double)newZ {
	// Returns bool indicating wasAdded. Relating to tolerance.
	return self.cpp->wrapper.addPoint(newX, newY, newZ);
}
- (void)listPlanePoints {
	self.cpp->wrapper.listPoints();
}
- (void)performConvexHull {
	self.cpp->wrapper.performConvexHull();
}
- (void)listConvexHullPoints {
	self.cpp->wrapper.listConvexHullPoints();
}
- (void)resetConvexHull {
	self.cpp->wrapper.resetConvexHull();
}

#pragma mark - Point-Within-Bounds Functions
- (void) addPreprocessedConvexHullPoint:(PointXYZIJ)aPoint
{
	self.cpp->wrapper.addPreprocessedConvexHullPoint(aPoint);
}
- (bool) processPointWithPreprocessedHullX:(double)x Y:(double)y Z:(double)z
{
	return self.cpp->wrapper.processPointWithPreprocessedHull(x, y, z);
}

@end
