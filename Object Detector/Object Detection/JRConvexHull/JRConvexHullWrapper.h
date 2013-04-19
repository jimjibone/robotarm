//
//  JRConvexHullWrapper.h
//  Convex Hull ObjC CPP
//
//  Created by James Reuss on 01/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "../JRPointTypes.h"

struct JRConvexHullWrapperOpaque;

@interface JRConvexHullWrapper : NSObject {
	struct JRConvexHullWrapperOpaque *_cpp;
}

- (unsigned int)convexHullPointsCount;
- (void)getConvexHullPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ;
- (void)getConvexHullPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ I:(double*)outI J:(double*)outJ;

- (void)setPlaneA:(double)newA B:(double)newB C:(double)newC D:(double)newD Tolerance:(double)newTol Invert:(bool)newInvert;
- (bool)addPointX:(double)newX Y:(double)newY Z:(double)newZ;
- (void)listPlanePoints;
- (void)performConvexHull;
- (void)listConvexHullPoints;
- (void)resetConvexHull;

// These next functions allow the processing a pre-processed convex hull and
// individual points for the use of point-within-bounds calculations.
- (void) addPreprocessedConvexHullPoint:(PointXYZIJ)aPoint;
- (bool) processPointWithPreprocessedHullX:(double)x Y:(double)y Z:(double)z;	// true = outside, false = inside.

@end
