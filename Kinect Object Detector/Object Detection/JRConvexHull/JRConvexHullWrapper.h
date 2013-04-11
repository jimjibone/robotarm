//
//  JRConvexHullWrapper.h
//  Convex Hull ObjC CPP
//
//  Created by James Reuss on 01/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>

struct JRConvexHullWrapperOpaque;

@interface JRConvexHullWrapper : NSObject {
	struct JRConvexHullWrapperOpaque *_cpp;
}

- (unsigned int)convexHullPointsCount;
- (void)getConvexHullPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ;

- (void)setPlaneA:(double)newA B:(double)newB C:(double)newC D:(double)newD Tolerance:(double)newTol;
- (bool)addPointX:(double)newX Y:(double)newY Z:(double)newZ;
- (void)listPlanePoints;
- (void)performConvexHull;
- (void)listConvexHullPoints;
- (void)resetConvexHull;

@end
