//
//  JRObjectDetector.m
//  Kinect Object Detector
//
//  Created by James Reuss on 10/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRObjectDetector.h"

#define MAIN_TIME_INTERVAL 0.1

@interface JRObjectDetector ()
//- (void)start;
- (void)addTimerForMainMethod;
//- (void)stop;

- (void)mainMethod;

// Data Management
- (BOOL)collectNewKinectFrames;
- (void)sendNewFramesToDisplay;

// RANSAC
- (void)initRANSAC;
- (void)freeRANSAC;
- (void)performRANSAC;
- (void)calculatePlane:(ABCDPlane*)plane Point1:(XYZPoint)a Point2:(XYZPoint)b Point3:(XYZPoint)c;

// Convex Hull
- (void)initConvexHull;
- (void)freeConvexHull;
- (void)performConvexHull;

// Helpers
//- (BOOL)isPoint:(uint8_t)point WithIndex:(unsigned int)index OnPlane:(ABCDPlane*)plane WithTolerance:(double)tolerance;
@end


@implementation JRObjectDetector

#pragma mark - Init/Dealloc Methods
- (id)init {
    self = [super init];
    if (self) {
		// Setup and start the kinect controller
        kinectController = [[JRKinectController alloc] initWithLEDColour:LED_GREEN];
		
		// Allocate memory for the kinect and display data
		_kinectDepth	= (uint16_t*)malloc(FREENECT_DEPTH_11BIT_SIZE);
		_kinectRGB		= (uint8_t*)malloc(FREENECT_VIDEO_RGB_SIZE);
        
        
		// Init RANSAC
		[self initRANSAC];
		[self initConvexHull];
		
		// Setup the rest of the program ready to start
		_runMainMethod = NO;
		_mainMethodCompleted = YES;
    }
    return self;
}
- (void)start {
	if (_runMainMethod == NO) {
		// Start the main method and to rest of the class
		_runMainMethod = YES;
		_mainMethodCompleted = YES;
		[self addTimerForMainMethod];
	}
}
- (void)addTimerForMainMethod {
	if (_runMainMethod) {
		[[NSRunLoop currentRunLoop] addTimer:[NSTimer timerWithTimeInterval:MAIN_TIME_INTERVAL
																  target:self
																selector:@selector(mainMethod)
																userInfo:nil
																 repeats:NO]
								  forMode:NSDefaultRunLoopMode];
		_isRunning = YES;
	} else {
		_isRunning = NO;
	}
}
- (void)stop {
	if (_runMainMethod == YES) {
		_runMainMethod = NO;
	}
	[kinectController stopKinectImmediately];
}
- (void)dealloc {
	// Stop the main method from running
	_runMainMethod = NO;
	
	// Stop and release kinect controller
    [kinectController stopKinectImmediately];
	[kinectController release];
	
	// Delete the kinect and display data
	free(_kinectDepth);
	free(_kinectRGB);
	_kinectDepth  = NULL;
	_kinectRGB    = NULL;
	
	// Dealloc RANSAC
	[self freeRANSAC];
	[self freeConvexHull];
	
    [super dealloc];
}




#pragma mark - Main Methods
- (void)mainMethod {
	
	if (_runMainMethod && _mainMethodCompleted) {
		_mainMethodCompleted = NO;
		
		if ([self collectNewKinectFrames]) {
			[self performRANSAC];
			[self performConvexHull];
			
			[self sendPlaneDataToView];
			[self sendNewFramesToDisplay];
		}
	}
	
	if (_runMainMethod) {
		[self addTimerForMainMethod];
	}
	
	_mainMethodCompleted = YES;
}




#pragma mark - Data Management
- (BOOL)collectNewKinectFrames {
	BOOL newData = NO;
	newData = [kinectController swapDepthData:&_kinectDepth];
	newData |= [kinectController swapRGBData:&_kinectRGB];
	return newData;
}
- (void)sendNewFramesToDisplay {
	[glView swapInNewDepthFrame:&_kinectDepth RGBFrame:&_kinectRGB];
}




#pragma mark - RANSAC
// http://en.wikipedia.org/wiki/RANSAC#The_algorithm
#define noConfidentPlanes 50
#define noPossPlanes 500
#define noPossInliers noPossPlanes*3	//must be a multiple of 3 for convenience.
#define noNonInliers (FREENECT_FRAME_PIX-noPossInliers)
#define ransacTolerance 10
#define confidenceIncrement 200			//higher number faster processing, lower confidence quality.
- (void)initRANSAC {
	_ransac.possibleInlierIndices = (unsigned int*)malloc(noPossInliers * sizeof(unsigned int));
	_ransac.nonInlierIndices = (unsigned int*)malloc(noNonInliers * sizeof(unsigned int));
	_ransac.possiblePlanes = (ABCDPlane*)malloc(noPossPlanes * sizeof(ABCDPlane));
	_ransac.noValidPlanes = 0;
	_ransac.confidentPlaneNo = 0;
	_ransac.confidentPlaneVal = 0;
	_ransac.confidentPlanes = (ABCDPlane*)malloc(noConfidentPlanes * sizeof(ABCDPlane));
	_ransac.confidentPlanesCount = 0;
	
	// Randomly select the possible inliers
	for (unsigned int i = 0; i < noPossInliers; i++) {
		//http://stackoverflow.com/questions/160890/generating-random-numbers-in-objective-c
		_ransac.possibleInlierIndices[i] = arc4random_uniform(FREENECT_FRAME_PIX);
	}
	
	// Find the indices of all the pixels that aren't possible inliers
	for (unsigned int i = 0; i < noNonInliers; i++) _ransac.nonInlierIndices[i] = 0;	//initialise non inliers.
	unsigned int nonInlierCount = 0;
	for (unsigned int i = 0; i < FREENECT_FRAME_PIX; i++) {
		BOOL isUsed = NO;
		for (unsigned int j = 0; j < noPossInliers-1; j++) {
			if (_ransac.possibleInlierIndices[j] == i) {
				isUsed = YES;
				break;
			}
		}
		if (!isUsed) {
			_ransac.nonInlierIndices[nonInlierCount++] = i;
			//nonInlierCount++;
		}
	}
	
	_showPlane = YES;
	_showPlaneChanged = YES;
	_showPlaneNo = 0;
	_resetRANSAC = YES;
}
- (void)freeRANSAC {
	free(_ransac.possibleInlierIndices);
	free(_ransac.nonInlierIndices);
	free(_ransac.possiblePlanes);
	free(_ransac.confidentPlanes);
	
	_ransac.possibleInlierIndices = NULL;
	_ransac.nonInlierIndices = NULL;
	_ransac.possiblePlanes = NULL;
	_ransac.confidentPlanes = NULL;
}
- (void)performRANSAC {
	static BOOL ransacPreviouslyCompleted = NO;
	
	if (_resetRANSAC) {
		_ransac.confidentPlanesCount = 0;
		_resetRANSAC = NO;
		ransacPreviouslyCompleted = NO;
	}
	
	if (_ransac.confidentPlanesCount < noConfidentPlanes) {
		NSLog(@"RANSAC current plane count = %d", _ransac.confidentPlanesCount);
		/* Iterate through 3 sets of possible inliers at a time.
		 Find the plane equation for them 3 and then compare all other
		 non-possible-inliers with that equation. Find the number of
		 points that satisfy the equation and become 'inliers'.
		 Finally, find the set/plane equation with the highest confidence.
		 */
		
		_ransac.noValidPlanes = 0;
		_ransac.confidentPlaneNo = 0;
		_ransac.confidentPlaneVal = 0;
		for (unsigned int i = 0, p = 0; i < noPossInliers; i+=3, p++) {
			// Find plane equation for the current 3 points.
			double wx1, wy1, wx2, wy2, wx3, wy3;
			worldFromIndex(&wx1, &wy1, _ransac.possibleInlierIndices[i+0], _kinectDepth[_ransac.possibleInlierIndices[i+0]]);
			worldFromIndex(&wx2, &wy2, _ransac.possibleInlierIndices[i+1], _kinectDepth[_ransac.possibleInlierIndices[i+1]]);
			worldFromIndex(&wx3, &wy3, _ransac.possibleInlierIndices[i+2], _kinectDepth[_ransac.possibleInlierIndices[i+2]]);
			
			[self calculatePlane:&_ransac.possiblePlanes[p]
						  Point1:(XYZPoint){wx1, wy1, _kinectDepth[_ransac.possibleInlierIndices[i+0]]}
						  Point2:(XYZPoint){wx2, wy2, _kinectDepth[_ransac.possibleInlierIndices[i+1]]}
						  Point3:(XYZPoint){wx3, wy3, _kinectDepth[_ransac.possibleInlierIndices[i+2]]}];
			BOOL goodData = _ransac.possiblePlanes[p].isValid;
			if (goodData) _ransac.noValidPlanes++;
			
			if (goodData) {
				// Randomly select a load of points and compare them to the current plane.
				// If the point lies within the plane ± tolerance then add 1 to the current
				// confidence value, if not then minus 1. If the current plane confidence value
				// is greater than the previous highest confidence plane then replace it as the
				// most confident.
				int currentConfidence = 0;
				ABCDPlane *v = &_ransac.possiblePlanes[p];	// Normal Indicies. Short for calcs.
				double distance = 0;				// Plane-Point Distance.
				for (unsigned int j = 10; j < noNonInliers; j+=confidenceIncrement) {
					if (j >= noNonInliers) break;
					// http://mathworld.wolfram.com/Point-PlaneDistance.html
					double wx, wy;
					worldFromIndex(&wx, &wy, _ransac.nonInlierIndices[j], _kinectDepth[_ransac.nonInlierIndices[j]]);
					if (_kinectDepth[_ransac.nonInlierIndices[j]] > 0) {
						distance = v->a*wx;
						distance += v->b*wy;
						distance += v->c*_kinectDepth[_ransac.nonInlierIndices[j]];
						distance += v->d;
						distance /= sqrt(pow(v->a, 2) + pow(v->b, 2) + pow(v->c, 2));
						distance = distance >= 0 ? distance : -distance;
						
						if (distance <= ransacTolerance) {
							currentConfidence++;
						} else {
							currentConfidence--;
						}
					}
				}
				
				if (currentConfidence > _ransac.confidentPlaneVal) {
					_ransac.confidentPlaneNo = p;
					_ransac.confidentPlaneVal = currentConfidence;
				}
			}
		}
	}
	
	if (_ransac.confidentPlanesCount < noConfidentPlanes) {
		// Add the most confident plane from the current
		// frame to the confidentPlanes array.
		
		ABCDPlane *currentPPlane = &_ransac.possiblePlanes[_ransac.confidentPlaneNo];
		ABCDPlane *currentCPlane = &_ransac.confidentPlanes[_ransac.confidentPlanesCount];
		
		currentCPlane->point1.x = currentPPlane->point1.x;
		currentCPlane->point1.y = currentPPlane->point1.y;
		currentCPlane->point1.z = currentPPlane->point1.z;
		currentCPlane->point2.x = currentPPlane->point2.x;
		currentCPlane->point2.y = currentPPlane->point2.y;
		currentCPlane->point2.z = currentPPlane->point2.z;
		currentCPlane->point3.x = currentPPlane->point3.x;
		currentCPlane->point3.y = currentPPlane->point3.y;
		currentCPlane->point3.z = currentPPlane->point3.z;
		currentCPlane->a = currentPPlane->a;
		currentCPlane->b = currentPPlane->b;
		currentCPlane->c = currentPPlane->c;
		currentCPlane->d = currentPPlane->d;
		currentCPlane->isValid = currentPPlane->isValid;
		
		_ransac.tablePlane.point1.x = currentPPlane->point1.x;
		_ransac.tablePlane.point1.y = currentPPlane->point1.y;
		_ransac.tablePlane.point1.z = currentPPlane->point1.z;
		_ransac.tablePlane.point2.x = currentPPlane->point2.x;
		_ransac.tablePlane.point2.y = currentPPlane->point2.y;
		_ransac.tablePlane.point2.z = currentPPlane->point2.z;
		_ransac.tablePlane.point3.x = currentPPlane->point3.x;
		_ransac.tablePlane.point3.y = currentPPlane->point3.y;
		_ransac.tablePlane.point3.z = currentPPlane->point3.z;
		_ransac.tablePlane.a = currentPPlane->a;
		_ransac.tablePlane.b = currentPPlane->b;
		_ransac.tablePlane.c = currentPPlane->c;
		_ransac.tablePlane.d = currentPPlane->d;
		_ransac.tablePlane.isValid = currentPPlane->isValid;
		
		_ransac.confidentPlanesCount++;
	}
		
	if (_ransac.confidentPlanesCount == noConfidentPlanes && ransacPreviouslyCompleted ==  NO) {
		// Once the algorithm has found the set of confident planes, average them.
		// Average the planes that have been found so far
		_ransac.tablePlane = (ABCDPlane){_ransac.confidentPlanes[0].point1,
										 _ransac.confidentPlanes[0].point2,
										 _ransac.confidentPlanes[0].point3,
										 0,0,0,0,YES};
		for (unsigned int k = 0; k < _ransac.confidentPlanesCount; k++) {
			// Check plane coefficients are similar to plane 0.
			//...code...
			// Sum plane coefficients
			_ransac.tablePlane.a += _ransac.confidentPlanes[k].a;
			_ransac.tablePlane.b += _ransac.confidentPlanes[k].b;
			_ransac.tablePlane.c += _ransac.confidentPlanes[k].c;
			_ransac.tablePlane.d += _ransac.confidentPlanes[k].d;
		}
		_ransac.tablePlane.a /= _ransac.confidentPlanesCount+1;
		_ransac.tablePlane.b /= _ransac.confidentPlanesCount+1;
		_ransac.tablePlane.c /= _ransac.confidentPlanesCount+1;
		_ransac.tablePlane.d /= _ransac.confidentPlanesCount+1;
		
		[_delegate objectDetectorDidCompleteRANSAC];
		
		ransacPreviouslyCompleted = YES;
	}
	
	if (_ransac.confidentPlanesCount > noConfidentPlanes) {
		NSLog(@"confidentPlanesCount has overflown (%d) NOT GOOD!", _ransac.confidentPlanesCount);
	}
}
- (void)calculatePlane:(ABCDPlane*)plane Point1:(XYZPoint)a Point2:(XYZPoint)b Point3:(XYZPoint)c {
    // First validate the input to check that Z values are not out-of-bounds of Kinect view.
    if (a.z < FREENECT_DEPTH_MM_NO_VALUE || a.z == FREENECT_DEPTH_MM_MAX_VALUE) {
		*plane = (ABCDPlane){a, b, c, 0, 0, 0, 0, NO};
		return;
    }
	if (b.z == FREENECT_DEPTH_MM_NO_VALUE || b.z == FREENECT_DEPTH_MM_MAX_VALUE) {
		*plane = (ABCDPlane){a, b, c, 0, 0, 0, 0, NO};
		return;
    }
	if (c.z == FREENECT_DEPTH_MM_NO_VALUE || c.z == FREENECT_DEPTH_MM_MAX_VALUE) {
		*plane = (ABCDPlane){a, b, c, 0, 0, 0, 0, NO};
		return;
    }
    
	//http://keisan.casio.com/has10/SpecExec.cgi# or
    //http://www.easycalculation.com/analytical/cartesian-plane-equation.php
    double Pa = (b.y-a.y)*(c.z-a.z) - (c.y-a.y)*(b.z-a.z);
    double Pb = (b.z-a.z)*(c.x-a.x) - (c.z-a.z)*(b.x-a.x);
    double Pc = (b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y);
    double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
    *plane = (ABCDPlane){a, b, c, Pa, Pb, Pc, Pd, YES};
}
- (void)sendPlaneDataToView {
	if (_ransac.noValidPlanes > 0 && _ransac.confidentPlanesCount > 0/* && _showPlaneChanged*/) {
		if (_ransac.tablePlane.isValid) {
			[glView setPlaneData:_ransac.tablePlane];
			_showPlane = YES;
		} else {
			_showPlane = NO;
			_showPlaneChanged = YES;
		}
	} else {
		_showPlane = NO;
		_showPlaneChanged = YES;
	}
	
	if (_showPlane/* && _showPlaneChanged*/) {
		[glView showPlane:YES];
	} else {
		[glView showPlane:NO];
	}
}




#pragma mark - Convex Hull
// http://en.wikipedia.org/wiki/Convex_hull_algorithms
// http://www.cs.umd.edu/~mount/754/Lects/754lects.pdf
#define pointOnTableTolerance 10
- (void)initConvexHull {
	_cHull.tableIndices		  = (unsigned int*)malloc(FREENECT_FRAME_PIX*sizeof(unsigned int));
	_cHull.tableIndiciesCount = 0;
	//_cHull.tableProjections	  = (IJXYZPoint*)malloc(FREENECT_FRAME_PIX*sizeof(IJXYZPoint));
	_cHull.tablePoints = (XYZPoint*)malloc(FREENECT_FRAME_PIX*sizeof(XYZPoint));
	_cHull.cHullIndices		  = (unsigned int*)malloc(FREENECT_FRAME_PIX*sizeof(unsigned int));
	_cHull.cHullIndexCount	  = 0;
	
	_resetConvexHull = YES;
}
- (void)freeConvexHull {
	free(_cHull.tableIndices);
	//free(_cHull.tableProjections);
	free(_cHull.tablePoints);
	free(_cHull.cHullIndices);
	
	_cHull.tableIndices		= NULL;
	//_cHull.tableProjections = NULL;
	_cHull.tablePoints		= NULL;
	_cHull.cHullIndices		= NULL;
}
- (void)performConvexHull {
	static BOOL convexHullPreviouslyCompleted = NO;
	
	if (_resetConvexHull) {
		_resetConvexHull = NO;
		convexHullPreviouslyCompleted = NO;
	}
	
	if (convexHullPreviouslyCompleted == NO) {
		// Find the the points relating to the table.
		//  - Filter points that lie on plane.
		//  - Map points on to plane.
		// Perform Convex Hull algorithm to find bounds of the table.
		
		// Only perform this algorithm once the RANSAC iterations are complete.
		if (_ransac.confidentPlanesCount >= noConfidentPlanes) {
			// Search for table bounds along the found plane equation.
			/* This will require filtering all the points that are within
			 a set distance from the plane equation, then mapping them to
			 the plane to get (i,j) coordinates.
			 */
			
			// Initialise counts.
			_cHull.tableIndiciesCount = 0;
			_cHull.cHullIndexCount	  = 0;
			
			// Filter the points that lie on the plane and get their indices.
			ABCDPlane *table = &_ransac.tablePlane;
			
			// Set variables as static to retain memory.
			// Save memory cycles for large arrays :(
			static double wx = 0;
			static double wy = 0;
			static double dist = 0;
			
			// The left-most point struct.
			struct {
				unsigned int index;
				int value;
			} static leftMost = {0, 100000};	// Very big positive number!
			
			// Iterate through all points to find the left most, ready for Convex Hull.
			for (unsigned int i = 0; i < FREENECT_FRAME_PIX; i++) {
				if (_kinectDepth[i] > 0) {
					
					// Calculate the distance from point to plane.
					BOOL isOnTable = NO;
					if (_kinectDepth[i] > 0) {
						// Get the relevant world x,y location.
						worldFromIndex(&wx, &wy, i, _kinectDepth[i]);
						// Does it lie on the plane or super close to it?
						// http://mathworld.wolfram.com/Point-PlaneDistance.html
						dist  = table->a * wx;
						dist += table->b * wy;
						dist += table->c * _kinectDepth[i];
						dist += table->d;
						dist /= sqrt(pow(table->a, 2) + pow(table->b, 2) + pow(table->c, 2));
						// Generate the absolute distance.
						dist  = (dist >= 0) ? dist : -dist;
						isOnTable = (dist <= pointOnTableTolerance) ? YES : NO;
					}
					
					// If the distance is within the tolerance then it is probably on the table/plane.
					if (isOnTable) {
						// Add the point to the indices
						_cHull.tableIndices[_cHull.tableIndiciesCount] = i;
						
						// NEW: DON'T DO PLANE-POINT PROJECTION! Overcomplication of algorithm.
						/*
						// Find the projected point to the plane
						// http://stackoverflow.com/questions/9605556/how-to-project-a-3d-point-to-a-3d-plane
						// projectedPoint = point + distance*normal;
						IJXYZPoint *cProj = &_cHull.tableProjections[_cHull.tableIndiciesCount];
						// 3D projected point
						cProj->x = wx + dist * table->a;
						cProj->y = wy + dist * table->b;
						cProj->z = _kinectDepth[i] + dist * table->c;
						// 2D project point w.r.t. plane
						cProj->i = sqrt(pow(cProj->x, 2) + pow(cProj->z, 2));
						cProj->j = sqrt(pow(cProj->y, 2) + pow(cProj->z, 2));
						 */
						
						// Add point to the tablePoints array for later reference.
						_cHull.tablePoints[_cHull.tableIndiciesCount].x = wx;
						_cHull.tablePoints[_cHull.tableIndiciesCount].y = wy;
						_cHull.tablePoints[_cHull.tableIndiciesCount].z = _kinectDepth[i]*1.0;//ensure data sent rather than pointer.
						
						
						// Check how left (x) this point is
						if (_cHull.tablePoints[_cHull.tableIndiciesCount].x < leftMost.value) {
							leftMost.value = _cHull.tablePoints[_cHull.tableIndiciesCount].x;
							leftMost.index = _cHull.tableIndiciesCount;
						}
						
						//printf("tablePoint[%d]   i:%.3f   j:%.3f\n", _cHull.tableIndiciesCount, cProj->i, cProj->j);
						
						// Increment the point count
						_cHull.tableIndiciesCount++;
					}
				}
			}
			
			// IDEA - Possibly search for nearby neighbours to remove outliers.
			
			// Perform the Jarvis' March algorithm (Gift-Wrapping / Convex Hull).
			// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
			// - Select the left-most point (on the x-axis), call it point 0
			// - In a for loop, find the CW angles between p0 and every other point.
			// - The next point in the convex hull is the point that gives the smallest angle against p0.
			// - This process is repeated until the smallest CW angle is >= 270 (or <= -270). Then
			// - This process is repeated until point n-1 has p0 as its smallest anti-cw angle.
			
			// @ Point 1 of Convex Hull.
			// Convex Hull Point 1 = Left Most Point.
			_cHull.cHullIndices[0] = leftMost.index;
			_cHull.cHullIndexCount = 1;
			
			typedef struct {
				unsigned int index;
				double value;
			} _smallAngle;
			_smallAngle smallAngle = {0, 4*PI_VAL};
			
			// WARNING!!! This next set of operations could take a VERY LONG TIME!
			// Consider running on a separate thread and report back results once complete.
			// The speed of this operation is not essential as it should only need to be done once.
			
			double angleTolerance = -PI_VAL/2;	// Start from an angle tolerance of -90 (-pi/2).
			// Iterate through the convex hull points as the next smallest angle point is found.
			unsigned int i = 0;
			for (i = 0; i < _cHull.tableIndiciesCount; i++) {
				
				// Reset the small angle for new convex hull points.
				smallAngle = (_smallAngle){0, 4*PI_VAL};
				
				// Now iterate through all the OTHER points on the table to find the next on the
				// convex hull (smallest angle).
				for (unsigned int j = 0; j < _cHull.tableIndiciesCount; j+=1) {
					
					// Check that the current table point is not the current convex hull point. If so then continue.
					if (_cHull.cHullIndices[i] == j) {
						continue;
					}
					
					// Find the clock-wise angle between the current convex hull point and the current table point.
					double thisAngle = atan2(_cHull.tablePoints[_cHull.cHullIndices[i]].y - _cHull.tablePoints[j].y,
											 // y[convex] - y[table]
											 _cHull.tablePoints[_cHull.cHullIndices[i]].x - _cHull.tablePoints[j].x);
											 // x[convex] - x[table]
					
					
					// Dependin upon the current angleTolerane we ue different angle calculations.
					if (angleTolerance != -6*PI_VAL/4) {
						// If the tolerance is currently set to -90deg.
						// If the angle is positive (Anti-CW) then convert it to CW.
						thisAngle = (thisAngle < 0) ? thisAngle : -2*PI_VAL + thisAngle;
					} else {
						// If the tolerance is currently set to -270deg.
						// No matter what sign the angle has we will perform the same operation.
						thisAngle = -2*PI_VAL + thisAngle;
					}
					
					// If the angle is less than the current angle tolerance then it is valid.
					// Else, this angle is not good for the convex hull so move onto the next table point.
					if ((thisAngle <= angleTolerance) && (fabs(thisAngle) < fabs(smallAngle.value))) {
						smallAngle.index = j;
						smallAngle.value = thisAngle;
					}
					
				}
				
				// Now we must of found that smallest-angle point, so make it the next point in the convex hull,
				// as long as it is not the same point (index) as convex hull point 0!
				if (_cHull.cHullIndices[0] != smallAngle.index) {
					
					// Set the next convex hull point.
					_cHull.cHullIndices[i+1] = smallAngle.index;
					_cHull.cHullIndexCount++;
					printf("CONVEXHULL points %7d(%4.3f,%4.3f):%7d(%.3f,%.3f) - angle: %f - indexCount: %d\n",
						   _cHull.cHullIndices[i],
						   _cHull.tablePoints[_cHull.cHullIndices[i]].x,
						   _cHull.tablePoints[_cHull.cHullIndices[i]].y,
						   _cHull.cHullIndices[i+1],
						   _cHull.tablePoints[_cHull.cHullIndices[i+1]].x,
						   _cHull.tablePoints[_cHull.cHullIndices[i+1]].y,
						   smallAngle.value*180/PI_VAL,
						   _cHull.cHullIndexCount);
					
					// Check to see if the small angle was greater than -270 (-3pi/4). If so then then we
					// must be at the right-most point and so the convex hull line will be going in the
					// reverse direction.
					if (fabs(smallAngle.value) >= 6*PI_VAL/4 && angleTolerance > -6*PI_VAL/4) {
						angleTolerance = -6*PI_VAL/4;
						printf("LOG: swapped angleTolerance to %f because of last smallAngle = %f\n", angleTolerance*180/PI_VAL, smallAngle.value*180/PI_VAL);
					}
					
				} else {
					// Convex hull point 0 = new convex hull point!
					// Close up the convex hull and finish this for loop!
					// If this doesn't happen before we reach the end of the 'i' for loop then there's
					// big problems.
					printf("ConvexHull: Made it back to the original point!\n");
					break;
				}
			}
			
			// Check for that BIG PROBLEM as defined a few lines up.
			if (i >= _cHull.tableIndiciesCount) {
				// BIG PROBLEM!
				// Initiate big problem ray.
				NSLog(@"WARNING (Convex Hull): The convex hull iterator reached the max number of table points. This should not happen! Finished at point (i): %d. Number f table points: %d. May be an error in programming.", i, _cHull.tableIndiciesCount);
			}
			
			convexHullPreviouslyCompleted = YES;
			[_delegate objectDetectorDidCompleteEdgeFinding];
			
			// Log the convex hull points for debugging/testing.
			printf("Convex Hull Complete. Total Points = %d\n", _cHull.cHullIndexCount);
			for (unsigned int p = 0; p < _cHull.cHullIndexCount; p++) {
				printf("\tpoint %5d/%d - i: %f j: %f\n", p, _cHull.cHullIndexCount, _cHull.tablePoints[_cHull.cHullIndices[p]].x, _cHull.tablePoints[_cHull.cHullIndices[p]].y);
			}
			
		} // END _ransac.confidentPlanesCount >= noConfidentPlanes
	} // END convexHullPreviouslyCompleted == NO

}




#pragma mark - Private Methods
/*- (BOOL)isPoint:(uint8_t)point WithIndex:(unsigned int)index OnPlane:(ABCDPlane*)plane WithTolerance:(double)tolerance {
	// Set variables as static to retain memory.
	// Save memory cycles for large arrays.
	static double wx = 0;
	static double wy = 0;
	static double dist = 0;
	bool result = false;
	
	if (point > 0) {
		// Get the relevant world x,y location.
		worldFromIndex(&wx, &wy, index, point);
		// Does it lie on the plane or super close to it?
		// http://mathworld.wolfram.com/Point-PlaneDistance.html
		dist  = plane->a * wx;
		dist += plane->b * wy;
		dist += plane->c * point;
		dist += plane->d;
		dist /= sqrt(pow(plane->a, 2) + pow(plane->b, 2) + pow(plane->c, 2));
		// Generate the absolute distance.
		dist  = (dist >= 0) ? dist : -dist;
		result = (dist <= tolerance) ? true : false;
	}
	
	return result;
}*///is point on plane method.




#pragma mark - Instance Methods
- (void)resetRANSACCapture {
	_resetRANSAC = YES;
	_resetConvexHull = YES;
}
- (void)showPlaneDataForSet:(unsigned int)setNo {
	_showPlane = YES;
	_showPlaneChanged = YES;
	_showPlaneNo = setNo;
}
- (void)hidePlaneData {
	_showPlane = NO;
	_showPlaneChanged = YES;
}




#pragma mark - Delegate Methods
- (void)setGLViewOutlet:(GLView*)newGLView {
	if (glView) {
		[glView release];
		glView = NULL;
	}
	glView = newGLView;
}
- (void)setKinectControllerDelegate:(id<JRKinectControllerDelegate>)delegate {
    [kinectController setDelegate:delegate];
}


@end
