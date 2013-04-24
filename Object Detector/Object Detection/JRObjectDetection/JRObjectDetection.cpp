//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

#pragma mark - 
#pragma mark Helping Functions

PointXYZ worldFromIndex(size_t index, int z)
{
	double x = index % FREENECT_FRAME_W;
	double y = (index - x) / FREENECT_FRAME_W;
	x = (double)((x - FREENECT_FRAME_W/2.0) * z * 0.00174);
	y = (double)((y - FREENECT_FRAME_H/2.0) * z * -0.00174);
	return PointXYZ(x, y, z);
}

// Function to get the plane equation from 3 points in 3D space.
PlaneCoefficients getPlaneCoefficients(PointXYZ a, PointXYZ b, PointXYZ c)
{
	// First validate the input to check that Z values are not out-of-bounds of Kinect view.
	if (!a.isValid() || !b.isValid() || !c.isValid()) {
		return PlaneCoefficients(0, 0, 0, 0);
	}
	
	//http://keisan.casio.com/has10/SpecExec.cgi# or
	//http://www.easycalculation.com/analytical/cartesian-plane-equation.php
	double Pa = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
	double Pb = (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x);
	double Pc = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
	double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
	
	return PlaneCoefficients(Pa, Pb, Pc, Pd);
}

double dot(PointXYZ a, PointXYZ b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double dot(PlaneCoefficients a, PlaneCoefficients b)
{
	return a.a*b.a + a.b*b.b + a.c*b.c;
}

double mod(PlaneCoefficients a)
{
	return sqrt( a.a*a.a + a.b*a.b + a.c*a.c );
}

double theta(PlaneCoefficients a, PlaneCoefficients b)
{
	// theta = acos ( a . b / ||a|| ||b|| )
	double moda = mod(a);
	double modb = mod(b);
	return (moda == 0 || modb == 0) ? 0 : acos( dot(a, b) / (moda * modb) );
}

// To normalise a normal equation you can do this
//	CombinedSquares = (Normal.x * Normal.x) +
//	(Normal.y * Normal.y) +
//	(Normal.z * Normal.z);
//	NormalisationFactor = sqrt(CombinedSquares);
//	Normal.x = Normal.x / NormalisationFactor;
//	Normal.y = Normal.y / NormalisationFactor;
//	Normal.z = Normal.z / NormalisationFactor;





#pragma mark -
#pragma mark Private Functions

bool ObjectDetection::compareNormalAngle(size_t a, size_t b)
{
	const double thresholdDistance = 10.0; // 10mm.
	const double thresholdAngle = PI_VALUE/9; // 20 deg in radians.
	
	// Compare the difference between the d component of the two points.
	//double distanceVal = fabs(input_cloud_normals[a].d - input_cloud_normals[b].d);
	double distanceVal = fabs(input_cloud[a].z - input_cloud[b].z);
	//double distanceVal = 0;
	bool distance = distanceVal < thresholdDistance;
	
	// Compare the angle between normal vectors by doing the dot product.
	//double angleVal = dot(input_cloud_normals[a], input_cloud_normals[b]);
	double angleVal = theta(input_cloud_normals[a], input_cloud_normals[b]);
	bool angle = angleVal < thresholdAngle;
	
	bool isValid = input_cloud_normals[a].isSet() && input_cloud_normals[b].isSet();
	
	/*static size_t count = 0;
	if (count > 1000) {
		count = 0;
		cout << "\tcompareNormalAngle  :  distance = " << distanceVal << " (" << distance << ")  " <<
				"angle = " << angleVal << " (" << angle << ")  " <<
				"isValid = " << isValid << "." << endl;
	} else {
		count++;
	}*/
	
	/*if (distance && angle && isValid) {
		cout << "\tcompareNormalAngle  :  distance = " << distanceVal << " (" << distance << ")  " <<
				"angle = " << angleVal << " (" << angle << ")  " <<
				"isValid = " << isValid << "." << endl;
	}*/
	
	
	return distance && angle && isValid;
}





#pragma mark -
#pragma mark Object Detection Functions

void ObjectDetection::setZPoints(uint16_t* zPoints, size_t size)
{
	// Remove all points from the input_cloud.
	input_cloud.erase(input_cloud.begin(), input_cloud.end());
	
	uint16_t depthCount = 0;
	// Iterate through all the new points and add them to the input_cloud
	// also calculate the world x and y values on input.
	for (size_t i = 0; i < size; i++) {
		depthCount += zPoints[i];
		input_cloud.emplace_back(worldFromIndex(i, zPoints[i]));
	}
	if (depthCount > 0) {
		validDepthData = true;
	} else {
		validDepthData = false;
		printf("updateDepthData dataIsNotValid\n");
	}
}

void ObjectDetection::calculateSurfaceNormals()
{
	if (validDepthData) {
		cout << "calculateSurfaceNormals started." << endl;
		
		// Iterate through all the points in the input_cloud and calcualte
		// the surface normals of each point.
		// First, remove all the current normals.
		input_cloud_normals.erase(input_cloud_normals.begin(), input_cloud_normals.end());
		
		// Some points will need to be skipped as the normals are calculated
		// by using 2 neighboring points to the right and below. Therefore the
		// right-most and bottom-most pixels will not have a complete set of
		// neighbours.
		size_t count = 0;
		for (size_t i = 0; i < input_cloud.size(); i++) {
			uint x = 0, y = 0;
			frameXYfromIndex(&x, &y, (uint)i);
			
			const int mult = 10;
			
			if (   x >= FREENECT_FRAME_W-mult	// is right-hand side
				&& y >= FREENECT_FRAME_H*mult-1	// and is bottom
				) {
				// This point is out-of-bounds! Set the normal to 0.
				input_cloud_normals.emplace_back(PlaneCoefficients(0, 0, 0, 0));
			} else {
				// This point is within bounds. Calculate the normal.
				input_cloud_normals.emplace_back(getPlaneCoefficients(input_cloud[i],
																	  input_cloud[i+mult],
																	  input_cloud[i+FREENECT_FRAME_W*mult]));
			}
			//if (count > 10000) {
				//cout << "\tNormal coefficients of point " << i << " are A = " << input_cloud_normals[i].a << "  B = " << input_cloud_normals[i].b << "  C = " << input_cloud_normals[i].c << "  D = " << input_cloud_normals[i].d << "." << endl;
			//	count = 0;
			//} else { count++; }
		}
		
		cout << "calculateSurfaceNormals completed." << endl;
	}/* END validDepthData */
}

// Mark all nodes unassigned.
//
// ITERATE through all nodes:
//		IF node unassigned:
//			ASSIGN node to a new component id C
//			DO a depth-first-search for all nodes connected to this one:
//				mark them with same component id C

void ObjectDetection::segmentPlanes()
{
	if (validDepthData) {
		cout << "segmentPlanes started." << endl;
		
		// Remove all the current points from the plane_cluster_nodes and reserve the size we need.
		// Also, set all the node values to -1 to show that they are all unassigned.
		vector<int> plane_cluster_nodes;
		plane_cluster_nodes.assign(FREENECT_FRAME_PIX, NODE_UNASSIGNED);
		
		int current_segment_id = -1;
		
		// Iterate through all nodes.
		for (size_t i = 0; i < FREENECT_FRAME_PIX; i++) {
			
			// Check if the current node is unassigned.
			if (plane_cluster_nodes[i] == NODE_UNASSIGNED) {
				
				//cout << "\tCurrent node is UNASSIGNED. Node = " << i << "   Current Segment = " << current_segment_id << "." << endl;
				
				// The node is unassigned then assign it a new segment_id.
				current_segment_id++;
				plane_cluster_nodes[i] = current_segment_id;
				
				// Find out if the current node has a neighbour.
				uint currentX, currentY = 0;
				frameXYfromIndex(&currentX, &currentY, (uint)i);
				
				bool rightIsNeighbour = (currentX < FREENECT_FRAME_W-1)&&(i < FREENECT_FRAME_PIX-1)				   ? compareNormalAngle(i, i+1)				   : false;
				bool belowIsNeighbour = (currentY < FREENECT_FRAME_H-1)&&(i < FREENECT_FRAME_PIX-FREENECT_FRAME_W) ? compareNormalAngle(i, i+FREENECT_FRAME_W) : false;
				
				// If it has neighbours then set their segment id to the current points id.
				if (rightIsNeighbour) {
					plane_cluster_nodes[i+1] = plane_cluster_nodes[i];
				}
				if (belowIsNeighbour) {
					plane_cluster_nodes[i+FREENECT_FRAME_W] = plane_cluster_nodes[i];
				}
				
			} else {
				
				// This node is assigned, so find its neighbours.
				// Find out if the current node has a neighbour.
				uint currentX, currentY = 0;
				frameXYfromIndex(&currentX, &currentY, (uint)i);
				
				bool rightIsNeighbour = (currentX < FREENECT_FRAME_W-1)&&(i < FREENECT_FRAME_PIX-1)				   ? compareNormalAngle(i, i+1)				   : false;
				bool belowIsNeighbour = (currentY < FREENECT_FRAME_H-1)&&(i < FREENECT_FRAME_PIX-FREENECT_FRAME_W) ? compareNormalAngle(i, i+FREENECT_FRAME_W) : false;
				
				//cout << "\tCurrent node is assigned. Node = " << i << "   Current Segment = " << current_segment_id << ".  rightIsNeighbour=" << rightIsNeighbour << "   belowIsNeighbour=" << belowIsNeighbour << "." << endl;
				
				// If it has neighbours then set their segment id to the current points id.
				if (rightIsNeighbour) {
					plane_cluster_nodes[i+1] = plane_cluster_nodes[i];
				}
				if (belowIsNeighbour) {
					plane_cluster_nodes[i+FREENECT_FRAME_W] = plane_cluster_nodes[i];
				}
				
			}
			
		}
		
		cout << "segmentPlanes completed. Found " << current_segment_id << "." << endl;
		
		// FILTERING
		
		// Run through all the points in plane_cluster_nodes and count how many
		// points belong to each one.
		vector<PointIndices> all_clusters;
		all_clusters.resize(current_segment_id);
		
		for (size_t i = 0; i < plane_cluster_nodes.size(); i++) {
			size_t segment_id = (size_t)plane_cluster_nodes[i];
			//cout << "\tassigning " << i << " to segment_id " << segment_id << ". all_clusters.size = " << all_clusters.size() << "." << endl;
			
			if (segment_id >= all_clusters.size()) {
				PointIndices newIndices;
				newIndices.indices.emplace_back(i);
				all_clusters.emplace_back(newIndices);
			} else {
				all_clusters[segment_id].indices.emplace_back(i);
			}
		}
		
		// Find all the clusters that have a point count greater than the threshold.
		plane_clusters.erase(plane_clusters.begin(), plane_clusters.end());
		size_t bigClusterCount = 0;
		for (size_t i = 0; i < all_clusters.size(); i++) {
			if (all_clusters[i].indices.size() > PLANE_CLUSTER_THRESHOLD) {
				bigClusterCount++;
				// Add this cluster to the plane_clusters.
				plane_clusters.emplace_back(all_clusters.at(i));
			}
		}
		cout << "segmentPlanes Filtered clusters down to " << bigClusterCount << " clusters. new plane_clusters.size = " << plane_clusters.size() << "." << endl;
		
		// Find the normal coefficients for the plane_clusters.
		// Do this by using the normal coefficients from the mode point in the cluster.
		plane_clusters_normals.erase(plane_clusters_normals.begin(), plane_clusters_normals.end());
		for (size_t i = 0; i < plane_clusters.size(); i++) {
			size_t index = plane_clusters[i].indices[(uint)plane_clusters[i].indices.size()/2];
			plane_clusters_normals.emplace_back(input_cloud_normals[index].a, input_cloud_normals[index].b, input_cloud_normals[index].c, input_cloud_normals[index].d);
			cout << "segmentPlanes set cluster " << i << " normals to A = " << input_cloud_normals[index].a << "  B = " << input_cloud_normals[index].b << "  C = " << input_cloud_normals[index].c << "  D = " << input_cloud_normals[index].d << " for an index of " << index << "." << endl;
		}
	}/* END validDepthData */
}




void ObjectDetection::segmentPlanes_old_old()
{
	if (validDepthData) {
		cout << "segmentPlanes started." << endl;
		
		// Create a vector containing all the indices to be clustered.
		vector<size_t> non_clustered_indices;
		for (size_t i = 0; i < input_cloud_normals.size(); i++) {
			non_clustered_indices.emplace_back(i);
		}
		
		// Remove all the previous plane clusters.
		plane_clusters.erase(plane_clusters.begin(), plane_clusters.end());
		
		// Iterate through each point in non_clustered and find which cluster it belongs to.
		while (non_clustered_indices.size() > 0) {
			// We need to process all the points until all the points belong to a cluster.
			
			// Now we are looking at creating a new cluster. Let's add the first available point
			// to it.
			PointIndices current_cluster_indices;
			current_cluster_indices.indices.emplace_back(*non_clustered_indices.begin());
			non_clustered_indices.erase(non_clustered_indices.begin());
			
			// Then compare all the other non_clustered points to the points in the current cluster.
			for (vector<size_t>::iterator nonc = non_clustered_indices.begin(); nonc != non_clustered_indices.end(); ++nonc) {
				//cout << "\t\t nonc start = " << *nonc << endl;
				bool neighbourFound = false;
				
				for (vector<size_t>::iterator curc = current_cluster_indices.indices.begin(); curc != current_cluster_indices.indices.end(); ++curc) {
					//cout << "\t\t\t nonc = " << *nonc << "   curc = " << *curc << "   :   nonc.size = " << non_clustered_indices.size() << "   curc.size = " << current_cluster_indices.indices.size() << endl;
					bool isNeighbour = compareNormalAngle(*nonc, *curc);
					
					if (isNeighbour) {
						//cout << "\t\t\t ^^ is neighbour" << endl;
						// This point in non_clustered is a neighbour to a point in current_cluster.
						// Therefore let's move it from non_clustered and into current_cluster.
						current_cluster_indices.indices.emplace_back(*nonc);
						non_clustered_indices.erase(nonc);
						neighbourFound = true;
						break;
					}
					
				}
				
				//cout << "\t\t nonc end = " << *nonc << endl;
				if (neighbourFound) {
					//something
				}
			}
			
			// We now should of found all the neighbours in current_cluster.
			// So now let's push back this current cluster to the plane_clusters vector.
			plane_clusters.emplace_back(current_cluster_indices);
			cout << "\tFinished cluster " << plane_clusters.size() << "." << endl;
		}
		
		cout << "segmentPlanes completed. Found " << plane_clusters.size() << " clusters." << endl;
	}/* END validDepthData */
}

void ObjectDetection::segmentPlanes_old()
{
	if (validDepthData) {
		cout << "segmentPlanes started." << endl;
		
		// Create a vector containing all the indices to be clustered.
		vector<size_t> non_clustered_indices;
		for (size_t i = 0; i < input_cloud_normals.size(); i++) {
			non_clustered_indices.emplace_back(i);
		}
		
		// Iterate through each point in non_clustered and find which cluster it belongs to.
		while (non_clustered_indices.size() > 0) {
			
			PointIndices current_cluster;
			vector<size_t> check_list_indices;
			
			// Start by adding the first available non-clustered point to be checked.
			check_list_indices.emplace_back(*non_clustered_indices.begin());
			non_clustered_indices.erase(non_clustered_indices.begin());
			
			/*if (current_cluster.indices.size() > 1000) {
			 cout << "\tStarting cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
			 } else if (plane_clusters.size() % 1000 == 0) {
			 cout << "\tStarting cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
			 }*/
			
			// Now compare each point in check_list to its neighbours.
			// If a point is a neighbour then add it to the check list.
			// Once a point in the check_list has finished comparing neighbours then
			// and it to the current_cluster.
			size_t currentCount = 0;
			while (check_list_indices.size() > 0) {
				
				size_t currentIndex = *check_list_indices.begin();
				size_t aboveIndex   = *check_list_indices.begin()-FREENECT_FRAME_W;
				size_t belowIndex   = *check_list_indices.begin()+FREENECT_FRAME_W;
				size_t rightIndex   = *check_list_indices.begin()+1;
				size_t leftIndex    = *check_list_indices.begin()-1;
				
				bool aboveIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), aboveIndex) != non_clustered_indices.end();
				bool belowIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), belowIndex) != non_clustered_indices.end();
				bool rightIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), rightIndex) != non_clustered_indices.end();
				bool leftIsValid  = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), leftIndex)  != non_clustered_indices.end();
				
				bool aboveIsNeighbour = compareNormalAngle(currentIndex, aboveIndex);
				bool belowIsNeighbour = compareNormalAngle(currentIndex, belowIndex);
				bool rightIsNeighbour = compareNormalAngle(currentIndex, rightIndex);
				bool leftIsNeighbour  = compareNormalAngle(currentIndex, leftIndex);
				
				//cout << "\t\tCurrent check_list_index = " << currentIndex << "  above = " << aboveIndex << "  below = " << belowIndex << "  right = " << rightIndex << "  left = " << leftIndex << "." << endl;
				//cout << "\t\t\t\t isValid  :" << "  above = " << aboveIsValid << "  below = " << belowIsValid << "  right = " << rightIsValid << "  left = " << leftIsValid << "." << endl;
				//cout << "\t\t\t\t isNeighbour  :";
				if (aboveIsValid) {
					if (aboveIsNeighbour) {
						//cout << "  above is  ";
						check_list_indices.emplace_back(aboveIndex);
						non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), aboveIndex), non_clustered_indices.end());
					}
				}
				
				if (belowIsValid) {
					if (belowIsNeighbour) {
						//cout << "  below is  ";
						check_list_indices.emplace_back(belowIndex);
						non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), belowIndex), non_clustered_indices.end());
					}
				}
				
				if (rightIsValid) {
					if (rightIsNeighbour) {
						//cout << "  right is  ";
						check_list_indices.emplace_back(rightIndex);
						non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), rightIndex), non_clustered_indices.end());
					}
				}
				
				if (leftIsValid) {
					if (leftIsNeighbour) {
						//cout << "  left is   ";
						check_list_indices.emplace_back(leftIndex);
						non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), leftIndex), non_clustered_indices.end());
					}
				}
				//cout << "." << endl;
				
				// Now move the current_index into the current_cluster.
				current_cluster.indices.emplace_back(currentIndex);
				check_list_indices.erase(std::remove(check_list_indices.begin(), check_list_indices.end(), currentIndex), check_list_indices.end());
				
				//cout << "\t\tCurrent check_list.size = " << check_list_indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "   current_cluster.size = " << current_cluster.indices.size() << ".\n------------------------" << endl;
				if (currentCount > 100) {
					currentCount = 0;
					cout << "\t\tCurrent check_list.size = " << check_list_indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "   current_cluster.size = " << current_cluster.indices.size() << "  :  Neighbours  above=" << aboveIsNeighbour << "  below=" << belowIsNeighbour << "  left=" << leftIsNeighbour << "  right=" << rightIsNeighbour << "." << endl;
				} else {
					currentCount++;
				}
				
			}
			
			if (current_cluster.indices.size() > 1000) {
				cout << "\tFinished cluster " << plane_clusters.size() << "   current_cluster.size = " << current_cluster.indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
			} else if (plane_clusters.size() % 1000 == 0) {
				cout << "\tFinished cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
			}
			
			// We have now finished with the current_cluster. So add it to the plane_clusters.
			plane_clusters.emplace_back(current_cluster);
			
		}
		
		size_t goodClusters = 0;
		//for (vector<PointIndices>::iterator cluster = plane_clusters.begin(); cluster != plane_clusters.end(); ++cluster) {
		for (size_t i = 0; i < plane_clusters.size(); i++) {
			size_t clusterCount = plane_clusters[i].indices.size();
			if (clusterCount > 1000) {
				goodClusters++;
			}
		}
		
		cout << "segmentPlanes completed. Found " << plane_clusters.size() << " clusters, of which " << goodClusters << "are over the threshold of 1000 points." << endl;
	}/* END validDepthData */
}
