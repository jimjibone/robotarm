#ifndef __robotarm__ObjectDetector__
#define __robotarm__ObjectDetector__

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

using namespace std;

template <typename PointType>
class ObjectDetector
{
private:
	
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	
	pcl::PassThrough<PointType> pass_through_;
	CloudConstPtr	cloud_;
	CloudPtr		cloud_pass_through_;
	CloudPtr		cloud_hull_;
	vector<pcl::Vertices> vertices_;
	bool new_cloud_;
	
    //ObjectDetector( ___ ) : ___ { }
    //virtual ~ObjectDetector();
    void processCloud(const CloudConstPtr& cloud);
};




ObjectDetector::ObjectDetector()
{
	pass_through_.setFilterFieldName("z");
	pass_through_.setFilterLimits(10.0, 3000.0);
}

ObjectDetector::~ObjectDetector() {}

void ObjectDetector::processCloud(const CloudConstPtr& cloud)
{
	cloud_pass_through_.reset(new Cloud);
	// Computation goes here
	pass_through_.setInputCloud(cloud);
	pass.filter(*cloud_pass_through_);
	
	// Estimate 3d convex hull
	pcl::ConvexHull<PointType> hr;
	hr.setInputCloud(cloud_pass_through_);
	cloud_hull_.reset(new Cloud);
	hr.reconstruct(*cloud_hull_, vertices_);
	
	cloud_ = cloud;
	new_cloud_ = true;
}


#endif /* defined(__robotarm__ObjectDetector__) */
