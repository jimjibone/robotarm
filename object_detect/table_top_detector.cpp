// Author: Nicolas Burrus
// Hacking the Kinect

#include "table_top_detector.h"

template <class PointT>
TableTopDetector<PointT> :: TableTopDetector()
{
    // Create all PCL objects and set their parameters
    setObjectVoxelSize();
    setBackgroundVoxelSize();
    setDepthLimits();
    setObjectHeightLimits();

    // Table model fitting parameters
    plane_distance_threshold_ = 0.02; // 2cm

    // Weigth of the normal distance term.
    normal_distance_weight_ = 0.1;

    // Clustering parameters
    object_cluster_tolerance_ = 0.05;        // 5cm between two objects
    object_cluster_min_size_  = 100;         // 100 points per object cluster
}

template <class PointT>
void TableTopDetector<PointT> :: initialize()
{
    grid_.setLeafSize (background_voxel_size_, background_voxel_size_, background_voxel_size_);
    grid_objects_.setLeafSize (object_voxel_size_, object_voxel_size_, object_voxel_size_);
    grid_.setFilterFieldName ("z");
    pass_through_filter_.setFilterFieldName ("z");

    grid_.setFilterLimits (min_z_bounds_, max_z_bounds_);
    pass_through_filter_.setFilterLimits (min_z_bounds_, max_z_bounds_);
    grid_.setDownsampleAllData (false);
    grid_objects_.setDownsampleAllData (false);

    normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
    clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
    clusters_tree_->setEpsilon (1);

    normal_estimator_.setKSearch (10); // 10 k-neighbors by default
    normal_estimator_.setSearchMethod (normals_tree_);

    segmentor_.setNormalDistanceWeight (normal_distance_weight_);
    segmentor_.setOptimizeCoefficients (true);
    segmentor_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    segmentor_.setMethodType (pcl::SAC_RANSAC);
    segmentor_.setProbability (0.99);
    segmentor_.setDistanceThreshold (plane_distance_threshold_);
    segmentor_.setMaxIterations (10000);

    project_inliers_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

    prism_.setHeightLimits (object_min_height_, object_max_height_);

    cluster_.setClusterTolerance (object_cluster_tolerance_);
    cluster_.setMinClusterSize (object_cluster_min_size_);
    cluster_.setSearchMethod (clusters_tree_);
}

template <class PointT>
bool TableTopDetector<PointT> :: detect(pcl::PointCloud<Point>& cloud)
{
    objects_.clear();
    initialize();

    PCL_INFO("PointCloud with %d data points.\n", cloud.width * cloud.height);

    // Convert the dataset
    cloud_ = cloud.makeShared();

    // Filter points outside of the defined range
    pcl::PointCloud<Point> cloud_filtered;
    pass_through_filter_.setInputCloud (cloud_);
    pass_through_filter_.filter (cloud_filtered);
    cloud_filtered_.reset (new pcl::PointCloud<Point> (cloud_filtered));
    PCL_INFO("Number of points left after filtering (%f -> %f): %d out of %d.\n", min_z_bounds_, max_z_bounds_, (int)cloud_filtered.points.size (), (int)cloud.points.size ());

    // Downsample the point cloud.
    pcl::PointCloud<Point> cloud_downsampled;
    grid_.setInputCloud (cloud_filtered_);
    grid_.filter (cloud_downsampled);
    cloud_downsampled_.reset (new pcl::PointCloud<Point> (cloud_downsampled));

    if ((int)cloud_filtered_->points.size () < 10)
    {
        PCL_INFO("WARNING Filtering returned %d points! Exiting.\n", (int)cloud_filtered_->points.size ());
        return false;
    }

    // Estimate the point normals
    pcl::PointCloud<pcl::Normal> cloud_normals;
    normal_estimator_.setInputCloud (cloud_downsampled_);
    normal_estimator_.compute (cloud_normals);
    cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
    PCL_INFO("%d normals estimated.", (int)cloud_normals.points.size ());

    // Perform segmentation
    pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
    segmentor_.setInputCloud (cloud_downsampled_);
    segmentor_.setInputNormals (cloud_normals_);
    segmentor_.segment (table_inliers, table_coefficients);

    // Store the table inliers indices.
    table_inliers_.reset (new pcl::PointIndices (table_inliers));
    table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
    if (table_coefficients.values.size () > 3)
        PCL_INFO("Model found with %d inliers: [%f %f %f %f].\n", (int)table_inliers.indices.size (),
                 table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);

    if (table_inliers_->indices.size () == 0)
        return false;

    // Extract the table plane cloud.
    pcl::ExtractIndices<Point> extract_object_indices;
    PointCloud table_cloud;
    extract_object_indices.setInputCloud (cloud_downsampled_);
    extract_object_indices.setIndices (table_inliers_);
    extract_object_indices.filter (table_cloud);
    table_cloud_.reset (new pcl::PointCloud<Point> (table_cloud));

    // Extract the points not being part of the table.
    non_table_cloud_.reset(new PointCloud);
    extract_object_indices.setNegative(true);
    extract_object_indices.filter (*non_table_cloud_);
    extract_object_indices.setNegative(false);

    plane_ = Eigen::Vector4f(table_coefficients.values[0],
                             table_coefficients.values[1],
                             table_coefficients.values[2],
                             table_coefficients.values[3]);

    // Project the table inliers onto the table plane.
    pcl::PointCloud<Point> table_projected;
    project_inliers_.setInputCloud (cloud_downsampled_);
    project_inliers_.setIndices (table_inliers_);
    project_inliers_.setModelCoefficients (table_coefficients_);
    project_inliers_.filter (table_projected);
    table_projected_.reset (new pcl::PointCloud<Point> (table_projected));
    PCL_INFO("Number of projected inliers: %d.\n", (int)table_projected.points.size ());

    // Estimate their convex hull
    std::vector<pcl::Vertices> hull_vertices;
    pcl::PointCloud<Point> table_hull;
    hull_.setInputCloud (table_projected_);
    hull_.reconstruct (table_hull, hull_vertices);
    table_hull_mesh_.reset(new pcl::PolygonMesh);
    pcl::toROSMsg(table_hull, table_hull_mesh_->cloud);
    table_hull_mesh_->polygons = hull_vertices;
    table_hull_.reset (new pcl::PointCloud<Point> (table_hull));

    // Get the objects on top of the table
    pcl::PointIndices cloud_object_indices;
    prism_.setInputCloud (cloud_);
    prism_.setInputPlanarHull (table_hull_);
    prism_.segment (cloud_object_indices);
    PCL_INFO("Number of object point indices: %d.\n", (int)cloud_object_indices.indices.size ());

    // Extract the cloud of objects lying on the table prism.
    pcl::PointCloud<Point> cloud_objects;
    extract_object_indices.setInputCloud (cloud_);
    extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
    extract_object_indices.filter (cloud_objects);
    cloud_objects_.reset (new pcl::PointCloud<Point> (cloud_objects));
    PCL_INFO("Number of object point candidates: %d.\n", (int)cloud_objects.points.size ());

    if (cloud_objects.points.size () == 0)
        return false;

    // Downsample the object points
    pcl::PointCloud<Point> cloud_objects_downsampled;
    grid_objects_.setInputCloud (cloud_objects_);
    grid_objects_.filter (cloud_objects_downsampled);
    cloud_objects_downsampled_.reset (new pcl::PointCloud<Point> (cloud_objects_downsampled));
    PCL_INFO("Number of object point candidates left after downsampling: %d.\n", (int)cloud_objects_downsampled.points.size ());

    // Split the objects into Euclidean clusters
    std::vector< pcl::PointIndices > object_clusters;
    cluster_.setInputCloud (cloud_objects_);
    cluster_.extract (object_clusters);
    PCL_INFO("Number of clusters found matching the given constraints: %d.\n", (int)object_clusters.size ());

    for (size_t i = 0; i < object_clusters.size (); ++i)
    {
        PointCloudPtr object_cloud (new PointCloud());
        extract_object_indices.setInputCloud (cloud_objects_);
        extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (object_clusters[i]));
        extract_object_indices.filter (*object_cloud);
        objects_.push_back(object_cloud);
    }

    return true;
}

// Explicit instanciations.
template class TableTopDetector<pcl::PointNormal>;
template class TableTopDetector<pcl::PointXYZ>;
