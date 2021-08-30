// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


// The arguments for the filtering cloud is the input cloud, the minimum and maximum points representing region of interest
// and filter resolution
// Eigen::Vector4f is a 4x1 float vector
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxel;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
    
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(filterRes, filterRes, filterRes);
    voxel.filter(*cloudFiltered);

    // Now what we need to do is to crop the region as we only care about a certain distance till which 
    // the car can see
    // Setting it tp true gives the points inside the cropped box
    pcl::CropBox<PointT> box_crop(true);
    box_crop.setMin(minPoint);
    box_crop.setMax(maxPoint);
    box_crop.setInputCloud(cloudFiltered);
    box_crop.filter(*cloud_crop);

    // Remove points from the roof of the car
    // Now pcl::CropBox<PointT> inherits the filter method that returns the std::vector<int> &indices that have
    // been filtered
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_crop);
    roof.filter(indices); // Return the filter indices

    // This will be used to obtain the inlier indices on the roof 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for (int point: indices)
        inliers->indices.push_back(point);
    // Imagine it this way
    // We create a extract indices object and input the cropped point cloud into it
    // After this we set the indices that we want to filter out from the cropped cloud 
    // and then filter the point cloud and save it into point cloud. This will remove the roof points

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_crop);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_crop);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    // Now add the inliers to the plane point cloud
    // Loop over the inlier indices and psh the values intp the road point;
    // plane->std::vector<PointT, Eigen::aligned_allocator<PointT> > points;
    for (int index: inliers->indices) {
        plane->points.push_back(cloud->points[index]);
    }

    // Now we need the obstacles indices
    // Use the extract object that subtracts the plane cloud from the input cloud
    extract.setInputCloud(cloud); // Provide pointer to input dataset
    extract.setIndices(inliers); // Provide a pointer to the vector of indices of the point cloud
    extract.setNegative(true); 
    extract.filter(*obstacles); // calls the filtering method and returns the filtered point cloud
    // Below is the syntax for initializing a std::pair<Ptr,Ptr>.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    // Basically use std::pair<> to hold the segmented point clouds. Obstacle points vs road points
    pcl::SACSegmentation<PointT> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Now we want to fit a plane to the point cloud data. Equation of a place is ax+by+cz+d = 0
    // This object will store the model parameters.
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients());
    segmentation.setModelType(pcl::SACMODEL_PLANE); // Fit a plane
    segmentation.setMethodType(pcl::SAC_RANSAC); // Random Sample Consensus
    segmentation.setDistanceThreshold(distanceThreshold); // Threshold over which point is considered an outlier
    segmentation.setMaxIterations(maxIterations);

    // Segment largest planar component from the point cloud
    segmentation.setInputCloud(cloud);

    // std::shared_ptr<int> x(new int(100))
    // *x = 100; --> Just for the sake of intuition
    // Below to avoid a copy it is being dereferenced
    segmentation.segment(*inliers, *coeffs); // inliers and coeffs are passed as reference
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // One instance of each detected cluster that is saved here. cluster_indices[0] constains all the indices of the first
    // cluster of our point cloud

    std::vector<pcl::PointIndices> cluster_indices;

    // const_iterator points to a onst value and returns as reference a const value and prevents modification
    // of referenced value
    std::vector<pcl::PointIndices>::const_iterator it;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creation of the Kd-Tree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    tree->setInputCloud(cloud);
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    // Basically the intuition here is that first access the vector index
    // After accessing the vector index iterate through pcl::PointIndices.
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end(); ++pit) 
            cloud_cluster->points.push_back((*cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    // Now after saving each cluster indices we need to seperate out the cluster indices to a new point cloud

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}