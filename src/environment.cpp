/* \author Shantam Bajpai */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac.cpp"
#include "cluster.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
    
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// Here viewer gets passed as a reference
// Basically means that  any changes to the viewer in the body of renderRays
// directly affects the viewer outside the function scope. 
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // We are creating the lidar object on the heap as it can hold large amounts of data
    // Stack has limited size 
    Lidar *lidar = new Lidar(cars,0);
    // Basically create a scan object 
    // This is a boost shared pointer
    // The below rendered rays are very sparse and we need to increase its resolution
    // const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderRays(viewer, lidar->position, cloud); // Render rays as line segments
    // renderPointCloud(viewer, cloud, "Point Cloud");
    // TODO:: Create point processor
    // This part is for detecting the parts of the cloud representing the cars in traffic
    // What we want to do is locate obstacles in our scene and things that are not obstacles
    // Any free space like a flat road is not an obstacle
    // PLANAR SEGMENTATION. Uses RANSAC Algorithm
    // Initialization on the heap
    
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloud, 1000, 0.2);
    // renderPointCloud(viewer, segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));
    // Get the cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.4 ,100,5000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size ";
        Box box = pointProcessor->BoundingBox(cluster);
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

// Here we are passing the input cloud as a const reference as we just need it as an input
// we are not modifying its value
// Initialize the process point cloud object on the heap
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {

    // Filter the input point cloud using Voxel grid downsampling
    // So when you open the window if you move towards the right that is the negative y axis
    // The max point is the top left corner and the minimum is the bottom right corner from your perspective
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPointCloud = pointProcessor->FilterCloud(inputCloud, 0.2, 
                                                              Eigen::Vector4f (-20, -7, -10, 1), Eigen::Vector4f (20, 8, 10, 1));

    // Segment the plane and the obstacle
    // This will return the indices for the obstacle and the plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segPlane = pointProcessor->SegmentPlane(filteredPointCloud,1000,0.2); 
    
    // Create the cluster object
    // In this we input the obstacle points only for clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segPlane.first, 0.4, 100, 5000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr clusters: cloudClusters) {
        std::cout << " Cluster size is ";
        pointProcessor->numPoints(clusters);
        Box box = pointProcessor->BoundingBox(clusters);
        renderPointCloud(viewer, clusters, "obstacle Cloud" + std::to_string(clusterId), colors[clusterId % 3]); // Dividing by 3 so that remained is always [0,2]
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
    // renderPointCloud(viewer, segPlane.first, "obstacle", Color(1,0,0));
    renderPointCloud(viewer, segPlane.second, "Plane", Color(0,1,0));
}

void ObstacleDetection(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud, ProcessPointClouds<pcl::PointXYZI> *pointProcessor) {
    // The first step is to filter the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPointCloud = pointProcessor->FilterCloud(inputCloud, 0.15, 
                                                                Eigen::Vector4f(-20, -6, -6, 1), Eigen::Vector4f(20,7,6,1));
    // Now we use RANSAC to Segment out our filtered cloud
    // This returns the inliers indices
    // Ransac takes the number of iterations and the distance tolerance for fitting plane

    std::unordered_set<int> inliers = Ransac<pcl::PointXYZI>(filteredPointCloud, 100, 0.2);
    // Create pcl::PointCloud objects for plane and obstacles
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZI>());

    for (int index = 0; index < filteredPointCloud->points.size(); index++) {
        // What this is doing is that it is returning the XYZI point values at std::vector points
        pcl::PointXYZI point = filteredPointCloud->points[index];
        
        // Now we will check whether the index exists in the set or not
        if (inliers.count(index)){
            plane->points.push_back(point);
        } else {
            obstacle->points.push_back(point);
        }
    }

    // Now lets construct the Kd-Tree using the filtered point cloud
    KdTree<pcl::PointXYZI> *tree = new KdTree<pcl::PointXYZI>();

    // Now we construct our KdTree and insert points into it
    for (int i = 0; i < obstacle->points.size(); i++) {
        tree->insert(obstacle->points[i], i);
    }

    // Now after constructing the KdTree we can pass it to the Euclidean clustering function
    std::vector<pcl::PointIndices> clusters = euclideanCluster<pcl::PointXYZI>(obstacle, tree, 28, 500, 3.5);
    // Render the clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (pcl::PointIndices cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::PointXYZI inherits this struct that stores the point and intensity information
        pcl::_PointXYZI p;
        for (auto indice:cluster.indices) {
            p.x = obstacle->points[indice].x;
            p.y = obstacle->points[indice].y;
            p.z = obstacle->points[indice].z;
            clusterCloud->points.push_back(pcl::PointXYZI(p));
        }
        Box box = pointProcessor->BoundingBox(clusterCloud);
        renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId), colors[clusterId%3]);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

    // renderPointCloud(viewer, obstacle, "obstacle", Color(1,0,0));
    renderPointCloud(viewer, plane, "plane", Color(0,1,0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>;
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

    // Point to the beginning of the dataset
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    // inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce();
    // }

    while (!viewer->wasStopped ())
    {
        // Clear the viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        ObstacleDetection(viewer, inputCloudI, pointProcessorI);
        // cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;

        // I think basically so that the viewer goes on for an infinite period of time
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        // Controls the frame rate
        viewer->spinOnce();
    } 
}