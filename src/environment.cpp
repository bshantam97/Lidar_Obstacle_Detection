/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0,3,30);
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // Create the point processor object on the heap
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    // Load the PCD File
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputPointCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Filter the input point cloud using Voxel grid downsampling
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPointCloud = pointProcessor->FilterCloud(inputPointCloud, 0.2 , 
                                                              Eigen::Vector4f (-15, -15, -15, 1), Eigen::Vector4f (15, 15, 15, 1));

    // Segment the plane and the obstacle
    // This will return the indices for the obstacle and the plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segPlane = pointProcessor->SegmentPlane(filteredPointCloud,1000,0.2); 
    renderPointCloud(viewer, segPlane.first, "obstacle", Color(1,0,0));
    renderPointCloud(viewer, segPlane.second, "Plane", Color(0,1,0));
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
    // simpleHighway(viewer);
    cityBlock(viewer);
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}