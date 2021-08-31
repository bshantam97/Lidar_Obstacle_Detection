/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

// #include "render/render.h"
#include <unordered_set>
// #include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "processPointClouds.cpp"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <chrono>

// pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
// {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//   	// Add inliers
//   	float scatter = 0.6;
//   	for(int i = -5; i < 5; i++)
//   	{
//   		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
//   		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
//   		pcl::PointXYZ point;
//   		point.x = i+scatter*rx;
//   		point.y = i+scatter*ry;
//   		point.z = 0;

//   		cloud->points.push_back(point);
//   	}
//   	// Add outliers
//   	int numOutliers = 10;
//   	while(numOutliers--)
//   	{
//   		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
//   		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
//   		pcl::PointXYZ point;
//   		point.x = 5*rx;
//   		point.y = 5*ry;
//   		point.z = 0;

//   		cloud->points.push_back(point);

//   	}
//   	cloud->width = cloud->points.size();
//   	cloud->height = 1;

//   	return cloud;

// }

// pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
// {
// 	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
// 	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
// }


// pcl::visualization::PCLVisualizer::Ptr initScene()
// {
// 	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
// 	viewer->setBackgroundColor (0, 0, 0);
//   	viewer->initCameraParameters();
//   	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
//   	viewer->addCoordinateSystem (1.0);
//   	return viewer;
// }

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	// TODO: Fill in this function
	std::unordered_set<int> inliersResult;
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	// Makes use of the internal clock and keeps chaning the seed 
	// so that the sequence of number generated is not the same
	// Sets seed for rand()
	srand(time(NULL)); // time in the ctime library
	int size = cloud->points.size(); // Will give us the size of the vector

	while (maxIterations--) {
		std::unordered_set<int> inliers;
		// Randomly sample 2 points
		while (inliers.size() < 3)
			inliers.insert(rand() % size);
		
		// Returns a pointer to the beginning of the unordered_set
		auto iterator = inliers.begin();
		auto x1 = cloud->points[*iterator].x;
		auto y1 = cloud->points[*iterator].y;
		auto z1 = cloud->points[*iterator].z;
		*iterator++;
		auto x2 = cloud->points[*iterator].x;
		auto y2 = cloud->points[*iterator].y;
		auto z2 = cloud->points[*iterator].z;
		*iterator++;
		auto x3 = cloud->points[*iterator].x;
		auto y3 = cloud->points[*iterator].y;
		auto z3 = cloud->points[*iterator].z;

		// Fit a line through these points
		// Calculate the line coefficients A, B and C
		auto A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		auto B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		auto C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		auto D = -(A*x1 + B*y1 + C*z1);

		// Now that we have the line coefficients we can iterate through all the points
		// To compute the distance from this line. 
		for (int index = 0; index < size; index++) {

			// Basically we check that whether the randomly sampled index is part of our line or not
			// If it is we continue if it is not we proceed with the following steps.
			//.count() checks for uniqueness of value.
			if (inliers.count(index)) {
				continue;
			}
			auto x = cloud->points[index].x;
			auto y = cloud->points[index].y;
			auto z = cloud->points[index].z;

			auto distance =  std::abs(A*x + B*y + C*z + D) 
						/ std::sqrt(std::pow(A,2) + std::pow(B,2) + std::pow(C,2));

			if (distance < distanceTol) {
				inliers.insert(index);
			}
		}

		// Basically out here if my inliers size is greater than the previously store
		// inlier indices replace it.
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac Took " << elapsedTime.count() << " milliseconds " << std::endl;
	return inliersResult;
}

// int main ()
// {

// 	// Create viewer
// 	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

// 	// Create data
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
// 	// Testing inbuilt Ransac function
// 	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
// 	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
// 	std::vector<int> inliersRansac;
// 	auto startTime = std::chrono::steady_clock::now();
// 	ransac.setDistanceThreshold (.01);
//     ransac.computeModel();
//     ransac.getInliers(inliersRansac);
// 	auto endTime = std::chrono::steady_clock::now();
// 	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
// 	std::cout << "Inbuilt Ransac Took " << elapsedTime.count() << " milliseconds " << std::endl;
	
// 	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
// 	// Custom Ransac Implementation
// 	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

// 	for(int index = 0; index < cloud->points.size(); index++)
// 	{
// 		pcl::PointXYZ point = cloud->points[index];
// 		// Basically as we can only have a single occurence of a key this is checking 
// 		// whether the index is true
// 		if(inliers.count(index))
// 			cloudInliers->points.push_back(point);
// 		else
// 			cloudOutliers->points.push_back(point);
// 	}


// 	// Render 2D point cloud with inliers and outliers
// 	if(inliers.size())
// 	{
// 		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
//   		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
// 	}
//   	else
//   	{
//   		renderPointCloud(viewer,cloud,"data");
//   	}
	
//   	while (!viewer->wasStopped ())
//   	{
//   	  viewer->spinOnce ();
//   	}
  	
// }