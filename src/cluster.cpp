/* \author Shantam Bajpai */

#include <chrono>
#include <string>
#include "kdtree.h"
#include <unordered_set>

template<typename PointT>
std::vector<pcl::PointIndices> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, int minClusterSize, int maxClusterSize, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	auto startTime = std::chrono::steady_clock::now();
	// This will return the cluster indices
	std::vector<pcl::PointIndices> clusters;
	
	// This vector will be used to check the points
	// pcl::PointIndices is an std::vector<int>
	pcl::PointIndices queue;

	// A set of vectors to store the explored elements
	std::unordered_set<int> Set;
	
	for (int i = 0; i < cloud->points.size(); i++) {
		queue.indices.push_back(i);
		if (Set.count(i)) {
			queue.indices.clear();
			continue;
		}
		// Kd-Tree Search for extracting nearest neighbors points
		std::vector<int> neighbors = tree->search(cloud->points[i], distanceTol);
		
		for (int ids : neighbors) {
			Set.insert(ids);
			queue.indices.push_back(ids);
		}
		if (queue.indices.size() > minClusterSize && queue.indices.size() < maxClusterSize) {
			clusters.push_back(queue);
		} else {
			queue.indices.clear();
		}
		queue.indices.clear();
	}

	auto endTime = std::chrono::steady_clock::now();
	auto processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout << "Euclidean Clustering took: " << processingTime.count() << "milliseconds and found: " << clusters.size() << " clusters " << std::endl; 
	return clusters;
}