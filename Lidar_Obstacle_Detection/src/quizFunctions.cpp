#include "sensors/lidar.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include <unordered_set>
#include <chrono>
#include <string>
#include "quiz/cluster/kdtree.h"

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// Steps:
	// a. Create loop for RANSAC iterations
    // b. Pick randomely 3 points to define a plane
	// c. Fit plane to the 3 points
	// d. Measure distance between every point and fitted plane
	// e. If distance is smaller than threshold count it as inlier
	// f. Return indicies of inliers from fitted plane with most inliers



	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers

	
	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliersIndiciesTemp;

		// Randomly sample subset and fit line
		while(inliersIndiciesTemp.size() < 3)
		{
			inliersIndiciesTemp.insert(rand() % cloud->points.size());
		}

		// Get the two points from the random indices (the first two inliers)
		auto it = inliersIndiciesTemp.begin();
		pcl::PointXYZI point1 = cloud->points[*it];
		it++;
		pcl::PointXYZI point2 = cloud->points[*it];
		it++;
		pcl::PointXYZI point3 = cloud->points[*it];

		// Use point1point1 as a reference and define two vectors on the plane v1v1 and v2v2 as follows:
		// Vector v1 travels from point1 to point2
		// Vector v2 travels from point1 to point3
		pcl::PointXYZI v1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
		pcl::PointXYZI v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
		// Calculate the normal vector of the plane using the cross product of v1 and v2
		pcl::PointXYZI norm;
		norm.x = v1.y * v2.z - v1.z * v2.y;
		norm.y = v1.z * v2.x - v1.x * v2.y;
		norm.z = v1.x * v2.y - v1.y * v2.x;


		// Fit line between the two points
		// line equation :: Ax+By+Cz+D=0
		// A = norm.x, B = norm.y, C = norm.z
		// D = -(A*x1 + B*y1 + C*z1) where (x1, y1, z1) is a point on the line (we can use point1)
		// This is the equation of the plane that contains the line
		float A = norm.x;
		float B = norm.y;
		float C = norm.z;
		float D = -(A * point1.x + B * point1.y + C * point1.z);


		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inliersIndiciesTemp.count(index) > 0)
				continue; // Skip already selected inliers

			pcl::PointXYZI point = cloud->points[index];
			// Calculate distance from point to line 
			// Equation:: d=∣A∗x+B∗y+C∗z+D∣/sqrt(A^2 +B^2+C^2)
			float distance  = A*point.x + B*point.y + C*point.z + D;
			distance = fabs(distance) / sqrt(A*A + B*B + C*C);

			if(distance <= distanceTol)
			{
				inliersIndiciesTemp.insert(index);
			}
		}

		// If the number of inliers is greater than the previous best, save this set of inliers
		if (inliersIndiciesTemp.size() > inliersResult.size())
		{
			inliersResult = inliersIndiciesTemp;
		}
	}

	
	return inliersResult;

}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RANSACsegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, float maxIterations, float distanceThreshold)
{
   std::unordered_set<int> inliers;
    inliers = RansacPlane(inputCloud, maxIterations, distanceThreshold);
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < inputCloud->points.size(); index++)
    {
        pcl::PointXYZI point = inputCloud->points[index];
        if(inliers.count(index))
            planeCloud->points.push_back(point);
        else
            obstacleCloud->points.push_back(point);
    }

    return std::make_pair(obstacleCloud, planeCloud);
}



std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster3D(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, float clusterTolerance, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    KdTree* tree = new KdTree;
    for (int i=0; i<inputCloud->points.size(); i++) 
    {
        std::vector<float> point = {inputCloud->points[i].x, inputCloud->points[i].y};
    	tree->insert(point,i); 
    }

    

    std::vector<bool> processed(inputCloud->points.size(), false);

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        if (processed[i]) continue; // Skip already processed points	

        // Start a new cluster
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<float> point = {inputCloud->points[i].x, inputCloud->points[i].y};
        std::vector<int> nearby = tree->search(point, clusterTolerance);
        for (int index : nearby)
        {
            if (!processed[index])
            {
                processed[index] = true; // Mark as processed
                cluster->points.push_back(inputCloud->points[index]); // Add to current cluster
            }
        }
        if (!cluster->points.empty())
        {
            cloudClusters.push_back(cluster); // Add the cluster to the list of clusters
        }
        else
        {
            std::cout << "No points found in cluster for point: " << i << std::endl;
        }
    }

    // Loop over the clusters
    // if clusters intersect by clusterTolerance, merge them
    for (size_t i = 0; i < cloudClusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterA = cloudClusters[i];
        for (size_t j = i + 1; j < cloudClusters.size(); j++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr clusterB = cloudClusters[j];
            bool shouldMerge = false;
            for (const auto& pointA : clusterA->points)
            {
                for (const auto& pointB : clusterB->points)
                {
                    float distance = sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2) + pow(pointA.z - pointB.z, 2));
                    if (distance <= clusterTolerance)
                    {
                        shouldMerge = true;
                        break;
                    }
                }
                if (shouldMerge) break;
            }
            if (shouldMerge)
            {
                *clusterA += *clusterB;
                cloudClusters.erase(cloudClusters.begin() + j);
                j--; // adjust index after erasing
            }
        }
    }

    // Filter clusters based on size
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filteredClusters;
    for (auto& cluster : cloudClusters)
    {
        if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
        {
            filteredClusters.push_back(cluster);
        }
    }

	return filteredClusters;

}

