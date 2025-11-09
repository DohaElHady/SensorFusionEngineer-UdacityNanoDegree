/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers


	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliersIndiciesTemp;

		// Randomly sample subset and fit line
		while(inliersIndiciesTemp.size() < 2)
		{
			inliersIndiciesTemp.insert(rand() % cloud->points.size());
		}

		// Get the two points from the random indices (the first two inliers)
		auto it = inliersIndiciesTemp.begin();
		pcl::PointXYZ point1 = cloud->points[*it];
		it++;
		pcl::PointXYZ point2 = cloud->points[*it];

		// Fit line between the two points
		// line equation :: Ax+By+C=0 :: (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0
		float A = point1.y - point2.y;
		float B = point2.x - point1.x;	
		float C = point1.x * point2.y - point2.x * point1.y;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inliersIndiciesTemp.count(index) > 0)
				continue; // Skip already selected inliers

			pcl::PointXYZ point = cloud->points[index];
			// Calculate distance from point to line 
			// Equation:: ∣Ax+By+C∣/sqrt(A^2 +B^2)
			float distance  = A*point.x + B*point.y + C;
			distance = fabs(distance) / sqrt(A*A + B*B);

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



std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		pcl::PointXYZ point1 = cloud->points[*it];
		it++;
		pcl::PointXYZ point2 = cloud->points[*it];
		it++;
		pcl::PointXYZ point3 = cloud->points[*it];

		// Use point1point1 as a reference and define two vectors on the plane v1v1 and v2v2 as follows:
		// Vector v1 travels from point1 to point2
		// Vector v2 travels from point1 to point3
		pcl::PointXYZ v1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
		pcl::PointXYZ v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
		// Calculate the normal vector of the plane using the cross product of v1 and v2
		pcl::PointXYZ norm;
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

			pcl::PointXYZ point = cloud->points[index];
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	
	bool use3D = true; // Change to true to use 3D data

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	if(use3D)
		cloud = CreateData3D();
	else
		cloud = CreateData();
	
	std::unordered_set<int> inliers;

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	if(use3D)
		inliers = RansacPlane(cloud, 100, 0.25);
	else
		inliers = RansacLine(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
