/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quizFunctions.cpp"


bool useRealPCD = true;
bool useRealPCDStream = true;
bool useQuizFunctions = true;
/********************************** Real PCD *********************************/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // Filter the cloud
    // Use voxel grid point reduction and region based filtering
    // Voxel filtering reduces the number of points by averaging points within each voxel (3D grid cell) into a single point
    // Bigger voxels means more reduction in number of points and less details; this is controled by the filterRes parameter
    // Region based filtering removes points outside a defined region of interest to focus on relevant areas; this is done using minPoint and maxPoint to define a 3D box
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-10, -6, -2, 1), Eigen::Vector4f (30, 7, 1, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    // Segment the cloud into obstacles and plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentResult = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    // renderPointCloud(viewer,segmentResult.second,"planeCloud",Color(0,1,0));
    // renderPointCloud(viewer,segmentResult.first,"obstCloud",Color(1,0,0));

    // Cluster the obstacle cloud
    // Adjust the distance tolerance, min and max cluster size for better clustering results
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentResult.first, 0.5, 10, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size "<< cluster->points.size() << std::endl;
        renderPointCloud(viewer,cluster,"obstCloudCluster"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

}


/************************** Real PCD Project *********************************/

void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    
    // Filter the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-10, -6, -1.9, 1), Eigen::Vector4f (30, 6, 1, 1));

    if (useQuizFunctions)
    {
        // Segment the cloud into obstacles and plane using quiz RANSAC function
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentResult = RANSACsegmentPlane(filterCloud, 180, 0.3);

        renderPointCloud(viewer,segmentResult.second,"planeCloud",Color(0,1,0));
        // renderPointCloud(viewer,segmentResult.first,"obstCloud",Color(1,0,0));

        // Cluster the obstacle cloud using quiz euclidean clustering function
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = euclideanCluster3D(segmentResult.first, 0.8, 10, 500);
        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; // red, yellow, blue
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size "<< cluster->points.size() << std::endl;
            renderPointCloud(viewer,cluster,"obstCloudCluster"+std::to_string(clusterId),colors[clusterId%colors.size()]);
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
        }

    }
    else{
         // Segment the cloud into obstacles and plane
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentResult = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
        renderPointCloud(viewer,segmentResult.second,"planeCloud",Color(0,1,0));

        // Cluster the obstacle cloud
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentResult.first, 0.5, 10, 500);
        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; // red, yellow, blue
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size "<< cluster->points.size() << std::endl;
            renderPointCloud(viewer,cluster,"obstCloudCluster"+std::to_string(clusterId),colors[clusterId%colors.size()]);
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
        }
    }
   

}



/********************************* Sample Environment *********************************/
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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;        // if renderScene is false, we will not render the highway or the cars
    bool renderPC = true;          // if renderPC is false, we will not render the point cloud
    bool renderPCRay = false;        // if renderPCRay is false, we will not render the rays from the lidar sensor
    bool renderSegCloud = false;    // if renderSegCloud is false, we will not render the segmented point cloud
    bool renderClusters = false;    // if renderClusters is false, we will not render the clustered point clouds
    bool renderBbox = false;        // if renderBbox is false, we will not render the bounding boxes around the clusters


    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // Created on the heap using a pointer to the Lidar class to save stack memory
    Lidar* lidar = new Lidar(cars, 0); // ground slope of 0 radians :: horizontal ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // render point cloud and rays
    if(renderPC)
    {
        // renderPointCloud(): takes the viewer, the point cloud, and a name for the point cloud 
        // we can have multiple point clouds in the viewer each with a unique name
        renderPointCloud(viewer, inputCloud, "inputCloud"); 
    }
    if(renderPCRay)
    {
        // renderRays(): takes the viewer, the lidar position, and the point cloud
        renderRays(viewer, lidar->position, inputCloud);
    }


    // TODO:: Create point processor
    // Create a point cloud processer on the stack using the template class directly, or on the heap using a pointer to the template class
    // ProcessPointClouds<pcl::PointXYZ> is a template class that takes a point type as a template parameter
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentResult = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles = segmentResult.first; // obstacles point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane = segmentResult.second; // plane point cloud  
    // render the segmented plane and obstacles
    if(renderSegCloud)
    {
        renderPointCloud(viewer, obstacles, "obstacles", Color(1,0,0)); // red for obstacles
        renderPointCloud(viewer, plane, "plane", Color(0,1,0)); // green for plane
    }


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(obstacles, 1.0, 3,20);
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    int clusterId =0;
    if(renderClusters)
    {
        for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            renderPointCloud(viewer, cluster, "obsCluster"+std::to_string(clusterId), colors[clusterId%colors.size()]);

            Box boundingBox= pointProcessor.BoundingBox(cluster);
            if(renderBbox)
                renderBox(viewer,boundingBox,clusterId);


            ++clusterId;
        }
            
    }
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
    if (useRealPCD)
    {
        // cityBlock(): creates a 3D viewer and displays the city block point cloud data
        if (useRealPCDStream)
        {
            ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
            std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
            auto streamIterator = stream.begin();
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
            while (!viewer->wasStopped ())
            {
                // Clear viewer
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                // Load pcd and run obstacle detection process
                inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
                cityBlockStream(viewer, pointProcessorI, inputCloudI);

                streamIterator++;
                if(streamIterator == stream.end())
                    streamIterator = stream.begin();

                viewer->spinOnce ();
            }
        }
        else
        {
            cityBlock(viewer);
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce ();
            } 
        } 
    }
    else
    {
        simpleHighway(viewer);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        } 
    }
}