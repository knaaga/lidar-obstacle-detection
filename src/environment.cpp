/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "quiz/cluster/cluster.cpp"
//#include "quiz/cluster/kdtree.h"

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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    double groundSlope = 0.0;
    Lidar* lidar = new Lidar(cars, groundSlope);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // render points clouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // // cluster the point clouds
    // // Clustering args: dist tolerance, min points, max points
    // std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    // int clusterId = 0;
    // std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

    // // render the cluster
    // for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "cluster size ";
    //     pointProcessor->numPoints(cluster);
    //     renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
    //     Box box = pointProcessor->BoundingBox(cluster);
    //     renderBox(viewer, box, clusterId);
    //     ++clusterId;
    // }
  
}


void cityBlockSinglePCD(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {
    // High res PCD
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // 1. Filtering and region cropping
    float FilterResolution = 0.3;
    Eigen::Vector4f MinPoint(-10, -6.0, -2, 1);
    Eigen::Vector4f MaxPoint(30, 6.5, 1, 1);
    inputCloud = pointProcessor.FilterCloud(inputCloud, FilterResolution, MinPoint, MaxPoint);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // 2. Segmentation 
    int maxIter = 50;
    float distThresh = 0.2; //0.3 for data_2
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, maxIter, distThresh);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0.5, 0.5, 0.5));

    // Segmentation using Ransac3d - too slow
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.RansacPlaneSegment(inputCloud, maxIter, distThresh);

    // ego car box
    // xmin, ymin, zmin, xmax, ymax, zmax
    Box ego_box = { -1.5, -1.2, -1, 2.6, 1.2, -0.4};
    renderBox(viewer, ego_box, 0, Color(1, 1, 0), 0.75);

    // 3. Clustering
    float ClusterTolerance = 0.5;
    int MinSize = 15; // 10 for data_2
    int MaxSize = 400;
    std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, ClusterTolerance, MinSize, MaxSize);
    int clusterId = 1;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    // render the cluster
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        // pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%3]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 0, 0), 0.5);
        ++clusterId;
    }

}


void simpleHighwayDriver() {
    std::cout << "starting simple highway enviroment" << std::endl;

    // create viewer object
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Simple Highway Viewer"));

    // camera setup
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // call single PCD function
    simpleHighway(viewer);

    // spin the viewer
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}


void singlePCDdriver() {

    std::cout << "Processing single PCD file" << std::endl;

    // create viewer object
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Single PCD Viewer"));

    // camera setup
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // create pointprocessor object on stack
    ProcessPointClouds<pcl::PointXYZI> pointProcessor; 

    // load PCD file
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // call single PCD function
    cityBlockSinglePCD(viewer, pointProcessor, inputCloud);

    // spin the viewer
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}


void streamPCDdriver() {

    std::cout << "Processing stream PCD file" << std::endl;

    // create viewer object
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Stream PCD Viewer"));

    // camera setup
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // create pointprocessor object on stack
    ProcessPointClouds<pcl::PointXYZI> pointProcessor; 

    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    int cnt = 0;
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor.loadPcd((*streamIterator).string());
        cityBlockSinglePCD(viewer, pointProcessor, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce(100);
        cnt++;
    }
}


int main (int argc, char** argv)
{
    simpleHighwayDriver();
    singlePCDdriver();
    streamPCDdriver();  
}
