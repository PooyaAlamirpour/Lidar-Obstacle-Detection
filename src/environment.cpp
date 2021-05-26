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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar(new Lidar(cars, 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->scan();
    
	// TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_processor ;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_processor.SegmentPlane(input_cloud, 100, 0.2);
    
    Color pcd_color3(1,1,1);
    renderPointCloud(viewer, segment_cloud.second, "road", pcd_color3);    
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = point_processor.Clustering(segment_cloud.first, 1.0, 3, 30);
	
	// Define cluster ID
    int cls_id = 0;
    std::vector<Color> cluster_color = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(auto single_cluster : cloud_clusters)
    {
        Box box = point_processor.BoundingBox(single_cluster);
        renderBox(viewer, box, cls_id, cluster_color[cls_id], 1.0);
        point_processor.numPoints(single_cluster);
        renderPointCloud(viewer, single_cluster, "cluster" + std::to_string(cls_id), cluster_color[cls_id]);
        ++cls_id;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> point_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //filtered pointCloud
  input_cloud = point_processor.FilterCloud(input_cloud, 0.4, Eigen::Vector4f(-10, -6.5, -2, 1), Eigen::Vector4f(30, 6.5, 1, 1));
 
  //segmetation of the pointCloud to two segments the ground and the other objects
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = point_processor.SegmentPlane(input_cloud, 40, 0.3);
  //clustering the objects in plane of objects returned from segmentPlane function
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clsuters = point_processor.Clustering(segment_cloud.first, 0.5, 10, 140);  
  //rendering clusters
  int cluster_id = 0;
  std::vector<Color> cluster_color = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
  for(auto cluster : clsuters)
  {
    renderPointCloud(viewer, cluster, "cluster"+std::to_string(cluster_id), cluster_color[cluster_id]);
    
    Box box = point_processor.BoundingBox(cluster);
    renderBox(viewer, box, cluster_id, cluster_color[0], 1.0);
    
    point_processor.numPoints(cluster);
    ++cluster_id;
  }

  //rendering the ground plane
  renderPointCloud(viewer, segment_cloud.second, "ground", Color(0,1,0));
  
  //renderPointCloud(viewer,input_cloud,"inputCloud");
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
    //viewer gives you the basic scene 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    //simpleHighway(viewer);
    cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointprocessorI(new ProcessPointClouds<pcl::PointXYZI>());  
    std::vector<boost::filesystem::path> stream = pointprocessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointprocessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, *pointprocessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}