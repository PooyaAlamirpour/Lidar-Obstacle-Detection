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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now(); //register the time of the beggining of the filtering process

    /*filter the input point cloudby voxel grid to keep only one point in each voxel cube its dimension is 0.4*04*0.4(0.4=>filter resolution)*/
    typename pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    
    vg.setInputCloud(cloud); //point cloud to be filtered
    vg.setLeafSize(filterRes, filterRes, filterRes); //setting the resolution of the voxel cube
    vg.filter(*filtered_cloud); //the output filtered point cloud

    
    /*croping the box of the road by excluding the two sides of the road in the point cloud and keep only the road and all its containings*/
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>());
    
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint); //the minimum point of the box
    region.setMax(maxPoint); //the maximum point of the box
    region.setInputCloud(filtered_cloud); //the input cloud you want to crop the box from
    region.filter(*cloud_region); //outup cloud ofthe cropped box

    /*the next line to filter the cloud from the scattered point because of the roof of the ego_car*/
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices); //the return of this value is the indcies of the points of the roof of the ego_car

    /*extracting the points of the roof of the go car from the cropped box of the road*/
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    for(int index : indices) //pushing the indcies of the unwanted points to pointer to indcies
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> exc;
    exc.setInputCloud(cloud_region);
    exc.setIndices(inliers);
    exc.setNegative(true);
    exc.filter(*cloud_region);

    /*calculating the the time has been take to filter the cloud*/
    auto endTime = std::chrono::steady_clock::now(); //register the time of the end of the filtering process
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obst_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    /*extracting the ground plan from the whole cloud*/   
    for(auto in : inliers->indices)
        plane_cloud->points.push_back(cloud->points[in]);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obst_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now(); 
    
    /*using the RANSAC approach ro segment the cloud into two planes the ground plane and any thing else as the other plane*/
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coff(new pcl::ModelCoefficients());
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coff); 
    
    if(inliers->indices.size() == 0)
        std::cout << "could not estimate a plannar  model for the given data set" << std::endl;
    
    /*calculating the the time has been take to segment the cloud*/
    auto endTime = std::chrono::steady_clock::now(); //register the time of the end of the segmentation process
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    /*passing to separateCloud function the indcies of the ground plane points and the whole point cloud 
    and return two pointers to pointCloud one of the ground and other of every thing else in cloud*/
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now(); 

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
	
	//minimum number of points
    ec.setMinClusterSize(minSize); 
	
	//maximum number of points
    ec.setMaxClusterSize(maxSize); 
	
    ec.setSearchMethod(tree); 
    ec.setInputCloud(cloud);
	
	//the output is vector
    ec.extract(cluster_indices); 
    for(auto cluster_index : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        for(auto index : cluster_index.indices)
            cloud_cluster->points.push_back(cloud->points[index]);
    
        cloud_cluster->width  = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

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