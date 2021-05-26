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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	
	
	// For max iterations 
	for(int i=0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;
	
		// Randomly sample subset and fit line
		while(inliers.size() < 2)
		{
			int index = (rand() % cloud->points.size());
			inliers.insert(index);
		}
		auto itr = inliers.begin();
		pcl::PointXYZ point1 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ point2 = cloud->points[*itr];
		
		// Measure distance between every point and fitted line
		//m = (DelatY)/(DeltaX)
		float m = (point1.y - point2.y) / (point1.x -point2.x); 
		float y_intercept = point1.y - m*point1.x;

		for(int index=0; index < cloud->points.size(); index++) 
		{
			if(inliers.count(index)>0)
				continue;
			
			float dist = std::fabs(m*(cloud->points[index].x) - cloud->points[index].y + y_intercept) / std::sqrt(pow(-1,2) + pow(m, 2));
			
			// If distance is smaller than threshold count it as inlier
			
			if(dist <= distanceTol)
			{
				inliers.insert(index);
			}

		}
		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	 
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for(int i=0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		int num_sample = 3;
		
		while(inliers.size() < num_sample) 
		{
			int index = ( rand() % cloud->points.size() );
			inliers.insert(index);
		}
		auto itr = inliers.begin();
		pcl::PointXYZ a = cloud->points[*itr];
		itr++;
		pcl::PointXYZ b = cloud->points[*itr];
		itr++;
		pcl::PointXYZ c = cloud->points[*itr];
		
		/*Ax + By + Cz + D = 0*/
		float A = (b.y - a.y) * (c.z - a.z) - (c.y - a.y) * (b.z - a.z);
		float B = (b.z - a.z) * (c.x - a.x) - (c.z - a.z) * (b.x - a.x);
		float C = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
		float D = -(A*a.x + B*a.y + C*a.z);
		
		for(int index=0; index < cloud->points.size(); index++) 
		{
			if(inliers.count(index)>0)
			{
				continue;
			}
			float dist = std::abs(A*cloud->points[index].x + B*cloud->points[index].y + C*cloud->points[index].z) / std::sqrt(A*A + B*B + C*C);
			if(dist <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	 
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac2D(cloud, 20, 0.5);

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
