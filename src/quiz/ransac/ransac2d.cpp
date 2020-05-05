/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include<math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 50; i++)
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
  	int numOutliers = 200;
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	//Final Set of Inliers
	std::unordered_set<int> inliersResult;
	
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
        for (int n = 0; n <= maxIterations; n++ ){
		
		//Temporary set of inliers
		std::unordered_set<int> tempResult;
		
		//Randomly generation three numbers between zero and cloud.Points.size()
		int firstnum = rand()%(cloud->points.size());
		int secondnum = rand()%(cloud->points.size());
		//int thirdnum = rand()%(cloud->points.size());
		//Avoid Generating two same numbers

		if (firstnum == secondnum){

			n--;
			break; 		
		}

		//Coordiante of First Number	
        	float x1 = cloud->points[firstnum].x;
		//float y1 = cloud->points[firstnum].y;
		float z1 = cloud->points[firstnum].z;

		//Coordinate of Second Number
		float x2 = cloud->points[secondnum].x;
		//float y2 = cloud->points[secondnum].y;
		float z2 = cloud->points[secondnum].z;

		//Coordinate of third Number 
		//float x3 = cloud->points[thirdnum].x;
		//float y3 = cloud->points[thirdnum].y;
		//float z3 = cloud->points[thirdnum].z;

		//line fitting 
		float A = z1-z2;
		float B = x2-x1;
		float C = x1*z2-x2*z1;
		//float D = -(A*x1 + B*y1 + C*z1);
		
		float dist;

		//Looping through all the points 
		for (int m = 0; m < cloud->points.size(); m++){
			
			float x4 = cloud->points[m].x;
			//float y4 = cloud->points[m].y;
			float z4 = cloud->points[m].z;

			dist = abs(A*x4 + B*z4 + C)/sqrt(A*A + B*B);
			

			if (dist <= distanceTol){

				tempResult.insert(m);
			} 	

		}
		
		if (tempResult.size() > inliersResult.size()){

			inliersResult = tempResult;	
		}

	
	}


	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance rguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 1.0);

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
