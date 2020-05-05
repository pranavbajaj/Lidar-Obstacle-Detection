// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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

//Filtering Cloud: Voxel Grid filtering and Cropping 3D data. 
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //Voxel Grid filtering 
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr cloudfiltered (new pcl::PointCloud<PointT>);
    sor.filter(*cloudfiltered);

    //Region of Interest
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloudfiltered);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    roi.filter(*cloudRegion);
  
    
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point: (indices)){
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloudRegion);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

//Separate Cloud helper function
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);

    typename pcl::PointCloud<PointT>::Ptr obscloud (new  pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planecloud (new  pcl::PointCloud<PointT>());

    extract.setNegative(false);
    extract.filter (*planecloud);
    // Alternate way
    //for (int index : inliers->indices)
    //     planecloud->points.puch_back(cloud->points[index]);

    extract.setNegative(true);
    extract.filter (*obscloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obscloud, planecloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool segmentation_builtin_function)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    //Using Builtin function for segmentation 
    if (segmentation_builtin_function){
   
    
    	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    
    	pcl::SACSegmentation<PointT> seg;
    	seg.setOptimizeCoefficients (true);
    	seg.setModelType (pcl::SACMODEL_PLANE);
    	seg.setMethodType (pcl::SAC_RANSAC);
    	seg.setMaxIterations (maxIterations);
    	seg.setDistanceThreshold (distanceThreshold);

    	seg.setInputCloud(cloud);
    	seg.segment (*inliers, *coefficients);

    }  
    // My Code For Segmentation. 	
    else {	

	// For max iterations 
        for (int n = 0; n <= maxIterations; n++ ){
		
		//Temporary set of inliers
                pcl::PointIndices::Ptr tempResult (new pcl::PointIndices ());
		
		//Randomly generation three numbers between zero and cloud.Points.size()
		int firstnum = rand()%(cloud->points.size());
		int secondnum = rand()%(cloud->points.size());
		int thirdnum = rand()%(cloud->points.size());
		//Avoid Generating two same numbers

		if (firstnum == secondnum || secondnum == thirdnum || firstnum == thirdnum){

			n--;
			break; 		
		}

		//Coordiante of First Number	
        	float x1 = cloud->points[firstnum].x;
		float y1 = cloud->points[firstnum].y;
		float z1 = cloud->points[firstnum].z;

		//Coordinate of Second Number
		float x2 = cloud->points[secondnum].x;
		float y2 = cloud->points[secondnum].y;
		float z2 = cloud->points[secondnum].z;

		//Coordinate of third Number 
		float x3 = cloud->points[thirdnum].x;
		float y3 = cloud->points[thirdnum].y;
		float z3 = cloud->points[thirdnum].z;

		//Plane fitting 
		float A = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		float B = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		float C = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		float D = -(A*x1 + B*y1 + C*z1);
		
		float dist;

		//Looping through all the points 
		for (int m = 0; m < cloud->points.size(); m++){
			
			float x4 = cloud->points[m].x;
			float y4 = cloud->points[m].y;
			float z4 = cloud->points[m].z;

			dist = abs(A*x4 + B*y4 + C*z4 + D)/sqrt(A*A + B*B + C*C);
			

			if (dist < distanceThreshold){

				tempResult->indices.push_back(m);
			} 	

		}
		
		if (tempResult->indices.size() > inliers->indices.size()){

			inliers = tempResult;	
		}

	
	}

    }
    if(inliers->indices.size() == 0){

	std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// Cluster Helper funtion. 
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, KdTree* tree, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, float distanceTol){
	
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearby_points = tree->search(points[indice], distanceTol);

	for(int id : nearby_points){
		
		if (!processed[id]){
			clusterHelper(id, tree, points, cluster, processed, distanceTol);
		}
	}
}

// Euclidean Cluster.
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters_idx;

	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size()){
		
		if (processed[i]){
			i++;
			continue;
		}
		
		std::vector<int> cluster;
		clusterHelper(i, tree, points, cluster, processed, distanceTol);
		clusters_idx.push_back(cluster);
		i++;
		
		
	}
	return clusters_idx;

}

//Clustering 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool clustering_builtin_function)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    if (clustering_builtin_function){
	
    
    	// Clustering Method
    	typename pcl::search::KdTree<PointT>::Ptr tree (new  pcl::search::KdTree<PointT>);
    	tree->setInputCloud(cloud);
    
    	std::vector<pcl::PointIndices> cluster_indices;
    	typename pcl::EuclideanClusterExtraction<PointT> ec;
    
    	// EuclideanClustering hyperparameters
    	ec.setClusterTolerance (clusterTolerance);
    	ec.setMinClusterSize(minSize);
    	ec.setMaxClusterSize(maxSize);
    	ec.setSearchMethod(tree);
    	//Extracting the Indices 
    	ec.setInputCloud(cloud);
    	ec.extract(cluster_indices);

   	for (pcl::PointIndices getIndices: cluster_indices){

		typename pcl::PointCloud<PointT>::Ptr cloudcluster (new pcl::PointCloud<PointT>);
	
		for (int index: getIndices.indices){
		
			cloudcluster->push_back(cloud->points[index]);
		}

		cloudcluster->width = cloudcluster->points.size();
		cloudcluster->height = 1;
		cloudcluster->is_dense = true;

		clusters.push_back(cloudcluster);
   	}
    }

    else {
    	std::vector<std::vector<float>> Points;
    	KdTree* tree = new KdTree;

    	for(int i = 0; i < cloud->points.size(); i++){

		Points.push_back({(float)cloud->points[i].x, (float)cloud->points[i].y});
		tree->insert(Points[i],i); 
    	}
    
    	std::vector<std::vector<int>> clusters_idx = euclideanCluster(Points, tree, clusterTolerance);

    	for (std::vector<int> group_idx: clusters_idx){
	
			typename pcl::PointCloud<PointT>::Ptr cloudcluster (new pcl::PointCloud<PointT>);

			for (int index: group_idx){
				cloudcluster->points.push_back(cloud->points[index]);
			}
			cloudcluster->width = cloudcluster->points.size();
			cloudcluster->height = 1;
			cloudcluster->is_dense = true;

			if (cloudcluster->points.size() >= minSize && cloudcluster->points.size() <= maxSize){
				clusters.push_back(cloudcluster);
			}
		
		}			
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

//Bounding Box
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

//Bounding BoxQ
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
	//Computing the Centroid, normalized covariance and Eigen Vector
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	
	//Transform the original cloud to the origin where the pricipal components corresponds to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

	//Get the minimum and maximum points of the transformed cloud.
	PointT minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
	
	BoxQ boxQ;
	
	boxQ.bboxQuaternion = eigenVectorsPCA;
	boxQ.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
	
	boxQ.cube_length = maxPoint.x-minPoint.x;
	boxQ.cube_width = maxPoint.y-minPoint.y;
	boxQ.cube_height = maxPoint.z-minPoint.z;	
	
	return boxQ;
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
