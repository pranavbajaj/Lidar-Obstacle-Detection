/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "render/box.h"

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
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, inputcloud, "Point Cloud");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* process_pc = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = process_pc->SegmentPlane(inputcloud, 100, 0.2, true);
    
    //renderPointCloud(viewer, segmentCloud.first, "obstcloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud");

    bool render_cluster = true;
    bool render_box = true;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = process_pc->Clustering(segmentCloud.first, 1.0, 10, 150, true);
    
    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
	
	if (render_cluster){
		std::cout << "cluster size ";
		process_pc->numPoints(cluster);
		renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterID), colors[clusterID]);
	}
	if (render_box){
		Box box = process_pc->BoundingBox(cluster);
		renderBox(viewer, box, clusterID);	
	}
	++clusterID;
    }
    
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
	

	// Parameters 
	bool clustering_builtin_function = false;
	bool segmentation_builtin_function = false;
	bool render_cluster = true;
    	bool render_box = true;
	bool BOXQ = false;
	
	//Initializing the processPointClouds 
	//ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	//pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
	//renderPointCloud(viewer,inputCloud,"inputCloud");

	//Filtering the data for increasing the effeciency of data processing. 
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10,-5,-2,1), Eigen::Vector4f (30,7,1,1));
        //renderPointCloud(viewer, filterCloud, "filterCloud");

	//Segmenting the cloud into road and obstacles. 
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 10, 0.2, segmentation_builtin_function);
	//renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
	renderPointCloud(viewer, segmentCloud.second, "planCloud", Color(0,1,0));
	
	//Bounding Box around the egocar top to find out the xyz range of roof top points 	
	/* 
	Box box;
	box.x_min = -1.5;
        box.x_max = 2.6;
	box.y_min = -1.7;
	box.y_max = 1.7;
	box.z_min = -1;
	box.z_max = 0.4;
        int clusterID = 1;
	renderBox(viewer, box, clusterID, Color(0.5,0,0.5));
	*/


	//Clustring the object cloud 
	float clusterTolerance = 0.5;
	int minSize = 10;
	int maxSize = 500;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusteredCloud = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize, clustering_builtin_function);
   	


	int clusterID = 0;
    	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

	for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusteredCloud){
		
		int clusterID_1 = clusterID%3;
		if (render_cluster){
			std::cout << "cluster size ";
			pointProcessorI->numPoints(cluster);
			renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterID), colors[clusterID_1]);
		}

		if (render_box && BOXQ){
			BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);
			renderBox(viewer, boxQ, clusterID);	
		}
		else if (render_box){
			Box box = pointProcessorI->BoundingBox(cluster);
			renderBox(viewer, box, clusterID);	
		}
		
		++clusterID;
    
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
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;     
 
    while (!viewer->wasStopped ())
    {
	//Clear viewer
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	
	//Load pcd and run obstacle detection process
	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
	cityBlock(viewer, pointProcessorI, inputCloudI);
	
	streamIterator++;
	if(streamIterator == stream.end()){
		streamIterator = stream.begin();
	}

	viewer->spinOnce();
	
    } 
}










