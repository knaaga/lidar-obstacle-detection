/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>
#include <filesystem>

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
    //std::cout << "Current path is " << std::filesystem::current_path() << '\n';
	return pointProcessor.loadPcd("src/sensors/data/pcd/simpleHighway.pcd");
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
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliers;
    srand(time(NULL));

    // TODO: Fill in this function
    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    // For max iterations 
    int i = 0;
    double line = 0.0;
    double dist = 0.0;
    double den = 0.0;
    int ind1, ind2 = 0;

    while (i < maxIterations) {

        // clear this every iteration
        inliers.clear();

        // ensure same indices are not picked
        while (true) {
            ind1 = rand() % cloud->points.size();
            ind2 = rand() % cloud->points.size();
            if (ind1 != ind2)
                break;
        }

        // points to construct the line
        pcl::PointXYZ p1 = cloud->points[ind1];
        pcl::PointXYZ p2 = cloud->points[ind2];

        for (int ind = 0; ind < cloud->points.size(); ind++) {
            double x = cloud->points[ind].x;
            double y = cloud->points[ind].y;
            line = (p1.y - p2.y) * x + (p2.x - p1.x) * y + (p1.x * p2.y - p2.x * p1.y);
            den = sqrt(pow((p1.y - p2.y), 2) + pow((p2.x - p1.x), 2));
            dist = fabs(line / den);
            std::cout << "distance " << dist << std::endl;
            if (dist < distanceTol) {
                inliers.insert(ind);
            }
        }
        // return the set with maximum number of inliers
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
        std::cout << "END: Iteration " << i << ", with " << inliersResult.size() << " inlier points.\n\n";
        i++;
    }
    return inliersResult;
}


std::unordered_set<int> Ransac3d(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliers;
    srand(time(NULL));

    // TODO: Fill in this function
    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    // For max iterations 
    int i = 0;
    double plane = 0.0;
    double dist = 0.0;
    double den = 0.0;
    int ind1, ind2, ind3 = 0;
    double a1, b1, c1, a2, b2, c2, a, b, c, d = 0.0;
    double x1, x2, y1, y2, z1, z2, x3, y3, z3 = 0.0;
    double x, y, z = 0.0;
    while (i < maxIterations) {

        // clear this every iteration
        inliers.clear();

        // ensure same indices are not picked
        while (true) {
            ind1 = rand() % cloud->points.size();
            ind2 = rand() % cloud->points.size();
            ind3 = rand() % cloud->points.size();
            if (ind1 != ind2 && ind2 != ind3 && ind1 != ind3)
                break;
        }

        // points to construct the plane
        pcl::PointXYZ p1 = cloud->points[ind1];
        pcl::PointXYZ p2 = cloud->points[ind2];
        pcl::PointXYZ p3 = cloud->points[ind3];

        x1 = p1.x;
        x2 = p2.x;
        x3 = p3.x;

        y1 = p1.y;
        y2 = p2.y;
        y3 = p3.y;

        z1 = p1.z;
        z2 = p2.z;
        z3 = p3.z;

        a1 = x2 - x1;
        b1 = y2 - y1;
        c1 = z2 - z1;
        a2 = x3 - x1;
        b2 = y3 - y1;
        c2 = z3 - z1;
        a = b1 * c2 - b2 * c1;
        b = a2 * c1 - a1 * c2;
        c = a1 * b2 - b1 * a2;
        d = (-a * x1 - b * y1 - c * z1);

        for (int ind = 0; ind < cloud->points.size(); ind++) {
            x = cloud->points[ind].x;
            y = cloud->points[ind].y;
            z = cloud->points[ind].z;
            plane = a * x + b * y + c * z + d;
            den = sqrt(a * a + b * b + c * c);
            dist = fabs(plane) / den;
            std::cout << "distance " << dist << std::endl;
            if (dist < distanceTol) {
                inliers.insert(ind);
            }
        }
        // return the set with maximum number of inliers
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
        std::cout << "END: Iteration " << i << ", with " << inliersResult.size() << " inlier points.\n\n";
        i++;
    }

    return inliersResult;

}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3d(cloud, 50, 0.5);

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