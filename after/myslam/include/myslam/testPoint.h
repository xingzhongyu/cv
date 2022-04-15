#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include "common_include.h"
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>

#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr pointCloud(new PointCloud);
PointCloud::Ptr tmp(new PointCloud);

static void updateCloud();
static void getCloudPoints(cv::Point2i location, Vector3d v,Mat color);