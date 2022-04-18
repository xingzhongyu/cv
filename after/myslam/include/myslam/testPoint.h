<<<<<<< HEAD
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
=======
#ifndef MYPOINT_H
#define MYPOINT_H
#define PCL_NO_PRECOMPILE
// #include <pcl/visualization/pcl_visualizer.h>
#include "common_include.h"

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



namespace myslam
{
    class testPoint
    {
    public:
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef shared_ptr<testPoint> Ptr;

         PointCloud::Ptr pointCloud;
        PointCloud::Ptr tmp;
       
        void init();
         testPoint(){
            init();
        }
        void updateCloud();
        void getCloudPoints(cv::Point2i location, Vector3d v, Mat color);
        void outPoints(vector<double> &v);
        ~testPoint(){

        }
    };
}
#endif
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
