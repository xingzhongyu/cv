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

         PointCloud::Ptr pointCloud;
        PointCloud::Ptr tmp;
       
        void init();
         testPoint(){
            init();
        }
        void updateCloud();
        void getCloudPoints(cv::Point2i location, Vector3d v, Mat color);
        ~testPoint(){

        }
    };
}
#endif