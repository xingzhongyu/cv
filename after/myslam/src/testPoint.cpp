#include "myslam/testPoint.h"
static void getCloudPoints(cv::Point2i location ,Vector3d v,Mat color){
    PointT t;
    t.x=v[0];
    t.y=v[1];
    t.z=v[2];
        
    t.b=color.data[int(location.y)*color.step+location.x*color.channels()];
    t.g=color.data[color.step+location.x*color.channels()+1];
    t.r=color.data[location.y*color.step+location.x*color.channels()+2];
    tmp->points.push_back(t);
    

}
static void updateCloud(){
    PointCloud::Ptr temp(new PointCloud);
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    statistical_filter.setMeanK(50);
    statistical_filter.setStddevMulThresh(1.0);
    statistical_filter.setInputCloud(tmp);
    statistical_filter.filter(*temp);
    (*pointCloud)+=(*tmp);
    
    tmp=new PointCloud()->makeShared();

}