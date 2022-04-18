#include "myslam/testPoint.h"
<<<<<<< HEAD
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

=======
namespace myslam
{
    void testPoint::init()
    {
        {
            pointCloud = (new PointCloud())->makeShared();
            tmp = (new PointCloud())->makeShared();
        }
    }

    void testPoint::getCloudPoints(cv::Point2i location, Vector3d v, Mat color)
    {
        PointT t;
        t.x = v[0];
        t.y = v[1];
        t.z = v[2];

        t.b = color.data[int(location.y) * color.step + location.x * color.channels()];
        t.g = color.data[color.step + location.x * color.channels() + 1];
        t.r = color.data[location.y * color.step + location.x * color.channels() + 2];
        tmp->points.push_back(t);
    }
    void testPoint::updateCloud()
    {
        if(true){
            
        PointCloud::Ptr temp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(tmp);
        statistical_filter.filter(*temp);
        (*pointCloud) += (*temp);
        pcl::VoxelGrid<PointT> voex_filter;
        voex_filter.setLeafSize(0.01,0.01,0.01);
        PointCloud::Ptr temp2(new PointCloud);
        voex_filter.setInputCloud(pointCloud);
        voex_filter.filter(*temp2);
        temp2->swap(*pointCloud);
        bool isSave=pcl::io::savePCDFileBinary("testmap.pcd",*pointCloud);
        cout<<isSave<<endl;
        cout<<"size="<<pointCloud->points.size()<<endl;
        tmp->clear();
        }else{
            (*pointCloud)+=(*tmp);
            pcl::io::savePCDFileBinary("testmap.pcd",*pointCloud);
            cout<<"size="<<pointCloud->points.size()<<endl;
            tmp->clear();

        }
        
    }
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
}